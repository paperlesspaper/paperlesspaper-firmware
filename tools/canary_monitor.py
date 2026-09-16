#!/usr/bin/env python3
"""
tools/canary_monitor.py
Canary Health-Gate Monitor für E-Paper Displays.
Überwacht eine definierte Gruppe von Pilotgeräten anhand von DynamoDB 'iotCatalog'
und 'iotPayload' auf Update-Erfolg, Batteriestabilität, Neustart-Zähler (Reboot-Loops)
und Event-Quittungen.

Funktionen:
- Query auf DynamoDB 'iotCatalog' (fwVersion, batLevel, StartCounter, lastUpdateTime).
- Query auf DynamoDB 'iotPayload' (Quittungs-Events wie 'update_ok').
- Erkennung von Crash-Loops (StartCounter >= 3) und toten Geräten (Heartbeat Timeout).
- Erzeugt Statusberichte in Markdown (für $GITHUB_STEP_SUMMARY) und JSON.
- Liefert Exit-Code 1 bei kritischen Anomalien, um CI/CD-Pipelines zu stoppen.
"""

import os
import sys
import json
import time
import argparse
from datetime import datetime, timezone

# Windows CLI Encoding-Fix
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

try:
    import boto3
    from botocore.exceptions import ClientError
except ImportError:
    boto3 = None
    ClientError = Exception


def get_dynamodb_resource(region=None):
    if boto3 is None:
        raise RuntimeError("Das Modul 'boto3' ist nicht installiert. Bitte 'pip install boto3' ausführen.")
    aws_region = region or os.environ.get("AWS_REGION", "eu-central-1")
    return boto3.resource("dynamodb", region_name=aws_region)


def load_target_devices(target_file=None, device_ids=None):
    """Lädt die Liste der zu überwachenden Zielgeräte aus Datei oder Argumenten."""
    targets = []
    if target_file and os.path.isfile(target_file):
        with open(target_file, "r", encoding="utf-8") as f:
            data = json.load(f)
            if isinstance(data, list):
                targets = [str(x).strip() for x in data if str(x).strip()]
            elif isinstance(data, dict):
                dev_list = data.get("devices") or data.get("target_devices") or []
                targets = [str(x).strip() for x in dev_list if str(x).strip()]
    elif device_ids:
        targets = [d.strip() for d in device_ids.split(",") if d.strip()]
    elif os.path.isfile("canary_target_devices.json"):
        with open("canary_target_devices.json", "r", encoding="utf-8") as f:
            data = json.load(f)
            if isinstance(data, list):
                targets = [str(x).strip() for x in data if str(x).strip()]
            elif isinstance(data, dict):
                dev_list = data.get("devices") or data.get("target_devices") or []
                targets = [str(x).strip() for x in dev_list if str(x).strip()]
    elif os.path.isfile("canary_target_devices.json.template"):
        with open("canary_target_devices.json.template", "r", encoding="utf-8") as f:
            data = json.load(f)
            if isinstance(data, list):
                targets = [str(x).strip() for x in data if str(x).strip()]
            elif isinstance(data, dict):
                dev_list = data.get("devices") or data.get("target_devices") or []
                targets = [str(x).strip() for x in dev_list if str(x).strip()]

    return targets


def query_device_catalog(catalog_table, device_id):
    """
    Fragt den Gerätezustand aus DynamoDB iotCatalog ab.
    Probiert Schlüsselvariationen (mit/ohne Prefix).
    """
    keys_to_try = [str(device_id)]
    if "-" in device_id:
        raw_mac = device_id.split("-", 1)[1]
        keys_to_try.extend([raw_mac, raw_mac.upper(), raw_mac.lower()])
    else:
        keys_to_try.extend([f"epd7-{device_id}", f"epd13-{device_id}", device_id.upper()])

    for k in keys_to_try:
        try:
            res = catalog_table.query(
                KeyConditionExpression="serialNumber = :s",
                ExpressionAttributeValues={":s": k},
                Limit=1
            )
            items = res.get("Items", [])
            if items:
                return items[0]
        except ClientError as e:
            print(f"⚠️ DynamoDB Catalog Query Fehler für '{k}': {e}")
            break
    return None


def query_device_payload_ack(payload_table, device_id, min_timestamp_ms=0):
    """
    Prüft in DynamoDB iotPayload, ob für das Gerät kürzlich ein Update-Quittierungs-Event
    (z. B. EventType 'state' oder EventMessage 'update_ok') abgelegt wurde.
    """
    candidates = [str(device_id)]
    if "-" in device_id:
        candidates.append(device_id.split("-", 1)[1])

    for cand in candidates:
        try:
            res = payload_table.query(
                KeyConditionExpression="DeviceId = :dev_id",
                ExpressionAttributeValues={":dev_id": cand},
                ScanIndexForward=False,
                Limit=5
            )
            for it in res.get("Items", []):
                ts = int(it.get("EventTimestamp") or it.get("AwsTimestamp") or 0)
                if ts >= min_timestamp_ms:
                    msg = str(it.get("EventMessage", "")).lower()
                    ev_type = str(it.get("EventType", "")).lower()
                    status = str(it.get("status", "")).lower()
                    if "update_ok" in msg or "update" in ev_type or "ok" in status:
                        return it
        except ClientError as e:
            print(f"⚠️ DynamoDB Payload Query Fehler für '{cand}': {e}")
            break
    return None


def evaluate_device_health(item, payload_ack, target_version=None, start_time_ts=0):
    """
    Bewertet den Gesundheitszustand eines Zielgeräts:
    - Version matchen (target_version erreicht?)
    - StartCounter (Crash-Loop-Erkennung)
    - Heartbeat / Timeout Überwachung
    - Batteriestand
    - Quittierungs-Event
    """
    if not item:
        return {
            "status": "NOT_FOUND",
            "is_healthy": False,
            "version_matched": False,
            "anomalies": ["Gerät nicht in DynamoDB iotCatalog gefunden"]
        }

    dev_data = item.get("deviceData", {}) if isinstance(item.get("deviceData"), dict) else {}
    current_version = str(item.get("fwVersion") or dev_data.get("firmware") or "0.0.0")

    # Version matchen
    version_matched = False
    if target_version:
        v_clean = target_version.lower().lstrip("bv")
        c_clean = current_version.lower().lstrip("bv")
        version_matched = (v_clean in c_clean) or (c_clean == v_clean)

    # StartCounter
    start_counter = dev_data.get("StartCounter") or item.get("StartCounter") or 0
    try:
        start_counter = int(start_counter)
    except (ValueError, TypeError):
        start_counter = 0

    # Batteriestand
    bat_raw = item.get("batLevel") or dev_data.get("batLevel") or item.get("vdd")
    bat_v = None
    if bat_raw is not None:
        try:
            b = float(bat_raw)
            bat_v = b / 1000.0 if b > 100 else b
        except (ValueError, TypeError):
            pass

    # Timeout & Last Update
    raw_timeout = dev_data.get("timeout") or item.get("timeout") or 180
    try:
        timeout_sec = int(raw_timeout)
    except (ValueError, TypeError):
        timeout_sec = 180

    last_update_raw = item.get("lastUpdateTime") or item.get("AwsTimestamp") or 0
    try:
        last_update_ms = int(last_update_raw)
        last_update_sec = last_update_ms / 1000.0 if last_update_ms > 10_000_000_000 else float(last_update_ms)
    except (ValueError, TypeError):
        last_update_sec = 0.0

    now = time.time()
    seconds_since_update = now - last_update_sec if last_update_sec > 0 else 999999

    # Anomalie-Erkennung
    anomalies = []

    # 1. Crash-Loop Erkennung
    if start_counter >= 3:
        anomalies.append(f"Achtung: Erhöhter StartCounter ({start_counter}) - möglicher Crash-Loop!")

    # 2. Akku-Kritisch
    if bat_v is not None and bat_v < 3.3:
        anomalies.append(f"Akku kritisch niedrig ({round(bat_v, 2)}V)")

    # 3. Heartbeat-Timeout (wenn länger als 3x das Timeout-Intervall stumm)
    max_silent_allowed = max(180, timeout_sec * 3.0)
    if start_time_ts > 0 and (last_update_sec < start_time_ts) and (now - start_time_ts > max_silent_allowed):
        anomalies.append(f"Gerät überfällig: Keine Meldung seit {int(now - start_time_ts)}s (Timeout: {timeout_sec}s)")

    # Status ermitteln
    if version_matched and not anomalies:
        health_status = "UPDATED_HEALTHY"
    elif version_matched and anomalies:
        health_status = "UPDATED_WARNING"
    elif not version_matched and anomalies:
        health_status = "FAILED_ANOMALY"
    elif seconds_since_update > max_silent_allowed:
        health_status = "PENDING_OVERDUE"
    else:
        health_status = "PENDING_WAITING"

    is_healthy = len(anomalies) == 0

    return {
        "status": health_status,
        "is_healthy": is_healthy,
        "version_matched": version_matched,
        "current_version": current_version,
        "target_version": target_version,
        "start_counter": start_counter,
        "bat_voltage": bat_v,
        "timeout_sec": timeout_sec,
        "last_update_sec": last_update_sec,
        "seconds_since_update": seconds_since_update,
        "has_ack": payload_ack is not None,
        "anomalies": anomalies
    }


def analyze_canary_fleet(catalog_table, payload_table, target_devices, target_version=None, start_time_ts=0, mock_items=None):
    """Fragt alle Zielgeräte ab und aggregiert Flottenmetriken."""
    results = {}
    updated_count = 0
    healthy_count = 0
    anomalies_count = 0

    for dev_id in target_devices:
        if mock_items and dev_id in mock_items:
            item = mock_items[dev_id].get("catalog")
            ack = mock_items[dev_id].get("payload")
        else:
            item = query_device_catalog(catalog_table, dev_id)
            min_ts_ms = int(start_time_ts * 1000) if start_time_ts else 0
            ack = query_device_payload_ack(payload_table, dev_id, min_timestamp_ms=min_ts_ms)

        evaluation = evaluate_device_health(
            item,
            ack,
            target_version=target_version,
            start_time_ts=start_time_ts
        )

        results[dev_id] = evaluation

        if evaluation["version_matched"]:
            updated_count += 1
        if evaluation["is_healthy"]:
            healthy_count += 1
        if evaluation["anomalies"]:
            anomalies_count += 1

    total = len(target_devices)
    adoption_rate = round((updated_count / total * 100.0), 1) if total > 0 else 0.0
    healthy_rate = round((healthy_count / total * 100.0), 1) if total > 0 else 0.0

    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "total_targets": total,
        "updated_count": updated_count,
        "adoption_rate_pct": adoption_rate,
        "healthy_count": healthy_count,
        "healthy_rate_pct": healthy_rate,
        "anomalies_count": anomalies_count,
        "target_version": target_version,
        "devices": results
    }


def generate_markdown_report(metrics, output_file="canary_health_report.md"):
    """Erzeugt einen detaillierten Markdown-Bericht für GitHub Actions und Pull Requests."""
    total = metrics["total_targets"]
    updated = metrics["updated_count"]
    adoption = metrics["adoption_rate_pct"]
    anomalies = metrics["anomalies_count"]
    target_v = metrics.get("target_version") or "nicht angegeben"

    status_badge = "🟢 BESTANDEN" if (adoption >= 100.0 and anomalies == 0) else (
        "🟡 IN ARBEIT" if anomalies == 0 else "🔴 ANOMALIEN ERKANNT"
    )

    lines = [
        "# 🛡️ Canary Health-Gate Statusbericht",
        f"**Gesamtstatus:** {status_badge} | **Ziel-Version:** `{target_v}`",
        f"*Zeitstempel: {metrics['timestamp']}*",
        "",
        "### 📊 Zusammenfassung",
        f"- **Überwachte Pilotgeräte:** {total}",
        f"- **Erfolgreich aktualisiert:** {updated} / {total} (**{adoption}%**)",
        f"- **Stabile Geräte:** {metrics['healthy_count']} / {total} (**{metrics['healthy_rate_pct']}%**)",
        f"- **Erkannte Anomalien/Warnungen:** **{anomalies}**",
        "",
        "### 📋 Gerätestatus-Details",
        "| Gerät / Thing | Status | Version | Akku | StartCounter | Quittung | Anomalien / Hinweise |",
        "| :--- | :--- | :--- | :--- | :--- | :--- | :--- |"
    ]

    for dev_id, d in metrics["devices"].items():
        st = d.get("status", "UNKNOWN")
        st_icon = {
            "UPDATED_HEALTHY": "🟢 Aktualisiert",
            "UPDATED_WARNING": "🟡 Aktualisiert (Warnung)",
            "PENDING_WAITING": "⏳ Ausstehend (Wartet)",
            "PENDING_OVERDUE": "🟠 Überfällig",
            "FAILED_ANOMALY": "🔴 Fehler",
            "NOT_FOUND": "⚪ Nicht gefunden"
        }.get(st, st)

        v_str = f"`{d.get('current_version', '-')}`"
        bat = d.get("bat_voltage")
        bat_str = f"{round(bat, 2)}V" if bat is not None else "-"
        sc = d.get("start_counter", "-")
        ack_str = "✅ Vorhanden" if d.get("has_ack") else "⏳ Ausstehend"
        anom_str = "; ".join(d.get("anomalies", [])) or "Keine"

        lines.append(f"| `{dev_id}` | {st_icon} | {v_str} | {bat_str} | {sc} | {ack_str} | {anom_str} |")

    lines.append("")

    if anomalies > 0:
        lines.append("> [!WARNING]")
        lines.append("> **Achtung:** Es wurden Anomalien bei den Pilotgeräten festgestellt!")
        lines.append("> Bitte prüfe Crash-Loops (`StartCounter`) oder Timeouts vor dem weiteren Rollout.")
        lines.append("")

    content = "\n".join(lines)
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"📄 Canary Health-Gate Markdown gespeichert: {output_file}")

    # Falls in GitHub Actions: In GITHUB_STEP_SUMMARY schreiben
    summary_path = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary_path and os.path.exists(os.path.dirname(summary_path)):
        try:
            with open(summary_path, "a", encoding="utf-8") as sf:
                sf.write("\n" + content + "\n")
            print("🚀 Bericht erfolgreich an $GITHUB_STEP_SUMMARY angehängt.")
        except Exception as e:
            print(f"⚠️ Konnte nicht in GITHUB_STEP_SUMMARY schreiben: {e}")


def generate_json_report(metrics, output_file="canary_health_report.json"):
    """Speichert die maschinenlesbaren Kennzahlen in einer JSON-Datei."""
    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)
    print(f"💾 Canary Health-Gate JSON gespeichert: {output_file}")


def resolve_target_version(target_version=None, s3_bucket=None, branch=None):
    """Ermittelt die Ziel-Firmware-Version automatisch aus S3-Manifest oder types.h."""
    if target_version:
        return target_version

    bucket = s3_bucket or os.environ.get("HIL_S3_BUCKET") or os.environ.get("S3_BUCKET_NAME") or "ul.epaperframe.de"
    branch_name = branch or os.environ.get("GITHUB_REF_NAME") or "dev"
    mode = "pre" if branch_name in ("main", "master") else "dev"

    if bucket:
        import urllib.request
        for target in ("epd7", "epd13"):
            url = f"http://{bucket}/espfota_{target}_{mode}.json"
            try:
                req = urllib.request.Request(url, headers={"User-Agent": "canary-monitor/1.0"})
                with urllib.request.urlopen(req, timeout=5) as res:
                    data = json.loads(res.read().decode("utf-8"))
                    v = data.get("version")
                    if v and v != "X.X.X" and v != "0.0.0":
                        print(f"🔍 Automatisch erkannte Ziel-Version von S3 ({url}): {v}")
                        return v
            except Exception:
                pass

    try:
        types_path = os.path.join(os.path.dirname(__file__), "..", "src", "types.h")
        if os.path.isfile(types_path):
            with open(types_path, "r", encoding="utf-8", errors="ignore") as f:
                content = f.read()
                import re
                m = re.search(r'#define\s+SOFTWARE_VERSION\s+"([^"]+)"', content)
                if m and m.group(1) not in ("0.0.0", "X.X.X"):
                    print(f"🔍 Automatisch erkannte Ziel-Version aus types.h: {m.group(1)}")
                    return m.group(1)
    except Exception:
        pass

    return None


def main():
    parser = argparse.ArgumentParser(
        description="Canary Health-Gate Monitor für E-Paper Displays (DynamoDB iotCatalog & iotPayload)"
    )
    parser.add_argument("--target-devices", help="Pfad zur JSON-Datei mit freigegebenen Zielgeräten (z. B. canary_target_devices.json)")
    parser.add_argument("--device-ids", help="Kommagetrennte Liste von Zielgeräten (z. B. 'epd7-xxx,epd13-yyy')")
    parser.add_argument("--target-version", help="Erwartete Firmware-Version nach dem Update (z. B. '3.0.57')")
    parser.add_argument("--table-catalog", default="iotCatalog", help="DynamoDB Tabelle iotCatalog")
    parser.add_argument("--table-payload", default="iotPayload", help="DynamoDB Tabelle iotPayload")
    parser.add_argument("--region", default=os.environ.get("AWS_REGION", "eu-central-1"), help="AWS Region")
    parser.add_argument("--output-md", default="canary_health_report.md", help="Pfad zum Markdown-Bericht")
    parser.add_argument("--output-json", default="canary_health_report.json", help="Pfad zum JSON-Bericht")
    parser.add_argument("--min-adoption", type=float, default=80.0, help="Erforderliche Erfolgsquote in Prozent (Standard: 80.0)")
    parser.add_argument("--max-anomalies", type=int, default=0, help="Maximal tolerierte Anomalien (Standard: 0)")
    parser.add_argument("--watch", action="store_true", help="Kontinuierliches Polling bis Erfolgsquote erreicht ist oder Timeout")
    parser.add_argument("--interval", type=int, default=30, help="Polling-Intervall in Sekunden (Standard: 30)")
    parser.add_argument("--timeout", type=int, default=600, help="Maximales Polling-Timeout in Sekunden (Standard: 600s / 10 Min)")
    parser.add_argument("--mock-file", help="Pfad zu Mock-Daten für Offline-/Testzwecke")
    args = parser.parse_args()

    print("=" * 65)
    print("🛡️ CANARY HEALTH-GATE MONITOR (PHASE 3)")
    print("=" * 65)

    targets = load_target_devices(target_file=args.target_devices, device_ids=args.device_ids)
    if not targets and not args.mock_file:
        print("❌ FEHLER: Keine Zielgeräte angegeben! Bitte --target-devices oder --device-ids übergeben.")
        sys.exit(1)

    mock_items = None
    if args.mock_file and os.path.isfile(args.mock_file):
        print(f"📦 Verwende Mock-Daten aus: {args.mock_file}")
        with open(args.mock_file, "r", encoding="utf-8") as f:
            mock_items = json.load(f)
            if not targets:
                targets = list(mock_items.keys())

    # Automatische Erkennung der Ziel-Version
    detected_version = resolve_target_version(
        target_version=args.target_version,
        s3_bucket=os.environ.get("HIL_S3_BUCKET") or os.environ.get("S3_BUCKET_NAME"),
        branch=os.environ.get("GITHUB_REF_NAME") or os.environ.get("BRANCH_NAME")
    )

    print(f"🎯 Überwachte Zielgeräte ({len(targets)}): {', '.join(targets)}")
    print(f"📌 Erwartete Version: {detected_version or 'jede (automatisch)'}")
    print(f"📊 Schwellenwerte: Min. Adoption={args.min_adoption}%, Max. Anomalien={args.max_anomalies}\n")

    catalog_table = None
    payload_table = None
    if not mock_items:
        dynamo = get_dynamodb_resource(region=args.region)
        catalog_table = dynamo.Table(args.table_catalog)
        payload_table = dynamo.Table(args.table_payload)

    start_time_ts = time.time()

    # Monitoring-Schleife oder Single-Pass
    while True:
        metrics = analyze_canary_fleet(
            catalog_table,
            payload_table,
            targets,
            target_version=detected_version,
            start_time_ts=start_time_ts,
            mock_items=mock_items
        )

        generate_markdown_report(metrics, output_file=args.output_md)
        generate_json_report(metrics, output_file=args.output_json)

        is_passed = (
            metrics["adoption_rate_pct"] >= args.min_adoption and
            metrics["anomalies_count"] <= args.max_anomalies
        )

        elapsed = int(time.time() - start_time_ts)
        print(f"⏱️ Laufzeit: {elapsed}s | Adoption: {metrics['adoption_rate_pct']}% | Anomalien: {metrics['anomalies_count']}")

        if not args.watch or is_passed:
            break

        if elapsed >= args.timeout:
            print(f"\n⏰ Timeout von {args.timeout}s erreicht, bevor alle Kriterien erfüllt wurden.")
            break

        time.sleep(args.interval)

    print("=" * 65)
    if is_passed:
        print("🎉 CANARY HEALTH-GATE ERFOLGREICH BESTANDEN!")
        print(f"   Alle {len(targets)} Pilotgeräte stabil auf Version {args.target_version}.")
        print("=" * 65)
        sys.exit(0)
    else:
        print("❌ CANARY HEALTH-GATE BLOCKIERT:")
        print(f"   Adoption: {metrics['adoption_rate_pct']}% (Ziel: {args.min_adoption}%)")
        print(f"   Anomalien: {metrics['anomalies_count']} (Max: {args.max_anomalies})")
        print("=" * 65)
        sys.exit(1)


if __name__ == "__main__":
    main()
