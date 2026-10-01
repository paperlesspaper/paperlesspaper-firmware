#!/usr/bin/env python3
"""
tools/select_pilot_devices.py
Flotten-Scan & Empfehlungs-Generator für Pilotkunden-Geräte (Canary Release).
Liest DynamoDB 'iotCatalog', filtert nach Modellen (EPD7/EPD13), Aufwachraten
(deviceData.timeout in [60, 180]) und Stabilitätsmetriken (batLevel, StartCounter).

SICHERHEITS-GARANTIE (Zero-OTA Safe-Mode):
- Läuft standardmäßig im schreibgeschützten Modus (Dry-Run / Read-Only).
- Erstellt eine Empfehlungsliste (Markdown & JSON).
- Der Anwender legt die tatsächlichen Zielgeräte selbst fest.
- Ein echtes Schreiben auf AWS IoT Named Shadows ('settings.otaUrl') erfordert
  zwingend sowohl '--apply' ALS AUCH '--confirm-ota I_CONFIRM_CANARY_OTA'
  sowie eine explizite Target-Geräteliste.
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

CONFIRMATION_PHRASE = "I_CONFIRM_CANARY_OTA"

_REGISTERED_MASKS = set()


def register_github_mask(value):
    """Registriert sensible Werte bei GitHub Actions als Secret für automatische Konsolenmaskierung."""
    if not value or not os.environ.get("GITHUB_ACTIONS"):
        return
    val_str = str(value).strip()
    if len(val_str) < 5 or val_str in _REGISTERED_MASKS:
        return
    if val_str.lower() in ("true", "false", "epd7", "epd13", "relay_hex", "eu-central-1", "all", "none"):
        return
    _REGISTERED_MASKS.add(val_str)
    print(f"::add-mask::{val_str}")


def mask_epd_id(uid):
    """
    Maskiert eine E-Paper Gerätekennung für Berichte und Protokolle.
    Sicherheits-Garantie:
    - Maskiert den Großteil der Seriennummer/MAC mit '***'
    - Macht AUF JEDEN FALL die vorletzte Stelle unkenntlich (ersetzt durch '*')
    - Verhindert Rückschlüsse auf Kunden oder Hardware-Batches

    Beispiele:
    - 'epd7-jidf3d4'  -> 'epd7-***3*4'  
    - 'epd13-ksdfj92d8' -> 'epd13-***2*8' 
    - 'epd7-704988'        -> 'epd7-***9*8'  
    - '58e6c5c29248'       -> '***2*8'
    """
    if not uid:
        return ""
    uid_str = str(uid).strip()
    register_github_mask(uid_str)

    prefix = ""
    rest = uid_str
    if "-" in uid_str:
        prefix, rest = uid_str.split("-", 1)
        prefix = f"{prefix}-"

    if len(rest) <= 2:
        return f"{prefix}***"
    if len(rest) == 3:
        return f"{prefix}***{rest[0]}*{rest[-1]}"
    return f"{prefix}***{rest[-3]}*{rest[-1]}"


def _load_env():
    possible_paths = [
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".env")),
        os.path.expanduser("~/.paperlesspaper.env"),
        os.path.expanduser("~/.env")
    ]
    for env_path in possible_paths:
        if os.path.isfile(env_path):
            try:
                with open(env_path, "r", encoding="utf-8", errors="ignore") as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith("#") and "=" in line:
                            k, v = line.split("=", 1)
                            k, v = k.strip(), v.strip().strip('"\'')
                            if k not in os.environ:
                                os.environ[k] = v
            except Exception:
                pass

_load_env()


def get_dynamodb_resource(region=None):
    if boto3 is None:
        raise RuntimeError("Das Modul 'boto3' ist nicht installiert. Bitte 'pip install boto3' ausführen.")
    aws_region = region or os.environ.get("AWS_REGION", "eu-central-1")
    return boto3.resource("dynamodb", region_name=aws_region)


def get_iot_data_client(region=None, endpoint_url=None):
    if boto3 is None:
        raise RuntimeError("Das Modul 'boto3' ist nicht installiert. Bitte 'pip install boto3' ausführen.")
    aws_region = region or os.environ.get("AWS_REGION", "eu-central-1")
    ep = endpoint_url or os.environ.get("AWS_IOT_ENDPOINT")
    kwargs = {"region_name": aws_region}
    if ep:
        kwargs["endpoint_url"] = ep
    return boto3.client("iot-data", **kwargs)


def scan_all_epaper_devices(table_name="iotCatalog", region=None):
    """
    Führt einen vollständigen paginierten Scan über die DynamoDB-Tabelle 'iotCatalog' durch,
    um alle registrierten E-Paper-Geräte abzurufen.
    """
    dynamo = get_dynamodb_resource(region=region)
    table = dynamo.Table(table_name)

    print(f"🔍 [SCAN] Starte Scan auf DynamoDB-Tabelle '{table_name}'...")
    devices = []
    scan_kwargs = {
        "FilterExpression": "begins_with(serialNumber, :prefix)",
        "ExpressionAttributeValues": {":prefix": "epd"}
    }

    page_num = 1
    while True:
        try:
            resp = table.scan(**scan_kwargs)
        except ClientError as e:
            print(f"❌ Fehler beim Scan auf DynamoDB-Tabelle '{table_name}': {e}")
            raise

        items = resp.get("Items", [])
        devices.extend(items)
        print(f"   ➔ Seite {page_num}: {len(items)} Geräte empfangen (Gesamt: {len(devices)})")

        last_key = resp.get("LastEvaluatedKey")
        if not last_key:
            break
        scan_kwargs["ExclusiveStartKey"] = last_key
        page_num += 1

    print(f"✅ [SCAN] Scan abgeschlossen: {len(devices)} E-Paper-Geräte gefunden.\n")
    return devices


def parse_device_metrics(item):
    """
    Normalisiert und extrahiert die relevanten Metriken eines DynamoDB-Items.
    """
    if not isinstance(item, dict):
        return {
            "serialNumber": str(item),
            "model": "UNKNOWN",
            "active": False,
            "activation_status": "invalid",
            "timeout": None,
            "start_counter": None,
            "bat_level": None,
            "last_update_ts": 0.0,
            "fw_version": "unbekannt",
            "organization": "-",
            "raw": {}
        }

    sn = str(item.get("serialNumber", "")).strip()

    # Modell erkennen
    model = "UNKNOWN"
    if sn.lower().startswith("epd7") or item.get("epdType") == "7INCH":
        model = "EPD7"
    elif sn.lower().startswith("epd13") or item.get("epdType") == "13INCH":
        model = "EPD13"

    # Aktivierungsstatus
    is_activated = item.get("activated") is True or str(item.get("activated", "")).lower() == "true"
    act_status = str(item.get("activation_status", "")).lower()
    active = is_activated and act_status in ("activated", "active")

    # deviceData Dict
    dev_data = item.get("deviceData", {})
    if not isinstance(dev_data, dict):
        dev_data = {}

    # Timeout (Aufwachintervall)
    timeout = None
    raw_timeout = dev_data.get("timeout") or item.get("timeout")
    if raw_timeout is not None:
        try:
            timeout = int(raw_timeout)
        except (ValueError, TypeError):
            timeout = None

    # StartCounter (Reboot-Zähler)
    start_counter = None
    raw_counter = dev_data.get("StartCounter") or item.get("StartCounter")
    if raw_counter is not None:
        try:
            start_counter = int(raw_counter)
        except (ValueError, TypeError):
            start_counter = None

    # Batteriestand / Spannung
    bat_level = None
    raw_bat = item.get("batLevel") or dev_data.get("batLevel") or item.get("vdd")
    if raw_bat is not None:
        try:
            bat_level = float(raw_bat)
        except (ValueError, TypeError):
            bat_level = None

    # Letzter Kontakt (ms oder s)
    last_update = 0
    raw_update = item.get("lastUpdateTime") or item.get("AwsTimestamp") or item.get("EventTimestamp") or 0
    try:
        last_update = int(raw_update)
        # Falls in ms angegeben (13 Stellen), in Sekunden umrechnen
        if last_update > 10_000_000_000:
            last_update_sec = last_update / 1000.0
        else:
            last_update_sec = float(last_update)
    except (ValueError, TypeError):
        last_update_sec = 0.0

    fw_version = str(item.get("fwVersion") or dev_data.get("firmware") or "unbekannt")
    org_name = str(item.get("organizationName") or dev_data.get("organizationName") or "-")

    return {
        "serialNumber": sn,
        "model": model,
        "active": active,
        "activation_status": act_status,
        "timeout": timeout,
        "start_counter": start_counter,
        "bat_level": bat_level,
        "last_update_ts": last_update_sec,
        "fw_version": fw_version,
        "organization": org_name,
        "raw": item
    }


def calculate_suitability_score(device_info, allowed_timeouts=(60, 180), max_age_days=7, max_age_hours=1.0):
    """
    Berechnet einen Eignungs-Score (0 bis 100) für Canary-Updates:
    - Nur aktive Geräte erhalten Punkte.
    - Nur Geräte, die vor weniger als max_age_hours online waren (Standard: 1.0h).
    - Timeout 60s (hohe Frequenz): +40 Pkt, Timeout 180s: +30 Pkt.
    - Kürzliche Aktivität: Je neuer, desto höher (bis zu +30 Pkt).
    - Niedriger StartCounter (<= 1): +15 Pkt, (== 2): +5 Pkt, (>= 3): -30 Pkt (Crash-Loop-Gefahr!).
    - Batteriestatus (>= 3.7V oder >= 3700mV oder >= 80%): +15 Pkt.
    """
    if not device_info["active"]:
        return 0.0, ["Gerät ist nicht aktiv/aktiviert"]

    now = time.time()
    age_sec = now - device_info["last_update_ts"] if device_info["last_update_ts"] > 0 else 99999999
    age_hours = age_sec / 3600.0

    if max_age_hours is not None and max_age_hours > 0 and age_hours > max_age_hours:
        return 0.0, [f"Zu lange inaktiv (vor > {max_age_hours}h online)"]

    notes = []
    score = 0.0

    # 1. Timeout / Aufwachrate
    timeout = device_info["timeout"]
    if timeout in allowed_timeouts:
        if timeout == 60:
            score += 40.0
            notes.append("Hohe Aufwachrate (60s)")
        elif timeout == 180:
            score += 30.0
            notes.append("Mittlere Aufwachrate (180s)")
        else:
            score += 20.0
            notes.append(f"Passendes Intervall ({timeout}s)")
    else:
        score += 5.0
        notes.append(f"Ungewöhnliches Intervall ({timeout}s)")

    # 2. Aktualität (lastUpdateTime)
    if age_hours <= 1.0:
        score += 30.0
        notes.append("Vor < 1h online")
    elif age_hours <= 24.0:
        score += 25.0
        notes.append("Vor < 24h online")
    elif age_hours <= max_age_days * 24.0:
        score += 15.0
        notes.append(f"Vor {int(age_hours/24)}d online")
    else:
        notes.append("Lange inaktiv (> 7 Tage)")
        score -= 20.0

    # 3. StartCounter (Reboot-Stabilität)
    sc = device_info["start_counter"]
    if sc is not None:
        if sc <= 1:
            score += 15.0
            notes.append("Sehr stabiler StartCounter (<=1)")
        elif sc == 2:
            score += 8.0
            notes.append("Normaler StartCounter (2)")
        elif sc >= 4:
            score -= 30.0
            notes.append(f"Achtung: Hoher StartCounter ({sc}) - möglicher Crash-Loop")
    else:
        score += 5.0

    # 4. Batteriestand
    bat = device_info["bat_level"]
    if bat is not None:
        # Falls in mV angegeben (z. B. 4150)
        voltage = bat / 1000.0 if bat > 100 else bat
        if voltage >= 3.8:
            score += 15.0
            notes.append(f"Akku sehr gut (~{round(voltage, 1)}V)")
        elif voltage >= 3.6:
            score += 10.0
            notes.append(f"Akku ausreichend (~{round(voltage, 1)}V)")
        elif voltage < 3.4:
            score -= 25.0
            notes.append(f"Akku schwach (~{round(voltage, 1)}V)")

    final_score = max(0.0, min(100.0, round(score, 1)))
    return final_score, notes


def generate_recommendations(devices, target_count=10, allowed_timeouts=(60, 180), max_age_hours=1.0):
    """
    Filtert und bewertet alle gescannten Geräte und liefert je Modell
    (EPD7 und EPD13) eine Liste der am besten geeigneten Kandidaten.
    Wählt für Empfehlungen ausschließlich Geräte aus, die vor weniger als max_age_hours (Standard: 1.0h)
    zuletzt online waren.
    """
    epd7_candidates = []
    epd13_candidates = []
    now = time.time()

    for dev in devices:
        parsed = parse_device_metrics(dev)
        score, notes = calculate_suitability_score(
            parsed,
            allowed_timeouts=allowed_timeouts,
            max_age_hours=max_age_hours
        )
        parsed["score"] = score
        parsed["notes"] = notes
        parsed["age_hours"] = (now - parsed["last_update_ts"]) / 3600.0 if parsed["last_update_ts"] > 0 else 99999999

        if parsed["model"] == "EPD7":
            epd7_candidates.append(parsed)
        elif parsed["model"] == "EPD13":
            epd13_candidates.append(parsed)

    # Nur aktive Geräte mit Score > 0 und Aktivität vor < max_age_hours für Empfehlungen zulassen
    def _is_eligible(d):
        if not d.get("active") or d.get("score", 0) <= 0:
            return False
        if max_age_hours is not None and max_age_hours > 0:
            return d.get("age_hours", 99999) <= max_age_hours
        return True

    eligible_epd7 = [d for d in epd7_candidates if _is_eligible(d)]
    eligible_epd13 = [d for d in epd13_candidates if _is_eligible(d)]

    # Nach Score absteigend sortieren, bei Punktgleichheit nach kürzestem Timeout (z. B. 60s vor 180s)
    # und anschließend nach neuester Aktivität
    def _sort_key(d):
        t = d.get("timeout")
        timeout_prio = -t if (t is not None and t > 0) else -999999
        return (d["score"], timeout_prio, d["last_update_ts"])

    eligible_epd7.sort(key=_sort_key, reverse=True)
    eligible_epd13.sort(key=_sort_key, reverse=True)

    recommended_epd7 = eligible_epd7[:target_count]
    recommended_epd13 = eligible_epd13[:target_count]

    return {
        "epd7": {
            "total_found": len(eligible_epd7),
            "total_catalog": len(epd7_candidates),
            "recommendations": recommended_epd7,
            "all_scored": epd7_candidates
        },
        "epd13": {
            "total_found": len(eligible_epd13),
            "total_catalog": len(epd13_candidates),
            "recommendations": recommended_epd13,
            "all_scored": epd13_candidates
        }
    }


def write_recommendations_markdown(results, output_file="pilot_recommendations.md"):
    """Schreibt einen übersichtlichen Markdown-Bericht mit Tabellen für EPD7 und EPD13."""
    lines = [
        "# 📋 Canary Pilot-Geräte Empfehlungsbericht",
        f"*Generiert am: {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')}*",
        "",
        "> [!NOTE]",
        "> **Sicherheitshinweis (Zero-OTA):** Dies ist eine reine Empfehlungsliste.",
        "> Es wurden **keine** Firmware-Updates ausgelöst. Übertrage die von dir gewünschten Geräte",
        "> in die Datei `canary_target_devices.json`.",
        ""
    ]

    for model_key, title in [("epd7", "EPD7 (7.5\" Hardware)"), ("epd13", "EPD13 (13.3\" Hardware)")]:
        data = results[model_key]
        recs = data["recommendations"]
        lines.append(f"## {title}")
        lines.append(f"*Gefundene Kandidaten (vor < 1h online): {data['total_found']} | Empfohlene Top-Auswahl: {len(recs)}*")
        lines.append("")

        if not recs:
            lines.append("*(Keine geeigneten aktiven Kandidaten mit den gewählten Kriterien gefunden)*\n")
            continue

        lines.append("| Rang | Seriennummer / Thing | Score | Timeout | Firmware | Batterie | Zuletzt aktiv | Begründung |")
        lines.append("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |")

        for idx, dev in enumerate(recs, 1):
            ts_str = "-"
            if dev["last_update_ts"] > 0:
                age_h = (time.time() - dev["last_update_ts"]) / 3600.0
                if age_h <= 1.0:
                    ts_str = "Vor < 1h"
                elif age_h <= 24.0:
                    ts_str = f"Vor ~{int(age_h)}h"
                elif age_h <= 168.0:
                    ts_str = f"Vor ~{int(age_h / 24)}d"
                else:
                    ts_str = "> 7 Tage"

            bat_str = "-"
            if dev["bat_level"] is not None:
                v = dev["bat_level"] / 1000.0 if dev["bat_level"] > 100 else dev["bat_level"]
                bat_str = f"~{round(v, 1)}V"

            notes_str = "; ".join(dev["notes"])
            masked_id = mask_epd_id(dev["serialNumber"])
            lines.append(
                f"| **#{idx}** | `{masked_id}` | **{dev['score']}/100** | {dev['timeout']}s | "
                f"`{dev['fw_version']}` | {bat_str} | {ts_str} | {notes_str} |"
            )
        lines.append("")

    content = "\n".join(lines)
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"📄 Markdown-Empfehlungsbericht gespeichert: {output_file}")


def write_recommendations_json(results, output_file="pilot_recommendations.json"):
    """Exportiert die strukturierte JSON-Empfehlungsliste mit datenschutzgerechter Maskierung."""
    def _mask_entries(entries):
        out = []
        for d in entries:
            v = None
            if d.get("bat_level") is not None:
                raw_v = d["bat_level"]
                v = round(raw_v / 1000.0 if raw_v > 100 else raw_v, 1)
            out.append({
                "serialNumber": mask_epd_id(d["serialNumber"]),
                "score": d["score"],
                "timeout": d["timeout"],
                "fw_version": d["fw_version"],
                "bat_level": v,
                "notes": d["notes"]
            })
        return out

    serializable = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "epd7": _mask_entries(results["epd7"]["recommendations"]),
        "epd13": _mask_entries(results["epd13"]["recommendations"])
    }

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(serializable, f, indent=2)
    print(f"💾 JSON-Empfehlungen gespeichert: {output_file}")


def write_template_target_file(results, template_file="canary_target_devices.json.template"):
    """Erzeugt eine beispielhafte Target-Vorlage mit Platzhaltern (schützt Kunden-Seriennummern), falls nicht vorhanden."""
    if os.path.exists(template_file):
        return

    template_data = {
        "description": "Vom Nutzer manuell freigegebene Zielgeräte für das Canary Pre-Release",
        "created_at": "2026-01-01T00:00:00+00:00",
        "candidate_version": "b3.0.X",
        "devices": [
            "epd7-XX",
            "epd7-XX",
            "epd7-XX",
            "epd13-XX",
            "epd13-XX",
            "epd13-XX"
        ]
    }
    with open(template_file, "w", encoding="utf-8") as f:
        json.dump(template_data, f, indent=2)
    print(f"📝 Vorlage für Zielgeräte erstellt: {template_file}")


def print_copy_paste_recommendations(results):
    """
    Gibt die empfohlenen Pilotgeräte als kommagetrennte Liste und fertige CLI-Befehle
    im Terminal aus, damit sie direkt für Canary-Monitor oder Rollout kopiert werden können.
    """
    epd7_ids = [d["serialNumber"] for d in results.get("epd7", {}).get("recommendations", [])]
    epd13_ids = [d["serialNumber"] for d in results.get("epd13", {}).get("recommendations", [])]
    all_ids = epd7_ids + epd13_ids

    for dev_id in all_ids:
        register_github_mask(dev_id)

    if not all_ids:
        print("\nℹ️ Keine geeigneten Pilotgeräte für Copy & Paste gefunden.")
        return

    all_ids_str = ",".join(all_ids)
    epd7_str = ",".join(epd7_ids)
    epd13_str = ",".join(epd13_ids)

    print("\n" + "=" * 65, flush=True)
    print("📋 COPY & PASTE FÜR CANARY MONITOR & ROLLOUT", flush=True)
    print("=" * 65, flush=True)
    print(f"🎯 Alle empfohlenen Pilotgeräte ({len(all_ids)}):", flush=True)
    print(f'"{all_ids_str}"', flush=True)

    if epd7_ids:
        print(f"\n  📺 Nur EPD7 ({len(epd7_ids)}):", flush=True)
        print(f'  "{epd7_str}"', flush=True)

    if epd13_ids:
        print(f"\n  📺 Nur EPD13 ({len(epd13_ids)}):", flush=True)
        print(f'  "{epd13_str}"', flush=True)

    print("\n💡 Fertige CLI-Befehle zum direkten Ausführen:", flush=True)
    print("▶ Canary Health-Gate Überwachung starten (Alle):", flush=True)
    print(f'  python tools/canary_monitor.py --device-ids "{all_ids_str}" --watch', flush=True)

    if epd7_str:
        print("\n▶ Canary Health-Gate Überwachung (Nur EPD7):", flush=True)
        print(f'  python tools/canary_monitor.py --device-ids "{epd7_str}" --watch', flush=True)

    if epd13_str:
        print("\n▶ Canary Health-Gate Überwachung (Nur EPD13):", flush=True)
        print(f'  python tools/canary_monitor.py --device-ids "{epd13_str}" --watch', flush=True)

    print("\n▶ Canary Rollout ausführen (Trockenlauf / Dry-Run):", flush=True)
    print(f'  python tools/select_pilot_devices.py --device-ids "{all_ids_str}"', flush=True)
    print("\n▶ Canary Rollout ausführen (Echt / Apply):", flush=True)
    print(f'  python tools/select_pilot_devices.py --device-ids "{all_ids_str}" --apply --confirm-ota {CONFIRMATION_PHRASE}', flush=True)
    print("=" * 65 + "\n", flush=True)


def apply_ota_to_selected_devices(target_devices, ota_url, dry_run=True, confirm_token="", region=None):
    """
    Setzt den Named Shadow 'settings.otaUrl' für die explizit vom Nutzer ausgewählten Geräte.
    Strikter Schutz: Führt bei dry_run=True KEINE Netzwerkanfragen durch.
    """
    print("=" * 65)
    print("🚀 CANARY OTA TARGETING ENGINE")
    print("=" * 65)
    print(f"  Ziel-FOTA URL:   {ota_url}")
    print(f"  Anzahl Zielgeräte: {len(target_devices)}")
    print(f"  Dry-Run Modus:    {dry_run}")
    print("=" * 65)

    if not target_devices:
        print("⚠️ Keine Zielgeräte übergeben. Vorgang beendet.")
        return

    if not dry_run:
        if confirm_token != CONFIRMATION_PHRASE:
            print(f"🛑 SICHERHEITSBLOCKADE: Ungültiger Bestätigungstoken!")
            print(f"   Erforderlich: --confirm-ota {CONFIRMATION_PHRASE}")
            print("   Keine Änderungen an AWS IoT Shadows vorgenommen.")
            sys.exit(1)

    iot_client = None if dry_run else get_iot_data_client(region=region)

    for idx, dev_id in enumerate(target_devices, 1):
        thing_name = dev_id.strip()
        register_github_mask(thing_name)
        dev_ota_url = ota_url
        if "{target}" in dev_ota_url:
            model = "epd13" if "13" in thing_name.lower() else "epd7"
            dev_ota_url = dev_ota_url.replace("{target}", model)
        elif "epd7" in dev_ota_url and "13" in thing_name.lower():
            dev_ota_url = dev_ota_url.replace("epd7", "epd13")
        elif "epd13" in dev_ota_url and ("epd7" in thing_name.lower() or "13" not in thing_name.lower()):
            dev_ota_url = dev_ota_url.replace("epd13", "epd7")

        shadow_payload = {
            "state": {
                "reported": {
                    "otaUrl": dev_ota_url
                }
            }
        }
        payload_bytes = json.dumps(shadow_payload).encode("utf-8")

        if dry_run:
            print(f" [DRY-RUN] #{idx}: Würde Shadow 'settings.otaUrl' für '{mask_epd_id(thing_name)}' setzen:")
            print(f"            Payload: {shadow_payload}")
        else:
            try:
                print(f" [APPLY] #{idx}: Setze Shadow 'settings.otaUrl' für '{mask_epd_id(thing_name)}'...")
                iot_client.update_thing_shadow(
                    thingName=thing_name,
                    shadowName="settings",
                    payload=payload_bytes
                )
                print(f"   ✅ Erfolgreich aktualisiert.")
            except ClientError as e:
                print(f"   ❌ Fehler beim Aktualisieren von Shadow für '{mask_epd_id(thing_name)}': {e}")


def resolve_main_ota_url(s3_bucket=None):
    """
    Ermittelt die FOTA Manifest URL der aktuellen regulären Hauptfirmware (main/production):
    - http://{bucket}/espfota_{target}.json
    """
    bucket = s3_bucket or os.environ.get("S3_BUCKET_NAME") or os.environ.get("HIL_S3_BUCKET") or "ul.epaperframe.de"
    return f"http://{bucket}/espfota_{{target}}.json"


def reset_ota_for_selected_devices(target_devices, dry_run=True, confirm_token="", region=None, s3_bucket=None, ota_url=None):
    """
    Rollt die aktuelle reguläre Hauptfirmware (main/production) auf die Zielgeräte aus:
    - Setzt Named Shadow 'settings.otaUrl' auf die Produktions-Manifest-URL (espfota_{target}.json)
    - Triggert das Gerät via MQTT ($aws/things/{thing}/epaper/receive)
    """
    main_url = ota_url or resolve_main_ota_url(s3_bucket=s3_bucket)
    print("=" * 65)
    print("🔄 CANARY SHADOW RESET ENGINE (MAIN FIRMWARE ROLLBACK)")
    print("=" * 65)
    print(f"  Hauptversion FOTA URL:           {main_url}")
    print(f"  Anzahl zurückzusetzender Geräte: {len(target_devices)}")
    print(f"  Dry-Run Modus:                   {dry_run}")
    print("=" * 65)

    if not target_devices:
        print("⚠️ Keine Zielgeräte übergeben. Vorgang beendet.")
        return

    if not dry_run:
        if confirm_token != CONFIRMATION_PHRASE:
            print("🛑 SICHERHEITSBLOCKADE: Ungültiger Bestätigungstoken!")
            print(f"   Erforderlich: --confirm-ota {CONFIRMATION_PHRASE}")
            sys.exit(1)

    iot_client = None if dry_run else get_iot_data_client(region=region)

    for idx, dev_id in enumerate(target_devices, 1):
        thing_name = dev_id.strip()
        register_github_mask(thing_name)
        model = "epd13" if "13" in thing_name.lower() else "epd7"
        dev_main_url = main_url
        if "{target}" in dev_main_url:
            dev_main_url = dev_main_url.replace("{target}", model)
        elif "epd7" in dev_main_url and "13" in thing_name.lower():
            dev_main_url = dev_main_url.replace("epd7", "epd13")
        elif "epd13" in dev_main_url and ("epd7" in thing_name.lower() or "13" not in thing_name.lower()):
            dev_main_url = dev_main_url.replace("epd13", "epd7")

        shadow_payload = {
            "state": {
                "reported": {
                    "otaUrl": dev_main_url
                }
            }
        }
        payload_bytes = json.dumps(shadow_payload).encode("utf-8")

        if dry_run:
            print(f" [DRY-RUN] #{idx}: Würde Shadow 'settings.otaUrl' für '{mask_epd_id(thing_name)}' auf Hauptversion setzen:")
            print(f"            Payload: {shadow_payload}")
        else:
            try:
                print(f" [RESET] #{idx}: Setze Shadow 'settings.otaUrl' für '{mask_epd_id(thing_name)}' auf Hauptversion ({dev_main_url})...")
                iot_client.update_thing_shadow(
                    thingName=thing_name,
                    shadowName="settings",
                    payload=payload_bytes
                )
                print("   ✅ Shadow erfolgreich auf Hauptversion gesetzt.")
            except ClientError as e:
                print(f"   ❌ Fehler beim Setzen von Shadow für '{mask_epd_id(thing_name)}': {e}")

            # Optional MQTT-Push zur sofortigen Signalisierung
            try:
                mqtt_topic = f"$aws/things/{thing_name}/epaper/receive"
                mqtt_msg = json.dumps({"ota": dev_main_url})
                iot_client.publish(topic=mqtt_topic, qos=1, payload=mqtt_msg.encode("utf-8"))
                print(f"   📡 MQTT Trigger an '$aws/things/{mask_epd_id(thing_name)}/epaper/receive' gesendet.")
            except Exception:
                pass


def resolve_ota_url(ota_url=None, s3_bucket=None, branch=None):
    """
    Ermittelt die FOTA Manifest URL automatisch basierend auf Branch und S3-Bucket:
    - main / master: espfota_{target}_pre.json (aus deploy_pre)
    - andere Branches (z. B. paper-l/dev): espfota_{target}_dev.json (aus deploy)
    """
    if ota_url:
        return ota_url
    bucket = s3_bucket or os.environ.get("S3_BUCKET_NAME") or os.environ.get("HIL_S3_BUCKET") or "ul.epaperframe.de"
    branch_name = branch or os.environ.get("GITHUB_REF_NAME") or "dev"
    mode = "pre" if branch_name in ("main", "master") else "dev"
    return f"http://{bucket}/espfota_{{target}}_{mode}.json"


def main():
    parser = argparse.ArgumentParser(
        description="Flotten-Scan & Empfehlungs-Generator für Canary Pilotgeräte (DynamoDB iotCatalog)"
    )
    parser.add_argument("--table", default="iotCatalog", help="Name der DynamoDB-Tabelle (Standard: iotCatalog)")
    parser.add_argument("--region", default=os.environ.get("AWS_REGION", "eu-central-1"), help="AWS Region")
    parser.add_argument("--count", type=int, default=10, help="Anzahl empfohlener Geräte je Display-Typ (Standard: 10)")
    parser.add_argument("--timeouts", default="60,180", help="Kommagetrennte Liste bevorzugter Aufwach-Timeouts (Standard: '60,180')")
    parser.add_argument("--output-md", default="pilot_recommendations.md", help="Pfad zur Markdown-Ausgabedatei")
    parser.add_argument("--output-json", default="pilot_recommendations.json", help="Pfad zur JSON-Ausgabedatei")
    parser.add_argument("--target-devices", help="Pfad zu einer JSON-Datei mit benutzerdefinierten Zielgeräten")
    parser.add_argument("--device-ids", help="Kommagetrennte Liste manueller Zielgeräte (z. B. 'epd7-xxx,epd13-yyy')")
    parser.add_argument("--ota-url", help="FOTA Manifest URL für das Pre-Release (Standard: automatisch nach Branch)")
    parser.add_argument("--deploy", action="store_true", help="Rollt die Canary OTA URL auf die Zielgeräte aus")
    parser.add_argument("--apply", action="store_true", help="Aktiviert das tatsächliche Setzen/Zurücksetzen der Shadows (erfordert --confirm-ota)")
    parser.add_argument("--reset", action="store_true", help="Setzt Zielgeräte auf die reguläre Hauptfirmware (main/production) zurück")
    parser.add_argument("--recommend", action="store_true", help="Führt den Flottenscan zur Ermittlung von Pilotgeräten aus")
    parser.add_argument("--confirm-ota", default="", help=f"Sicherheits-Bestätigungstoken: '{CONFIRMATION_PHRASE}'")
    parser.add_argument("--max-age-hours", type=float, default=1.0, help="Maximales Alter des letzten Kontakts in Stunden für Empfehlungen (Standard: 1.0 = vor < 1h)")
    parser.add_argument("--mock-file", help="Pfad zu einer JSON-Datei mit Test-Geräten (für Offline-/Testbench-Betrieb ohne AWS)")
    args = parser.parse_args()

    print("=" * 65)
    print("🎯 DYNAMODB PILOT-SELECTOR & CANARY ENGINE")
    print("=" * 65)

    # Zielgeräte ermitteln (Prio: CLI device-ids > target-devices > canary_target_devices.json > canary_target_devices.json.template)
    raw_targets = []
    if args.device_ids:
        raw_targets = [d.strip() for d in args.device_ids.replace("\n", ",").split(",") if d.strip()]
    elif args.target_devices and os.path.isfile(args.target_devices):
        with open(args.target_devices, "r", encoding="utf-8") as f:
            t_data = json.load(f)
            raw_targets = t_data.get("devices", []) if isinstance(t_data, dict) else t_data
    elif os.path.isfile("canary_target_devices.json"):
        with open("canary_target_devices.json", "r", encoding="utf-8") as f:
            t_data = json.load(f)
            raw_targets = t_data.get("devices", []) if isinstance(t_data, dict) else t_data
    elif os.path.isfile("canary_target_devices.json.template"):
        with open("canary_target_devices.json.template", "r", encoding="utf-8") as f:
            t_data = json.load(f)
            raw_targets = t_data.get("devices", []) if isinstance(t_data, dict) else t_data

    # Bereinigen von Platzhaltern wie "epd7-XX"
    selected_targets = [d for d in raw_targets if d and not d.endswith("-XX") and d != "XX"]

    # 1. Falls Reset-Modus
    if args.reset:
        if not selected_targets:
            print("❌ FEHLER: Für --reset müssen gültige Zielgeräte übergeben werden (--device-ids oder canary_target_devices.json).")
            sys.exit(1)
        is_dry_run = not args.apply
        reset_ota_for_selected_devices(
            selected_targets,
            dry_run=is_dry_run,
            confirm_token=args.confirm_ota,
            region=args.region,
            s3_bucket=os.environ.get("S3_BUCKET_NAME") or os.environ.get("HIL_S3_BUCKET"),
            ota_url=args.ota_url
        )
        return

    # 2. Falls Deploy-Modus oder Zielgeräte definiert sind (und kein expliziter --recommend Scan gewünscht ist)
    if args.deploy or (selected_targets and not args.recommend):
        if not selected_targets:
            print("❌ FEHLER: Keine gültigen Zielgeräte definiert! Bitte übergebe --device-ids oder passe canary_target_devices.json an.")
            sys.exit(1)

        auto_url = resolve_ota_url(
            args.ota_url,
            s3_bucket=os.environ.get("S3_BUCKET_NAME") or os.environ.get("HIL_S3_BUCKET"),
            branch=os.environ.get("GITHUB_REF_NAME")
        )
        is_dry_run = not args.apply
        print(f"📡 Ziel-FOTA URL (Modell-Platzhalter {{target}}): {auto_url}")
        apply_ota_to_selected_devices(
            selected_targets,
            auto_url,
            dry_run=is_dry_run,
            confirm_token=args.confirm_ota,
            region=args.region
        )
        return

    # 3. Flottenscan (nur bei explizitem --recommend oder wenn keine Geräte übergeben wurden)
    try:
        allowed_timeouts = tuple(int(x.strip()) for x in args.timeouts.split(",") if x.strip())
    except ValueError:
        allowed_timeouts = (60, 180)

    if args.mock_file and os.path.isfile(args.mock_file):
        print(f"📦 Verwende Mock-Gerätedaten aus: {args.mock_file}")
        with open(args.mock_file, "r", encoding="utf-8") as f:
            raw_loaded = json.load(f)
            if isinstance(raw_loaded, list):
                devices = raw_loaded
            elif isinstance(raw_loaded, dict) and "devices" in raw_loaded:
                devices = raw_loaded["devices"]
            elif isinstance(raw_loaded, dict):
                devices = list(raw_loaded.values()) if any(isinstance(v, dict) for v in raw_loaded.values()) else [raw_loaded]
            else:
                devices = []
    else:
        devices = scan_all_epaper_devices(table_name=args.table, region=args.region)

    results = generate_recommendations(
        devices,
        target_count=args.count,
        allowed_timeouts=allowed_timeouts,
        max_age_hours=args.max_age_hours
    )

    age_crit_str = f"vor < {args.max_age_hours}h online" if args.max_age_hours and args.max_age_hours > 0 else "alle"
    print(f"📊 Auswertungsergebnis (Kriterien: aktiv & {age_crit_str}):")
    print(f"   - EPD7  Kandidaten im Katalog: {results['epd7']['total_found']} (Top {len(results['epd7']['recommendations'])} empfohlen)")
    print(f"   - EPD13 Kandidaten im Katalog: {results['epd13']['total_found']} (Top {len(results['epd13']['recommendations'])} empfohlen)")

    write_recommendations_markdown(results, output_file=args.output_md)
    write_recommendations_json(results, output_file=args.output_json)
    write_template_target_file(results, template_file="canary_target_devices.json.template")
    print_copy_paste_recommendations(results)


if __name__ == "__main__":
    main()

