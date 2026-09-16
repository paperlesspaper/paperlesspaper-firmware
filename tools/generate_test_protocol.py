#!/usr/bin/env python3
"""
tools/generate_test_protocol.py
Aggregiert HIL-Testergebnisse (JUnit XML) für EPD7 und EPD13, den Deployment-Status
und die KI-Risikoanalyse zu einem strukturierten Testprotokoll.
Schreibt die Zusammenfassung in $GITHUB_STEP_SUMMARY, erzeugt ein Markdown-Artefakt
für den Pull Request und ein JSON-Artefakt für automatische CI-Bots.
"""

import sys
import os
import re
import json
import argparse
import xml.etree.ElementTree as ET
from datetime import datetime

# Windows CLI Encoding-Fix
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

PHASE_TITLES = {
    "test_00": "Phase 0: Factory-Reset (6x Power-Cycles)",
    "test_01": "Phase 1: BLE-WLAN-Provisionierung",
    "test_02": "Phase 2: Produktions-Firmware OTA (Manifest JSON)",
    "test_03": "Phase 3: Kandidaten-Firmware OTA & S3-Bereinigung",
    "test_04": "Phase 4: Autonome REST-Aktivierung & Handshake",
    "test_05": "Phase 5: Presigned URL Bild-Upload, Rendering & Quittung",
    "test_06": "Phase 6: REST-Deaktivierung & Deep Sleep"
}

def parse_junit_xml(xml_path):
    """Parst eine JUnit-XML-Datei und extrahiert detaillierte Testergebnisse."""
    if not xml_path or not os.path.isfile(xml_path):
        return None

    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
    except Exception as e:
        print(f"⚠️ Warnung beim Parsen von '{xml_path}': {e}")
        return None

    testsuite = root if root.tag == "testsuite" else root.find("testsuite")
    if testsuite is None:
        return None

    tests_total = int(testsuite.attrib.get("tests", 0))
    failures_total = int(testsuite.attrib.get("failures", 0))
    errors_total = int(testsuite.attrib.get("errors", 0))
    skipped_total = int(testsuite.attrib.get("skipped", 0))
    time_total = float(testsuite.attrib.get("time", 0.0))

    cases = []
    for tc in testsuite.findall("testcase"):
        tc_name = tc.attrib.get("name", "")
        tc_time = float(tc.attrib.get("time", 0.0))
        classname = tc.attrib.get("classname", "")

        status = "PASSED"
        message = ""

        fail = tc.find("failure")
        err = tc.find("error")
        skip = tc.find("skipped")

        if fail is not None:
            status = "FAILED"
            message = fail.attrib.get("message", "") or fail.text or ""
        elif err is not None:
            status = "ERROR"
            message = err.attrib.get("message", "") or err.text or ""
        elif skip is not None:
            status = "SKIPPED"
            message = skip.attrib.get("message", "") or skip.text or "Übersprungen"

        # Lesbaren Phasen-Titel ermitteln
        phase_key = next((k for k in PHASE_TITLES if k in tc_name), tc_name)
        phase_title = PHASE_TITLES.get(phase_key, tc_name)

        cases.append({
            "name": tc_name,
            "phase": phase_title,
            "classname": classname,
            "duration": round(tc_time, 2),
            "status": status,
            "message": message.strip()
        })

    passed_total = tests_total - failures_total - errors_total - skipped_total

    return {
        "file": xml_path,
        "total": tests_total,
        "passed": max(0, passed_total),
        "failed": failures_total + errors_total,
        "skipped": skipped_total,
        "duration": round(time_total, 2),
        "cases": cases
    }

def load_risk_report(report_path):
    """Liest die KI-Risikoanalyse ein und extrahiert Ampelbewertung und Empfehlung."""
    if not report_path or not os.path.isfile(report_path):
        return {
            "available": False,
            "rating": "UNBEKANNT",
            "recommendation": "KEINE ANALYSE VORHANDEN",
            "raw_text": "Keine KI-Risikoanalyse verfügbar."
        }

    try:
        with open(report_path, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read().strip()

        rating = "GERING (LOW)"
        if "HOCH (HIGH)" in content:
            rating = "HOCH (HIGH)"
        elif "MITTEL (MEDIUM)" in content:
            rating = "MITTEL (MEDIUM)"

        recommendation = "GENEHMIGT"
        if "BLOCKIERT" in content:
            recommendation = "BLOCKIERT"
        elif "MANUELLE PRÜFUNG" in content:
            recommendation = "MANUELLE PRÜFUNG EMPFOHLEN"

        return {
            "available": True,
            "rating": rating,
            "recommendation": recommendation,
            "raw_text": content
        }
    except Exception as e:
        return {
            "available": False,
            "rating": "FEHLER",
            "recommendation": f"Fehler beim Einlesen: {e}",
            "raw_text": ""
        }

def generate_markdown(fw_version, commit_sha, deploy_status, epd7_data, epd13_data, risk_data):
    """Erzeugt das vollständige Markdown-Testprotokoll."""
    lines = []

    # Gesamtbewertung ermitteln
    any_test_failed = False
    epd7_passed = epd7_data and epd7_data["failed"] == 0 and epd7_data["passed"] > 0
    epd13_passed = epd13_data and epd13_data["failed"] == 0 and epd13_data["passed"] > 0
    deploy_passed = deploy_status in ("success", "ok", "passed", "")

    if (epd7_data and epd7_data["failed"] > 0) or (epd13_data and epd13_data["failed"] > 0) or not deploy_passed:
        any_test_failed = True

    risk_high = "HOCH" in risk_data.get("rating", "") or "BLOCKIERT" in risk_data.get("recommendation", "")

    if any_test_failed or risk_high:
        verdict = "🔴 NICHT BEREIT FÜR MERGE (FEHLER / BLOCKIERT)"
        verdict_badge = "❌ **BLOCKIERT**"
    elif risk_data.get("rating") == "MITTEL (MEDIUM)" or (epd13_data and epd13_data["skipped"] > 0):
        verdict = "🟡 MANUELLE PRÜFUNG VOR MERGE EMPFOHLEN"
        verdict_badge = "⚠️ **MANUELLE FREIGABE EMPFOHLEN**"
    else:
        verdict = "🟢 FREIGABE ERTEILT (BEREIT FÜR AUTOMATISCHEN PULL REQUEST)"
        verdict_badge = "✅ **AUTOMATISCH GENEHMIGT**"

    lines.append("# 🔬 HIL Testprotokoll & Release-Bewertung")
    lines.append("")
    lines.append(f"**Firmware-Version:** `{fw_version}` | **Commit:** `{commit_sha[:7] if commit_sha else 'HEAD'}` | **Zeitstempel:** `{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}`")
    lines.append(f"### Gesamtstatus: {verdict_badge}")
    lines.append(f"> **Fazit für Pull Request:** {verdict}")
    lines.append("")
    lines.append("---")
    lines.append("")

    # 1. Übersichtstabelle
    lines.append("## 📊 Übersicht der Test- und Deployment-Ergebnisse")
    lines.append("")
    lines.append("| Komponente / Target | Status | Details |")
    lines.append("| :--- | :---: | :--- |")

    # Deployment
    if deploy_status == "skipped":
        deploy_icon = "⚪ Übersprungen (Hardware-Tests nicht bestanden)"
    elif deploy_passed:
        deploy_icon = "✅ Erfolgreich"
    else:
        deploy_icon = "❌ Fehlgeschlagen"
    lines.append(f"| **S3 Deployment (`dev`)** | {deploy_icon} | Upload von Firmware & JSON Manifests |")

    # EPD7
    if epd7_data:
        epd7_icon = "✅ Bestanden" if epd7_data["failed"] == 0 else "❌ Fehlgeschlagen"
        lines.append(f"| **EPD7 (7.5\" Hardware)** | {epd7_icon} | {epd7_data['passed']}/{epd7_data['total']} Tests bestanden ({epd7_data['duration']}s) |")
    else:
        lines.append("| **EPD7 (7.5\" Hardware)** | ⚪ Nicht ausgeführt | Kein Testergebnis vorhanden |")

    # EPD13
    if epd13_data:
        if epd13_data["failed"] > 0:
            epd13_icon = "❌ Fehlgeschlagen"
            epd13_desc = f"{epd13_data['passed']}/{epd13_data['total']} Tests ({epd13_data['failed']} fehlgeschlagen)"
        elif epd13_data["passed"] > 0:
            epd13_icon = "✅ Bestanden"
            epd13_desc = f"{epd13_data['passed']}/{epd13_data['total']} Tests bestanden ({epd13_data['duration']}s)"
        else:
            epd13_icon = "⚪ Übersprungen"
            epd13_desc = "Hardware nicht angeschlossen (sauber übersprungen)"
        lines.append(f"| **EPD13 (13.3\" Hardware)** | {epd13_icon} | {epd13_desc} |")
    else:
        lines.append("| **EPD13 (13.3\" Hardware)** | ⚪ Nicht ausgeführt | Kein Testergebnis vorhanden |")

    # KI Risiko
    risk_icon = "🟢 Gering"
    if "HOCH" in risk_data.get("rating", ""):
        risk_icon = "🔴 Hoch"
    elif "MITTEL" in risk_data.get("rating", ""):
        risk_icon = "🟡 Mittel"
    lines.append(f"| **KI-Risikoanalyse (Gemini)** | {risk_icon} | {risk_data.get('rating', 'Unbekannt')} – {risk_data.get('recommendation', '')} |")
    lines.append("")
    lines.append("---")
    lines.append("")

    # 2. Detaillierte Hardware-Testergebnisse EPD7
    if epd7_data and epd7_data.get("cases"):
        lines.append("## 📺 Detailergebnisse: EPD7 (7.5\" Display)")
        lines.append("")
        lines.append("| Testphase | Status | Dauer | Bemerkung |")
        lines.append("| :--- | :---: | :---: | :--- |")
        for tc in epd7_data["cases"]:
            icon = "✅" if tc["status"] == "PASSED" else ("⚠️" if tc["status"] == "SKIPPED" else "❌")
            note = tc["message"].replace("\n", " ")[:80] if tc["message"] else "Erfolgreich quittiert"
            lines.append(f"| {tc['phase']} | {icon} `{tc['status']}` | {tc['duration']}s | {note} |")
        lines.append("")

    # 3. Detaillierte Hardware-Testergebnisse EPD13
    if epd13_data and epd13_data.get("cases"):
        lines.append("## 📺 Detailergebnisse: EPD13 (13.3\" Display)")
        lines.append("")
        lines.append("| Testphase | Status | Dauer | Bemerkung |")
        lines.append("| :--- | :---: | :---: | :--- |")
        for tc in epd13_data["cases"]:
            icon = "✅" if tc["status"] == "PASSED" else ("⚠️" if tc["status"] == "SKIPPED" else "❌")
            note = tc["message"].replace("\n", " ")[:80] if tc["message"] else "Erfolgreich quittiert"
            lines.append(f"| {tc['phase']} | {icon} `{tc['status']}` | {tc['duration']}s | {note} |")
        lines.append("")

    # 4. KI Risikoanalyse
    if risk_data.get("available"):
        lines.append("---")
        lines.append("")
        lines.append("## 🛡️ KI-Firmware Risikoanalyse Zusammenfassung")
        lines.append("")
        lines.append(risk_data.get("raw_text", ""))
        lines.append("")

    return "\n".join(lines)

def main():
    parser = argparse.ArgumentParser(description="Aggregiert HIL-Testergebnisse & generiert Protokoll für Pull Requests")
    parser.add_argument("--junit-epd7", help="Pfad zur JUnit XML für EPD7")
    parser.add_argument("--junit-epd13", help="Pfad zur JUnit XML für EPD13")
    parser.add_argument("--risk-report", help="Pfad zum KI-Risikobericht (risk_report.md)")
    parser.add_argument("--fw-version", default=os.environ.get("FW_VERSION", "0.0.0"), help="Firmware-Version")
    parser.add_argument("--commit-sha", default=os.environ.get("GITHUB_SHA", "HEAD"), help="Git Commit SHA")
    parser.add_argument("--deploy-status", default=os.environ.get("DEPLOY_STATUS", "success"), help="Status des S3 Deployments")
    parser.add_argument("--output-md", help="Ausgabepfad für Markdown-Protokoll (z.B. hil_test_protocol.md)")
    parser.add_argument("--output-json", help="Ausgabepfad für JSON-Protokoll (z.B. hil_test_protocol.json)")
    parser.add_argument("--strict", action="store_true", help="Beende mit Exit-Code 1 wenn ein Test fehlgeschlagen ist")
    args = parser.parse_args()

    epd7_data = parse_junit_xml(args.junit_epd7)
    epd13_data = parse_junit_xml(args.junit_epd13)
    risk_data = load_risk_report(args.risk_report)

    md_report = generate_markdown(
        fw_version=args.fw_version,
        commit_sha=args.commit_sha,
        deploy_status=args.deploy_status,
        epd7_data=epd7_data,
        epd13_data=epd13_data,
        risk_data=risk_data
    )

    print("\n" + "=" * 65)
    print("📋 HIL TESTPROTOKOLL GENERIERT")
    print("=" * 65)
    print(md_report)
    print("=" * 65 + "\n")

    # In Step Summary schreiben
    step_summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if step_summary:
        try:
            with open(step_summary, "a", encoding="utf-8") as sf:
                sf.write("\n\n" + md_report + "\n")
            print(f"✅ In GITHUB_STEP_SUMMARY geschrieben: {step_summary}")
        except Exception as e:
            print(f"⚠️ Warnung beim Schreiben von GITHUB_STEP_SUMMARY: {e}")

    # Als Markdown-Datei speichern
    if args.output_md:
        try:
            with open(args.output_md, "w", encoding="utf-8") as mf:
                mf.write(md_report)
            print(f"📄 Markdown-Protokoll gespeichert: {args.output_md}")
        except Exception as e:
            print(f"⚠️ Fehler beim Speichern von {args.output_md}: {e}")

    # Als strukturierte JSON-Datei speichern (für automatische PR-Bots)
    if args.output_json:
        try:
            json_payload = {
                "version": args.fw_version,
                "commit": args.commit_sha,
                "timestamp": datetime.now().isoformat(),
                "deploy_status": args.deploy_status,
                "risk_analysis": {
                    "rating": risk_data.get("rating"),
                    "recommendation": risk_data.get("recommendation"),
                },
                "epd7": epd7_data,
                "epd13": epd13_data,
                "overall_success": not (
                    (epd7_data and epd7_data["failed"] > 0) or
                    (epd13_data and epd13_data["failed"] > 0) or
                    args.deploy_status not in ("success", "ok", "passed", "") or
                    "HOCH" in risk_data.get("rating", "") or
                    "BLOCKIERT" in risk_data.get("recommendation", "")
                )
            }
            with open(args.output_json, "w", encoding="utf-8") as jf:
                json.dump(json_payload, jf, indent=2)
            print(f"📦 JSON-Protokoll gespeichert: {args.output_json}")
        except Exception as e:
            print(f"⚠️ Fehler beim Speichern von {args.output_json}: {e}")

    # Bei Fehlern strikt abbrechen
    if args.strict:
        has_failure = (
            (epd7_data and epd7_data["failed"] > 0) or
            (epd13_data and epd13_data["failed"] > 0) or
            args.deploy_status not in ("success", "ok", "passed", "") or
            "HOCH" in risk_data.get("rating", "") or
            "BLOCKIERT" in risk_data.get("recommendation", "")
        )
        if has_failure:
            print("❌ STRIKTER PIPELINE-ABBRUCH: Fehler im Deployment, in den Hardware-Tests oder KI-Risikobewertung!")
            sys.exit(1)

    sys.exit(0)

if __name__ == "__main__":
    main()
