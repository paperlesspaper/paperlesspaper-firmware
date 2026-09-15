#!/usr/bin/env python3
"""
ai_risk_analysis.py
Automatische KI-Code-Risikoanalyse für Firmware-Änderungen (ESP32-C6 / E-Paper) mittels Google Gemini API.
Analysiert Git-Diffs auf Firmware-Sicherheitsrisiken (FOTA, Watchdog, Memory Leaks, Deep Sleep).
"""

import sys
import os
import re
import subprocess
import json
import urllib.request
import urllib.error
import argparse

# Windows CLI Encoding-Fix
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

GEMINI_MODEL = os.environ.get("GEMINI_MODEL", "gemini-3.8-flash")

SYSTEM_PROMPT = """Du bist ein hochqualifizierter Senior Embedded Systems & Firmware Security Auditor, spezialisiert auf ESP32 Microcontroller (C++/Arduino), E-Paper Displays und AWS IoT Cloud-Anbindung.

Deine Aufgabe ist es, das übergebene Git-Diff einer neuen Firmware-Version auf Sicherheits-, Stabilitäts- und Regressionsrisiken zu prüfen.

Achte besonders auf:
1. **FOTA & Bootloader:** Manipulation von OTA-URLs, Partitionsgrenzen, SPIFFS/Flash-Offsets, esp32FOTA-Logik.
2. **Watchdog & Deadlocks:** Blockierende Schleifen (`while (...)`) ohne Watchdog-Reset (`tickerFailsave`), Timeouts.
3. **Power Management & Deep Sleep:** Fehlerhafte GPIOs vor Deep Sleep, Endlosschleifen vor `gotToDeepSleep()`, Akku-Drain.
4. **Speicherstabilität:** Häufige dynamische Allokationen (`new`, `malloc`, `String += ...`) in Loops (Heap-Fragmentierung).
5. **Hardcodierte Secrets:** Passwörter, API-Keys oder Zertifikate.

WICHTIGE ANWEISUNG ZUR FORM:
- Halte die Zusammenfassung und Auswertung gerade bei niedrigem Risiko (LOW) SEHR KURZ und prägnant (maximal 2-3 knappe Aufzählungspunkte insgesamt, keine ausschweifenden Erklärungen).
- Nur bei tatsächlichen Risiken (MEDIUM oder HIGH) sind detaillierte Ausführungen erforderlich.

Gib deine Antwort in folgendem Markdown-Format auf Deutsch aus:
# 🛡️ KI-Firmware Risikoanalyse

### Gesamtbewertung: [🟢 GERING (LOW) | 🟡 MITTEL (MEDIUM) | 🔴 HOCH (HIGH)]
**Empfehlung:** [GENEHMIGT | MANUELLE PRÜFUNG EMPFOHLEN | BLOCKIERT]

### Zusammenfassung & Risikobewertung
- Bei LOW: Max. 2-3 prägnante Stichpunkte zur Änderung und Bestätigung, dass keine Risiken vorliegen.
- Bei MEDIUM/HIGH: Konkrete Schwachstellen mit Zeilen- und Funktionsbezug.

### HIL-Testfokus
- 1-2 wesentliche Punkte, die auf der echten Hardware verifiziert werden sollten.
"""

def sanitize_diff(diff_text):
    """Filtert sensible Werte wie Passwörter, Private Keys und Tokens aus dem Diff vor der Übertragung."""
    # Redaktiere Zertifikatsblöcke
    diff_text = re.sub(r"-----BEGIN [A-Z ]+-----[^-]+-----END [A-Z ]+-----", "[REDACTED_CERTIFICATE]", diff_text)
    # Redaktiere gängige Key/Secret-Muster
    diff_text = re.sub(r'(?i)(password|secret|token|api_?key|auth|pass)\s*[:=]\s*["\']([^"\']+)["\']', r'\1: "[REDACTED]"', diff_text)
    return diff_text

def get_git_diff(base_ref=None, staged_only=False):
    try:
        # Schließe sensible Dateien strikt aus dem Diff aus
        exclude_specs = [
            ":!src/secrets.h",
            ":!.env*",
            ":!*.key",
            ":!*.crt",
            ":!*.pem",
            ":!*.bin"
        ]

        if staged_only:
            # Nur gestagte Änderungen prüfen (git diff --cached)
            cmd = ["git", "diff", "--cached", "--", "."] + exclude_specs
            res = subprocess.run(cmd, capture_output=True, text=True, check=True, encoding="utf-8", errors="ignore")
            return sanitize_diff(res.stdout.strip())

        if base_ref:
            cmd = ["git", "diff", f"{base_ref}...HEAD", "--", "."] + exclude_specs
            res = subprocess.run(cmd, capture_output=True, text=True, check=True, encoding="utf-8", errors="ignore")
            return sanitize_diff(res.stdout.strip())

        # Lokal: Zuerst prüfen, ob uncommittete Änderungen vorliegen (gestagt oder im Arbeitsverzeichnis vs HEAD)
        cmd_head = ["git", "diff", "HEAD", "--", "."] + exclude_specs
        res_head = subprocess.run(cmd_head, capture_output=True, text=True, check=True, encoding="utf-8", errors="ignore")
        diff_head = res_head.stdout.strip()
        if diff_head:
            return sanitize_diff(diff_head)

        # Wenn Arbeitsbereich sauber ist (z.B. in CI nach Commit), den letzten Commit prüfen
        cmd_commit = ["git", "diff", "HEAD~1", "--", "."] + exclude_specs
        res_commit = subprocess.run(cmd_commit, capture_output=True, text=True, check=True, encoding="utf-8", errors="ignore")
        return sanitize_diff(res_commit.stdout.strip())

    except Exception as e:
        print(f"Hinweis: Git-Diff konnte nicht ermittelt werden: {e}")
        return ""

def call_gemini(diff_text, api_key):
    # Diff bei sehr großen Änderungen kürzen, um Kontextgrenzen nicht zu überlasten
    max_chars = 30000
    if len(diff_text) > max_chars:
        diff_text = diff_text[:max_chars] + "\n\n[... Diff gekürzt auf 30.000 Zeichen ...]"

    payload = {
        "contents": [
            {
                "parts": [
                    {"text": SYSTEM_PROMPT},
                    {"text": f"Hier ist das zu prüfende Firmware Git-Diff (sensible Secrets vorab maskiert):\n\n```diff\n{diff_text}\n```"}
                ]
            }
        ],
        "generationConfig": {
            "temperature": 0.2,
            "maxOutputTokens": 2048
        }
    }

    # Bevorzugtes Modell: gemini-3.8-flash, gefolgt von erprobten Fallbacks
    models_to_try = [os.environ.get("GEMINI_MODEL", "gemini-3.8-flash")]
    for fallback in ["gemini-3.6-flash", "gemini-2.0-flash", "gemini-1.5-flash"]:
        if fallback not in models_to_try:
            models_to_try.append(fallback)

    last_error = None
    for model in models_to_try:
        url = f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
        req = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "x-goog-api-key": api_key
            }
        )

        try:
            with urllib.request.urlopen(req, timeout=45) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                text = data["candidates"][0]["content"]["parts"][0]["text"]
                return text, model
        except urllib.error.HTTPError as e:
            if e.code == 404:
                # Modell in dieser API-Version nicht verfügbar -> nächstes Fallback-Modell testen
                last_error = e
                continue
            error_body = e.read().decode("utf-8", errors="ignore")
            raise RuntimeError(f"Gemini API HTTP {e.code}: {error_body}")
        except Exception as e:
            last_error = e
            continue

    raise RuntimeError(f"Keines der Modelle ({models_to_try}) konnte aufgerufen werden. Letzter Fehler: {last_error}")

def load_env_file():
    """Liest Umgebungsvariablen aus der lokalen .env-Datei (für lokale Tests), ohne externe Abhängigkeiten."""
    env_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".env"))
    if not os.path.isfile(env_path):
        return {}

    env_vars = {}
    try:
        with open(env_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split("=", 1)
                if len(parts) == 2:
                    k = parts[0].strip()
                    v = parts[1].strip()
                    if (v.startswith('"') and v.endswith('"')) or (v.startswith("'") and v.endswith("'")):
                        v = v[1:-1]
                    env_vars[k] = v
    except Exception:
        pass
    return env_vars

def main():
    parser = argparse.ArgumentParser(description="KI-Firmware Risikoanalyse via Google Gemini API")
    parser.add_argument("--base", help="Git Basis-Ref für den Diff (z.B. origin/main oder HEAD~1)")
    parser.add_argument("--diff-file", help="Optionaler Pfad zu einer Datei, die das Git-Diff enthält")
    parser.add_argument("--staged", action="store_true", help="Nur gestagte Änderungen prüfen (git diff --cached)")
    parser.add_argument("--output", help="Optionaler Pfad zur Ausgabe des Markdown-Reports")
    parser.add_argument("--strict", action="store_true", help="Beende mit Exit-Code 1 bei HOHEM Risiko oder fehlendem Key")
    args = parser.parse_args()

    # Priorität: 1. Umgebungsvariable (z.B. GitHub Actions Secret), 2. lokale .env Datei
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        env_vars = load_env_file()
        api_key = env_vars.get("GEMINI_API_KEY")

    if not api_key:
        print("⚠️ GEMINI_API_KEY weder in Umgebungsvariablen noch in .env gefunden. Überspringe KI-Risikoanalyse.")
        if args.strict:
            sys.exit(1)
        sys.exit(0)

    # In GitHub Actions Logs maskieren
    print(f"::add-mask::{api_key}")

    # Diff ermitteln
    if args.diff_file and os.path.isfile(args.diff_file):
        with open(args.diff_file, "r", encoding="utf-8", errors="ignore") as f:
            diff_text = sanitize_diff(f.read().strip())
    else:
        diff_text = get_git_diff(args.base, staged_only=args.staged)

    if not diff_text:
        print("ℹ️ Keine relevanten Code-Änderungen im Diff gefunden. Keine KI-Analyse erforderlich.")
        sys.exit(0)

    print("=" * 60)
    print("🤖 STARTE KI-RISIKOANALYSE")
    print(f"📏 Diff-Größe: {len(diff_text):,} Zeichen")
    print("=" * 60)

    try:
        report, used_model = call_gemini(diff_text, api_key)
        print(f"✨ Modell: {used_model}")
        print(report)

        if args.output:
            with open(args.output, "w", encoding="utf-8") as f:
                f.write(report)
            print(f"\n📄 Bericht gespeichert in: {args.output}")

        # In GitHub Actions Step Summary einbinden
        github_summary = os.environ.get("GITHUB_STEP_SUMMARY")
        if github_summary:
            try:
                with open(github_summary, "a", encoding="utf-8") as gf:
                    gf.write(f"\n\n{report}\n")
            except Exception as e:
                print(f"Hinweis: GitHub Step Summary konnte nicht geschrieben werden: {e}")

        # Bei strengem Modus prüfen, ob hohes Risiko vorliegt
        if args.strict and ("HOCH (HIGH)" in report or "BLOCKIERT" in report):
            print("\n❌ Build durch KI-Risikoanalyse blockiert (HOCH / BLOCKIERT erkannt)!")
            sys.exit(1)

        sys.exit(0)

    except Exception as e:
        print(f"❌ Fehler bei der KI-Risikoanalyse: {e}")
        if args.strict:
            sys.exit(1)
        sys.exit(0)

if __name__ == "__main__":
    main()
