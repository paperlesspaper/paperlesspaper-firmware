#!/usr/bin/env python3
"""
ai_risk_analysis.py
Automatische KI-Code-Risikoanalyse für Firmware-Änderungen (ESP32-C6 / E-Paper) mittels Google Gemini API.
Analysiert Git-Diffs auf Firmware-Sicherheitsrisiken (FOTA, Watchdog, Memory Leaks, Deep Sleep).
"""

import sys
import os
import re
import time
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

SYSTEM_PROMPT = """Du bist ein hochqualifizierter Senior Embedded Systems & Firmware Security Auditor, spezialisiert auf das PaperlessPaper E-Paper Ökosystem (ESP32-C6 Microcontroller EPD7/EPD13, PaperlessPaper Web/Mobile App und AWS IoT Cloud-Anbindung).
Dokumentation des Gesamtsystems: https://github.com/paperlesspaper und https://github.com/paperlesspaper/paperlesspaper-docs.

Deine Hauptaufgabe ist es, die aktuellen Änderungen (Git-Diff) und deren Auswirkung auf das Gesamtsystem zu prüfen sowie konkrete, umsetzbare Endprodukt- & App-Testaufgaben für den Anwender zu definieren.
Der bereitgestellte Gesamtcode dient als Architektur- und Sicherheitskontext.

WICHTIGE PRIORISIERUNGS- UND STRUKTURREGELN:
1. **Abschnitt 1: Bewertung der aktuellen Code-Änderungen (Git-Diff):** Steht an erster Stelle!
   - Was wurde im Diff geändert?
   - Welche Risiken, Seiteneffekte oder architektonischen Auswirkungen auf das Endprodukt oder das Verhalten der App ergeben sich?
   - Bei LOW: 2-3 prägnante Kernpunkte, was geändert wurde und warum kein Risiko besteht.
   - Bei MEDIUM/HIGH: Detaillierte Schwachstellen mit Datei-, Zeilen- und Funktionsbezug.

2. **Abschnitt 2: Endprodukt- & App-Testtasks für den Nutzer:** Steht an zweiter Stelle und ist besonders wichtig!
   - **STRIKTES VERBOT VON ENTWICKLER-/BUILD-TASKS:** Formuliere KEINE Aufgaben wie "Kompiliere mit PlatformIO", "Flashe per USB-C", "Starte Python-Skripte", "Prüfe Git-Diff" oder Compiler-Warnungen!
   - **NUR TESTAUFGABEN AM ECHTEN ENDPRODUKT IM ZUSAMMENSPIEL MIT DER PAPERLESSPAPER-APP:**
     Definiere 2 bis 4 konkrete, handlungsorientierte Aufgaben mit Markdown-Checkboxen (`- [ ] **Task X: ...**`), die ein Nutzer am fertigen E-Paper Rahmen (EPD7 / EPD13) zusammen mit der PaperlessPaper App durchführen kann, um die einwandfreie Funktion zu gewährleisten:
     * **BLE-Onboarding & WLAN-Provisionierung via App:** Gerät einschalten / zurücksetzen -> Startbildschirm/QR-Code auf dem E-Paper -> In der App per Bluetooth koppeln, WLAN wählen, Credentials senden -> Display bestätigt Verbindung.
     * **Bildübertragung & Rendering via App:** In der App ein Bild, Dokument oder Widget an den Rahmen senden -> Display wacht auf, lädt Bild, führt sauberen Refresh aus (Farbdarstellung, Dithering) und quittiert erfolgreich in der App.
     * **Einstellungen & Deep Sleep:** In der App Bild-Rotation, Update-Intervall oder Ruhemodus ändern -> Prüfen, ob Display die Settings übernimmt und danach sauber in den Deep Sleep (stromsparend) wechselt.
     * **Firmware-Update (OTA) via Cloud/App:** In der App ein Update anstoßen -> Display zeigt "Updating Software...", flasht ohne Abbruch und bootet sauber in die neue Version.
     * **Hardware-Taster am Rahmen:** 1x Taster drücken für manuellen Refresh; 5x Taster drücken für Werksreset (Rückkehr zum QR-Code Onboarding).
   - Jeder Task muss enthalten:
     * **Test-Fokus:** Was wird am Endprodukt verifiziert?
     * **Aktion (App & Gerät):** Genaue Schritt-für-Schritt Anleitung, was der Nutzer in der App klickt und am Gerät tut.
     * **Erwartetes Verhalten am Endprodukt:** Was genau sieht der Nutzer auf dem E-Paper Display und in der App?

3. **Abschnitt 3: Ganzheitliche Systemanalyse (Gesamtcode):** Steht an letzter Stelle!
   - **WICHTIG BEI GERINGEM RISIKO (LOW):** Halte diesen Abschnitt **extrem kurz und kompakt** (maximal 3-4 knappe Bullet-Points als Bestätigung, dass Boot-Loop, OTA, Deep Sleep und Watchdog in der Gesamtarchitektur solide und unbeeinträchtigt sind).
   - **Nur bei MITTEL oder HOCH:** Führe eine detaillierte Risikoaufschlüsselung der Schwachstellen im Gesamtcode durch.

Gib deine Antwort in folgendem Markdown-Format auf Deutsch aus:

# 🛡️ KI-Firmware Risikoanalyse & Audit-Report

### Gesamtbewertung: [🟢 GERING (LOW) | 🟡 MITTEL (MEDIUM) | 🔴 HOCH (HIGH)]
**Empfehlung:** [GENEHMIGT | MANUELLE PRÜFUNG EMPFOHLEN | BLOCKIERT]

---

### 1. 🔍 Bewertung der aktuellen Code-Änderungen (Git-Diff)
- [Präzise Analyse der Änderungen im Diff und Einfluss auf Gerät & App. Bei LOW: 2-3 Kernpunkte. Bei MEDIUM/HIGH: Detaillierte Schwachstellen.]

---

### 2. 📱 Endprodukt- & App-Testtasks für den Nutzer
- [ ] **Task 1: [Prägnanter Titel, z. B. Bildübertragung & Rendering via App]**
  - **Test-Fokus:** [Was am Gerät/App geprüft wird]
  - **Aktion (App & Gerät):** [Schritt-für-Schritt Anleitung für den Nutzer]
  - **Erwartetes Verhalten:** [Was auf dem E-Paper Display und in der App sichtbar passiert]
- [ ] **Task 2: [Prägnanter Titel, z. B. BLE-Onboarding oder Settings-Sync via App]**
  - **Test-Fokus:** [Was am Gerät/App geprüft wird]
  - **Aktion (App & Gerät):** [Schritt-für-Schritt Anleitung für den Nutzer]
  - **Erwartetes Verhalten:** [Was auf dem E-Paper Display und in der App sichtbar passiert]
- [ ] **Task 3: [Prägnanter Titel, z. B. Manueller Taster-Refresh & Sleep-Verhalten]**
  - **Test-Fokus:** [Was am Gerät/App geprüft wird]
  - **Aktion (App & Gerät):** [Schritt-für-Schritt Anleitung für den Nutzer]
  - **Erwartetes Verhalten:** [Was auf dem E-Paper Display und in der App sichtbar passiert]

---

### 3. 🌐 Ganzheitliche Systemanalyse (Gesamtcode)
*(Kompakt bei LOW; detailliert nur bei MEDIUM/HIGH)*
- **Boot-Loop-Resilienz:** [Kurzbestätigung bei LOW / Detailanalyse bei MEDIUM/HIGH]
- **OTA-Funktionalität:** [Kurzbestätigung bei LOW / Detailanalyse bei MEDIUM/HIGH]
- **Power Management & Deep Sleep:** [Kurzbestätigung bei LOW / Detailanalyse bei MEDIUM/HIGH]
- **Watchdog & Stabilität:** [Kurzbestätigung bei LOW / Detailanalyse bei MEDIUM/HIGH]
"""


def sanitize_diff(diff_text):
    """Filtert sensible Werte wie Passwörter, Private Keys und Tokens aus dem Text vor der Übertragung."""
    # Redaktiere Zertifikatsblöcke
    diff_text = re.sub(r"-----BEGIN [A-Z ]+-----[^-]+-----END [A-Z ]+-----", "[REDACTED_CERTIFICATE]", diff_text)
    # Redaktiere gängige Key/Secret-Muster
    diff_text = re.sub(r'(?i)(password|secret|token|api_?key|auth|pass)\s*[:=]\s*["\']([^"\']+)["\']', r'\1: "[REDACTED]"', diff_text)
    return diff_text


def get_codebase_context():
    """
    Sammelt den vollständigen, funktionsrelevanten Firmware-Quellcode zur ganzheitlichen Systemanalyse:
    - Analysiert alle Code-Dateien im Ordner 'src/' (.cpp, .c, .h, .hpp).
    - Beschränkt die Codebase auf sinnvolle Parts der Firmware:
      * Reine Bitmap-Assets (z.B. icons.h mit 66 KB statischen Hex-Arrays) werden nicht als Roh-Bytes übertragen,
        sondern als Asset-Hinweis deklariert, um das Token-Budget für Logik zu reservieren.
      * Sensible Dateien (secrets.h, secrets.h.sample, .env*) sind strikt ausgeschlossen und alle Inhalte werden maskiert.
    - Bezieht fundamentale Hardware- & Build-Konfigurationen (part.csv, deploy/platformio.ini) ein.
    """
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    src_dir = os.path.join(root_dir, "src")

    context_parts = []

    # 1. Relevante Konfigurationsdateien außerhalb von src/
    config_files = [
        ("part.csv", "Partitions-Tabelle & Flash-Offsets"),
        ("deploy/platformio.ini", "PlatformIO Konfiguration & Build-Flags")
    ]
    for rel_path, desc in config_files:
        full_path = os.path.join(root_dir, rel_path)
        if os.path.isfile(full_path):
            try:
                with open(full_path, "r", encoding="utf-8", errors="ignore") as f:
                    content = sanitize_diff(f.read())
                    context_parts.append(f"#### Datei: `{rel_path}` ({desc})\n```ini\n{content}\n```")
            except Exception as e:
                print(f"Hinweis: Konnte {rel_path} nicht lesen: {e}")

    # 2. Alle relevanten Code-Dateien im Ordner src/
    EXCLUDED_FILENAMES = {
        "secrets.h",
        "secrets.h.sample",
        "cmakelists.txt"
    }

    ASSET_ONLY_FILES = {
        "icons.h": "Statische UI-Bitmap-Assets (imageArrow, Icons etc. - ausgelassen zur Vermeidung von Token-Ballast)"
    }

    if os.path.isdir(src_dir):
        for entry in sorted(os.listdir(src_dir)):
            full_path = os.path.join(src_dir, entry)
            if not os.path.isfile(full_path):
                continue

            entry_lower = entry.lower()

            # Sicherheitscheck: Niemals Secrets oder Build-Dateien aufnehmen
            if "secret" in entry_lower or entry_lower in EXCLUDED_FILENAMES:
                continue

            # Reine Asset-Dateien kennzeichnen statt 66 KB Hex-Arrays zu senden
            if entry in ASSET_ONLY_FILES:
                context_parts.append(f"#### Datei: `src/{entry}`\n/* {ASSET_ONLY_FILES[entry]} */\n")
                continue

            # Nur C/C++ Quellcode- und Headerdateien aufnehmen
            ext = os.path.splitext(entry)[1].lower()
            if ext in (".cpp", ".c", ".h", ".hpp"):
                try:
                    with open(full_path, "r", encoding="utf-8", errors="ignore") as f:
                        content = sanitize_diff(f.read())
                        context_parts.append(f"#### Datei: `src/{entry}`\n```cpp\n{content}\n```")
                except Exception as e:
                    print(f"Hinweis: Konnte src/{entry} nicht lesen: {e}")

    return "\n\n".join(context_parts)



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


def call_gemini(diff_text, codebase_text, api_key):
    # Diff bei extrem großen Änderungen kürzen
    max_diff_chars = 40000
    if diff_text and len(diff_text) > max_diff_chars:
        diff_text = diff_text[:max_diff_chars] + "\n\n[... Diff gekürzt auf 40.000 Zeichen ...]"

    prompt_parts = [
        "Führe die Firmware-Risikoanalyse durch. Fokussiere dich primär auf das Git-Diff und leite konkrete Test-Tasks für den Endnutzer am fertigen Gerät im Zusammenspiel mit der PaperlessPaper-App ab (keine Entwickler-/Build-Befehle!). Die ganzheitliche Systemanalyse des Gesamtcodes steht an letzter Stelle und ist bei geringem Risiko kompakt zu halten.\n\n"
    ]

    if diff_text:
        prompt_parts.append(
            f"=================================================================\n"
            f"### 1. AKTUELLE CODE-ÄNDERUNGEN (GIT-DIFF):\n"
            f"=================================================================\n"
            f"```diff\n{diff_text}\n```\n\n"
        )
    else:
        prompt_parts.append(
            "=================================================================\n"
            "### 1. AKTUELLE CODE-ÄNDERUNGEN:\n"
            "=================================================================\n"
            "*(Keine spezifischen Zeilenänderungen im Diff – Vollständiger Audit des Repository-Stands)*\n\n"
        )

    prompt_parts.append(
        f"=================================================================\n"
        f"### 2. VOLLSTÄNDIGER FIRMWARE-QUELLCODE & KONFIGURATION:\n"
        f"=================================================================\n"
        f"{codebase_text}\n"
    )

    user_prompt_text = "".join(prompt_parts)

    payload = {
        "contents": [
            {
                "parts": [
                    {"text": SYSTEM_PROMPT},
                    {"text": user_prompt_text}
                ]
            }
        ],
        "generationConfig": {
            "temperature": 0.2,
            "maxOutputTokens": 4096
        }
    }

    # Bevorzugtes Modell: gemini-3.8-flash, gefolgt von erprobten Fallbacks
    models_to_try = [os.environ.get("GEMINI_MODEL", "gemini-3.8-flash")]
    for fallback in ["gemini-3.7-flash", "gemini-3.6-flash", "gemini-3.5-flash", "gemini-flash-latest"]:
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
            with urllib.request.urlopen(req, timeout=90) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                text = data["candidates"][0]["content"]["parts"][0]["text"]
                return text, model
        except urllib.error.HTTPError as e:
            error_body = e.read().decode("utf-8", errors="ignore")
            if e.code == 429:
                print(f"⚠️ Modell '{model}' meldet HTTP 429 (Quota/Prepayment Credits erschöpft).")
                last_error = f"HTTP 429 (Quota/Credits erschöpft): {error_body}"
                time.sleep(1)
                continue
            elif e.code in (404, 500, 502, 503, 504):
                print(f"⚠️ Modell '{model}' meldet HTTP {e.code}. Wechsle automatisch zum nächsten Fallback-Modell...")
                last_error = f"HTTP {e.code}: {error_body}"
                time.sleep(1)
                continue
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
    parser = argparse.ArgumentParser(description="KI-Firmware Risikoanalyse & Gesamtsystem-Audit via Google Gemini API")
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

    # Gesamten Firmware-Quellcode laden
    codebase_text = get_codebase_context()

    if not diff_text and not codebase_text:
        print("ℹ️ Weder Code-Änderungen noch Quellcode-Dateien gefunden. Keine KI-Analyse erforderlich.")
        sys.exit(0)

    print("=" * 65)
    print("🤖 STARTE KI-RISIKOANALYSE & GESAMTSYSTEM-AUDIT")
    if diff_text:
        print(f"📏 Diff-Größe:      {len(diff_text):,} Zeichen")
    else:
        print("📏 Diff:            Keine uncommitteten Änderungen (Gesamtaudit)")
    print(f"📦 Quellcode-Basis: {len(codebase_text):,} Zeichen (alle funktionalen src/* Dateien, part.csv, platformio.ini)")
    print("=" * 65)

    try:
        report, used_model = call_gemini(diff_text, codebase_text, api_key)
        print(f"✨ Verwendetes Modell: {used_model}")
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

        # Wenn HOCH (HIGH) oder BLOCKIERT erkannt wird, IMMER abbrechen!
        if "HOCH (HIGH)" in report or "BLOCKIERT" in report:
            print("\n❌ Build durch KI-Risikoanalyse blockiert (HOCH / BLOCKIERT erkannt)!")
            print("::error::Firmware AI Risk Analysis flagged HIGH risk / BLOCK!")
            sys.exit(1)

        sys.exit(0)

    except Exception as e:
        print(f"❌ Fehler bei der KI-Risikoanalyse: {e}")
        print("::error::KI-Risikoanalyse fehlgeschlagen! Pipeline wird abgebrochen.")
        sys.exit(1)


if __name__ == "__main__":
    main()
