# 🛠️ PaperlessPaper Firmware Tools

Zentrale Sammlung von CLI- und CI/CD-Tools für Validierung, Build-Prüfung, KI-Risikoanalyse, HIL-Testprotokollierung und Canary-Deployments der ESP32-C6 E-Paper Firmware (**EPD7** & **EPD13**).

---

## 📋 Übersicht der Tools

| Skript | Beschreibung | Primäre Umgebung | Hauptabhängigkeiten |
| :--- | :--- | :--- | :--- |
| [`ai_risk_analysis.py`](#1-ai_risk_analysispy--ki-quellcode--firmware-risikoanalyse) | Ganzheitlicher KI-Sicherheits- und Code-Audit via Google Gemini API | CI/CD & Lokal | Python Standardbibliothek (`urllib`) |
| [`select_pilot_devices.py`](#2-select_pilot_devicespy--canary-rollout--pilot-selector) | Canary-Rollout, Shadow-Updates (`settings.otaUrl`) und Flottenscan | GitHub Actions & CLI | `boto3` |
| [`canary_monitor.py`](#3-canary_monitorpy--canary-health-gate-monitor) | Überwachung von Pilotgeräten auf Adoption, Boot-Loops & Stabilität | CI/CD & Lokal | `boto3` |
| [`check_firmware_size.py`](#4-check_firmware_sizepy--partitions--binärgrößen-check) | Prüft `firmware.bin` gegen OTA-Partitionsgrenzen in `part.csv` | Build-Pipeline | Python Standardbibliothek |
| [`validate_firmware_config.py`](#5-validate_firmware_configpy--pre-flight-konfigurationsprüfung) | Pre-Flight Validierung vor dem Kompilieren (Mutex, Version, Partitions) | Pre-Build | Python Standardbibliothek |
| [`generate_test_protocol.py`](#6-generate_test_protocolpy--hil-testprotokoll-generator) | Aggregiert HIL-JUnit-XMLs, Deployment-Status & KI-Audit für PRs | CI/CD (GitHub Actions) | Python Standardbibliothek |

---

## 1. `ai_risk_analysis.py` – KI-Quellcode- & Firmware-Risikoanalyse

Führt einen automatisierten statischen Sicherheits- und Architektur-Audit des **gesamten Firmware-Codes** sowie der aktuellen Änderungen (**Git-Diff**) mittels Google Gemini API (`gemini-3.8-flash`) durch.

### Funktionsumfang
- **Ganzheitlicher Quellcode-Kontext:** Liest automatisch alle funktionalen C/C++ Dateien im Ordner `src/` ([`main.cpp`](../src/main.cpp), [`epaper_display.cpp`](../src/epaper_display.cpp), [`epaper_display.h`](../src/epaper_display.h), [`flash_config.h`](../src/flash_config.h), [`types.h`](../src/types.h)) sowie fundamentale Systemkonfigurationen ([`part.csv`](../part.csv), [`deploy/platformio.ini`](../deploy/platformio.ini)) ein.
- **Fokus auf sinnvolle Firmware-Parts:** Reine Bitmap-Assets ([`icons.h`](../src/icons.h) mit 66 KB statischen Hex-Arrays) werden nicht als Roh-Bytes übertragen, sondern als Asset-Hinweis markiert, um das Kontext- und Token-Budget für Logik zu reservieren.
- **Sicherheits- & Regressionsprüfung:**
  - **OTA-Sicherheit:** Prüft FOTA-Pfade (MQTT `doc["ota"]` mit `forceUpdate()` vs. synchroner Boot-Check via `myEsp32FOTA.execHTTPcheck()`), Manifest-Parsing und Partitionsgrenzen.
  - **Boot-Loop- & Crash-Resilienz:** Validiert `StartCounter`-NVS-Logik, Reset-Ursachen (Panics/Brownouts) und Watchdog-Abdeckung (`tickerFailsave`).
  - **Power Management & Deep Sleep:** Erkennt potenzielle Endlosschleifen (Akku-Drain bei Verbindungsverlust) und prüft Bus-/Display-Abschaltungen.
  - **Speicher- & Heap-Stabilität:** Analysiert dynamische Allokationen und Puffergrenzen.
- **Strikter Schutz von Secrets:** Dateien wie `secrets.h`, `secrets.h.sample` und `.env*` sind strikt ausgeschlossen. Zertifikate (`.pem`, `.crt`), Private Keys, Passwörter und Tokens werden vor der Übertragung regex-maskiert.
- **Berichts-Priorisierung:**
  1. **Diff-Analyse (Prio 1):** Konkrete Bewertung der Änderungen und direkten Auswirkungen.
  2. **Endprodukt- & App-Testtasks (Prio 2):** Strukturierte Test-Aufgaben (`- [ ] **Task X: ...**`) für das Zusammenspiel zwischen dem physischen E-Paper Rahmen und der PaperlessPaper App (BLE-Onboarding, Bildübertragung/Rendering, Settings-Sync, Taster). Reine Entwickler- und Compiler-Befehle sind strikt ausgeschlossen.
  3. **Ganzheitliche Systemanalyse (Prio 3):** Steht an letzter Stelle – bei geringem Risiko extrem kompakt und prägnant gehalten.
- **Zero-Dependency:** Nutzt ausschließlich Python-Standardmodule (`urllib`), liest Keys aus der Umgebung oder `.env`.

### Wichtigste Optionen
- `--base <ref>`: Git Basis-Ref für den Diff (z. B. `origin/main` oder `HEAD~1`).
- `--staged`: Prüft nur gestagte Änderungen (`git diff --cached`).
- `--output <pfad>`: Speichert den Bericht als Markdown (z. B. `risk_report.md`).
- `--strict`: Beendet mit Exit-Code 1 bei Risikostufe `HOCH (HIGH)` oder Status `BLOCKIERT`.

### Nutzung
```bash
# Lokaler Gesamtaudit (Quellcode + uncommittete Änderungen)
python tools/ai_risk_analysis.py

# In CI/CD mit Markdown-Export und strengem Abbruch bei Risiken
python tools/ai_risk_analysis.py --output risk_report.md --strict
```

---

## 2. `select_pilot_devices.py` – Canary Rollout & Pilot-Selector

Verwaltet das gezielte Rollout von Pre-Release Firmware auf Pilotgeräte über AWS DynamoDB und AWS IoT Device Shadows.

### Funktionsumfang
- **Zero-OTA Sicherheitsgarantie (Dry-Run per Default):** Führt Änderungen nur aus, wenn explizit `--apply` **und** `--confirm-ota I_CONFIRM_CANARY_OTA` übergeben werden.
- **Automatischer URL-Resolver:** Wählt abhängig vom Branch automatisch das passende FOTA-Manifest:
  - Branch `main`/`master`: `http://<bucket>/espfota_{target}_pre.json`
  - Feature-/Dev-Branches: `http://<bucket>/espfota_{target}_dev.json`
- **Canary Deploy:** Schreibt die FOTA-URL in den AWS IoT Named Shadow `settings.otaUrl` der Zielgeräte.
- **Rollback / Reset:** Setzt Zielgeräte auf die reguläre Hauptfirmware (`espfota_{target}.json`) zurück.
- **Flottenscan (`--recommend`):** Durchsucht DynamoDB `iotCatalog` nach geeigneten Pilotgeräten (Filter nach EPD7/EPD13, Wakeup-Intervallen 60/180s, Akkustand > 50%, `StartCounter == 0`).

### Wichtigste Optionen
- `--device-ids <liste>`: Kommagetrennte Geräte-IDs (z. B. `"epd7-001,epd13-002"`).
- `--target-devices <pfad>`: JSON-Datei mit Zielgeräten (alternativ `canary_target_devices.json`).
- `--apply`: Führt das Schreiben auf AWS IoT Shadows tatsächlich aus (erfordert `--confirm-ota`).
- `--confirm-ota <token>`: Bestätigungs-Token (`I_CONFIRM_CANARY_OTA`).
- `--reset`: Setzt Geräte auf die Hauptfirmware zurück.
- `--recommend`: Führt Flottenscan aus und generiert Empfehlungslisten (`pilot_recommendations.md`).

### Nutzung
```bash
# 1. Trockenlauf (Dry-Run): Zeigt an, welche Shadows aktualisiert würden
python tools/select_pilot_devices.py --device-ids "epd7-010203,epd13-040506"

# 2. Echtes Rollout auf Pilotgeräte ausführen
python tools/select_pilot_devices.py --device-ids "epd7-010203,epd13-040506" --apply --confirm-ota I_CONFIRM_CANARY_OTA

# 3. Rollback der Pilotgeräte auf die reguläre Produktions-Firmware
python tools/select_pilot_devices.py --device-ids "epd7-010203,epd13-040506" --reset --apply --confirm-ota I_CONFIRM_CANARY_OTA

# 4. Flottenscan nach stabilen Kandidaten durchführen
python tools/select_pilot_devices.py --recommend --count 5
```

---

## 3. `canary_monitor.py` – Canary Health-Gate Monitor

Überwacht den Zustand einer Pilotgeräte-Gruppe nach einem Canary-Rollout und fungiert als automatisches Qualitäts-Gate vor dem globalen Release.

### Funktionsumfang
- **DynamoDB Telemetrie-Auswertung:**
  - Liest Gerätestatus aus `iotCatalog` (`fwVersion`, `batLevel`, `StartCounter`, `lastUpdateTime`).
  - Prüft Event-Quittungen in `iotPayload` (z. B. Bestätigung von `update_ok`).
- **Anomalie-Erkennung:**
  - **Boot-Loops:** Erkennt Geräte mit `StartCounter >= 3`.
  - **Batterie-Einbrüche:** Markiert Spannungsabfälle > 10% nach dem Update.
  - **Offline-Geräte:** Identifiziert ausgebliebene Heartbeats anhand des Wakeup-Intervalls.
- **Automatische Versionsauflösung:** Erkennt die erwartete Zielversion direkt aus dem S3-Manifest oder über `--target-version`.
- **Watch-Modus:** Kontinuierliches Polling mit konfigurierbarem Intervall und Timeout.

### Wichtigste Optionen
- `--device-ids <liste>`: Zu überwachende Geräte-IDs.
- `--target-version <v>`: Erwartete Versionsnummer (z. B. `3.0.58`; optional, sonst Auto-Detect).
- `--min-adoption <pct>`: Erforderliche Erfolgsquote in Prozent (Standard: `80.0`).
- `--max-anomalies <anz>`: Maximal tolerierte Anomalien (Standard: `0`).
- `--watch`: Kontinuierliches Monitoring bis Schwellenwerte erreicht sind oder Timeout greift.
- `--interval <sek>`: Polling-Intervall in Sekunden (Standard: `30`).
- `--timeout <sek>`: Maximales Timeout in Sekunden (Standard: `600`).
- `--reset`: Überwacht das Zurücksetzen auf die Hauptfirmware.

### Nutzung
```bash
# Snapshot-Prüfung im Terminal
python tools/canary_monitor.py --device-ids "epd7-010203,epd13-040506"

# Kontinuierliches Polling (Watch-Modus) für CI/CD Health-Gate
python tools/canary_monitor.py --device-ids "epd7-010203,epd13-040506" --watch --interval 20 --timeout 300
```

---

## 4. `check_firmware_size.py` – Partitions- & Binärgrößen-Check

Validiert kompilierte Binärdateien gegen die in [`part.csv`](../part.csv) definierten Flash-Partitionsgrenzen.

### Funktionsumfang
- Liest die maximale App-Partitionsgröße (`app0` / `0x1D0000` = 1.900.544 Bytes) direkt aus der CSV.
- Berechnet exakte Dateigröße, Speicherauslastung in Prozent und den verbleibenden Puffer.
- Generiert SHA-256 Checksummen zur Integritätsprüfung im Release.
- Schreibt Metadaten nach `$GITHUB_STEP_SUMMARY` und warnt ab 85% Auslastung.
- Beendet bei Partitionsüberlauf mit Exit-Code 1 (verhindert FOTA-Flash-Abbrüche).

### Nutzung
```bash
python tools/check_firmware_size.py .pio/build/epd7/firmware.bin --target epd7 --output-json build_meta.json
```

---

## 5. `validate_firmware_config.py` – Pre-Flight Konfigurationsprüfung

Stellt vor dem Build-Prozess sicher, dass alle Konfigurationsdateien konsistent sind.

### Funktionsumfang
- **Compile-Time Mutex:** Verifiziert, dass [`src/types.h`](../src/types.h) den `#error`-Schutz gegen gleichzeitiges Aktivieren von `EPD_TYPE_7INCH` und `EPD_TYPE_13INCH` enthält.
- **Versions-Check:** Liest `SOFTWARE_VERSION` aus.
- **Partitions-Layout:** Überprüft das Vorhandensein beider OTA-Partitionen (`app0` und `app1`) in [`part.csv`](../part.csv).
- **Environment-Konsistenz:** Prüft, ob `[env:epd7]` und `[env:epd13]` in [`platformio.ini`](../platformio.ini) deklariert sind.

### Nutzung
```bash
python tools/validate_firmware_config.py --target epd7
```

---

## 6. `generate_test_protocol.py` – HIL-Testprotokoll-Generator

Führt die Ergebnisse aller Test- und Analysephasen zu einem einheitlichen Gesamtbericht zusammen.

### Funktionsumfang
- **JUnit-XML Parser:** Liest Testergebnisse der 7 Testphasen (Power-Cycles, BLE, FOTA, REST-Aktivierung, Bild-Upload, Sleep) für EPD7 und EPD13 ein.
- **Multi-Source Aggregation:** Kombiniert HIL-Testergebnisse, S3-Deployment-Status und die KI-Risikoanalyse (`risk_report.md`).
- **Artefakt-Erzeugung:**
  - Markdown-Report für Pull-Request-Kommentare und `$GITHUB_STEP_SUMMARY`.
  - Strukturierte JSON-Datei für automatisierte CI-Bots.
- **Pipeline-Gate (`--strict`):** Bricht bei fehlgeschlagenen Hardware-Tests oder KI-Risiko `HOCH` mit Exit-Code 1 ab.

### Nutzung
```bash
python tools/generate_test_protocol.py \
  --junit-epd7 hil_results_epd7.xml \
  --junit-epd13 hil_results_epd13.xml \
  --risk-report risk_report.md \
  --output-md hil_test_protocol.md \
  --output-json hil_test_protocol.json \
  --strict
```

---

## 🔐 Sicherheits- & Integrationshinweise

1. **Keine Klartext-Secrets:** Kein Skript im `tools/`-Ordner loggt oder überträgt AWS-Keys, Passwörter oder Zertifikate.
2. **Lokale Authentifizierung:** AWS-Tools (`select_pilot_devices.py`, `canary_monitor.py`) und die Gemini-KI (`ai_risk_analysis.py`) laden Umgebungsvariablen automatisch aus einer lokalen [`.env`](../.env)-Datei im Repository-Root, falls vorhanden.
3. **CI/CD Beständigkeit:** Alle Tools unterstützen sowohl Headless-Betrieb (Exit-Codes 0/1) als auch formatierte Konsolenausgaben für Windows PowerShell und Linux-Runner.
