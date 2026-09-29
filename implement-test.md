# E-Paper Firmware Test- & Staged-Rollout-Pipeline (EPD7 & EPD13)
# Tracking- & Implementierungsdokument: implement-test.md

Dieses Dokument dient zur Nachverfolgung der Implementierungsschritte für die CI/CD- und Hardware-in-the-Loop (HIL) Testpipeline der Firmware für **EPD7** und **EPD13**.

---

## 📋 Vorgaben & Rahmenbedingungen

* **Testbench Host:** Windows PC (Intel-basiert) mit angeschlossenen Testgeräten (1x EPD7, 1x EPD13) über USB.
* **Hardware-Reset:** Steuerbar über USB-Serial (DTR/RTS bzw. Serial-Relais) zur Auslösung von Hard-Resets / Bootloader.
* **Netzwerk:** Lokales "Deploy WiFi" (`DEFAULT_WIFI_SSID`) am Testplatz.
* **Verifikation des Bildrenders:** Payload- & Telemetrie-basiert über DynamoDB `iotPayload` (`epaper-updatestate` MQTT-Quittung) + serielle Schnittstelle.
* **Pilotgruppe (Canary):** Dynamische Auswahl von je 10 EPD7- und 10 EPD13-Geräten aus der DynamoDB `iotCatalog`, die `deviceData.timeout == 60` oder `180` besitzen (hohe Aufwachfrequenz).
* **AI-Risikoanalyse:** Auswertung von PR/Commit Diffs über Google Gemini API.
* **Referenzen & Bausteine:** Muster aus `epaper-shipping-tool` / `epaper-deploy` (Serieller Reset, Flashen, Logparsing).

---

## 🎯 Phasen- & Arbeitspaket-Übersicht

```
[ Phase 1: Günstig & Sicher (Sofortige Repo-Absicherung) ]
       │
       ▼
[ Phase 2: Remote-Testbench & HIL-Testsuite (Windows Host) ]
       │
       ▼
[ Phase 3: Canary-Auswahl & DynamoDB Health Gate ]
       │
       ▼
[ Phase 4: Staged Production Rollout & Rollback ]
```

---

## 📦 Arbeitspakete & Fortschritt

### Phase 1: Günstig & Sicher (Sofortige Repo-Absicherung)
*Ziel: Beseitigung fragiler Skripte im bestehenden Repository, Absicherung der Display-Konfigurationen und KI-Pre-Flight Check.*

- [x] **AP 1.1: Display-Typ Mutex in `types.h` & Pre-Build Validierung**
  - [x] Ersetzen der anfälligen `sed`-Ersetzungen in GitHub Workflows durch dedizierte Preprocessor-Checks (`#error` bei doppelter Definition, sicherer Fallback).
  - [x] Erstellen eines Validierungsskripts `tools/validate_firmware_config.py`, das vor dem Kompilieren prüft, ob alle Schalter konsistent gesetzt sind (erfolgreich getestet).
- [x] **AP 1.2: PlatformIO Environment-Separation**
  - [x] Saubere Umgebungen in `platformio.ini` und `deploy/platformio.ini` definiert (`[env:epd7]` und `[env:epd13]`), sodass `pio run -e epd7` und `pio run -e epd13` ohne Code-Dateimanipulationen reproduzierbar gebaut werden können.
- [x] **AP 1.3: Binary- & Partitionsgrößenprüfung**
  - [x] Skript `tools/check_firmware_size.py` zur Überprüfung der generierten `firmware.bin` gegen die maximalen App-Partitionen in `part.csv` (1.81 MB Limit, 85% Warnschwelle, SHA-256 Checksumme).
- [x] **AP 1.4: Gemini AI Code-Risk-Check Step**
  - [x] Skript `tools/ai_risk_analysis.py` implementiert (nutzt Google Gemini API mit Zero-Dependency Standardbibliothek, analysiert Git-Diffs auf FOTA, Watchdogs, Deep Sleep und Memory Leaks).
- [x] **AP 1.5: Pipeline-Integration (.github/workflows/dev.yml & prod.yml)**
  - [x] `pre_flight` Job vor Matrix-Build eingefügt (Checkout fetch-depth 2, Pre-Flight Validierung, AI Risk Analysis).
  - [x] Fragile `sed`-Ersetzungen aus beiden Workflows entfernt; Umstellung auf `pio run -e ${{ matrix.suffix }}`.
  - [x] `tools/check_firmware_size.py` nach dem Build eingefügt (prüft Binaries und Partitionen).
  - [x] Pfade zur Artefakterstellung auf `./.pio/build/${{ matrix.suffix }}/firmware.bin` angepasst.
  - [x] Resilienz gegen HTTP 503 / 429 (automatischer Modell-Failover 3.8 -> 3.6 -> 2.0).
  - [x] Hartes Gating: `HOCH / BLOCKIERT` stoppt den Build immer; `prod.yml` läuft im `--strict`-Modus.

---

### Phase 2: Remote-Testbench & HIL-Suite (Windows Host)
*Ziel: Vollautomatischer Test auf echten Geräten am Windows-PC via Python/PyTest.*

- [x] **AP 2.1: Testbench Controller (`testbench/hardware_controller.py`)**
  - [x] COM-Port-Erkennung: Displays werden als **CP210x**, Relais als **CH340** automatisch erkannt.
  - [x] Automatische Relais-Display-Zuordnung: Selektiver Power-Cycle einzelner CH340-Relais ermittelt durch Boot-Log-Beobachtung (`[MAIN] INIT Device V: ...`), welches Relais welches Display steuert.
  - [x] Automatische Seriennummern- & Typ-Erkennung: Auslesen von MAC-Adresse (`CLIENT_ID` Suffix) und Display-Typ (`epd7` vs. `epd13`) aus den Boot-Logs der Firmware.
  - [x] Hardware-Reset ausschließlich über USB-Relais (`relay_hex`), da DTR und Boot-Tasten hardwareseitig nicht angebunden sind.
  - [x] Asynchroner Serial-Monitor mit Timeout & Event-Regex (`[MAIN] INIT Device V: ...`, `[NETWORK] WiFi Connected`, `[OTA] ...`).
  - [x] Automatische Vorab-Prüfung (`verify_and_pair_hardware()`): Blockiert Tests mit klarer Diagnose, falls ein Display oder Relais fehlt.
- [x] **AP 2.2: Shadow-basierte FOTA-Steuerung & Manifest-Verwaltung (`testbench/aws_client.py` & `flasher.py`)**
  - [x] FOTA-Updates werden rein über die AWS IoT Named Shadow API (`https://<iot-endpoint>/things/{thingName}/shadow?name=settings`) gesteuert.
  - [x] Automatisches Herunterladen und Parsen der aktuellen Produktions-Manifeste (`espfota_epd7.json` / `espfota_epd13.json`) sowie Kandidaten-Manifeste (`espfota_epd7_pre.json` / `espfota_epd13_pre.json`).
- [x] **AP 2.3: End-to-End HIL Testfälle (`testbench/test_epd_lifecycle.py` & `run_testbench.py`)**
  - [x] Test 1: Sicherstellung Baseline: Prüft aktuelle Firmware; falls abweichend, setzt Shadow `otaUrl` auf Produktion und erzwingt OTA via Relais-Power-Cycle.
  - [x] Test 2: Trigger OTA auf Kandidaten-Firmware via Shadow API (`settings.otaUrl`) -> Relais Power-Cycle -> Log-Quittung -> Reboot in neue Firmware mit Versionsprüfung.
  - [x] Test 3: Device Activation Handshake (`$aws/things/+/activateepaper`) und Validierung in DynamoDB `iotCatalog`.
  - [x] Test 4: Image Request -> Render -> Prüfung des Bestätigungs-Events in DynamoDB `iotPayload`.
  - [x] Test 5: Deactivate -> Verifikation des Eintritts in Deep Sleep.
  - [x] CLI-Runner `testbench/run_testbench.py` mit `--verify`, `--list-ports` (CP210x vs. CH340) und obligatorischer Vorab-Prüfung vor dem Testlauf.
- [x] **AP 2.4: Windows GitHub Actions Self-Hosted Runner Konfiguration**
  - [x] Setup-Skript `testbench/setup_runner.ps1` für Windows Runner (Python, Treiber, Abhängigkeiten, Runner-Registrierung).
  - [x] Dedizierter GitHub Actions Workflow `.github/workflows/hil-test.yml` mit `workflow_dispatch` Parametern.

---

### Phase 3: Canary-Auswahl & DynamoDB Health Gate
*Ziel: Gezielte Zuweisung an 10 Pilotkunden mit hoher Aufwachrate und automatische Stabilitätsüberwachung.*

- [x] **AP 3.1: DynamoDB Pilot-Selector (`tools/select_pilot_devices.py`)**
  - [x] Paginierter Multi-Page Scan auf DynamoDB `iotCatalog`.
  - [x] Filtert und bewertet je 10 Geräte `EPD7` und `EPD13` mit `deviceData.timeout in [60, 180]`, aktuellem Aktivitätsstatus und Batteriezustand.
  - [x] Generiert strukturierte Empfehlungsberichte (`pilot_recommendations.md` & `pilot_recommendations.json`).
  - [x] Zero-OTA Safe-Mode (Standard: Dry-Run / Read-Only). Zielgeräte werden vom Nutzer bestimmt (`canary_target_devices.json`).
  - [x] Setzt via AWS IoT Core Named Shadow (`settings.otaUrl`) die Pre-Release FOTA URL nur bei explizitem `--apply --confirm-ota`.
- [x] **AP 3.2: Canary Health Gate Monitor (`tools/canary_monitor.py`)**
  - [x] Regelmäßige Abfrage von DynamoDB `iotCatalog` und `iotPayload` für die Pilotgeräte.
  - [x] Auswertung: Update-Erfolgsquote (Adoption-Rate %), Batterieverhalten (`batLevel`), Neustart-Zähler / Crash-Loops (`StartCounter`) und Event-Quittungen.
  - [x] Generierung eines strukturierten Statusreports in GitHub Actions (`$GITHUB_STEP_SUMMARY` & Artefakte `canary_health_report.md` / `.json`).
  - [x] Abbruch-Trigger bei Anomalien (Exit Code 1 für Pipeline-Gating).
  - [x] Dedizierter Dispatch-Workflow `.github/workflows/canary-health-gate.yml`.

---

### Phase 4: Staged Production Rollout & Rollback
*Ziel: Sicheres manuelles Release-Gate und automatisierter Rollout.*

- [ ] **AP 4.1: Manual Approval Gate & Promotion Job in GitHub Actions**
  - [ ] GitHub Environment `production` mit Reviewer-Pflicht.
  - [ ] Promotion-Skript: Kopiert `_pre.bin` auf S3 zu `firmware_epd7.bin` und `firmware_epd13.bin`.
  - [ ] Aktualisiert die Produktions-Manifeste `espfota_epd7.json` / `espfota_epd13.json`.
- [ ] **AP 4.2: 1-Click Rollback Workflow**
  - [ ] Manueller Workflow zum sofortigen Zurücksetzen auf die vorherige Versionsnummer auf S3 im Fehlerfall.

---

## 📝 Changelog & Bearbeitungsstand
* **2026-09-15:** Dokument initial erstellt. Phase 1 als erste Umsetzungstranche priorisiert.
* **2026-09-16:** Phase 2 (HIL-Testbench Pipeline-Integration, Relais-HEX Power-Cycling, REST-Aktivierung & Presigned S3 Tests) erfolgreich abgeschlossen.
* **2026-09-16:** Phase 3 (Canary Flotten-Scan, Empfehlungssystem mit Zero-OTA Safe-Mode und DynamoDB Health Gate Monitor `tools/canary_monitor.py`) vollständig implementiert.
