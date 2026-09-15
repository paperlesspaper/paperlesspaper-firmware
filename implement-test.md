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

---

### Phase 2: Remote-Testbench & HIL-Suite (Windows Host)
*Ziel: Vollautomatischer Test auf echten Geräten am Windows-PC via Python/PyTest.*

- [ ] **AP 2.1: Testbench Controller (`testbench/hardware_controller.py`)**
  - [ ] COM-Port-Erkennung für EPD7 und EPD13.
  - [ ] Serial-Relais / DTR-RTS Reset-Funktion.
  - [ ] Asynchroner Serial-Monitor mit Timeout & Event-Regex (`[MAIN] INIT Device V: ...`, `[NETWORK] WiFi Connected`, `[OTA] ...`).
- [ ] **AP 2.2: Automated Flasher (`testbench/flasher.py`)**
  - [ ] Ansteuerung von `esptool.py` zum Flashen einer definierten Baseline-Altversion vor dem Test.
- [ ] **AP 2.3: End-to-End HIL Testfälle (`testbench/test_epd_lifecycle.py`)**
  - [ ] Test 1: Baseline Flash -> Boot -> Deploy WiFi Connect.
  - [ ] Test 2: Trigger OTA (Kandidaten-Firmware) -> Serial Log Quittung -> Reboot in neue Firmware.
  - [ ] Test 3: Device Activation Handshake (`$aws/things/+/activateepaper`).
  - [ ] Test 4: Image Request -> Render -> Prüfung des Bestätigungs-Events in DynamoDB `iotPayload`.
  - [ ] Test 5: Deactivate -> Verifikation des Eintritts in Deep Sleep.
- [ ] **AP 2.4: Windows GitHub Actions Self-Hosted Runner Konfiguration**
  - [ ] Setup-Anleitung und PowerShell-Startskripte für den Windows-Testrechner.

---

### Phase 3: Canary-Auswahl & DynamoDB Health Gate
*Ziel: Gezielte Zuweisung an 10 Pilotkunden mit hoher Aufwachrate und automatische Stabilitätsüberwachung.*

- [ ] **AP 3.1: DynamoDB Pilot-Selector (`tools/select_pilot_devices.py`)**
  - [ ] Scan/Query auf DynamoDB `iotCatalog`.
  - [ ] Filtert je 10 Geräte `EPD7` und `EPD13` mit `deviceData.timeout in [60, 180]`.
  - [ ] Setzt via AWS IoT Core Named Shadow (`settings.otaUrl`) die Pre-Release FOTA URL.
  - [ ] Exportiert `canary_target_devices.json` für das Health-Gate.
- [ ] **AP 3.2: Canary Health Gate Monitor (`tools/canary_monitor.py`)**
  - [ ] Regelmäßige Abfrage von DynamoDB `iotCatalog` und `iotPayload` für die 20 Pilotgeräte.
  - [ ] Auswertung: Update-Erfolgsquote (100% Ziel), Batterieverhalten (`batLevel`), Neustart-Zähler (`StartCounter`).
  - [ ] Generierung eines Statusreports in GitHub Actions.
  - [ ] Abbruch-Trigger bei Anomalien.

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
