# 🔬 PaperlessPaper HIL Testbench

Hardware-in-the-Loop (HIL) Testsuite für ESP32-C6 basierte E-Paper Displays (**EPD7** 7.5" und **EPD13** 13.3") auf einem Windows Self-Hosted GitHub Runner.

---

## 🏗️ Hardware-Setup & Architektur

* **Displays:** ESP32-C6 Boards, angeschlossen über **Silicon Labs CP210x** USB-zu-UART Bridges.
* **Stromversorgung & Reset:** Schaltung über **CH340 USB-Relais** (`relay_hex`), da DTR/RTS hardwareseitig nicht angebunden sind.
* **Automatische Zuordnung (`verify_and_pair_hardware`):**
  Die Testbench erkennt alle CP210x- und CH340-Ports automatisch, schaltet jedes Relais kurz stromlos und analysiert die seriellen Boot-Logs (`[MAIN] INIT Device V: ...`), um Relais-Port, Display-Port, MAC-Adresse und Display-Typ (EPD7 vs. EPD13) zuzuordnen. Die Zuordnung wird in `testbench/hardware_mapping.json` (git-ignoriert) gecacht.

```
+--------------------------------------------------------------------+
|                       Windows Runner Host                          |
|                                                                    |
|  +--------------------+               +-------------------------+  |
|  | CH340 USB-Relais   |--[Power VCC]->| ESP32-C6 Display (EPD)  |  |
|  +--------------------+               +-------------------------+  |
|            ^                                       |               |
|            |                                   [UART Logs]         |
|            |                                       v               |
|  +--------------------------------------------------------------+  |
|  | testbench.run_testbench / pytest (ESP32HardwareController)  |  |
|  +--------------------------------------------------------------+  |
|            |                                       |               |
|       [REST API]                              [BLE GATT]           |
|            v                                       v               |
|  +--------------------+               +-------------------------+  |
|  | AWS API Gateway    |               | Display BLE Advertising |  |
|  | (Auth0 M2M Token)  |               | (WLAN Provisioning)     |  |
|  +--------------------+               +-------------------------+  |
+--------------------------------------------------------------------+
```

---

## 📋 Voraussetzungen

1. **Python 3.10+** im System-PATH.
2. **Abhängigkeiten installieren:**
   ```powershell
   pip install -r testbench/requirements.txt
   ```
3. **Automatische Runner-Einrichtung (optional):**
   ```powershell
   powershell -ExecutionPolicy Bypass -File testbench/setup_runner.ps1
   ```

---

## ⚙️ Konfiguration (`.env`)

Kopiere [`.env.example`](file:///c:/WSL/paperlesspaper-firmware/.env.example) nach `.env` im Projektstamm:

```env
# WLAN für Hardware-Tests
HIL_WIFI_SSID="DeinTestWLAN"
HIL_WIFI_PASSWORD="DeinWLANPasswort"

# Auth0 M2M Authentifizierung
AUTH0_DOMAIN="dein-tenant.eu.auth0.com"
AUTH0_CLIENT_ID="deine-client-id"
AUTH_0_CLIENT_SECRET="dein-client-secret"
AUTH_0_AUDIENCE="localhost:3000/"
AUTH_0_GRANT_TYPE="client_credentials"

# AWS Backend & Endpunkte
HIL_API_BASE_URL="https://<api-id>.execute-api.eu-central-1.amazonaws.com/Prod"
HIL_S3_BUCKET="dein-s3-bucket"
AWS_IOT_ENDPOINT="https://<ats-id>.iot.eu-central-1.amazonaws.com"
AWS_REGION="eu-central-1"

# AWS IAM Credentials (zur Verifikation in DynamoDB & S3)
AWS_ACCESS_KEY_ID="AKIA..."
AWS_SECRET_ACCESS_KEY="..."
```

> **Sicherheit:** Der Auth0-Bearer-Token wird verschlüsselt auf TTL-Basis in `testbench/cache/.auth0_token_cache.json` für 30 Tage gecacht und bei `401 Unauthorized` automatisch invalidiert. Keine Credentials landen im Git.

---

## 🚀 Testbench ausführen (CLI)

### 1. COM-Ports anzeigen
Listet angeschlossene CP210x-Displays und CH340-Relais auf:
```powershell
python -m testbench.run_testbench --list-ports
```

### 2. Hardware-Zuordnung prüfen & neu einmessen
Führt selektive Relais-Pulse durch und ordnet Ports den Displays zu:
```powershell
python -m testbench.run_testbench --verify
```

### 3. Factory-Reset per Hardware-Power-Cycles
Setzt das Display durch mindestens 6 aufeinanderfolgende Relais-Power-Cycles auf Werkseinstellungen zurück:
```powershell
python -m testbench.run_testbench --factory-reset --target epd7
```

### 4. Fokussierter BLE-WLAN-Provisionierungs-Test
Führt Factory-Reset durch und verifiziert BLE-Scanning sowie GATT-Provisioning:
```powershell
python -m testbench.run_testbench --test-ble --target epd7
```

### 5. Vollständige Testsuite ausführen
Führt die komplette Lifecycle-Prüfung aus (Factory-Reset -> BLE -> Aktivierung -> Bild-Upload & Render -> Deaktivierung & Deep Sleep):
```powershell
# Nur EPD7
python -m testbench.run_testbench --target epd7 -v

# Nur EPD13
python -m testbench.run_testbench --target epd13 -v

# Alle angeschlossenen Geräte
python -m testbench.run_testbench --target all -v

# Optional inkl. zeitintensiver OTA-Firmware-Update-Tests
python -m testbench.run_testbench --target epd7 --run-ota -v
```

### 6. Direkt über PyTest
```powershell
python -m pytest testbench/test_epd_lifecycle.py -k "TestEPD7Lifecycle" -s -v
```

---

## 🔄 Lifecycle-Testphasen (`test_epd_lifecycle.py`)

| Phase | Testfall | Beschreibung |
| :--- | :--- | :--- |
| **0** | `test_00_factory_reset_via_power_cycles` | Setzt das Gerät vor dem Test durch mindestens 6 aufeinanderfolgende Relais-Power-Cycles auf Werkseinstellungen zurück (`StartCounter >= 5`) und verifiziert `[MAIN] Reset - ACT 1 \| WIFI 1`. |
| **1** | `test_01_ble_wifi_provisioning` | Prüft BLE-Advertising des werksfrischen Geräts, liest gescannte Netze via GATT und überträgt WLAN-Zugangsdaten. |
| **2** | `test_02_production_firmware_ota` | Flasht die offizielle Produktions-Firmware via OTA-Manifest JSON (`http://<bucket-domain>/espfota_<target>.json`) und verifiziert den Reboot in Produktionsversion (z.B. V3.0.17). |
| **3** | `test_03_candidate_firmware_ota` | Lädt das frisch gebuildete Firmware-Binary temporär als `test-firmware-<target>.bin` nach S3 hoch, triggert Direkt-OTA via URL, verifiziert den Reboot in die Kandidaten-Version und löscht die Test-Binärdatei anschließend sofort wieder aus S3. |
| **4** | `test_04_device_activation` | Ruft REST-API `POST /activatedevice` auf, wartet auf autonomen Handshake des Displays (`[AWS RX] Device is activated`) ohne Relais-Reset und validiert in DynamoDB `iotCatalog`. |
| **5** | `test_05_picture_render_and_payload` | Fordert Presigned URL an (`POST /uploads`), lädt Testbild via HTTP PUT hoch, weckt Display auf, prüft Download (`[DL] Done`), Rendering (`[EPD] Set Image Done`) und Quittung in DynamoDB `iotPayload`. |
| **6** | `test_06_deactivate_and_deep_sleep` | Deaktiviert das Gerät per REST-API (`reset: True`), bereinigt Testbilder und verifiziert den sauberen Eintritt in Deep Sleep (`[MAIN] Going to Sleep for 0 seconds`). |


---

## 🤖 CI/CD & GitHub Actions Pipeline

Die HIL-Testsuite ist vollständig in die kontinuierliche Build- & Deployment-Pipeline ([`.github/workflows/dev.yml`](file:///c:/WSL/paperlesspaper-firmware/.github/workflows/dev.yml)) sowie in den manuellen Test-Workflow ([`.github/workflows/hil-test.yml`](file:///c:/WSL/paperlesspaper-firmware/.github/workflows/hil-test.yml)) integriert.

### Pipeline-Architektur (`dev.yml`)

```
  +-------------------------------------------------------------+
  | 1. Pre-Flight & AI Risk Analysis (Gemini Flash)             |
  +-------------------------------------------------------------+
                                 |
                                 v
  +-------------------------------------------------------------+
  | 2. Matrix Build (EPD7 & EPD13 Binaries via PlatformIO)      |
  +-------------------------------------------------------------+
                                 |
                                 v
  +-------------------------------------------------------------+
  | 3. HIL Test: EPD7 (7.5" Hardware)                           |
  |    (Sequentiell auf Windows Runner, generiert junit_epd7)   |
  +-------------------------------------------------------------+
                                 |
                                 v  (needs: hil_test_epd7)
  +-------------------------------------------------------------+
  | 4. HIL Test: EPD13 (13.3" Hardware)                         |
  |    (Streng nacheinander, verhindert Relay-/BLE-Kollision)   |
  +-------------------------------------------------------------+
                                 |
                                 v  (needs: [build, hil_test_epd7, hil_test_epd13])
  +-------------------------------------------------------------+
  | 5. S3 Deployment (Upload nach dev-Bucket & Manifests)       |
  |    (GATING: Wird NUR ausgeführt, wenn alle Tests bestehen!) |
  +-------------------------------------------------------------+
                                 |
                                 v  (if: always())
  +-------------------------------------------------------------+
  | 6. Testprotokoll & Release-Bewertung                        |
  |    - Aggregation der JUnit XMLs & KI-Risikoanalyse          |
  |    - Schreibt formatiertes Protokoll in $GITHUB_STEP_SUMMARY|
  |    - Exportiert hil_test_protocol.md & .json für Auto-PR    |
  |    - STRIKTER ABBRUCH bei Fehlern im Deploy oder in Tests   |
  +-------------------------------------------------------------+
```

### Strenge Qualitäts- & Sicherheits-Garantien
1. **Deployment-Gating durch echte Hardware:** Das S3-Deployment (`deploy`) läuft **erst nach** erfolgreicher Hardware-Verifikation. Schlägt ein HIL-Test fehl, wird die Firmware gar nicht erst in den S3-Bucket hochgeladen, sodass niemals fehlerhafte Software an Test- oder Produktivgeräte verteilt wird.
2. **Kein vorheriges Deployment nötig:** Der HIL-Test benötigt kein vorheriges Deployment, da er die frisch kompilierten Binärdateien direkt aus den GitHub Actions Build-Artefakten bezieht und für das Test-OTA eine isolierte, temporäre S3-Testdatei (`test-firmware-<target>.bin`) nutzt, die direkt nach dem Test gelöscht wird.
3. **Kein gleichzeitiger Hardware-Zugriff:** EPD7 und EPD13 laufen **immer sequentiell** (Job `hil_test_epd13` wartet auf `hil_test_epd7`), um COM-Port-, Relais- und BLE-Kollisionen auf dem Host auszuschließen.
4. **Strikter Abbruch:** Schlägt die Risikoanalyse, der Build, ein Hardware-Test oder das Deployment fehl, bricht die Pipeline mit Exit-Code 1 ab.
5. **Automatischer Pull Request:** Das generierte JSON-Artefakt (`hil_test_protocol.json`) liefert `overall_success: true/false`, Kennzahlen und Einzeltestergebnisse für automatische Freigabe- und Mergebots.

