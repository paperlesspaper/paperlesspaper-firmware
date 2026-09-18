"""
testbench/config.py
Zentrale Konfiguration für die Hardware-in-the-Loop (HIL) Testbench (Windows Runner).
Alle Werte können über Umgebungsvariablen oder eine lokale .env-Datei überschrieben werden.
"""

import os
import sys

# Windows CLI Encoding-Fix
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Optional .env laden (Projektordner, User-Home oder festes Testbench-Verzeichnis)
def _load_env():
    possible_paths = [
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".env")),
        os.path.expanduser("~/.paperlesspaper.env"),
        os.path.expanduser("~/.env"),
        "C:\\testbench\\.env"
    ]
    for env_path in possible_paths:
        if os.path.isfile(env_path):
            try:
                with open(env_path, "r", encoding="utf-8", errors="ignore") as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith("#"):
                            parts = line.split("=", 1)
                            if len(parts) == 2:
                                k, v = parts[0].strip(), parts[1].strip().strip('"\'')
                                if k not in os.environ:
                                    os.environ[k] = v
            except Exception:
                pass

_load_env()

# Aus Cache hardware_mapping.json laden falls vorhanden
_mapping_file = os.path.join(os.path.dirname(__file__), "hardware_mapping.json")
_cached_map = {}
if os.path.isfile(_mapping_file):
    try:
        import json
        with open(_mapping_file, "r", encoding="utf-8") as f:
            _cached_map = json.load(f)
    except Exception:
        pass

_epd7_cache = _cached_map.get("epd7", {})
_epd13_cache = _cached_map.get("epd13", {})

# Serieller Port & Hardware-Zuweisung (Standardwerte aus Cache oder Env)
EPD7_COM_PORT = os.environ.get("EPD7_COM_PORT", _epd7_cache.get("port") or "COM3")
EPD13_COM_PORT = os.environ.get("EPD13_COM_PORT", _epd13_cache.get("port") or "COM4")
EPD7_RELAY_PORT = os.environ.get("EPD7_RELAY_PORT", _epd7_cache.get("relay_port") or None)
EPD13_RELAY_PORT = os.environ.get("EPD13_RELAY_PORT", _epd13_cache.get("relay_port") or None)
RELAY_COM_PORT = os.environ.get("RELAY_COM_PORT", None)

SERIAL_BAUDRATE = int(os.environ.get("SERIAL_BAUDRATE", "115200"))
FLASH_BAUDRATE = int(os.environ.get("FLASH_BAUDRATE", "921600"))

# Reset-Methode: 'relay_hex' ist Pflicht (DTR hardwareseitig nicht angeschlossen)
RESET_METHOD = os.environ.get("RESET_METHOD", "relay_hex")

# WLAN & BLE Konfiguration (HIL_WIFI_SSID hat höchste Priorität)
WIFI_SSID = os.environ.get("HIL_WIFI_SSID", os.environ.get("ENV_WIFI_SSID_DEPLOY", ""))
WIFI_PASSWORD = os.environ.get("HIL_WIFI_PASSWORD", os.environ.get("ENV_WIFI_PW_DEPLOY", ""))

# BLE GATT UUIDs (gemäß src/main.cpp)
BLE_SERVICE_WIFI_DATA = "0515c086-7b0c-11ed-a1eb-0242ac120002"
BLE_CHAR_WIFI_SSID = "090b0ef2-7b0d-11ed-a1eb-0242ac120002"
BLE_CHAR_WIFI_PASSWORD = "a62eed84-7b0d-11ed-a1eb-0242ac120002"

BLE_SERVICE_DEVICE_DATA = "7f74170e-7b0e-11ed-a1eb-0242ac120002"
BLE_CHAR_WIFI_CONNECTED = "4c578d4c-7b0e-11ed-a1eb-0242ac120002"
BLE_CHAR_WIFI_SCAN = "5131a3fc-7b0e-11ed-a1eb-0242ac120002"

# Definierte Test-Seriennummern / Client-IDs
EPD7_DEVICE_ID = os.environ.get("EPD7_DEVICE_ID", _epd7_cache.get("uid") or "epd7-test001")
EPD13_DEVICE_ID = os.environ.get("EPD13_DEVICE_ID", _epd13_cache.get("uid") or "epd13-test001")

# AWS Konfiguration & IoT Shadow Endpoint
AWS_REGION = os.environ.get("AWS_REGION", "eu-central-1")
_raw_iot_ep = os.environ.get("AWS_IOT_ENDPOINT", os.environ.get("ENV_AWS_IOT_ENDPOINT", ""))
if _raw_iot_ep and not _raw_iot_ep.startswith("http"):
    _raw_iot_ep = f"https://{_raw_iot_ep}"
AWS_IOT_ENDPOINT = _raw_iot_ep
DYNAMODB_CATALOG_TABLE = os.environ.get("DYNAMODB_CATALOG_TABLE", "iotCatalog")
DYNAMODB_PAYLOAD_TABLE = os.environ.get("DYNAMODB_PAYLOAD_TABLE", "iotPayload")

# Timeouts in Sekunden
BOOT_TIMEOUT = int(os.environ.get("HIL_BOOT_TIMEOUT", "20"))
WIFI_CONNECT_TIMEOUT = int(os.environ.get("HIL_WIFI_TIMEOUT", "30"))
OTA_UPDATE_TIMEOUT = int(os.environ.get("HIL_OTA_TIMEOUT", "180"))
RENDER_TIMEOUT = int(os.environ.get("HIL_RENDER_TIMEOUT", "60"))
DEEP_SLEEP_TIMEOUT = int(os.environ.get("HIL_SLEEP_TIMEOUT", "20"))

# Standard-Pfade zu Firmware-Dateien & Produktions-Manifests
WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CACHE_DIR = os.path.join(WORKSPACE_ROOT, "testbench", "cache")

# S3 Bucket für OTA Firmware-Distribution & Sicherheits-Blacklist für Produktivdateien
def _detect_ota_bucket():
    bucket = os.environ.get("HIL_OTA_BUCKET") or os.environ.get("HIL_S3_BUCKET")
    if not bucket and os.environ.get("PRODUCTION_MANIFEST_URL_EPD7"):
        try:
            from urllib.parse import urlparse
            bucket = urlparse(os.environ.get("PRODUCTION_MANIFEST_URL_EPD7")).netloc
        except Exception:
            pass
    return bucket or "ul.epaperframe.de"

HIL_OTA_BUCKET = _detect_ota_bucket()

def _detect_prod_manifest(target):
    env_key = "PRODUCTION_MANIFEST_URL_EPD7" if target == "epd7" else "PRODUCTION_MANIFEST_URL_EPD13"
    legacy_key = "ENV_OTA_URL" if target == "epd7" else "ENV_OTA_URL_13"
    url = os.environ.get(env_key) or os.environ.get(legacy_key)
    if not url and HIL_OTA_BUCKET:
        url = f"http://{HIL_OTA_BUCKET}/espfota_{target}.json"
    return url or ""

PRODUCTION_MANIFEST_URL_EPD7 = _detect_prod_manifest("epd7")
PRODUCTION_MANIFEST_URL_EPD13 = _detect_prod_manifest("epd13")
CANDIDATE_MANIFEST_URL_EPD7 = os.environ.get("CANDIDATE_MANIFEST_URL_EPD7", os.environ.get("ENV_OTA_URL_DEV", ""))
CANDIDATE_MANIFEST_URL_EPD13 = os.environ.get("CANDIDATE_MANIFEST_URL_EPD13", os.environ.get("ENV_OTA_URL_DEV_13", ""))

DEFAULT_BASELINE_EPD7 = os.path.join(CACHE_DIR, "firmware_epd7_production.bin")
DEFAULT_BASELINE_EPD13 = os.path.join(CACHE_DIR, "firmware_epd13_production.bin")

DEFAULT_CANDIDATE_EPD7 = os.environ.get(
    "CANDIDATE_FIRMWARE_EPD7",
    os.path.join(WORKSPACE_ROOT, ".pio", "build", "epd7", "firmware.bin")
)
DEFAULT_CANDIDATE_EPD13 = os.environ.get(
    "CANDIDATE_FIRMWARE_EPD13",
    os.path.join(WORKSPACE_ROOT, ".pio", "build", "epd13", "firmware.bin")
)
PROTECTED_S3_KEYS = frozenset([
    "firmware.bin", "firmware_dev.bin", "firmware_v2.bin",
    "firmware_epd7.bin", "firmware_epd7_dev.bin", "firmware_epd7_pre.bin",
    "firmware_epd13.bin", "firmware_epd13_dev.bin", "firmware_epd13_pre.bin",
    "espfota.json", "espfota_v2.json",
    "espfota_epd7.json", "espfota_epd7_dev.json", "espfota_epd7_pre.json",
    "espfota_epd13.json", "espfota_epd13_dev.json", "espfota_epd13_pre.json"
])

