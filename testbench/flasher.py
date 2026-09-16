"""
testbench/flasher.py
Automatisiertes Flashen von ESP32-C6 Firmware via esptool.py für HIL-Tests.
Ermöglicht das Aufspielen von Baseline-Versionen und das Zurücksetzen von Partitionen.
"""

import sys
import os
import subprocess
import time
import json
import urllib.request
import re

def read_device_mac(port, baudrate=115200, chip="esp32c6"):
    """
    Liest die MAC-Adresse des ESP32-Chips über esptool.py read_mac aus.
    Gibt ein Tuple zurück: (mac_with_colons, mac_compact_uppercase)
    Beispiel: ("AA:BB:CC:11:22:33", "AABBCC112233")
    """
    cmd = [
        sys.executable, "-m", "esptool",
        "--port", port,
        "--baud", str(baudrate),
        "--chip", chip,
        "read_mac"
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        if res.returncode != 0:
            raise RuntimeError(f"esptool read_mac fehlgeschlagen: {res.stderr}")

        match = re.search(r"MAC:\s*([0-9a-fA-F:]{17})", res.stdout)
        if match:
            mac_colons = match.group(1).upper()
            mac_compact = mac_colons.replace(":", "")
            return mac_colons, mac_compact
        raise ValueError(f"MAC-Adresse nicht in esptool-Ausgabe gefunden:\n{res.stdout}")
    except Exception as e:
        raise RuntimeError(f"Fehler beim Auslesen der MAC-Adresse auf {port}: {e}")

def fetch_production_firmware(manifest_url, cache_dir=None):
    """
    Lädt das Produktions-Manifest von der URL herunter (z. B. PRODUCTION_MANIFEST_URL_EPD7)
    und lädt die darin referenzierte Produktions-Firmware in den lokalen Cache herunter.
    Gibt (bin_filepath, version_string) zurück.
    """
    if cache_dir is None:
        cache_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "cache"))
    os.makedirs(cache_dir, exist_ok=True)

    print(f"📥 Lade Produktions-Manifest: {manifest_url}...")
    try:
        req = urllib.request.Request(manifest_url, headers={"User-Agent": "HIL-Testbench/1.0"})
        with urllib.request.urlopen(req, timeout=15) as resp:
            manifest = json.loads(resp.read().decode("utf-8"))
    except Exception as e:
        raise RuntimeError(f"Konnte Produktions-Manifest nicht abrufen ({manifest_url}): {e}")

    prod_version = manifest.get("version")
    bin_url = manifest.get("url")
    if not (prod_version and bin_url):
        raise ValueError(f"Ungültiges Manifest-Format in {manifest_url}: {manifest}")

    filename = os.path.basename(bin_url)
    target_path = os.path.join(cache_dir, f"{os.path.splitext(filename)[0]}_{prod_version}.bin")

    if os.path.isfile(target_path):
        print(f"ℹ️ Produktions-Firmware V{prod_version} bereits im Cache vorhanden: {target_path}")
        return target_path, prod_version

    print(f"⬇️ Lade aktuelle Produktions-Firmware V{prod_version} herunter: {bin_url}...")
    try:
        urllib.request.urlretrieve(bin_url, target_path)
        print(f"✅ Download abgeschlossen: {target_path} ({os.path.getsize(target_path):,} Bytes)")
        return target_path, prod_version
    except Exception as e:
        raise RuntimeError(f"Download der Produktions-Firmware fehlgeschlagen ({bin_url}): {e}")

def read_chip_info(port, baudrate=115200, chip="esp32c6"):
    """Liest Chip-Informationen (MAC-Adresse, Features) über esptool aus."""
    cmd = [
        sys.executable, "-m", "esptool",
        "--port", port,
        "--baud", str(baudrate),
        "--chip", chip,
        "chip_id"
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        if res.returncode != 0:
            raise RuntimeError(f"Chip-Info Abfrage fehlgeschlagen: {res.stderr}")
        return res.stdout
    except Exception as e:
        raise RuntimeError(f"Fehler bei Chip-Info Abfrage auf {port}: {e}")

def flash_firmware(port, binary_path, offset=0x10000, baudrate=921600, chip="esp32c6", erase_nvs=False):
    """
    Flasht eine Firmware-Binärdatei an den Offset 0x10000 (app0).
    Optional kann NVS gelöscht werden, um einen sauberen Werkszustand zu simulieren.
    """
    if not os.path.isfile(binary_path):
        raise FileNotFoundError(f"Firmware-Binärdatei nicht gefunden: {binary_path}")

    print("=" * 60)
    print(f"⚡ FLASHING START: {os.path.basename(binary_path)} -> {port} @ 0x{offset:X}")
    print("=" * 60)

    start_time = time.time()

    cmd = [
        sys.executable, "-m", "esptool",
        "--port", port,
        "--baud", str(baudrate),
        "--chip", chip,
        "--before", "default_reset",
        "--after", "hard_reset",
        "write_flash",
        "-z",
        f"0x{offset:X}", binary_path
    ]

    if erase_nvs:
        # NVS Partition liegt bei 0x9000 mit Größe 0x5000 (aus part.csv)
        cmd.extend(["--erase-all"])

    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        elapsed = round(time.time() - start_time, 2)

        if res.returncode != 0:
            print(f"❌ Flashen fehlgeschlagen nach {elapsed}s:")
            print(res.stderr)
            raise RuntimeError(f"esptool write_flash fehlgeschlagen: {res.stderr}")

        print(f"✅ Firmware erfolgreich in {elapsed}s geflasht.")
        return True

    except subprocess.TimeoutExpired:
        raise TimeoutError(f"Flash-Vorgang auf {port} nach 120s abgebrochen (Timeout)!")
    except Exception as e:
        raise RuntimeError(f"Unerwarteter Fehler beim Flashen: {e}")

def erase_partition(port, offset, size, baudrate=921600, chip="esp32c6"):
    """Löscht einen spezifischen Flash-Bereich (z.B. NVS oder SPIFFS)."""
    cmd = [
        sys.executable, "-m", "esptool",
        "--port", port,
        "--baud", str(baudrate),
        "--chip", chip,
        "erase_region", f"0x{offset:X}", f"0x{size:X}"
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    if res.returncode != 0:
        raise RuntimeError(f"erase_region fehlgeschlagen: {res.stderr}")
    print(f"✅ Flash-Region 0x{offset:X} (Größe: 0x{size:X}) gelöscht.")
    return True
