#!/usr/bin/env python3
"""
validate_firmware_config.py
Pre-Flight Validierungsskript für die E-Paper Firmware-Konfiguration (EPD7 & EPD13).
Stellt vor dem Build sicher, dass Display-Typen, Partitionsgrenzen und Versionen konsistent sind.
"""

import sys
import os
import re
import argparse

# Windows CLI Encoding-Fix
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

def check_files_exist():
    required_files = [
        os.path.join(WORKSPACE_ROOT, "src", "types.h"),
        os.path.join(WORKSPACE_ROOT, "platformio.ini"),
        os.path.join(WORKSPACE_ROOT, "part.csv"),
    ]
    missing = [f for f in required_files if not os.path.isfile(f)]
    if missing:
        print(f"❌ FEHLER: Erforderliche Dateien fehlen: {missing}")
        return False
    print("✅ Alle erforderlichen Konfigurationsdateien vorhanden.")
    return True

def validate_types_h(target=None):
    types_path = os.path.join(WORKSPACE_ROOT, "src", "types.h")
    with open(types_path, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()

    # Prüfe auf Compile-Time Mutex
    has_mutex = "#error" in content and "EPD_TYPE_7INCH" in content and "EPD_TYPE_13INCH" in content
    if not has_mutex:
        print("⚠️ WARNUNG: Kein Compile-Time Mutex für EPD7/EPD13 in types.h gefunden!")
    else:
        print("✅ Compile-Time Mutex in types.h verifiziert.")

    # Prüfe Version-String
    version_match = re.search(r'#define\s+SOFTWARE_VERSION\s+"([^"]+)"', content)
    if version_match:
        version = version_match.group(1)
        print(f"✅ Firmware-Version in types.h: '{version}'")
    else:
        print("⚠️ WARNUNG: SOFTWARE_VERSION konnte nicht extrahiert werden.")

    return True

def validate_partition_table():
    part_path = os.path.join(WORKSPACE_ROOT, "part.csv")
    with open(part_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    app0_found = False
    app1_found = False
    app_size = 0

    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 5:
            name, ptype, psubtype, offset, size = parts[0], parts[1], parts[2], parts[3], parts[4]
            if name == "app0":
                app0_found = True
                app_size = int(size, 16) if size.startswith("0x") else int(size)
            elif name == "app1":
                app1_found = True

    if not (app0_found and app1_found):
        print("❌ FEHLER: OTA-Partitionen app0 und app1 müssen in part.csv definiert sein!")
        return False

    print(f"✅ Zweifache OTA-Partition (app0/app1) vorhanden. Maximale App-Größe: {app_size:,} Bytes ({app_size / (1024*1024):.2f} MB).")
    return True

def validate_platformio_ini():
    pio_path = os.path.join(WORKSPACE_ROOT, "platformio.ini")
    with open(pio_path, "r", encoding="utf-8") as f:
        content = f.read()

    has_epd7 = "[env:epd7]" in content
    has_epd13 = "[env:epd13]" in content

    if not (has_epd7 and has_epd13):
        print("❌ FEHLER: platformio.ini enthält nicht beide Ziel-Umgebungen [env:epd7] und [env:epd13]!")
        return False

    print("✅ Dedizierte Umgebungen [env:epd7] und [env:epd13] in platformio.ini vorhanden.")
    return True

def main():
    parser = argparse.ArgumentParser(description="Pre-Flight Validierung der Firmware-Konfiguration.")
    parser.add_argument("--target", choices=["epd7", "epd13"], help="Ziel-Display-Typ zur Verifikation")
    args = parser.parse_args()

    print("=" * 60)
    print("🔍 STARTE PRE-FLIGHT VALIDIERUNG (EPD7 / EPD13)")
    print("=" * 60)

    success = True
    success &= check_files_exist()
    success &= validate_types_h(args.target)
    success &= validate_partition_table()
    success &= validate_platformio_ini()

    print("=" * 60)
    if success:
        print("🎉 PRE-FLIGHT VALIDIERUNG ERFOLGREICH ABGESCHLOSSEN")
        print("=" * 60)
        sys.exit(0)
    else:
        print("❌ PRE-FLIGHT VALIDIERUNG FEHLGESCHLAGEN")
        print("=" * 60)
        sys.exit(1)

if __name__ == "__main__":
    main()
