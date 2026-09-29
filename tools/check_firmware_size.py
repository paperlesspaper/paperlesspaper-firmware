#!/usr/bin/env python3
"""
check_firmware_size.py
Überprüft die generierte Firmware-Binärdatei gegen die Partitionsgrenzen in part.csv,
berechnet SHA-256 Checksummen und erzeugt Build-Metadaten.
"""

import sys
import os
import hashlib
import json
import argparse

# Windows CLI Encoding-Fix
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_PART_CSV = os.path.join(WORKSPACE_ROOT, "part.csv")

def get_app_partition_size(part_csv_path):
    if not os.path.isfile(part_csv_path):
        raise FileNotFoundError(f"Partitionstabelle nicht gefunden: {part_csv_path}")

    with open(part_csv_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 5 and parts[0] == "app0":
                size_str = parts[4]
                return int(size_str, 16) if size_str.startswith("0x") else int(size_str)

    # Fallback Standard ESP32 OTA Partition
    return 1900544  # 0x1D0000

def calculate_sha256(filepath):
    sha = hashlib.sha256()
    with open(filepath, "rb") as f:
        while chunk := f.read(65536):
            sha.update(chunk)
    return sha.hexdigest()

def check_firmware(bin_path, target_name, part_csv_path=DEFAULT_PART_CSV, warn_threshold=0.85):
    if not os.path.isfile(bin_path):
        print(f"❌ FEHLER: Firmware-Binärdatei existiert nicht: {bin_path}")
        return False, None

    max_size = get_app_partition_size(part_csv_path)
    file_size = os.path.getsize(bin_path)
    usage_pct = (file_size / max_size) * 100
    sha256 = calculate_sha256(bin_path)

    metadata = {
        "target": target_name,
        "binary_path": bin_path,
        "size_bytes": file_size,
        "max_bytes": max_size,
        "usage_percent": round(usage_pct, 2),
        "sha256": sha256,
        "status": "OK"
    }

    print("=" * 60)
    print(f"📊 FIRMWARE-GRÖSSENCHECK: {target_name.upper()}")
    print("=" * 60)
    print(f"📁 Datei:             {bin_path}")
    print(f"📏 Dateigröße:        {file_size:,} Bytes ({file_size / (1024*1024):.2f} MB)")
    print(f"🧱 Max. Partition:    {max_size:,} Bytes ({max_size / (1024*1024):.2f} MB)")
    print(f"📈 Speichernutzung:   {usage_pct:.1f} %")
    print(f"🔒 SHA-256 Checksum:  {sha256}")

    # GitHub Actions Step Summary & Warnings
    github_summary_path = os.environ.get("GITHUB_STEP_SUMMARY")
    if github_summary_path:
        try:
            with open(github_summary_path, "a", encoding="utf-8") as gf:
                gf.write(f"\n### Firmware Build `{target_name}`\n")
                gf.write(f"- **Größe:** {file_size:,} Bytes ({usage_pct:.1f} % von {max_size:,} Bytes)\n")
                gf.write(f"- **SHA-256:** `{sha256}`\n")
        except Exception as e:
            print(f"Hinweis: GitHub Step Summary konnte nicht geschrieben werden: {e}")

    if file_size > max_size:
        print(f"\n❌ KRITISCHER FEHLER: Firmware überschreitet die Partition um {file_size - max_size:,} Bytes!")
        print("::error::Firmware size exceeds OTA partition size!")
        metadata["status"] = "OVERFLOW"
        return False, metadata
    elif usage_pct >= (warn_threshold * 100):
        print(f"\n⚠️ ACHTUNG: Firmware belegt {usage_pct:.1f} % des Partitionsspeichers (Warnschwelle {warn_threshold*100:.0f}% erreicht)!")
        print("::warning::Firmware size is approaching partition limit!")
        metadata["status"] = "WARNING"
        return True, metadata
    else:
        print(f"\n✅ Firmware liegt sicher innerhalb der Partitionsgrenzen (Puffer: {max_size - file_size:,} Bytes).")
        return True, metadata

def main():
    parser = argparse.ArgumentParser(description="Überprüfung der Firmware-Binärgröße gegen part.csv")
    parser.add_argument("binary", help="Pfad zur firmware.bin")
    parser.add_argument("--target", default="firmware", help="Name des Targets (z.B. epd7 oder epd13)")
    parser.add_argument("--part", default=DEFAULT_PART_CSV, help="Pfad zur part.csv")
    parser.add_argument("--output-json", help="Optionaler Pfad zum Speichern der Metadaten als JSON")
    args = parser.parse_args()

    success, metadata = check_firmware(args.binary, args.target, args.part)

    if metadata and args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2)
        print(f"📄 Metadaten gespeichert in: {args.output_json}")

    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
