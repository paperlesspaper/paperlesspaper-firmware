"""
testbench/ble_provisioner.py
Automatisierter BLE-Provisionierungs-Client für ESP32 E-Paper Displays.
Simuliert das Übertragen von WLAN-Zugangsdaten (SSID & Passwort) über Bluetooth Low Energy (GATT).
Basiert auf den in src/main.cpp definierten BLE-Services und Characteristics.
"""

import sys
import os
import time
import asyncio
import argparse

# Windows CLI Encoding-Absicherung
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

try:
    from bleak import BleakClient, BleakScanner
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False

from . import config

class BLEProvisioningError(RuntimeError):
    """Wird ausgelöst, wenn die BLE-Übertragung fehlschlägt."""
    pass

class BLEProvisioner:
    """Verwaltet BLE-Scans und das Übertragen von WiFi-Credentials an ESP32-Displays."""

    @staticmethod
    def is_supported():
        return BLEAK_AVAILABLE

    @classmethod
    async def async_scan_devices(cls, timeout=6.0, name_prefix=None):
        """Scant nach BLE-Geräten und filtert optional nach Namens-Präfix (z.B. 'epd7' oder 'epd13')."""
        if not BLEAK_AVAILABLE:
            raise RuntimeError("Das Modul 'bleak' ist nicht installiert. Bitte 'pip install bleak' ausführen.")

        devices = await BleakScanner.discover(timeout=timeout)
        matched = []
        for d in devices:
            d_name = d.name or ""
            if name_prefix:
                if name_prefix.lower() in d_name.lower():
                    matched.append(d)
            elif "epd" in d_name.lower():
                matched.append(d)
            elif d_name:
                matched.append(d)
        return matched

    @classmethod
    def scan_devices(cls, timeout=6.0, name_prefix=None):
        """Synchrone Variante von async_scan_devices."""
        return asyncio.run(cls.async_scan_devices(timeout=timeout, name_prefix=name_prefix))

    @classmethod
    async def async_find_device(cls, device_name_or_uid, timeout=10.0):
        """Sucht ein bestimmtes Gerät anhand von UID / Advertised Name."""
        if not BLEAK_AVAILABLE:
            raise RuntimeError("'bleak' ist nicht installiert.")

        target = device_name_or_uid.lower().strip()
        print(f"🔍 [BLE] Scanne nach Bluetooth-Gerät '{device_name_or_uid}' (Timeout: {timeout}s)...")
        start_t = time.time()
        
        while (time.time() - start_t) < timeout:
            devices = await BleakScanner.discover(timeout=2.5)
            for d in devices:
                d_name = (d.name or "").lower().strip()
                d_addr = d.address.lower().strip()
                if target in d_name or target in d_addr:
                    print(f"🎯 [BLE] Gerät gefunden: {d.name} [{d.address}]")
                    return d
            await asyncio.sleep(0.5)

        return None

    @classmethod
    async def async_provision_wifi(cls, device_name_or_uid, ssid=None, password=None, timeout=15.0):
        """
        Verbindet sich via BLE mit dem Display und überträgt SSID und Passwort.
        Schreibt auf die beiden GATT Characteristics:
          - SSID: config.BLE_CHAR_WIFI_SSID
          - PASS: config.BLE_CHAR_WIFI_PASSWORD
        """
        if not BLEAK_AVAILABLE:
            raise RuntimeError("Das Modul 'bleak' ist nicht installiert.")

        ssid = ssid or config.WIFI_SSID
        password = password or config.WIFI_PASSWORD

        if not ssid:
            raise ValueError("Keine WLAN-SSID für die BLE-Provisionierung angegeben.")

        device = await cls.async_find_device(device_name_or_uid, timeout=timeout)
        if not device:
            raise BLEProvisioningError(
                f"❌ BLE-Gerät '{device_name_or_uid}' nicht gefunden! "
                "Ist das Display im BLE-Advertising-Modus ([BLE] BLE Advertising started)?"
            )

        print(f"🔗 [BLE] Verbinde mit {device.name} ({device.address})...")
        async with BleakClient(device, timeout=25.0) as client:
            if not client.is_connected:
                raise BLEProvisioningError(f"Konnte keine BLE-Verbindung zu {device.name} aufbauen.")

            print(f"✅ [BLE] Verbunden. Übertrage WLAN-Zugangsdaten...")
            print(f"   ➔ SSID: {ssid}")
            await client.write_gatt_char(config.BLE_CHAR_WIFI_SSID, ssid.encode("utf-8"), response=True)
            await asyncio.sleep(0.3)

            print(f"   ➔ Passwort: {'*' * len(password)}")
            await client.write_gatt_char(config.BLE_CHAR_WIFI_PASSWORD, password.encode("utf-8"), response=True)
            await asyncio.sleep(0.5)

            print(f"🎉 [BLE] WLAN-Credentials an {device.name} übermittelt.")
            print(f"⏳ [BLE] Warte auf Verbindungsüberprüfung durch das Display (max. 10s)...")

            # Das Display versucht nun bis zu 10s die Verbindung herzustellen.
            # Bei Fehlschlag setzt es wifiConnectedCharacteristic auf 0 (false).
            # Bei Erfolg beendet es BLE oder bleibt auf 1 (true).
            connection_confirmed = False
            for sec in range(1, 12):
                await asyncio.sleep(1.0)
                try:
                    status_raw = await client.read_gatt_char(config.BLE_CHAR_WIFI_CONNECTED)
                    if status_raw and status_raw[0] == 1:
                        connection_confirmed = True
                        break
                except Exception:
                    # Wenn das Display nach erfolgreicher Verbindung BLE trennt/beendet
                    connection_confirmed = True
                    break

            if not connection_confirmed:
                raise BLEProvisioningError(
                    f"❌ WLAN-Verbindung zu '{ssid}' fehlgeschlagen! "
                    "Das Display hat die Verbindung innerhalb von 12s nicht bestätigt."
                )

            print(f"✅ [BLE] WLAN-Verbindung zu '{ssid}' erfolgreich vom Display bestätigt!")
            return True

    @classmethod
    def provision_wifi(cls, device_name_or_uid, ssid=None, password=None, timeout=15.0):
        """Synchrone Variante von async_provision_wifi."""
        return asyncio.run(cls.async_provision_wifi(device_name_or_uid, ssid=ssid, password=password, timeout=timeout))

    @classmethod
    async def async_read_wifi_scan(cls, device_name_or_uid, timeout=10.0):
        """Liest die vom ESP32 gescannten WLAN-Netzwerke über BLE aus."""
        device = await cls.async_find_device(device_name_or_uid, timeout=timeout)
        if not device:
            raise BLEProvisioningError(f"Gerät '{device_name_or_uid}' nicht gefunden.")

        async with BleakClient(device, timeout=25.0) as client:
            raw = await client.read_gatt_char(config.BLE_CHAR_WIFI_SCAN)
            text = raw.decode("utf-8", errors="replace")
            # Format im Code: SSID´RSSI´´SSID´RSSI´´...
            entries = text.split("´´")
            networks = []
            for e in entries:
                if "´" in e:
                    parts = e.split("´")
                    s = parts[0].strip()
                    r = parts[1].strip() if len(parts) > 1 else ""
                    if s:
                        networks.append({"ssid": s, "rssi": r})
            return networks

    @classmethod
    def read_wifi_scan(cls, device_name_or_uid, timeout=10.0):
        """Synchrone Variante von async_read_wifi_scan."""
        return asyncio.run(cls.async_read_wifi_scan(device_name_or_uid, timeout=timeout))


def main():
    parser = argparse.ArgumentParser(description="BLE WiFi Provisioner für ESP32 E-Paper Displays")
    parser.add_argument("--scan", action="store_true", help="Scant nach erreichbaren E-Paper Displays via BLE")
    parser.add_argument("--target", default="epd7", help="Zielgerät (UID, Display-Name oder 'epd7'/'epd13')")
    parser.add_argument("--ssid", default=None, help="WLAN-SSID")
    parser.add_argument("--password", default=None, help="WLAN-Passwort")
    parser.add_argument("--read-scan", action="store_true", help="Liest die vom Gerät gescannten WLANs aus")
    parser.add_argument("--timeout", type=float, default=15.0, help="Timeout in Sekunden")
    args = parser.parse_args()

    if not BLEProvisioner.is_supported():
        print("❌ 'bleak' ist nicht installiert. Bitte 'pip install bleak' ausführen.")
        sys.exit(1)

    if args.scan:
        print("🔍 Scanne nach BLE-Geräten in der Umgebung...")
        devices = BLEProvisioner.scan_devices(timeout=args.timeout)
        print(f"\nGefundene BLE-Geräte ({len(devices)}):")
        for d in devices:
            print(f"  - {d.name or '<Unbekannt>'} ({d.address})")
        sys.exit(0)

    # Ermittle den Target-Namen (z.B. aus config oder übergeben)
    target_name = args.target
    if target_name.lower() == "epd7":
        target_name = config.EPD7_DEVICE_ID or "epd7"
    elif target_name.lower() == "epd13":
        target_name = config.EPD13_DEVICE_ID or "epd13"

    if args.read_scan:
        print(f"📡 Lese WLAN-Scan vom Gerät '{target_name}' via BLE...")
        try:
            nets = BLEProvisioner.read_wifi_scan(target_name, timeout=args.timeout)
            print(f"Erkannte Netzwerke am Display ({len(nets)}):")
            for n in nets:
                print(f"  📶 {n['ssid']} (Signal: {n['rssi']} dBm)")
            sys.exit(0)
        except Exception as e:
            print(f"❌ Fehler: {e}")
            sys.exit(1)

    # Provisionierung durchführen
    ssid = args.ssid or config.WIFI_SSID
    pw = args.password or config.WIFI_PASSWORD
    print(f"🚀 Starte BLE-WLAN-Provisionierung für '{target_name}'...")
    print(f"   SSID: {ssid}")

    try:
        success = BLEProvisioner.provision_wifi(target_name, ssid=ssid, password=pw, timeout=args.timeout)
        if success:
            print("\n🎉 BLE-Provisionierung erfolgreich abgeschlossen.")
            sys.exit(0)
    except Exception as e:
        print(f"\n❌ Fehler bei der BLE-Provisionierung: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
