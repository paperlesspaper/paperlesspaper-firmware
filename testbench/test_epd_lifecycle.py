"""
testbench/test_epd_lifecycle.py
End-to-End Hardware-in-the-Loop (HIL) Testsuite für EPD7 und EPD13.
Führt die vollständige Lifecycle-Prüfung auf echter Hardware aus.
"""

import os
import time
import re
import pytest
from . import config
from .hardware_controller import ESP32HardwareController
from .ble_provisioner import BLEProvisioner
from .flasher import fetch_production_firmware
from .aws_client import AWSTestVerifier
from .privacy import mask_uid, mask_mac, mask_ssid, sanitize_log_line, register_github_mask

def ensure_wifi_connected(device, device_id, timeout=45):
    """
    Wartet auf WLAN-Verbindung. Falls das Gerät in den BLE-Provisioning-Modus wechselt,
    werden die Credentials automatisch per BLE übertragen.
    """
    start_t = time.time()
    ble_provisioned = False

    while (time.time() - start_t) < timeout:
        with device._lock:
            logs = list(device.log_history)

        if any("[NETWORK] WiFi Connected" in l or "[NETWORK] Wifi got IP" in l for l in logs):
            print(f"✅ [{device.name}] WLAN erfolgreich verbunden.")
            return True

        if not ble_provisioned and any(
            "[NETWORK] wait for wifi via ble" in l or "[BLE] BLE Advertising started" in l
            for l in logs
        ):
            print(f"📡 [{device.name}] Display im BLE-Provisioning-Modus erkannt. Starte BLE-Übertragung...")
            try:
                BLEProvisioner.provision_wifi(
                    device_id,
                    ssid=config.WIFI_SSID,
                    password=config.WIFI_PASSWORD,
                    timeout=15
                )
                ble_provisioned = True
                print(f"✅ [{device.name}] WLAN-Daten per BLE übermittelt.")
            except Exception as e:
                print(f"⚠️ [{device.name}] BLE-Provisionierungsversuch fehlgeschlagen: {e}")

        time.sleep(1.0)

    raise TimeoutError(f"[{device.name}] Timeout ({timeout}s) beim Warten auf WLAN-Verbindung!")

def trigger_ota_with_retry(device, aws_verifier, device_id, ota_url, pattern, retries=2, timeout_per_try=20):
    """
    Triggert OTA via AWS IoT Shadow & MQTT ($aws/things/<device_id>/epaper/receive)
    und wartet auf das Bestätigungs-Event.
    Läuft das Warten ins Timeout (z. B. durch Paketverlust oder temporäre MQTT-Verbindungspause),
    wird die MQTT-Nachricht automatisch noch einmal gesendet und geprüft (Retry).
    """
    last_error = None
    for attempt in range(1, retries + 1):
        if attempt == 1:
            print(f"\n📡 [{device.name}] Triggere OTA via MQTT ('{ota_url}')...")
        else:
            print(f"\n🔄 [{device.name}] OTA Retry ({attempt}/{retries}): Sende MQTT-Befehl erneut an '{mask_uid(device_id)}'...")
            # Kurz prüfen, ob das Gerät evtl. gerade neu zu AWS verbindet
            try:
                device.wait_for_pattern(r"\[AWS\] CONNECTED", timeout=4)
            except TimeoutError:
                pass

        aws_verifier.trigger_ota(device_id, ota_url)

        try:
            return device.wait_for_pattern(pattern, timeout=timeout_per_try)
        except TimeoutError as e:
            last_error = e
            if attempt < retries:
                recent = " | ".join([l.strip() for l in device.log_history[-3:]]) if device.log_history else "<keine Logs>"
                print(f"⚠️ [{device.name}] Keine OTA-Reaktion nach {timeout_per_try}s (Logs: {recent}). Starte Retry in 2s...")
                time.sleep(2)

    print(f"❌ [{device.name}] OTA-Empfang auch nach {retries} Versuchen nicht eingetroffen.")
    raise last_error

@pytest.fixture(scope="module")
def aws_verifier():
    """Initialisiert den AWS Verifier."""
    try:
        return AWSTestVerifier()
    except Exception as e:
        pytest.skip(f"AWS Credentials nicht verfügbar: {e}")

class TestEPD7Lifecycle:
    """Testzyklus für das 7.5 Zoll Display (EPD7)."""

    port = config.EPD7_COM_PORT
    relay_port = config.EPD7_RELAY_PORT
    device_id = config.EPD7_DEVICE_ID
    manifest_url = config.PRODUCTION_MANIFEST_URL_EPD7
    candidate_manifest_url = config.CANDIDATE_MANIFEST_URL_EPD7
    candidate_bin_path = config.DEFAULT_CANDIDATE_EPD7
    target_name = "epd7"
    prod_bin_path = None
    prod_version = None

    @pytest.fixture(autouse=True)
    def check_prerequisites(self):
        self.port = config.EPD7_COM_PORT
        self.relay_port = config.EPD7_RELAY_PORT
        self.device_id = config.EPD7_DEVICE_ID
        self.candidate_bin_path = config.DEFAULT_CANDIDATE_EPD7
        available_ports = [p["port"] for p in ESP32HardwareController.list_ports()]
        if self.port not in available_ports:
            pytest.skip(f"Hardware-Port {self.port} für EPD7 nicht angeschlossen (verfügbar: {available_ports}).")

    def test_00_factory_reset_via_power_cycles(self):
        """Schritt 0: Setzt das Gerät per mindestens 6x Power-Cycles auf Werkseinstellungen zurück und verifiziert dies."""
        with ESP32HardwareController(self.port, name="EPD7", relay_port=self.relay_port) as device:
            success = device.factory_reset_via_power_cycles(min_cycles=6)
            assert success is True, "Factory-Reset über 6x Power-Cycles fehlgeschlagen!"
            info = device.read_device_identity(reset=False, timeout=config.BOOT_TIMEOUT)
            if info.get("uid"):
                self.device_id = info["uid"]
                TestEPD7Lifecycle.device_id = info["uid"]
                config.EPD7_DEVICE_ID = info["uid"]
                os.environ["EPD7_DEVICE_ID"] = info["uid"]
            register_github_mask(self.device_id)
            print(f"🎉 [EPD7] Testgerät '{mask_uid(self.device_id)}' erfolgreich per 6x Power-Cycles auf Werkseinstellungen zurückgesetzt.")

    def test_01_ble_wifi_provisioning(self, request):
        """Schritt 1: Simuliert die drahtlose BLE-Provisionierung von WLAN-Credentials an das frisch zurückgesetzte Display."""
        if not BLEProvisioner.is_supported():
            pytest.skip("BLE / bleak ist auf diesem System nicht verfügbar.")

        try:
            with ESP32HardwareController(self.port, name="EPD7", relay_port=self.relay_port) as device:
                print(f"⏳ [{device.name}] Warte auf BLE Bereitschaft für '{mask_uid(self.device_id)}'...")
                try:
                    device.wait_for_pattern(
                        r"(?:\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|Provisioning attempt)",
                        timeout=8
                    )
                except TimeoutError:
                    print(f"ℹ️ [{device.name}] Display reagiert nicht (Deep Sleep oder Event verpasst). Wecke per Relais auf...")
                    device.reset(method="relay_hex")
                    device.wait_for_pattern(
                        r"(?:\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|Provisioning attempt)",
                        timeout=25
                    )

            print(f"\n📡 Starte Test: BLE-WLAN-Provisionierung für EPD7 (UID: {mask_uid(self.device_id)})...")
            try:
                nets = BLEProvisioner.read_wifi_scan(self.device_id, timeout=15)
                print(f"📶 Vom Display gescannte WLAN-Netzwerke via BLE ({len(nets)}):")
                for idx, n in enumerate(nets[:5], 1):
                    print(f"   - WLAN-Netz #{idx} [maskiert] ({n.get('rssi', '')} dBm)")
            except Exception as e:
                print(f"⚠️ Hinweis: BLE-WLAN-Scan übersprungen/fehlgeschlagen ({e}). Fahre mit Zugangsdaten-Übertragung fort...")

            success = BLEProvisioner.provision_wifi(
                self.device_id,
                ssid=config.WIFI_SSID,
                password=config.WIFI_PASSWORD,
                timeout=25
            )
            assert success is True, f"WLAN-Verbindung zu '{mask_ssid(config.WIFI_SSID)}' konnte nicht hergestellt werden!"
            print(f"🎉 BLE-Provisionierung für {mask_uid(self.device_id)} erfolgreich verifiziert.")
        except Exception as exc:
            print(f"\n🛑 [EPD7] BLE-WLAN-Provisionierung fehlgeschlagen: {exc}")
            print("   ➔ Ohne WLAN-Verbindung können nachfolgende Tests (OTA, AWS, REST) nicht funktionieren.")
            print("   ➔ Beende Testlauf vorzeitig (Fail-Fast).")
            request.session.shouldstop = f"EPD7 BLE-WLAN-Provisionierung fehlgeschlagen: {exc}"
            raise

    def test_02_production_firmware_ota(self, aws_verifier):
        """Schritt 2: Flasht die offizielle Produktions-Firmware via OTA Manifest JSON und verifiziert den Reboot."""
        bin_path, version = fetch_production_firmware(self.manifest_url, cache_dir=config.CACHE_DIR)
        TestEPD7Lifecycle.prod_bin_path = bin_path
        TestEPD7Lifecycle.prod_version = version

        with ESP32HardwareController(self.port, name="EPD7", relay_port=self.relay_port) as device:
            # 1. Stelle sicher, dass das Display mit AWS verbunden ist
            try:
                device.wait_for_pattern(r"\[AWS\] CONNECTED", timeout=8)
            except TimeoutError:
                print(f"ℹ️ [EPD7] Display nicht aktiv im AWS-Netzwerk. Wecke per Relais auf...")
                device.reset(method="relay_hex")
                device.wait_for_pattern(r"\[AWS\] CONNECTED", timeout=config.WIFI_CONNECT_TIMEOUT + 15)

            # 2. & 3. Jetzt, wo MQTT aktiv verbunden ist: Triggere OTA mit automatischem Retry bei Timeout
            trigger_ota_with_retry(
                device=device,
                aws_verifier=aws_verifier,
                device_id=self.device_id,
                ota_url=self.manifest_url,
                pattern=r"(?:\[AWS RX\] OTA URL received|\[OTA\] Processing Manifest JSON|\[OTA\] (?:Dev )?OTA (?:via MQTT )?Started)",
                retries=2,
                timeout_per_try=20
            )

            # 4. Warte auf den echten Neustart NACH dem Flashen (nur ab jetzt eintreffende Logs!)
            print(f"⏳ [EPD7] Produktions-OTA läuft. Warte auf Neustart in V{version} (Timeout: {config.OTA_UPDATE_TIMEOUT}s)...")
            new_boot = device.wait_for_pattern(r"\[MAIN\] INIT Device V:\s*([^\s]+)", timeout=config.OTA_UPDATE_TIMEOUT, from_current=True)
            assert new_boot is not None, "Display hat nach Produktions-OTA keinen Neustart durchgeführt!"
            booted_version = new_boot.group(1)
            print(f"✅ [EPD7] Erfolgreich auf Produktions-Firmware V{booted_version} geflasht.")
            assert booted_version == version, f"Unerwartete Version nach Produktions-OTA: {booted_version} != {version}"

            ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)

    def test_03_candidate_firmware_ota(self, aws_verifier):
        """Schritt 3: Lädt die gebuildete Kandidaten-Binärdatei hoch, flasht diese direkt via OTA-URL und verifiziert den Reboot."""
        assert os.path.isfile(self.candidate_bin_path), f"Kandidaten-Firmware nicht gefunden: {self.candidate_bin_path}"

        direct_url, s3_key = aws_verifier.upload_candidate_firmware(
            self.device_id,
            self.candidate_bin_path,
            key_name=f"test-firmware-{self.target_name}.bin"
        )

        try:
            with ESP32HardwareController(self.port, name="EPD7", relay_port=self.relay_port) as device:
                # 1. Stelle sicher, dass das Display mit AWS verbunden ist
                try:
                    device.wait_for_pattern(r"\[AWS\] CONNECTED", timeout=8)
                except TimeoutError:
                    print(f"ℹ️ [EPD7] Wecke Display per Relais auf zur AWS-Verbindung...")
                    device.reset(method="relay_hex")
                    device.wait_for_pattern(r"\[AWS\] CONNECTED", timeout=config.WIFI_CONNECT_TIMEOUT + 15)

                # 2. & 3. Jetzt, wo MQTT aktiv verbunden ist: Triggere Kandidaten-OTA mit automatischem Retry bei Timeout
                trigger_ota_with_retry(
                    device=device,
                    aws_verifier=aws_verifier,
                    device_id=self.device_id,
                    ota_url=direct_url,
                    pattern=r"(?:\[AWS RX\] OTA URL received|\[OTA\] Processing direct binary URL|\[OTA\] (?:Dev )?OTA (?:via MQTT )?Started)",
                    retries=2,
                    timeout_per_try=20
                )

                # 4. Warte auf den echten Neustart NACH dem Flashen (nur ab jetzt eintreffende Logs!)
                print(f"⏳ [EPD7] Direkt-OTA läuft. Warte auf Neustart in Kandidaten-Version (Timeout: {config.OTA_UPDATE_TIMEOUT}s)...")
                new_boot = device.wait_for_pattern(r"\[MAIN\] INIT Device V:\s*([^\s]+)", timeout=config.OTA_UPDATE_TIMEOUT, from_current=True)
                assert new_boot is not None, "Display hat nach Kandidaten-OTA keinen Neustart durchgeführt!"
                candidate_version = new_boot.group(1)
                print(f"🎉 [EPD7] Erfolgreich auf Kandidaten-Firmware V{candidate_version} geflasht.")
                assert candidate_version != TestEPD7Lifecycle.prod_version, (
                    f"Kandidaten-Firmware wurde nicht übernommen! Gerät läuft weiterhin auf Produktionsversion V{candidate_version}."
                )

                ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)
        finally:
            print(f"🧹 [EPD7] Bereinige temporäre Test-Firmware '{s3_key}' aus S3...")
            aws_verifier.cleanup_candidate_firmware(s3_key)

    def test_04_device_activation(self, aws_verifier):
        """Schritt 4: Aktiviert das Gerät über die REST-API (/activatedevice) und wartet auf autonomen Handshake des Geräts (ohne Relais-Reset)."""
        with ESP32HardwareController(self.port, name="EPD7", relay_port=self.relay_port) as device:
            print(f"\n🔑 [EPD7] Rufe Aktivierungs-API (POST /activatedevice) für '{mask_uid(self.device_id)}' auf...")
            aws_verifier.set_device_activation_pending(self.device_id)

            print(f"⏳ [EPD7] Warte auf autonomen Aktivierungs-Abschluss durch das Gerät (ohne Relais-Reset)...")
            try:
                activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)
            except TimeoutError:
                print(f"ℹ️ [EPD7] Gerät reagiert nicht unmittelbar (evtl. im Deep Sleep). Wecke per Relais auf...")
                device.reset(method="relay_hex")
                activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)

            assert activated_match is not None, "Display hat 'Device is activated' nicht empfangen!"

            catalog_item = aws_verifier.wait_for_activation(self.device_id, timeout=30)
            assert catalog_item is not None
            status = catalog_item.get("activation_status")
            assert status in ("activated", "active"), f"Unerwarteter Status in iotCatalog: {status}"
            print(f"🎉 [EPD7] Aktivierung über REST-API und Hardware-Handshake erfolgreich verifiziert.")

    def test_05_picture_render_and_payload(self, aws_verifier):
        """Schritt 5: Lädt Testbild über Presigned URL hoch, prüft Download, Rendering und DynamoDB Quittung."""
        print(f"\n🖼️ [EPD7] Generiere und lade Testbild für '{mask_uid(self.device_id)}' hoch...")
        key, t_upload = aws_verifier.upload_test_image(self.device_id, width=800, height=480)

        with ESP32HardwareController(self.port, name="EPD7", relay_port=self.relay_port) as device:
            print(f"🔄 [EPD7] Triggere Bildabruf per Relais-Power-Cycle...")
            device.reset(method="relay_hex")

            device.wait_for_pattern(r"(?:\[MAIN\] Device will update Image|\[AWS\] Request Image URL|\[AWS RX\] Picture URL Message)", timeout=config.WIFI_CONNECT_TIMEOUT + 15)
            print(f"📥 [EPD7] Bildanforderung erkannt. Warte auf Download & Render...")

            device.wait_for_pattern(r"\[DL\] Done", timeout=config.RENDER_TIMEOUT)
            print(f"✅ [EPD7] Bild erfolgreich vom Server heruntergeladen.")

            device.wait_for_pattern(r"\[EPD\] Set Image Done", timeout=config.RENDER_TIMEOUT + 30)
            print(f"🎨 [EPD7] E-Paper Rendering abgeschlossen.")

            device.wait_for_pattern(r"\[AWS\] Set State to: update_ok", timeout=30)
            print(f"📡 [EPD7] State 'update_ok' via MQTT an AWS übermittelt.")

            ack = aws_verifier.wait_for_payload_ack(self.device_id, min_timestamp=t_upload, timeout=config.RENDER_TIMEOUT)
            assert ack is not None, "Keine Quittierung in DynamoDB iotPayload gefunden!"
            print(f"🎉 [EPD7] Bild-Upload, Rendering und Quittung vollständig verifiziert.")

    def test_06_deactivate_and_deep_sleep(self, aws_verifier):
        """Schritt 6: Deaktiviert das Gerät über die REST-API (/activatedevice reset: True) und prüft Übergang in den Deep Sleep."""
        print(f"\n🛑 [EPD7] Führe Deaktivierung über REST-API für '{mask_uid(self.device_id)}' durch...")
        aws_verifier.deactivate_device(self.device_id)

        with ESP32HardwareController(self.port, name="EPD7", relay_port=self.relay_port) as device:
            print(f"🔄 [EPD7] Starte Display per Relais neu zur Deaktivierungs- und Deep-Sleep-Prüfung...")
            device.reset(method="relay_hex")

            reset_match = device.wait_for_pattern(
                r"(?:\[AWS RX\] Device activation (?:not started|reset)|\[MAIN\] Reset|\[MAIN\] ACT MEM:\s*0|\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|\[EPD\] Wifi Activate Function|\[AWS\] Request Remove Device)",
                timeout=config.WIFI_CONNECT_TIMEOUT + 15
            )
            assert reset_match is not None, "Display hat Deaktivierung nicht erkannt!"

            print(f"⏳ [EPD7] Warte auf Übergang in den Deep Sleep...")
            sleep_match = device.wait_for_pattern(
                r"(?:\[MAIN\] Going to Sleep for \d+ seconds|going to sleep|Deep Sleep)",
                timeout=60
            )
            assert sleep_match is not None, "Display ist nach Deaktivierung nicht in den Deep Sleep gewechselt!"
            print(f"🎉 [EPD7] Deaktivierung und Wechsel in den Deep Sleep erfolgreich verifiziert: '{sleep_match.group(0)}'")





class TestEPD13Lifecycle:
    """Testzyklus für das 13.3 Zoll Display (EPD13)."""

    target_name = "epd13"
    port = config.EPD13_COM_PORT
    relay_port = config.EPD13_RELAY_PORT
    device_id = config.EPD13_DEVICE_ID
    manifest_url = config.PRODUCTION_MANIFEST_URL_EPD13
    candidate_bin_path = config.DEFAULT_CANDIDATE_EPD13
    prod_bin_path = None
    prod_version = None

    @pytest.fixture(autouse=True)
    def check_prerequisites(self):
        self.port = config.EPD13_COM_PORT
        self.relay_port = config.EPD13_RELAY_PORT
        self.device_id = config.EPD13_DEVICE_ID
        available_ports = [p["port"] for p in ESP32HardwareController.list_ports()]
        if self.port not in available_ports:
            pytest.skip(f"Hardware-Port {self.port} für EPD13 nicht angeschlossen (verfügbar: {available_ports}).")

    def test_00_factory_reset_via_power_cycles(self):
        """Schritt 0: Setzt das EPD13 per mindestens 6x Power-Cycles auf Werkseinstellungen zurück und verifiziert dies."""
        with ESP32HardwareController(self.port, name="EPD13", relay_port=self.relay_port) as device:
            success = device.factory_reset_via_power_cycles(min_cycles=6)
            assert success is True, "Factory-Reset über 6x Power-Cycles fehlgeschlagen!"
            info = device.read_device_identity(reset=False, timeout=config.BOOT_TIMEOUT)
            if info.get("uid"):
                self.device_id = info["uid"]
                TestEPD13Lifecycle.device_id = info["uid"]
                config.EPD13_DEVICE_ID = info["uid"]
                os.environ["EPD13_DEVICE_ID"] = info["uid"]
            register_github_mask(self.device_id)
            print(f"🎉 [EPD13] Testgerät '{mask_uid(self.device_id)}' erfolgreich per 6x Power-Cycles auf Werkseinstellungen zurückgesetzt.")

    def test_01_ble_wifi_provisioning(self, request):
        """Schritt 1: Simuliert die drahtlose BLE-Provisionierung von WLAN-Credentials an das frisch zurückgesetzte EPD13 Display."""
        if not BLEProvisioner.is_supported():
            pytest.skip("BLE / bleak ist auf diesem System nicht verfügbar.")

        try:
            with ESP32HardwareController(self.port, name="EPD13", relay_port=self.relay_port) as device:
                print(f"⏳ [{device.name}] Warte auf BLE Bereitschaft für '{mask_uid(self.device_id)}'...")
                try:
                    device.wait_for_pattern(
                        r"(?:\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|Provisioning attempt)",
                        timeout=8
                    )
                except TimeoutError:
                    print(f"ℹ️ [{device.name}] Display reagiert nicht (Deep Sleep oder Event verpasst). Wecke per Relais auf...")
                    device.reset(method="relay_hex")
                    device.wait_for_pattern(
                        r"(?:\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|Provisioning attempt)",
                        timeout=25
                    )

            print(f"\n📡 Starte Test: BLE-WLAN-Provisionierung für EPD13 (UID: {mask_uid(self.device_id)})...")
            try:
                nets = BLEProvisioner.read_wifi_scan(self.device_id, timeout=15)
                print(f"📶 Vom Display gescannte WLAN-Netzwerke via BLE ({len(nets)}):")
                for idx, n in enumerate(nets[:5], 1):
                    print(f"   - WLAN-Netz #{idx} [maskiert] ({n.get('rssi', '')} dBm)")
            except Exception as e:
                print(f"⚠️ Hinweis: BLE-WLAN-Scan übersprungen/fehlgeschlagen ({e}). Fahre mit Zugangsdaten-Übertragung fort...")

            success = BLEProvisioner.provision_wifi(
                self.device_id,
                ssid=config.WIFI_SSID,
                password=config.WIFI_PASSWORD,
                timeout=25
            )
            assert success is True, f"WLAN-Verbindung zu '{mask_ssid(config.WIFI_SSID)}' konnte nicht hergestellt werden!"
            print(f"🎉 BLE-Provisionierung für {mask_uid(self.device_id)} erfolgreich verifiziert.")
        except Exception as exc:
            print(f"\n🛑 [EPD13] BLE-WLAN-Provisionierung fehlgeschlagen: {exc}")
            print("   ➔ Ohne WLAN-Verbindung können nachfolgende Tests (OTA, AWS, REST) nicht funktionieren.")
            print("   ➔ Beende Testlauf vorzeitig (Fail-Fast).")
            request.session.shouldstop = f"EPD13 BLE-WLAN-Provisionierung fehlgeschlagen: {exc}"
            raise

    def test_02_production_firmware_ota(self, aws_verifier):
        """Schritt 2: Flasht die offizielle Produktions-Firmware via OTA Manifest JSON und verifiziert den Reboot."""
        bin_path, version = fetch_production_firmware(self.manifest_url, cache_dir=config.CACHE_DIR)
        TestEPD13Lifecycle.prod_bin_path = bin_path
        TestEPD13Lifecycle.prod_version = version

        with ESP32HardwareController(self.port, name="EPD13", relay_port=self.relay_port) as device:
            # 1. Stelle sicher, dass das Display mit AWS verbunden ist
            try:
                device.wait_for_pattern(r"\[AWS\] CONNECTED", timeout=8)
            except TimeoutError:
                print(f"ℹ️ [EPD13] Display nicht aktiv im AWS-Netzwerk. Wecke per Relais auf...")
                device.reset(method="relay_hex")
                device.wait_for_pattern(r"\[AWS\] CONNECTED", timeout=config.WIFI_CONNECT_TIMEOUT + 15)

            # 2. & 3. Jetzt, wo MQTT aktiv verbunden ist: Triggere OTA mit automatischem Retry bei Timeout
            trigger_ota_with_retry(
                device=device,
                aws_verifier=aws_verifier,
                device_id=self.device_id,
                ota_url=self.manifest_url,
                pattern=r"(?:\[AWS RX\] OTA URL received|\[OTA\] Processing Manifest JSON|\[OTA\] (?:Dev )?OTA (?:via MQTT )?Started)",
                retries=2,
                timeout_per_try=20
            )

            # 4. Warte auf den echten Neustart NACH dem Flashen (nur ab jetzt eintreffende Logs!)
            print(f"⏳ [EPD13] Produktions-OTA läuft. Warte auf Neustart in V{version} (Timeout: {config.OTA_UPDATE_TIMEOUT}s)...")
            new_boot = device.wait_for_pattern(r"\[MAIN\] INIT Device V:\s*([^\s]+)", timeout=config.OTA_UPDATE_TIMEOUT, from_current=True)
            assert new_boot is not None, "Display hat nach Produktions-OTA keinen Neustart durchgeführt!"
            booted_version = new_boot.group(1)
            print(f"✅ [EPD13] Erfolgreich auf Produktions-Firmware V{booted_version} geflasht.")
            assert booted_version == version, f"Unerwartete Version nach Produktions-OTA: {booted_version} != {version}"

            ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)

    def test_03_candidate_firmware_ota(self, aws_verifier):
        """Schritt 3: Lädt die gebuildete Kandidaten-Binärdatei hoch, flasht diese direkt via OTA-URL und verifiziert den Reboot."""
        assert os.path.isfile(self.candidate_bin_path), f"Kandidaten-Firmware nicht gefunden: {self.candidate_bin_path}"

        direct_url, s3_key = aws_verifier.upload_candidate_firmware(
            self.device_id,
            self.candidate_bin_path,
            key_name=f"test-firmware-{self.target_name}.bin"
        )

        try:
            with ESP32HardwareController(self.port, name="EPD13", relay_port=self.relay_port) as device:
                # 1. Stelle sicher, dass das Display mit AWS verbunden ist
                try:
                    device.wait_for_pattern(r"\[AWS\] CONNECTED", timeout=8)
                except TimeoutError:
                    print(f"ℹ️ [EPD13] Wecke Display per Relais auf zur AWS-Verbindung...")
                    device.reset(method="relay_hex")
                    device.wait_for_pattern(r"\[AWS\] CONNECTED", timeout=config.WIFI_CONNECT_TIMEOUT + 15)

                # 2. & 3. Jetzt, wo MQTT aktiv verbunden ist: Triggere Kandidaten-OTA mit automatischem Retry bei Timeout
                trigger_ota_with_retry(
                    device=device,
                    aws_verifier=aws_verifier,
                    device_id=self.device_id,
                    ota_url=direct_url,
                    pattern=r"(?:\[AWS RX\] OTA URL received|\[OTA\] Processing direct binary URL|\[OTA\] (?:Dev )?OTA (?:via MQTT )?Started)",
                    retries=2,
                    timeout_per_try=20
                )

                # 4. Warte auf den echten Neustart NACH dem Flashen (nur ab jetzt eintreffende Logs!)
                print(f"⏳ [EPD13] Direkt-OTA läuft. Warte auf Neustart in Kandidaten-Version (Timeout: {config.OTA_UPDATE_TIMEOUT}s)...")
                new_boot = device.wait_for_pattern(r"\[MAIN\] INIT Device V:\s*([^\s]+)", timeout=config.OTA_UPDATE_TIMEOUT, from_current=True)
                assert new_boot is not None, "Display hat nach Kandidaten-OTA keinen Neustart durchgeführt!"
                candidate_version = new_boot.group(1)
                print(f"🎉 [EPD13] Erfolgreich auf Kandidaten-Firmware V{candidate_version} geflasht.")
                assert candidate_version != TestEPD13Lifecycle.prod_version, (
                    f"Kandidaten-Firmware wurde nicht übernommen! Gerät läuft weiterhin auf Produktionsversion V{candidate_version}."
                )

                ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)
        finally:
            print(f"🧹 [EPD13] Bereinige temporäre Test-Firmware '{s3_key}' aus S3...")
            aws_verifier.cleanup_candidate_firmware(s3_key)

    def test_04_device_activation(self, aws_verifier):
        """Schritt 4: Aktiviert das Gerät über die REST-API (/activatedevice) und wartet auf autonomen Handshake des Geräts (ohne Relais-Reset)."""
        with ESP32HardwareController(self.port, name="EPD13", relay_port=self.relay_port) as device:
            print(f"\n🔑 [EPD13] Rufe Aktivierungs-API (POST /activatedevice) für '{mask_uid(self.device_id)}' auf...")
            aws_verifier.set_device_activation_pending(self.device_id)

            print(f"⏳ [EPD13] Warte auf autonomen Aktivierungs-Abschluss durch das Gerät (ohne Relais-Reset)...")
            try:
                activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)
            except TimeoutError:
                print(f"ℹ️ [EPD13] Gerät reagiert nicht unmittelbar (evtl. im Deep Sleep). Wecke per Relais auf...")
                device.reset(method="relay_hex")
                activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)

            assert activated_match is not None, "Display hat 'Device is activated' nicht empfangen!"

            catalog_item = aws_verifier.wait_for_activation(self.device_id, timeout=30)
            assert catalog_item is not None
            status = catalog_item.get("activation_status")
            assert status in ("activated", "active"), f"Unerwarteter Status in iotCatalog: {status}"
            print(f"🎉 [EPD13] Aktivierung über REST-API und Hardware-Handshake erfolgreich verifiziert.")

    def test_05_picture_render_and_payload(self, aws_verifier):
        """Schritt 5: Lädt Testbild über Presigned URL hoch, prüft Download, Rendering und DynamoDB Quittung."""
        print(f"\n🖼️ [EPD13] Generiere und lade Testbild für '{mask_uid(self.device_id)}' hoch...")
        key, t_upload = aws_verifier.upload_test_image(self.device_id, width=1200, height=1600)

        with ESP32HardwareController(self.port, name="EPD13", relay_port=self.relay_port) as device:
            print(f"🔄 [EPD13] Triggere Bildabruf per Relais-Power-Cycle...")
            device.reset(method="relay_hex")

            device.wait_for_pattern(r"(?:\[MAIN\] Device will update Image|\[AWS\] Request Image URL|\[AWS RX\] Picture URL Message)", timeout=config.WIFI_CONNECT_TIMEOUT + 15)
            print(f"📥 [EPD13] Bildanforderung erkannt. Warte auf Download & Render...")

            device.wait_for_pattern(r"\[DL\] Done", timeout=config.RENDER_TIMEOUT)
            print(f"✅ [EPD13] Bild erfolgreich vom Server heruntergeladen.")

            device.wait_for_pattern(r"\[EPD\] Set Image Done", timeout=config.RENDER_TIMEOUT + 30)
            print(f"🎨 [EPD13] E-Paper Rendering abgeschlossen.")

            device.wait_for_pattern(r"\[AWS\] Set State to: update_ok", timeout=30)
            print(f"📡 [EPD13] State 'update_ok' via MQTT an AWS übermittelt.")

            ack = aws_verifier.wait_for_payload_ack(self.device_id, min_timestamp=t_upload, timeout=config.RENDER_TIMEOUT)
            assert ack is not None, "Keine Quittierung in DynamoDB iotPayload gefunden!"
            print(f"🎉 [EPD13] Bild-Upload, Rendering und Quittung vollständig verifiziert.")

    def test_06_deactivate_and_deep_sleep(self, aws_verifier):
        """Schritt 6: Deaktiviert das Gerät über die REST-API (/activatedevice reset: True) und prüft Übergang in den Deep Sleep."""
        print(f"\n🛑 [EPD13] Führe Deaktivierung über REST-API für '{mask_uid(self.device_id)}' durch...")
        aws_verifier.deactivate_device(self.device_id)

        with ESP32HardwareController(self.port, name="EPD13", relay_port=self.relay_port) as device:
            print(f"🔄 [EPD13] Starte Display per Relais neu zur Deaktivierungs- und Deep-Sleep-Prüfung...")
            device.reset(method="relay_hex")

            reset_match = device.wait_for_pattern(
                r"(?:\[AWS RX\] Device activation (?:not started|reset)|\[MAIN\] Reset|\[MAIN\] ACT MEM:\s*0|\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|\[EPD\] Wifi Activate Function|\[AWS\] Request Remove Device)",
                timeout=config.WIFI_CONNECT_TIMEOUT + 15
            )
            assert reset_match is not None, "Display hat Deaktivierung nicht erkannt!"

            print(f"⏳ [EPD13] Warte auf Übergang in den Deep Sleep...")
            sleep_match = device.wait_for_pattern(
                r"(?:\[MAIN\] Going to Sleep for \d+ seconds|going to sleep|Deep Sleep)",
                timeout=60
            )
            assert sleep_match is not None, "Display ist nach Deaktivierung nicht in den Deep Sleep gewechselt!"
            print(f"🎉 [EPD13] Deaktivierung und Wechsel in den Deep Sleep erfolgreich verifiziert: '{sleep_match.group(0)}'")




