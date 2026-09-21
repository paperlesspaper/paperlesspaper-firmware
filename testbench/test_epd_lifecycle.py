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

def ensure_wifi_connected(device, device_id, timeout=60):
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
                    timeout=25
                )
                ble_provisioned = True
                print(f"✅ [{device.name}] WLAN-Daten per BLE übermittelt.")
            except Exception as e:
                print(f"⚠️ [{device.name}] BLE-Provisionierungsversuch fehlgeschlagen: {e}")

        time.sleep(1.0)

    raise TimeoutError(f"[{device.name}] Timeout ({timeout}s) beim Warten auf WLAN-Verbindung!")

def trigger_ota_with_retry(device, aws_verifier, device_id, ota_url, pattern, retries=2, timeout_per_try=45):
    """
    Triggert OTA: Schreibt die OTA-URL VORAB in den AWS IoT Shadow ('settings.otaUrl')
    und weckt anschließend das Display per Relais auf.
    Das Display holt sich beim Booten die URL autonom aus dem Shadow / Backend.
    """
    last_error = None
    for attempt in range(1, retries + 1):
        if attempt == 1:
            print(f"\n📡 [{device.name}] Setze OTA-URL vorab in Shadow: '{ota_url}'...")
        else:
            print(f"\n🔄 [{device.name}] OTA Retry ({attempt}/{retries}): Erneuere Shadow & starte erneut...")

        # 1. Vorab in AWS IoT Shadow schreiben (und redundante MQTT-Nachricht senden)
        aws_verifier.trigger_ota(device_id, ota_url)

        # 2. Display per Relais-Power-Cycle starten, damit es den Shadow beim Booten abfragt
        print(f"🔄 [{device.name}] Starte Display per Relais (Power-Cycle) zur OTA-Abfrage...")
        device.reset(method="relay_hex")

        # 3. Warte auf Erkennung des OTA-Starts (nur ab jetzt eintreffende Logs!)
        try:
            match = device.wait_for_pattern(pattern, timeout=timeout_per_try, from_current=True)
            # Nach erfolgreichem Empfang den Shadow bereinigen, um Schleifen zu verhindern
            aws_verifier.clear_ota_shadow(device_id)
            return match
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
        print(f"ℹ️ AWSTestVerifier nicht verfügbar: {e}")
        return None

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

    @pytest.fixture(scope="class")
    @classmethod
    def device(cls):
        """Hält den seriellen Port für den gesamten EPD7-Testzyklus dauerhaft offen."""
        port = config.EPD7_COM_PORT
        relay_port = config.EPD7_RELAY_PORT

        # Stromversorgungs-Isolation: EPD13 AUS, EPD7 AN
        print("⚡ [EPD7 Testzyklus] Schalte Relais für EPD13 AUS und für EPD7 AN...")
        if config.EPD13_RELAY_PORT:
            ESP32HardwareController.set_relay_power(config.EPD13_RELAY_PORT, power_on=False)
        if relay_port:
            ESP32HardwareController.set_relay_power(relay_port, power_on=True)

        # Bis zu 3s warten, falls Windows den CP210x Port nach dem Zuschalten enumeriert
        port_found = False
        available_ports = []
        for _ in range(6):
            available_ports = [p["port"] for p in ESP32HardwareController.list_ports()]
            if port in available_ports:
                port_found = True
                break
            time.sleep(0.5)

        if not port_found:
            pytest.skip(f"Hardware-Port {port} für EPD7 nicht angeschlossen (verfügbar: {available_ports}).")

        ctrl = ESP32HardwareController(port, name="EPD7", relay_port=relay_port)
        ctrl.connect()
        try:
            yield ctrl
        finally:
            ctrl.disconnect()
            if relay_port:
                ESP32HardwareController.set_relay_power(relay_port, power_on=False)

    @pytest.fixture(autouse=True)
    def check_prerequisites(self):
        self.port = config.EPD7_COM_PORT
        self.relay_port = config.EPD7_RELAY_PORT
        self.device_id = config.EPD7_DEVICE_ID
        self.candidate_bin_path = config.DEFAULT_CANDIDATE_EPD7

        port_found = False
        available_ports = []
        for _ in range(4):
            available_ports = [p["port"] for p in ESP32HardwareController.list_ports()]
            if self.port in available_ports:
                port_found = True
                break
            time.sleep(0.5)

        if not port_found:
            pytest.skip(f"Hardware-Port {self.port} für EPD7 nicht angeschlossen (verfügbar: {available_ports}).")

    def test_00_factory_reset_via_power_cycles(self, device, aws_verifier):
        """Schritt 0: Deaktiviert das Gerät vorab über die REST-API und setzt es per 6x Power-Cycles auf Werkseinstellungen zurück."""
        if aws_verifier:
            print(f"🛑 [{device.name}] Deaktiviere Gerät vorab via REST-API für '{mask_uid(self.device_id)}'...")
            try:
                aws_verifier.deactivate_device(self.device_id)
            except Exception as e:
                print(f"ℹ️ [{device.name}] Hinweis bei Vorab-Deaktivierung: {e}")

        success = device.factory_reset_via_power_cycles(min_cycles=6)
        assert success is True, "Factory-Reset über 6x Power-Cycles fehlgeschlagen!"
        info = device.read_device_identity(reset=False, timeout=config.BOOT_TIMEOUT)
        if info.get("uid"):
            self.device_id = info["uid"]
            TestEPD7Lifecycle.device_id = info["uid"]
            config.EPD7_DEVICE_ID = info["uid"]
            os.environ["EPD7_DEVICE_ID"] = info["uid"]
        register_github_mask(self.device_id)
        print(f"🎉 [{device.name}] Testgerät '{mask_uid(self.device_id)}' erfolgreich per 6x Power-Cycles auf Werkseinstellungen zurückgesetzt.")

    def test_01_ble_wifi_provisioning(self, device, request):
        """Schritt 1: Simuliert die drahtlose BLE-Provisionierung von WLAN-Credentials an das frisch zurückgesetzte Display."""
        if not BLEProvisioner.is_supported():
            pytest.skip("BLE / bleak ist auf diesem System nicht verfügbar.")

        try:
            print(f"⏳ [{device.name}] Warte auf BLE Bereitschaft für '{mask_uid(self.device_id)}'...")
            ble_pattern = r"(?:\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|Provisioning attempt|BLE reprovisioning)"
            ble_timeout = max(config.BLE_READY_TIMEOUT, 60)
            try:
                # E-Paper Refresh (20s) + QR-Code Rendering + WiFi-Scan (5s) benötigt 35-50s
                device.wait_for_pattern(ble_pattern, timeout=ble_timeout)
            except TimeoutError:
                print(f"ℹ️ [{device.name}] Display reagiert nicht (Deep Sleep oder Event verpasst). Wecke per Relais auf...")
                device.reset(method="relay_hex")
                device.wait_for_pattern(ble_pattern, timeout=ble_timeout)

            print(f"\n📡 Starte Test: BLE-WLAN-Provisionierung für {device.name} (UID: {mask_uid(self.device_id)})...")
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
            print(f"\n🛑 [{device.name}] BLE-WLAN-Provisionierung fehlgeschlagen: {exc}")
            print("   ➔ Ohne WLAN-Verbindung können nachfolgende Tests (OTA, AWS, REST) nicht funktionieren.")
            print("   ➔ Beende Testlauf vorzeitig (Fail-Fast).")
            request.session.shouldstop = f"{device.name} BLE-WLAN-Provisionierung fehlgeschlagen: {exc}"
            raise

    def test_02_device_activation(self, device, aws_verifier):
        """Schritt 2: Aktiviert das Gerät über die REST-API (/activatedevice), damit OTA-Updates und AWS-Dienste empfangen werden können."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)

        print(f"\n🔑 [{device.name}] Rufe Aktivierungs-API (POST /activatedevice) für '{mask_uid(self.device_id)}' auf...")
        aws_verifier.set_device_activation_pending(self.device_id)

        print(f"⏳ [{device.name}] Warte auf autonomen Aktivierungs-Abschluss durch das Gerät (ohne Relais-Reset)...")
        try:
            activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)
        except TimeoutError:
            print(f"ℹ️ [{device.name}] Gerät reagiert nicht unmittelbar (evtl. im Deep Sleep). Wecke per Relais auf...")
            device.reset(method="relay_hex")
            activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)

        assert activated_match is not None, "Display hat 'Device is activated' nicht empfangen!"

        catalog_item = aws_verifier.wait_for_activation(self.device_id, timeout=30)
        assert catalog_item is not None
        status = catalog_item.get("activation_status")
        assert status in ("activated", "active"), f"Unerwarteter Status in iotCatalog: {status}"
        print(f"🎉 [{device.name}] Aktivierung über REST-API und Hardware-Handshake erfolgreich verifiziert.")

    def test_03_production_firmware_ota(self, device, aws_verifier):
        """Schritt 3: Flasht die offizielle Produktions-Firmware via OTA Manifest JSON und verifiziert den Reboot."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        bin_path, version = fetch_production_firmware(self.manifest_url, cache_dir=config.CACHE_DIR)
        TestEPD7Lifecycle.prod_bin_path = bin_path
        TestEPD7Lifecycle.prod_version = version

        # 1. & 2. OTA-URL vorab in Shadow schreiben und Display per Relais neu starten
        trigger_ota_with_retry(
            device=device,
            aws_verifier=aws_verifier,
            device_id=self.device_id,
            ota_url=self.manifest_url,
            pattern=r"(?:\[AWS RX\] OTA URL received|\[OTA\] Processing Manifest JSON|\[OTA\] (?:Dev )?OTA (?:via MQTT )?Started)",
            retries=2,
            timeout_per_try=45
        )

        # 3. Warte auf den echten Neustart NACH dem Flashen (nur ab jetzt eintreffende Logs!)
        print(f"⏳ [EPD7] Produktions-OTA läuft. Warte auf Neustart in V{version} (Timeout: {config.OTA_UPDATE_TIMEOUT}s)...")
        new_boot = device.wait_for_pattern(r"\[MAIN\] INIT Device V:\s*([^\s]+)", timeout=config.OTA_UPDATE_TIMEOUT, from_current=True)
        assert new_boot is not None, "Display hat nach Produktions-OTA keinen Neustart durchgeführt!"
        booted_version = new_boot.group(1)
        print(f"✅ [EPD7] Erfolgreich auf Produktions-Firmware V{booted_version} geflasht.")
        assert booted_version == version, f"Unerwartete Version nach Produktions-OTA: {booted_version} != {version}"

        ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)

    def test_04_candidate_firmware_ota(self, device, aws_verifier):
        """Schritt 4: Lädt die gebuildete Kandidaten-Binärdatei hoch, flasht diese direkt via OTA-URL und verifiziert den Reboot."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        assert os.path.isfile(self.candidate_bin_path), f"Kandidaten-Firmware nicht gefunden: {self.candidate_bin_path}"

        direct_url, s3_key = aws_verifier.upload_candidate_firmware(
            self.device_id,
            self.candidate_bin_path,
            key_name=f"test-firmware-{self.target_name}.bin"
        )

        try:
            # 1. & 2. Direkt-OTA-URL vorab in Shadow schreiben und Display per Relais neu starten
            trigger_ota_with_retry(
                device=device,
                aws_verifier=aws_verifier,
                device_id=self.device_id,
                ota_url=direct_url,
                pattern=r"(?:\[AWS RX\] OTA URL received|\[OTA\] Processing direct binary URL|\[OTA\] (?:Dev )?OTA (?:via MQTT )?Started)",
                retries=2,
                timeout_per_try=45
            )

            # 3. Warte auf den echten Neustart NACH dem Flashen (nur ab jetzt eintreffende Logs!)
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

    def test_05_candidate_factory_reset(self, device, aws_verifier):
        """Schritt 5: Führt 6x Power-Cycles Factory-Reset auf der frisch geflashten Kandidaten-Firmware durch und deaktiviere via API."""
        print(f"\n🏭 [EPD7] Führe Factory-Reset auf der neuen Kandidaten-Firmware durch...")
        success = device.factory_reset_via_power_cycles(min_cycles=6)
        assert success is True, "Kandidaten-Factory-Reset über 6x Power-Cycles fehlgeschlagen!"

        if aws_verifier:
            print(f"🛑 [EPD7] Deaktiviere Gerät nach Kandidaten-Reset über REST-API für '{mask_uid(self.device_id)}'...")
            try:
                aws_verifier.deactivate_device(self.device_id)
            except Exception as e:
                print(f"ℹ️ [EPD7] Hinweis: API-Deaktivierung übersprungen/fehlgeschlagen: {e}")

        info = device.read_device_identity(reset=False, timeout=config.BOOT_TIMEOUT)
        if info.get("uid"):
            self.device_id = info["uid"]
            TestEPD7Lifecycle.device_id = info["uid"]
            config.EPD7_DEVICE_ID = info["uid"]
            os.environ["EPD7_DEVICE_ID"] = info["uid"]
        register_github_mask(self.device_id)
        print(f"🎉 [EPD7] Kandidaten-Firmware erfolgreich per 6x Power-Cycles zurückgesetzt und in Cloud deaktiviert.")

    def test_06_candidate_ble_wifi_provisioning(self, device, request):
        """Schritt 6: Prüft die BLE-WLAN-Provisionierung auf der frisch zurückgesetzten Kandidaten-Firmware."""
        if not BLEProvisioner.is_supported():
            pytest.skip("BLE / bleak ist auf diesem System nicht verfügbar.")

        try:
            print(f"⏳ [EPD7] Warte auf BLE Bereitschaft der Kandidaten-Firmware für '{mask_uid(self.device_id)}'...")
            ble_pattern = r"(?:\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|Provisioning attempt|BLE reprovisioning)"
            ble_timeout = max(config.BLE_READY_TIMEOUT, 60)
            try:
                device.wait_for_pattern(ble_pattern, timeout=ble_timeout)
            except TimeoutError:
                print(f"ℹ️ [EPD7] Display reagiert nicht (Deep Sleep oder Event verpasst). Wecke per Relais auf...")
                device.reset(method="relay_hex")
                device.wait_for_pattern(ble_pattern, timeout=ble_timeout)

            print(f"\n📡 Starte BLE-WLAN-Provisionierung für Kandidaten-Firmware ({mask_uid(self.device_id)})...")
            success = BLEProvisioner.provision_wifi(
                self.device_id,
                ssid=config.WIFI_SSID,
                password=config.WIFI_PASSWORD,
                timeout=25
            )
            assert success is True, f"Kandidaten-Firmware: WLAN-Verbindung zu '{mask_ssid(config.WIFI_SSID)}' konnte nicht hergestellt werden!"
            print(f"🎉 Kandidaten-Firmware: BLE-Provisionierung für {mask_uid(self.device_id)} erfolgreich verifiziert.")
        except Exception as exc:
            print(f"\n🛑 [EPD7] BLE-WLAN-Provisionierung der Kandidaten-Firmware fehlgeschlagen: {exc}")
            request.session.shouldstop = f"EPD7 Kandidaten BLE-WLAN-Provisionierung fehlgeschlagen: {exc}"
            raise

    def test_07_candidate_device_activation(self, device, aws_verifier):
        """Schritt 7: Aktiviert die Kandidaten-Firmware über die REST-API (/activatedevice) und validiert den autonomen Handshake."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)

        print(f"\n🔑 [EPD7] Rufe Aktivierungs-API (POST /activatedevice) für Kandidaten-Firmware '{mask_uid(self.device_id)}' auf...")
        aws_verifier.set_device_activation_pending(self.device_id)

        print(f"⏳ [EPD7] Warte auf autonomen Aktivierungs-Abschluss durch das Gerät (ohne Relais-Reset)...")
        try:
            activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)
        except TimeoutError:
            print(f"ℹ️ [EPD7] Gerät reagiert nicht unmittelbar (evtl. im Deep Sleep). Wecke per Relais auf...")
            device.reset(method="relay_hex")
            activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)

        assert activated_match is not None, "Display hat 'Device is activated' auf Kandidaten-Firmware nicht empfangen!"

        catalog_item = aws_verifier.wait_for_activation(self.device_id, timeout=30)
        assert catalog_item is not None
        status = catalog_item.get("activation_status")
        assert status in ("activated", "active"), f"Unerwarteter Status in iotCatalog: {status}"
        print(f"🎉 [EPD7] Aktivierung der Kandidaten-Firmware über REST-API und Hardware-Handshake erfolgreich verifiziert.")

    def test_08_candidate_picture_render_and_payload(self, device, aws_verifier):
        """Schritt 8: Lädt Testbild über Presigned URL hoch, prüft Download, Rendering und DynamoDB Quittung auf Kandidaten-Firmware."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        print(f"\n🖼️ [EPD7] Generiere und lade Testbild für '{mask_uid(self.device_id)}' hoch...")
        key, t_upload = aws_verifier.upload_test_image(self.device_id, width=800, height=480)

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

    def test_09_candidate_deactivate_and_deep_sleep(self, device, aws_verifier):
        """Schritt 9: Deaktiviert die Kandidaten-Firmware über die REST-API (/activatedevice reset: True) und prüft Übergang in Deep Sleep."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        print(f"\n🛑 [EPD7] Führe Deaktivierung über REST-API für '{mask_uid(self.device_id)}' durch...")
        aws_verifier.deactivate_device(self.device_id)

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
    candidate_manifest_url = config.CANDIDATE_MANIFEST_URL_EPD13
    candidate_bin_path = config.DEFAULT_CANDIDATE_EPD13
    prod_bin_path = None
    prod_version = None

    @pytest.fixture(scope="class")
    @classmethod
    def device(cls):
        """Hält den seriellen Port für den gesamten EPD13-Testzyklus dauerhaft offen."""
        port = config.EPD13_COM_PORT
        relay_port = config.EPD13_RELAY_PORT

        # Stromversorgungs-Isolation: EPD7 AUS, EPD13 AN
        print("⚡ [EPD13 Testzyklus] Schalte Relais für EPD7 AUS und für EPD13 AN...")
        if config.EPD7_RELAY_PORT:
            ESP32HardwareController.set_relay_power(config.EPD7_RELAY_PORT, power_on=False)
        if relay_port:
            ESP32HardwareController.set_relay_power(relay_port, power_on=True)

        # Bis zu 3s warten, falls Windows den CP210x Port nach dem Zuschalten enumeriert
        port_found = False
        available_ports = []
        for _ in range(6):
            available_ports = [p["port"] for p in ESP32HardwareController.list_ports()]
            if port in available_ports:
                port_found = True
                break
            time.sleep(0.5)

        if not port_found:
            pytest.skip(f"Hardware-Port {port} für EPD13 nicht angeschlossen (verfügbar: {available_ports}).")

        ctrl = ESP32HardwareController(port, name="EPD13", relay_port=relay_port)
        ctrl.connect()
        try:
            yield ctrl
        finally:
            ctrl.disconnect()
            if relay_port:
                ESP32HardwareController.set_relay_power(relay_port, power_on=False)

    @pytest.fixture(autouse=True)
    def check_prerequisites(self):
        self.port = config.EPD13_COM_PORT
        self.relay_port = config.EPD13_RELAY_PORT
        self.device_id = config.EPD13_DEVICE_ID
        self.candidate_bin_path = config.DEFAULT_CANDIDATE_EPD13

        port_found = False
        available_ports = []
        for _ in range(4):
            available_ports = [p["port"] for p in ESP32HardwareController.list_ports()]
            if self.port in available_ports:
                port_found = True
                break
            time.sleep(0.5)

        if not port_found:
            pytest.skip(f"Hardware-Port {self.port} für EPD13 nicht angeschlossen (verfügbar: {available_ports}).")

    def test_00_factory_reset_via_power_cycles(self, device, aws_verifier):
        """Schritt 0: Deaktiviert das EPD13 vorab über die REST-API und setzt es per 6x Power-Cycles auf Werkseinstellungen zurück."""
        if aws_verifier:
            print(f"🛑 [{device.name}] Deaktiviere Gerät vorab via REST-API für '{mask_uid(self.device_id)}'...")
            try:
                aws_verifier.deactivate_device(self.device_id)
            except Exception as e:
                print(f"ℹ️ [{device.name}] Hinweis bei Vorab-Deaktivierung: {e}")

        success = device.factory_reset_via_power_cycles(min_cycles=6)
        assert success is True, "Factory-Reset über 6x Power-Cycles fehlgeschlagen!"
        info = device.read_device_identity(reset=False, timeout=config.BOOT_TIMEOUT)
        if info.get("uid"):
            self.device_id = info["uid"]
            TestEPD13Lifecycle.device_id = info["uid"]
            config.EPD13_DEVICE_ID = info["uid"]
            os.environ["EPD13_DEVICE_ID"] = info["uid"]
        register_github_mask(self.device_id)
        print(f"🎉 [{device.name}] Testgerät '{mask_uid(self.device_id)}' erfolgreich per 6x Power-Cycles auf Werkseinstellungen zurückgesetzt.")

    def test_01_ble_wifi_provisioning(self, device, request):
        """Schritt 1: Simuliert die drahtlose BLE-Provisionierung von WLAN-Credentials an das frisch zurückgesetzte EPD13 Display."""
        if not BLEProvisioner.is_supported():
            pytest.skip("BLE / bleak ist auf diesem System nicht verfügbar.")

        try:
            print(f"⏳ [{device.name}] Warte auf BLE Bereitschaft für '{mask_uid(self.device_id)}'...")
            ble_pattern = r"(?:\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|Provisioning attempt|BLE reprovisioning)"
            ble_timeout = max(config.BLE_READY_TIMEOUT, 65)
            try:
                # E-Paper Refresh (20-30s) + QR-Code Rendering + WiFi-Scan (5s) benötigt 40-60s
                device.wait_for_pattern(ble_pattern, timeout=ble_timeout)
            except TimeoutError:
                print(f"ℹ️ [{device.name}] Display reagiert nicht (Deep Sleep oder Event verpasst). Wecke per Relais auf...")
                device.reset(method="relay_hex")
                device.wait_for_pattern(ble_pattern, timeout=ble_timeout)

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

    def test_02_device_activation(self, device, aws_verifier):
        """Schritt 2: Aktiviert das EPD13 über die REST-API (/activatedevice), damit OTA-Updates und AWS-Dienste empfangen werden können."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)

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

    def test_03_production_firmware_ota(self, device, aws_verifier):
        """Schritt 3: Flasht die offizielle Produktions-Firmware via OTA Manifest JSON und verifiziert den Reboot."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        bin_path, version = fetch_production_firmware(self.manifest_url, cache_dir=config.CACHE_DIR)
        TestEPD13Lifecycle.prod_bin_path = bin_path
        TestEPD13Lifecycle.prod_version = version

        # 1. & 2. OTA-URL vorab in Shadow schreiben und Display per Relais neu starten
        trigger_ota_with_retry(
            device=device,
            aws_verifier=aws_verifier,
            device_id=self.device_id,
            ota_url=self.manifest_url,
            pattern=r"(?:\[AWS RX\] OTA URL received|\[OTA\] Processing Manifest JSON|\[OTA\] (?:Dev )?OTA (?:via MQTT )?Started)",
            retries=2,
            timeout_per_try=45
        )

        # 3. Warte auf den echten Neustart NACH dem Flashen (nur ab jetzt eintreffende Logs!)
        print(f"⏳ [EPD13] Produktions-OTA läuft. Warte auf Neustart in V{version} (Timeout: {config.OTA_UPDATE_TIMEOUT}s)...")
        new_boot = device.wait_for_pattern(r"\[MAIN\] INIT Device V:\s*([^\s]+)", timeout=config.OTA_UPDATE_TIMEOUT, from_current=True)
        assert new_boot is not None, "Display hat nach Produktions-OTA keinen Neustart durchgeführt!"
        booted_version = new_boot.group(1)
        print(f"✅ [EPD13] Erfolgreich auf Produktions-Firmware V{booted_version} geflasht.")
        assert booted_version == version, f"Unerwartete Version nach Produktions-OTA: {booted_version} != {version}"

        ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)

    def test_04_candidate_firmware_ota(self, device, aws_verifier):
        """Schritt 4: Lädt die gebuildete Kandidaten-Binärdatei hoch, flasht diese direkt via OTA-URL und verifiziert den Reboot."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        assert os.path.isfile(self.candidate_bin_path), f"Kandidaten-Firmware nicht gefunden: {self.candidate_bin_path}"

        direct_url, s3_key = aws_verifier.upload_candidate_firmware(
            self.device_id,
            self.candidate_bin_path,
            key_name=f"test-firmware-{self.target_name}.bin"
        )

        try:
            # 1. & 2. Direkt-OTA-URL vorab in Shadow schreiben und Display per Relais neu starten
            trigger_ota_with_retry(
                device=device,
                aws_verifier=aws_verifier,
                device_id=self.device_id,
                ota_url=direct_url,
                pattern=r"(?:\[AWS RX\] OTA URL received|\[OTA\] Processing direct binary URL|\[OTA\] (?:Dev )?OTA (?:via MQTT )?Started)",
                retries=2,
                timeout_per_try=45
            )

            # 3. Warte auf den echten Neustart NACH dem Flashen (nur ab jetzt eintreffende Logs!)
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

    def test_05_candidate_factory_reset(self, device, aws_verifier):
        """Schritt 5: Führt 6x Power-Cycles Factory-Reset auf der frisch geflashten Kandidaten-Firmware durch und deaktiviere via API."""
        print(f"\n🏭 [EPD13] Führe Factory-Reset auf der neuen Kandidaten-Firmware durch...")
        success = device.factory_reset_via_power_cycles(min_cycles=6)
        assert success is True, "Kandidaten-Factory-Reset über 6x Power-Cycles fehlgeschlagen!"

        if aws_verifier:
            print(f"🛑 [EPD13] Deaktiviere Gerät nach Kandidaten-Reset über REST-API für '{mask_uid(self.device_id)}'...")
            try:
                aws_verifier.deactivate_device(self.device_id)
            except Exception as e:
                print(f"ℹ️ [EPD13] Hinweis: API-Deaktivierung übersprungen/fehlgeschlagen: {e}")

        info = device.read_device_identity(reset=False, timeout=config.BOOT_TIMEOUT)
        if info.get("uid"):
            self.device_id = info["uid"]
            TestEPD13Lifecycle.device_id = info["uid"]
            config.EPD13_DEVICE_ID = info["uid"]
            os.environ["EPD13_DEVICE_ID"] = info["uid"]
        register_github_mask(self.device_id)
        print(f"🎉 [EPD13] Kandidaten-Firmware erfolgreich per 6x Power-Cycles zurückgesetzt und in Cloud deaktiviert.")

    def test_06_candidate_ble_wifi_provisioning(self, device, request):
        """Schritt 6: Prüft die BLE-WLAN-Provisionierung auf der frisch zurückgesetzten Kandidaten-Firmware."""
        if not BLEProvisioner.is_supported():
            pytest.skip("BLE / bleak ist auf diesem System nicht verfügbar.")

        try:
            print(f"⏳ [EPD13] Warte auf BLE Bereitschaft der Kandidaten-Firmware für '{mask_uid(self.device_id)}'...")
            ble_pattern = r"(?:\[BLE\] BLE Advertising started|\[NETWORK\] wait for wifi via ble|Provisioning attempt|BLE reprovisioning)"
            ble_timeout = max(config.BLE_READY_TIMEOUT, 65)
            try:
                device.wait_for_pattern(ble_pattern, timeout=ble_timeout)
            except TimeoutError:
                print(f"ℹ️ [EPD13] Display reagiert nicht (Deep Sleep oder Event verpasst). Wecke per Relais auf...")
                device.reset(method="relay_hex")
                device.wait_for_pattern(ble_pattern, timeout=ble_timeout)

            print(f"\n📡 Starte BLE-WLAN-Provisionierung für Kandidaten-Firmware ({mask_uid(self.device_id)})...")
            success = BLEProvisioner.provision_wifi(
                self.device_id,
                ssid=config.WIFI_SSID,
                password=config.WIFI_PASSWORD,
                timeout=25
            )
            assert success is True, f"Kandidaten-Firmware: WLAN-Verbindung zu '{mask_ssid(config.WIFI_SSID)}' konnte nicht hergestellt werden!"
            print(f"🎉 Kandidaten-Firmware: BLE-Provisionierung für {mask_uid(self.device_id)} erfolgreich verifiziert.")
        except Exception as exc:
            print(f"\n🛑 [EPD13] BLE-WLAN-Provisionierung der Kandidaten-Firmware fehlgeschlagen: {exc}")
            request.session.shouldstop = f"EPD13 Kandidaten BLE-WLAN-Provisionierung fehlgeschlagen: {exc}"
            raise

    def test_07_candidate_device_activation(self, device, aws_verifier):
        """Schritt 7: Aktiviert die Kandidaten-Firmware über die REST-API (/activatedevice) und validiert den autonomen Handshake."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        ensure_wifi_connected(device, self.device_id, timeout=config.WIFI_CONNECT_TIMEOUT)

        print(f"\n🔑 [EPD13] Rufe Aktivierungs-API (POST /activatedevice) für Kandidaten-Firmware '{mask_uid(self.device_id)}' auf...")
        aws_verifier.set_device_activation_pending(self.device_id)

        print(f"⏳ [EPD13] Warte auf autonomen Aktivierungs-Abschluss durch das Gerät (ohne Relais-Reset)...")
        try:
            activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)
        except TimeoutError:
            print(f"ℹ️ [EPD13] Gerät reagiert nicht unmittelbar (evtl. im Deep Sleep). Wecke per Relais auf...")
            device.reset(method="relay_hex")
            activated_match = device.wait_for_pattern(r"\[AWS RX\] Device is activated", timeout=config.WIFI_CONNECT_TIMEOUT + 15)

        assert activated_match is not None, "Display hat 'Device is activated' auf Kandidaten-Firmware nicht empfangen!"

        catalog_item = aws_verifier.wait_for_activation(self.device_id, timeout=30)
        assert catalog_item is not None
        status = catalog_item.get("activation_status")
        assert status in ("activated", "active"), f"Unerwarteter Status in iotCatalog: {status}"
        print(f"🎉 [EPD13] Aktivierung der Kandidaten-Firmware über REST-API und Hardware-Handshake erfolgreich verifiziert.")

    def test_08_candidate_picture_render_and_payload(self, device, aws_verifier):
        """Schritt 8: Lädt Testbild über Presigned URL hoch, prüft Download, Rendering und DynamoDB Quittung auf Kandidaten-Firmware."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        print(f"\n🖼️ [EPD13] Generiere und lade Testbild für '{mask_uid(self.device_id)}' hoch...")
        key, t_upload = aws_verifier.upload_test_image(self.device_id, width=1200, height=1600)

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

    def test_09_candidate_deactivate_and_deep_sleep(self, device, aws_verifier):
        """Schritt 9: Deaktiviert die Kandidaten-Firmware über die REST-API (/activatedevice reset: True) und prüft Übergang in Deep Sleep."""
        if aws_verifier is None:
            pytest.skip("AWS Credentials nicht verfügbar.")

        print(f"\n🛑 [EPD13] Führe Deaktivierung über REST-API für '{mask_uid(self.device_id)}' durch...")
        aws_verifier.deactivate_device(self.device_id)

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
