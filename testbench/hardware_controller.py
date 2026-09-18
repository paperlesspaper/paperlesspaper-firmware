"""
testbench/hardware_controller.py
Hardware-Controller für ESP32-C6 E-Paper Testgeräte.
Verwaltet serielle Kommunikation, asynchrones Log-Matching und Hardware-Resets (DTR/RTS & Relais).
"""

import time
import threading
import re
import subprocess
import sys
import os
import json

# Windows CLI Encoding-Absicherung
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

from .privacy import mask_uid, mask_mac, mask_path, sanitize_log_line, register_github_mask

class HardwareSetupError(RuntimeError):
    """Wird ausgelöst, wenn Displays oder Relais bei der automatischen Hardware-Prüfung fehlen oder fehlerhaft sind."""
    pass

class ESP32HardwareController:
    def __init__(self, port, baudrate=115200, name="ESP32-Device", relay_port=None):
        self.port = port
        self.baudrate = baudrate
        self.name = name
        self.relay_port = relay_port
        self.ser = None
        self._running = False
        self._thread = None
        self.log_history = []
        self._new_line_event = threading.Event()
        self._lock = threading.Lock()

    def connect(self):
        """Öffnet die serielle Schnittstelle und startet den Hintergrund-Reader."""
        if serial is None:
            raise RuntimeError("Das Modul 'pyserial' ist nicht installiert. Bitte 'pip install pyserial' ausführen.")

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self._running = True
            self._thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._thread.start()
            print(f"🔌 [{self.name}] Verbunden mit {self.port} @ {self.baudrate} Baud.")
        except Exception as e:
            raise ConnectionError(f"[{self.name}] Konnte nicht mit {self.port} verbinden: {e}")

    def disconnect(self):
        """Schließt den seriellen Port und beendet den Hintergrund-Reader."""
        self._running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        print(f"🔌 [{self.name}] Verbindung zu {self.port} getrennt.")

    def clear_logs(self):
        """Löscht die bisherige Log-Historie."""
        with self._lock:
            self.log_history.clear()
            self._new_line_event.clear()

    def get_logs(self):
        """Gibt eine threadsichere Kopie aller bisherigen Logzeilen zurück."""
        with self._lock:
            return list(self.log_history)


    def _reader_loop(self):
        """Liest zeilenweise asynchron aus dem seriellen Port."""
        while self._running and self.ser and self.ser.is_open:
            try:
                line_bytes = self.ser.readline()
                if line_bytes:
                    line = line_bytes.decode("utf-8", errors="replace").strip()
                    if line:
                        with self._lock:
                            self.log_history.append(line)
                            self._new_line_event.set()
            except Exception:
                if not self._running:
                    break
                time.sleep(0.05)

    def wait_for_pattern(self, pattern, timeout=30, case_sensitive=False, from_current=False):
        """
        Wartet auf das Erscheinen eines Regex-Musters in den seriellen Logs.
        Gibt das Match-Objekt zurück oder löst einen TimeoutError aus.
        Wenn from_current=True ist, werden nur Log-Zeilen geprüft, die ab jetzt eintreffen.
        """
        flags = 0 if case_sensitive else re.IGNORECASE
        regex = re.compile(pattern, flags)
        start_time = time.time()
        with self._lock:
            checked_index = len(self.log_history) if from_current else 0

        print(f"⏳ [{self.name}] Warte auf Event '{pattern}' (Timeout: {timeout}s)...")

        while (time.time() - start_time) < timeout:
            with self._lock:
                current_logs = list(self.log_history)

            # Prüfe nur neue Zeilen seit dem letzten Check
            for idx in range(checked_index, len(current_logs)):
                line = current_logs[idx]
                match = regex.search(line)
                if match:
                    elapsed = round(time.time() - start_time, 2)
                    print(f"🎯 [{self.name}] Event erkannt nach {elapsed}s: '{sanitize_log_line(line)}'")
                    return match

            checked_index = len(current_logs)
            self._new_line_event.wait(timeout=0.2)
            self._new_line_event.clear()

        elapsed = round(time.time() - start_time, 2)
        recent_tail = "\n".join([sanitize_log_line(l) for l in self.log_history[-10:]]) if self.log_history else "<keine Logs>"
        raise TimeoutError(
            f"[{self.name}] Timeout ({timeout}s) beim Warten auf Muster '{pattern}'!\n"
            f"Letzte Log-Zeilen:\n{recent_tail}"
        )

    @staticmethod
    def pulse_relay(relay_port, off_duration=0.8):
        """
        Schaltet ein LCUS-1 USB-Relais (CH340) kurz aus und wieder ein (Power-Cycle).
        Hex-Befehle:
          A0 01 01 A2 -> Relais zieht an / Öffner (NC) trennt Stromversorgung
          A0 01 00 A1 -> Relais fällt ab / Stromversorgung wiederhergestellt
        """
        if serial is None:
            raise RuntimeError("Das Modul 'pyserial' ist nicht installiert.")
        try:
            with serial.Serial(relay_port, 9600, timeout=1) as ser:
                ser.write(b"\xA0\x01\x01\xA2")
                time.sleep(off_duration)
                ser.write(b"\xA0\x01\x00\xA1")
                time.sleep(0.2)
        except Exception as e:
            raise RuntimeError(f"Fehler beim Schalten des Relais an Port {relay_port}: {e}")

    def reset(self, method="relay_hex", relay_port=None):
        """
        Führt einen Hardware-Reset des Testgeräts durch.
        Standard ist 'relay_hex' über das zugeordnete CH340 USB-Relais.
        DTR-Reset wird hardwareseitig nicht unterstützt und löst einen Fehler aus.
        """
        if method == "dtr_rts":
            raise RuntimeError(
                f"[{self.name}] DTR-Reset wird hardwareseitig nicht unterstützt (kein DTR/BOOT-Pin verdrahtet). "
                "Bitte 'relay_hex' verwenden!"
            )

        print(f"🔄 [{self.name}] Führe Reset durch (Methode: {method})...")
        self.clear_logs()

        if method == "relay_hex":
            target_relay = relay_port or self.relay_port
            if not target_relay:
                try:
                    from . import config
                    if "7" in self.name:
                        target_relay = config.EPD7_RELAY_PORT
                    elif "13" in self.name:
                        target_relay = config.EPD13_RELAY_PORT
                    target_relay = target_relay or config.RELAY_COM_PORT
                except Exception:
                    pass

            if not target_relay:
                raise RuntimeError(
                    f"[{self.name}] Kein Relais-Port für den Power-Cycle konfiguriert oder zugeordnet!"
                )

            print(f"⚡ [{self.name}] Schalte USB-Relais an Port {target_relay}...")
            was_open = self._running and self.ser and self.ser.is_open
            if was_open:
                self.disconnect()

            self.pulse_relay(target_relay)
            time.sleep(0.5)

            if was_open:
                self.connect()
                self.clear_logs()

            print(f"✅ [{self.name}] USB-Relais Power-Cycle erfolgreich.")

        elif method == "esptool":
            raise RuntimeError(
                f"[{self.name}] esptool hard_reset wird hardwareseitig nicht unterstützt (kein Boot-Mode möglich)."
            )
        else:
            raise ValueError(f"Unbekannte Reset-Methode: {method}")

    def factory_reset_via_power_cycles(self, min_cycles=6, max_wait_per_cycle=6.0):
        """
        Führt einen vollständigen Factory-Reset über mindestens 6 aufeinanderfolgende Relais-Power-Cycles durch.
        Die Firmware zählt jeden Button-/Power-Wake in NVS ('counter').
        Wenn der Zähler >= 5 erreicht, führt die Firmware in wifiSmart() den Reset aus:
          - NVS Zähler wird zurückgesetzt (startupCounter(true))
          - WLAN-Credentials (SSID & Passwort) im Flash werden gelöscht
          - Aktivierungsstatus wird gelöscht (resetAll(true, true))
          - Bestätigt via Serial: '[MAIN] Reset - ACT 1 | WIFI 1'

        Nach dem Reset wird das Gerät einmalig per Relais geweckt, damit es sauber im
        Auslieferungszustand hochfährt und das BLE-Advertising startet.
        """
        target_relay = self.relay_port
        if not target_relay:
            try:
                from . import config
                if "7" in self.name:
                    target_relay = config.EPD7_RELAY_PORT
                elif "13" in self.name:
                    target_relay = config.EPD13_RELAY_PORT
                target_relay = target_relay or config.RELAY_COM_PORT
            except Exception:
                pass

        if not target_relay:
            raise RuntimeError(f"[{self.name}] Kein Relais-Port für den Factory-Reset zugeordnet!")

        print(f"\n" + "=" * 65)
        print(f"🏭 [{self.name}] FACTORY-RESET VIA {min_cycles}x POWER-CYCLES STARTEN")
        print(f"=" * 65)

        if not (self._running and self.ser and self.ser.is_open):
            self.connect()

        last_counter = 0
        reset_triggered = False

        for cycle in range(1, min_cycles + 1):
            print(f"⚡ [{self.name}] Power-Cycle {cycle}/{min_cycles} (Relais {target_relay})...")
            self.reset(method="relay_hex", relay_port=target_relay)

            # Auf Boot & NVS Zähler-Log lauschen
            t_start = time.time()
            counter_found = None
            while (time.time() - t_start) < max_wait_per_cycle:
                with self._lock:
                    logs = list(self.log_history)
                for l in logs:
                    m = re.search(r"Button wake detected! NVS Counter:\s*(\d+)/5", l)
                    if m:
                        counter_found = int(m.group(1))
                        break
                    if "Reset - ACT 1 | WIFI 1" in l:
                        reset_triggered = True
                if counter_found is not None:
                    break
                time.sleep(0.1)

            if counter_found is not None:
                last_counter = counter_found
                print(f"   ➔ Boot {cycle}: NVS Counter = {counter_found}/5")
            else:
                print(f"   ⚠️ Kein NVS-Counter-Log innerhalb von {max_wait_per_cycle}s erhalten.")

            # Kurze Pause vor dem nächsten Zyklus
            if cycle < min_cycles:
                time.sleep(0.5)

        # Nach den Zyklen auf Ausführung des Factory-Resets in den Logs warten
        print(f"⏳ [{self.name}] Warte auf Bestätigung des Factory-Resets ([MAIN] Reset - ACT 1 | WIFI 1)...")
        t_wait = time.time() + 12.0
        while time.time() < t_wait:
            with self._lock:
                logs = list(self.log_history)
            for l in logs:
                if "Reset - ACT 1 | WIFI 1" in l or "Startup Counter RESET" in l:
                    reset_triggered = True
                    break
            if reset_triggered:
                break
            time.sleep(0.3)

        assert reset_triggered, (
            f"[{self.name}] Factory-Reset konnte nicht verifiziert werden! "
            f"Letzter Zähler: {last_counter}/5. Letzte Logs: {self.get_logs()[-10:]}"
        )
        print(f"✅ [{self.name}] Factory-Reset erfolgreich verifiziert! ([MAIN] Reset - ACT 1 | WIFI 1)")

        # Nach dem Reset geht die Firmware in Deep Sleep (gotToDeepSleep(0)).
        # Wir wecken sie per Relais-Puls auf, damit sie sauber im Reset-Zustand hochfährt
        # und direkt in das BLE-Advertising wechselt.
        print(f"🔄 [{self.name}] Wecke Display per Relais auf für initialen Werks-Boot (BLE Bereitschaft)...")
        time.sleep(1.0)
        self.reset(method="relay_hex", relay_port=target_relay)
        print(f"🎉 [{self.name}] Testgerät ist im sauberen Werkszustand und bereit für Provisioning.")
        return True

    def read_device_identity(self, reset=False, timeout=12, fallback_esptool=False):
        """
        Ermittelt automatisch UID (z.B. epd7-404CCA41C534), Seriennummer (MAC-Adresse),
        Display-Typ (epd7/epd13) und Firmware-Version über serielle Boot-Logs des Geräts.
        Die Seriennummer entspricht der Hardware-MAC-Adresse des ESP32.
        Falls reset=True, wird vorab ein Relais-Power-Cycle ausgelöst.
        """
        if reset:
            self.reset(method="relay_hex")

        device_uid = None
        version = None
        mac_compact = None
        mac_colons = None
        display_type = "unknown"

        start_time = time.time()
        checked_index = 0

        uid_regex = re.compile(r"\[MAIN\] UID:\s*((epd(\d+))-([0-9A-Fa-f]{12}))", re.IGNORECASE)
        alt_uid_regex = re.compile(r"\[MAIN\] UID:\s*([^\s]+)", re.IGNORECASE)
        version_regex = re.compile(r"\[MAIN\] INIT Device V:\s*([^\s]+)", re.IGNORECASE)
        mac_regex = re.compile(r"\[WIFI\] MAC:\s*([0-9A-Fa-f:]{17})", re.IGNORECASE)

        print(f"⏳ [{self.name}] Lese Identität und Firmware-Version über serielle Schnittstelle (Timeout: {timeout}s)...")

        while (time.time() - start_time) < timeout:
            with self._lock:
                current_logs = list(self.log_history)

            for idx in range(checked_index, len(current_logs)):
                line = current_logs[idx]

                if not device_uid:
                    m_uid = uid_regex.search(line)
                    if m_uid:
                        device_uid = m_uid.group(1)
                        display_type = f"epd{m_uid.group(3)}"
                        mac_compact = m_uid.group(4).upper()
                    else:
                        m_alt = alt_uid_regex.search(line)
                        if m_alt:
                            device_uid = m_alt.group(1)
                            if device_uid.startswith("epd7-"):
                                display_type = "epd7"
                                mac_compact = device_uid.split("-", 1)[1].upper()
                            elif device_uid.startswith("epd13-"):
                                display_type = "epd13"
                                mac_compact = device_uid.split("-", 1)[1].upper()

                if not version:
                    m_ver = version_regex.search(line)
                    if m_ver:
                        version = m_ver.group(1)

                if not mac_colons:
                    m_mac = mac_regex.search(line)
                    if m_mac:
                        mac_colons = m_mac.group(1).upper()
                        if not mac_compact:
                            mac_compact = mac_colons.replace(":", "")

            checked_index = len(current_logs)

            # Sobald UID und Version vorliegen, können wir vorzeitig abschließen
            if device_uid and version:
                break

            self._new_line_event.wait(timeout=0.2)
            self._new_line_event.clear()

        # Formatierte MAC rekonstruieren falls nur kompakte MAC vorliegt
        if mac_compact and not mac_colons and len(mac_compact) == 12:
            mac_colons = ":".join(mac_compact[i:i+2] for i in range(0, 12, 2))

        # Fallback auf esptool read_mac über serielle Schnittstelle
        if (not mac_compact or display_type == "unknown") and fallback_esptool:
            try:
                from .flasher import read_device_mac
                print(f"ℹ️ [{self.name}] Nutze esptool-Fallback zum Auslesen der MAC-Adresse über Serial...")
                was_open = self.ser and self.ser.is_open
                if was_open:
                    self.disconnect()
                f_colons, f_compact = read_device_mac(self.port, baudrate=self.baudrate)
                mac_colons = mac_colons or f_colons
                mac_compact = mac_compact or f_compact
                if was_open:
                    self.connect()
            except Exception as e:
                print(f"⚠️ [{self.name}] esptool MAC-Abfrage fehlgeschlagen: {e}")

        # Seriennummer ist die Hardware-MAC-Adresse
        serial_number = mac_compact or device_uid

        return {
            "uid": device_uid,
            "serial_number": serial_number,
            "mac": mac_colons,
            "mac_compact": mac_compact,
            "display_type": display_type,
            "version": version,
            "port": self.port
        }

    @classmethod
    def find_ports_by_chip(cls, retries=3, delay=0.5):
        """
        Kategorisiert alle COM-Ports im System nach:
          - CP210x: E-Paper Displays (Silicon Labs UART Bridge)
          - CH340: USB-Relais (QinHeng Electronics)
        Mit automatischer Wiederholung, falls Geräte nach einem Relais-Puls kurz re-enumerieren.
        """
        for attempt in range(retries):
            all_ports = cls.list_ports()
            cp210x_ports = []
            ch340_ports = []
            other_ports = []

            for p in all_ports:
                desc = (p.get("desc") or "").lower()
                hwid = (p.get("hwid") or "").lower()
                port = p["port"]

                if "cp210" in desc or "10c4:ea60" in hwid or "silicon labs" in desc or "silabser" in hwid:
                    cp210x_ports.append(port)
                elif "ch340" in desc or "1a86:7523" in hwid or "ch341" in desc or "serial2" in hwid:
                    ch340_ports.append(port)
                else:
                    other_ports.append(port)

            if (cp210x_ports or ch340_ports) or attempt == retries - 1:
                break
            time.sleep(delay)

        return {
            "cp210x": cp210x_ports,
            "ch340": ch340_ports,
            "other": other_ports,
            "all": all_ports
        }

    @classmethod
    def verify_and_pair_hardware(cls, timeout_per_relay=6, save_cache=True, required_targets=None):
        """
        Automatische Vorab-Prüfung und Zuordnung:
          1. Erkennt Displays (CP210x) und Relais (CH340).
          2. Schaltet jedes Relais einzeln (Display-Ports bleiben während des Schaltens geschlossen,
             um Windows USB-Treiberfehler durch Spannungsabfall zu verhindern).
          3. Öffnet Display-Ports nach 0.5s Re-Enumeration und lauscht auf serielle Boot-Logs.
          4. Liest UID, MAC/Seriennummer, Typ (epd7/epd13) und Firmware aus.
          5. Verifiziert mindestens ein Display bzw. die in required_targets angeforderten Targets.
        """
        print("=" * 65)
        print("🔍 AUTOMATISCHE HARDWARE-VERIFIKATION & RELAIS-ZUORDNUNG")
        print("=" * 65)

        categorized = cls.find_ports_by_chip()
        display_candidates = list(categorized["cp210x"])
        relay_candidates = list(categorized["ch340"])

        # Fallback falls Beschreibungen generisch sind
        if not display_candidates and categorized["other"]:
            print("ℹ️ Keine expliziten CP210x-Ports im Treibernamen, nutze sonstige Ports als Display-Kandidaten...")
            display_candidates = list(categorized["other"])
        if not relay_candidates and categorized["other"]:
            print("ℹ️ Keine expliziten CH340-Ports im Treibernamen, nutze sonstige Ports als Relais-Kandidaten...")
            relay_candidates = [p for p in categorized["other"] if p not in display_candidates]

        print(f"  📺 Display-Kandidaten (CP210x): {display_candidates or 'Keine'}")
        print(f"  ⚡ Relais-Kandidaten  (CH340):  {relay_candidates or 'Keine'}")

        if not relay_candidates:
            raise HardwareSetupError(
                f"❌ Keine CH340-Relais an den COM-Ports gefunden! Verfügbare Ports: {[p['port'] for p in categorized['all']]}.\n"
                "Bitte USB-Relais anschließen und Treiber prüfen."
            )
        if not display_candidates:
            raise HardwareSetupError(
                f"❌ Keine CP210x E-Paper Displays gefunden! Verfügbare Ports: {[p['port'] for p in categorized['all']]}.\n"
                "Bitte Displays anschließen."
            )

        paired = {}
        paired_displays = set()

        for r_port in relay_candidates:
            remaining_displays = [d for d in display_candidates if d not in paired_displays]
            if not remaining_displays:
                break

            print(f"\n⚡ Schalte Relais an {r_port} (Power-Cycle) und lausche auf Display-Neustart...")
            try:
                cls.pulse_relay(r_port, off_duration=0.8)
            except Exception as e:
                print(f"  ⚠️ Fehler beim Schalten von Relais {r_port}: {e}")
                continue

            # 0.5s Pause für Windows USB Re-Enumeration nach Spannungswiederkehr
            time.sleep(0.5)

            booted_port = None
            display_info = None

            # Prüfe verbleibende Display-Kandidaten
            for d_port in remaining_displays:
                ctrl = None
                try:
                    ctrl = cls(d_port, baudrate=115200, name=f"Probe-{d_port}")
                    ctrl.connect()

                    # Bis zu 4 Sekunden auf Boot-Meldungen lauschen
                    start_t = time.time()
                    found_boot = False
                    while (time.time() - start_t) < 4.0:
                        with ctrl._lock:
                            logs = list(ctrl.log_history)
                        if any("[MAIN] INIT" in line or "[MAIN] UID" in line or "[WIFI] MAC" in line for line in logs):
                            found_boot = True
                            break
                        time.sleep(0.15)

                    if found_boot:
                        display_info = ctrl.read_device_identity(reset=False, timeout=3, fallback_esptool=False)
                        booted_port = d_port
                        break
                except Exception:
                    pass
                finally:
                    if ctrl:
                        ctrl.disconnect()

            if booted_port and display_info:
                dtype = display_info.get("display_type", "unknown")
                uid = display_info.get("uid")
                sn = display_info.get("serial_number")
                ver = display_info.get("version")

                register_github_mask(uid)
                register_github_mask(sn)
                register_github_mask(display_info.get("mac"))

                print(f"  🎯 Treffer! Relais {r_port} steuert Display an {booted_port}")
                print(f"     ➔ Typ: {dtype.upper()} | UID: {mask_uid(uid)} | SN/MAC: {mask_mac(sn)} | Firmware: V{ver}")

                display_info["relay_port"] = r_port
                display_info["port"] = booted_port

                if dtype in ("epd7", "epd13"):
                    paired[dtype] = display_info
                    paired_displays.add(booted_port)
                else:
                    print(f"  ⚠️ Unerwarteter Display-Typ '{dtype}' an {booted_port}")
                    paired[f"unknown_{booted_port}"] = display_info
            else:
                print(f"  ⚪ Kein Display-Neustart nach Relais-Puls an {r_port} erkannt.")

        print("\n" + "=" * 65)
        print("📊 STATUS DER HARDWARE-VERIFIKATION:")
        print("=" * 65)

        for expected in ("epd7", "epd13"):
            if expected in paired:
                info = paired[expected]
                register_github_mask(info.get("uid"))
                register_github_mask(info.get("serial_number"))
                register_github_mask(info.get("mac"))
                print(f"  ✅ {expected.upper()}: Display={info['port']} (CP210x) ➔ Relais={info['relay_port']} (CH340)")
                print(f"     UID: {mask_uid(info['uid'])} | SN/MAC: {mask_mac(info['serial_number'])} | Firmware: V{info['version']}")
            else:
                print(f"  ⚪ {expected.upper()}: Nicht angeschlossen / nicht erkannt")

        print("=" * 65)

        # Validierung
        if required_targets:
            missing = [t for t in required_targets if t.lower() not in paired]
            if missing:
                raise HardwareSetupError(
                    f"❌ Benötigtes Zielgerät {', '.join(missing).upper()} fehlt oder schaltet nicht sauber über das Relais.\n"
                    "Bitte Anschlüsse und CH340-Relais überprüfen."
                )
        else:
            if not paired:
                raise HardwareSetupError(
                    "❌ Keine betriebsbereiten E-Paper Displays mit Relais-Steuerung gefunden.\n"
                    "Bitte Anschlüsse und CH340-Relais überprüfen."
                )

        # In-Memory config aktualisieren
        try:
            from . import config
            if "epd7" in paired:
                config.EPD7_COM_PORT = paired["epd7"]["port"]
                config.EPD7_RELAY_PORT = paired["epd7"]["relay_port"]
                config.EPD7_DEVICE_ID = paired["epd7"]["uid"]

            if "epd13" in paired:
                config.EPD13_COM_PORT = paired["epd13"]["port"]
                config.EPD13_RELAY_PORT = paired["epd13"]["relay_port"]
                config.EPD13_DEVICE_ID = paired["epd13"]["uid"]
        except Exception:
            pass

        if save_cache:
            cache_file = os.path.join(os.path.dirname(__file__), "hardware_mapping.json")
            try:
                with open(cache_file, "w", encoding="utf-8") as f:
                    json.dump(paired, f, indent=2)
            except Exception:
                pass

        return paired

    @classmethod
    def auto_discover_devices(cls, ports=None, timeout=6):
        """Führt automatische Erkennung und Zuordnung durch."""
        return cls.verify_and_pair_hardware(timeout_per_relay=timeout)

    @staticmethod
    def list_ports():
        """
        Listet alle aktuell im System sichtbaren COM-Ports auf.
        Kombiniert pyserial SetupAPI mit Windows Registry HARDWARE\\DEVICEMAP\\SERIALCOMM als robuster Fallback.
        """
        ports_dict = {}

        # 1. pyserial Standard-Erkennung
        if serial is not None:
            try:
                for p in serial.tools.list_ports.comports():
                    ports_dict[p.device.upper()] = {
                        "port": p.device,
                        "desc": p.description or "",
                        "hwid": p.hwid or ""
                    }
            except Exception:
                pass

        # 2. Windows Registry Fallback / Ergänzung
        if sys.platform == "win32":
            try:
                import winreg
                key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
                for i in range(winreg.QueryInfoKey(key)[1]):
                    val_name, com_port, _ = winreg.EnumValue(key, i)
                    com_upper = com_port.upper()
                    if com_upper not in ports_dict:
                        desc = "Serielles Gerät"
                        vn_lower = val_name.lower()
                        if "silabser" in vn_lower:
                            desc = "Silicon Labs CP210x USB to UART Bridge"
                        elif "ch34" in vn_lower or "serial2" in vn_lower:
                            desc = "USB-SERIAL CH340"
                        ports_dict[com_upper] = {
                            "port": com_port,
                            "desc": desc,
                            "hwid": val_name
                        }
            except Exception:
                pass

        return list(ports_dict.values())

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
