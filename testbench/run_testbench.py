#!/usr/bin/env python3
"""
testbench/run_testbench.py
CLI-Runner für die Hardware-in-the-Loop Testsuite.
Ermöglicht manuelle und CI-gesteuerte Testläufe auf dem Windows-Testbench-Host.
"""

import sys
import os
import time
import argparse

# Windows CLI Encoding-Fix
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

try:
    import pytest
except ImportError:
    pytest = None

from .hardware_controller import ESP32HardwareController, HardwareSetupError
from . import config
from .privacy import mask_uid, mask_path, register_github_mask

def print_banner():
    register_github_mask(config.EPD7_DEVICE_ID)
    register_github_mask(config.EPD13_DEVICE_ID)
    print("=" * 65)
    print("🔬 HARDWARE-IN-THE-LOOP (HIL) TESTBENCH RUNNER")
    print("=" * 65)
    print("Hardware-Konfiguration:")
    print(f"  - EPD7:  Display={config.EPD7_COM_PORT} | Relais={config.EPD7_RELAY_PORT} | UID={mask_uid(config.EPD7_DEVICE_ID)}")
    print(f"  - EPD13: Display={config.EPD13_COM_PORT} | Relais={config.EPD13_RELAY_PORT} | UID={mask_uid(config.EPD13_DEVICE_ID)}")
    print(f"  - Reset-Methode: {config.RESET_METHOD}")
    print("=" * 65)

def isolate_inactive_relays(active_target=None):
    """
    Schaltet nach der Verifikation das Relais des Geräts ab, welches NICHT getestet wird.
    Das aktive Zielgerät bleibt unterbrechungsfrei mit Strom versorgt.
    """
    if active_target == "epd13":
        print("\n" + "=" * 65)
        print(f"⚡ [Hardware-Isolation] Schalte Relais für inaktives Gerät EPD7 ({config.EPD7_RELAY_PORT}) AUS...")
        print("=" * 65)
        if config.EPD7_RELAY_PORT:
            ESP32HardwareController.set_relay_power(config.EPD7_RELAY_PORT, power_on=False)
        if config.EPD13_RELAY_PORT:
            ESP32HardwareController.set_relay_power(config.EPD13_RELAY_PORT, power_on=True)
    elif active_target == "epd7":
        print("\n" + "=" * 65)
        print(f"⚡ [Hardware-Isolation] Schalte Relais für inaktives Gerät EPD13 ({config.EPD13_RELAY_PORT}) AUS...")
        print("=" * 65)
        if config.EPD13_RELAY_PORT:
            ESP32HardwareController.set_relay_power(config.EPD13_RELAY_PORT, power_on=False)
        if config.EPD7_RELAY_PORT:
            ESP32HardwareController.set_relay_power(config.EPD7_RELAY_PORT, power_on=True)

def is_mapping_valid_for_target(target):
    """
    Prüft, ob für das gewünschte Target bereits eine valide Zuordnung (Display-Port und Relais-Port)
    in hardware_mapping.json / config vorliegt.
    """
    if target == "epd7":
        return bool(config._epd7_cache.get("port") and config._epd7_cache.get("relay_port"))
    elif target == "epd13":
        return bool(config._epd13_cache.get("port") and config._epd13_cache.get("relay_port"))
    elif target == "all":
        has_epd7 = bool(config._epd7_cache.get("port") and config._epd7_cache.get("relay_port"))
        has_epd13 = bool(config._epd13_cache.get("port") and config._epd13_cache.get("relay_port"))
        return has_epd7 and has_epd13
    return False

def main():
    parser = argparse.ArgumentParser(description="HIL Testbench CLI Runner")
    parser.add_argument("--target", choices=["epd7", "epd13", "all"], default="all", help="Zielgerät für den Test")
    parser.add_argument("--list-ports", action="store_true", help="Listet alle erkannten COM-Ports auf (CP210x & CH340)")
    parser.add_argument("--verify", action="store_true", help="Prüft Relais-Zuordnung und Vorhandensein von EPD7 und EPD13")
    parser.add_argument("--auto-detect", action="store_true", help="Alias für --verify (Erkennung und Zuordnung)")
    parser.add_argument("--force-verify", action="store_true", help="Erzwingt vollständige Neu-Kalibrierung aller Relais vor dem Testlauf (ignoriert Cache)")
    parser.add_argument("--skip-check", action="store_true", help="Überspringt die automatische Hardware-Prüfung vor dem Testlauf")
    parser.add_argument("--factory-reset", action="store_true", help="Führt isoliert einen 6x Power-Cycle Factory-Reset auf dem Zielgerät durch")
    parser.add_argument("--run-ota", action="store_true", help="Führt zusätzlich die zeitintensiven OTA-Firmware-Update-Tests aus")
    parser.add_argument("--test-ble", action="store_true", help="Führt den fokussierten BLE-WLAN-Provisionierungs-Testlauf durch (Relais-Wakeup -> BLE Scan -> GATT Provisioning)")
    parser.add_argument("--junitxml", help="Pfad zur JUnit-XML-Ausgabedatei für Testprotokolle")
    parser.add_argument("--test-relay", help="Schaltet testweise ein Relais (z.B. COM8) und zeigt an, welches Display reagiert")
    parser.add_argument("--candidate-bin", help="Pfad zur Kandidaten-Firmware-Binärdatei (überschreibt CANDIDATE_FIRMWARE_EPD7/13)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Ausführliche PyTest-Ausgabe")
    args = parser.parse_args()

    if args.candidate_bin:
        abs_bin = os.path.abspath(args.candidate_bin)
        if args.target == "epd7":
            os.environ["CANDIDATE_FIRMWARE_EPD7"] = abs_bin
        elif args.target == "epd13":
            os.environ["CANDIDATE_FIRMWARE_EPD13"] = abs_bin
        else:
            os.environ["CANDIDATE_FIRMWARE_EPD7"] = abs_bin
            os.environ["CANDIDATE_FIRMWARE_EPD13"] = abs_bin

    if args.run_ota:
        os.environ["HIL_RUN_OTA"] = "1"

    if args.test_relay:
        r_port = args.test_relay.upper()
        print(f"⚡ Schalte Test-Puls an Relais {r_port} (off_duration=0.8s)...")
        cat = ESP32HardwareController.find_ports_by_chip()
        display_ports = cat["cp210x"] or cat["other"]
        active_controllers = {}
        for dp in display_ports:
            try:
                c = ESP32HardwareController(dp, name=f"Probe-{dp}")
                c.connect()
                if c.ser and c.ser.is_open:
                    try:
                        c.ser.reset_input_buffer()
                    except Exception:
                        pass
                c.clear_logs()
                active_controllers[dp] = c
            except Exception:
                pass
        try:
            ESP32HardwareController.pulse_relay(r_port, off_duration=0.8)
            print(f"✅ Relais {r_port} geschaltet. Lausche bis zu 5s auf Boot-Meldungen auf {list(active_controllers.keys())}...")
            t0 = time.time()
            detected = None
            while (time.time() - t0) < 5.0:
                for dp, c in active_controllers.items():
                    with c._lock:
                        lines = list(c.log_history)
                    if any("[MAIN] INIT" in l or "[MAIN] UID" in l or "Button wake detected!" in l or "[WAKE]" in l for l in lines):
                        detected = dp
                        break
                if detected:
                    break
                time.sleep(0.1)
            if detected:
                info = active_controllers[detected].read_device_identity(reset=False, timeout=3)
                print(f"🎯 Treffer! Relais {r_port} hat Display {detected} ({info.get('display_type', '').upper()} / {info.get('uid')}) neu gestartet!")
            else:
                print(f"⚪ Kein Display-Neustart nach Relais-Puls an {r_port} registriert.")
        finally:
            for c in active_controllers.values():
                try:
                    c.disconnect()
                except Exception:
                    pass
        sys.exit(0)

    if args.list_ports:
        cat = ESP32HardwareController.find_ports_by_chip()
        print("Erkannte serielle Ports im System:")
        print("  📺 Displays (CP210x):")
        for p in cat["cp210x"]:
            print(f"     - {p}")
        if not cat["cp210x"]:
            print("     (keine CP210x Ports gefunden)")

        print("  ⚡ USB-Relais (CH340):")
        for p in cat["ch340"]:
            print(f"     - {p}")
        if not cat["ch340"]:
            print("     (keine CH340 Ports gefunden)")

        if cat["other"]:
            print("  ⚪ Sonstige COM-Ports:")
            for p in cat["other"]:
                print(f"     - {p}")
        sys.exit(0)

    target_req = [args.target] if args.target in ("epd7", "epd13") else None

    if args.verify or args.auto_detect:
        try:
            ESP32HardwareController.verify_and_pair_hardware(required_targets=target_req, use_cached_first=not args.force_verify)
            print("\n🎉 Hardware-Setup vollständig und betriebsbereit.")
            sys.exit(0)
        except HardwareSetupError as e:
            print(f"\n{e}")
            sys.exit(1)

    if args.factory_reset:
        # Vor dem Factory-Reset Hardware prüfen/verifizieren nur falls noch keine Zuordnung vorliegt
        if args.force_verify or (not is_mapping_valid_for_target(args.target) and not args.skip_check and not os.environ.get("HIL_SKIP_HARDWARE_CHECK")):
            try:
                targets_req = [args.target] if args.target in ("epd7", "epd13") else None
                ESP32HardwareController.verify_and_pair_hardware(required_targets=targets_req, use_cached_first=not args.force_verify)
            except Exception as e:
                print(f"⚠️ Hinweis bei automatischer Hardware-Prüfung vor dem Factory-Reset: {e}")

        targets = ["epd7", "epd13"] if args.target == "all" else [args.target]
        for t in targets:
            port = config.EPD7_COM_PORT if t == "epd7" else config.EPD13_COM_PORT
            relay = config.EPD7_RELAY_PORT if t == "epd7" else config.EPD13_RELAY_PORT
            name = t.upper()
            available = [p["port"] for p in ESP32HardwareController.list_ports()]
            if port not in available:
                print(f"⚠️ Port {port} für {name} nicht angeschlossen - überspringe.")
                continue
            isolate_inactive_relays(active_target=t)
            print(f"\n🏭 Führe 6x Power-Cycle Factory-Reset für {name} an Port {port} (Relais {relay}) durch...")
            with ESP32HardwareController(port, name=name, relay_port=relay) as dev:
                dev.factory_reset_via_power_cycles(min_cycles=6)
                print(f"🎉 {name} erfolgreich per 6x Power-Cycles auf Werkseinstellungen zurückgesetzt.")
        sys.exit(0)

    # Automatische Vorab-Prüfung vor jedem Testlauf:
    # Falls Zuordnung bereits in hardware_mapping.json gecacht ist, überspringen wir das Abklappern aller Relais.
    paired_devices = {}
    has_valid_config = is_mapping_valid_for_target(args.target)

    if args.force_verify or (not has_valid_config and not args.skip_check and not os.environ.get("HIL_SKIP_HARDWARE_CHECK")):
        print("\n🔍 Keine vollständige Hardware-Zuordnung vorhanden oder --force-verify gesetzt.")
        print("   Führe automatische Hardware-Erkennung und Relais-Zuordnung durch...")
        try:
            paired_devices = ESP32HardwareController.verify_and_pair_hardware(required_targets=target_req)
        except HardwareSetupError as e:
            print(f"\n{e}")
            print("❌ TEST ABGEBROCHEN: Hardware-Setup unvollständig.")
            sys.exit(1)
        except Exception as e:
            print(f"⚠️ Warnung bei der Hardware-Prüfung: {e}")
    else:
        print("\n" + "=" * 65)
        print("📋 HARDWARE-KONFIGURATION GELADEN (aus hardware_mapping.json)")
        print("=" * 65)
        if args.target in ("epd7", "all") and config._epd7_cache.get("port"):
            print(f"  📺 EPD7:  Display={config.EPD7_COM_PORT} | Relais={config.EPD7_RELAY_PORT} | UID={mask_uid(config.EPD7_DEVICE_ID)}")
        if args.target in ("epd13", "all") and config._epd13_cache.get("port"):
            print(f"  📺 EPD13: Display={config.EPD13_COM_PORT} | Relais={config.EPD13_RELAY_PORT} | UID={mask_uid(config.EPD13_DEVICE_ID)}")
        print("  ⚡ Gespeicherte Zuordnung aktiv – Relais-Scan übersprungen.")
        print("     (Zum erneuten Kalibrieren aller Relais: --verify oder --force-verify nutzen)")
        print("=" * 65)

    isolate_target = args.target if args.target in ("epd7", "epd13") else None
    if args.target == "all":
        if paired_devices:
            if "epd7" in paired_devices and "epd13" not in paired_devices:
                isolate_target = "epd7"
            elif "epd13" in paired_devices and "epd7" not in paired_devices:
                isolate_target = "epd13"
        else:
            if config._epd7_cache.get("port") and not config._epd13_cache.get("port"):
                isolate_target = "epd7"
            elif config._epd13_cache.get("port") and not config._epd7_cache.get("port"):
                isolate_target = "epd13"

    # Nach der Zuordnung sofort das Relais des inaktiven Displays abschalten
    isolate_inactive_relays(active_target=isolate_target)

    print_banner()

    if pytest is None:
        print("❌ FEHLER: 'pytest' ist nicht installiert. Bitte 'pip install pytest' ausführen.")
        sys.exit(1)

    # Pytest Argumente vorbereiten
    test_file = os.path.join(os.path.dirname(__file__), "test_epd_lifecycle.py")
    pytest_args = [test_file, "-s"]

    if args.verbose:
        pytest_args.append("-v")

    if args.test_ble:
        target_name = "EPD7" if args.target == "epd7" else ("EPD13" if args.target == "epd13" else "")
        if target_name:
            pytest_args.extend(["-k", f"Test{target_name}Lifecycle and (test_00 or test_01)"])
        else:
            pytest_args.extend(["-k", "test_00 or test_01"])
    elif args.target == "epd7":
        pytest_args.extend(["-k", "TestEPD7Lifecycle"])
    elif args.target == "epd13":
        pytest_args.extend(["-k", "TestEPD13Lifecycle"])
    elif args.target == "all" and paired_devices:
        if "epd7" in paired_devices and "epd13" not in paired_devices:
            print("ℹ️ Nur EPD7 verifiziert – führe Testlauf für EPD7 aus.\n")
            pytest_args.extend(["-k", "TestEPD7Lifecycle"])
        elif "epd13" in paired_devices and "epd7" not in paired_devices:
            print("ℹ️ Nur EPD13 verifiziert – führe Testlauf für EPD13 aus.\n")
            pytest_args.extend(["-k", "TestEPD13Lifecycle"])

    if args.junitxml:
        pytest_args.append(f"--junitxml={args.junitxml}")

    try:
        clean_pytest_args = [mask_path(a) for a in pytest_args]
        print(f"🚀 Starte PyTest mit Argumenten: {clean_pytest_args}\n")
        exit_code = pytest.main(pytest_args)
    finally:
        print("\n⚡ [Hardware-Isolation] Beende Testlauf: Schalte alle Relais AUS...")
        if config.EPD7_RELAY_PORT:
            ESP32HardwareController.set_relay_power(config.EPD7_RELAY_PORT, power_on=False)
        if config.EPD13_RELAY_PORT:
            ESP32HardwareController.set_relay_power(config.EPD13_RELAY_PORT, power_on=False)

    print("=" * 65)
    if exit_code == 0:
        print("🎉 HIL TESTBENCH ERFOLGREICH: Alle Hardware-Tests bestanden.")
    else:
        print(f"❌ HIL TESTBENCH FEHLGESCHLAGEN: Exit Code {exit_code}")
    print("=" * 65)

    sys.exit(exit_code)

if __name__ == "__main__":
    main()
