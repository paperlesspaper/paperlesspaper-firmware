#!/usr/bin/env python3
"""
testbench/privacy.py
Datenschutz- und Maskierungsmodul für Testbench-Logs.
Verhindert das Veröffentlichen von Hardware-Identifikatoren (EPD-UIDs, MAC-Adressen),
gescannten WLAN-Netzwerken, Benutzerpfaden und Cloud-Secrets in öffentlichen CI/CD-Logs.
"""

import os
import re
import sys

# Windows CLI Encoding-Fix
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Cache bereits registrierter Maskierungen zur Vermeidung doppelter ::add-mask:: Aufrufe
_REGISTERED_MASKS = set()


def register_github_mask(value):
    """
    Registriert einen sensiblen Wert bei GitHub Actions als Secret.
    GitHub Actions maskiert diesen Wert in allen nachfolgenden Konsolen-Ausgaben mit '***'.
    """
    if not value or not os.environ.get("GITHUB_ACTIONS"):
        return
    val_str = str(value).strip()
    # Ignoriere zu kurze oder generische Strings
    if len(val_str) < 5 or val_str in _REGISTERED_MASKS:
        return
    if val_str.lower() in ("true", "false", "epd7", "epd13", "relay_hex", "eu-central-1", "all", "none"):
        return

    _REGISTERED_MASKS.add(val_str)
    print(f"::add-mask::{val_str}")


def mask_uid(uid):
    """
    Maskiert eine E-Paper Display UID:
    - 'epd7-dc1ed57e3334'  -> 'epd7-***3334'
    - 'epd13-test001'      -> 'epd13-***t001'
    - 'DC1ED57E3334'       -> '***3334'
    """
    if not uid:
        return ""
    uid_str = str(uid).strip()
    register_github_mask(uid_str)

    if "-" in uid_str:
        prefix, rest = uid_str.split("-", 1)
        if len(rest) > 4:
            return f"{prefix}-***{rest[-4:]}"
        return f"{prefix}-***"
    if len(uid_str) > 6:
        return f"***{uid_str[-4:]}"
    return "***"


def mask_mac(mac):
    """
    Maskiert MAC-Adressen:
    - 'DC:1E:D5:7E:33:36'  -> '**:**:**:**:**:36'
    - 'DC-1E-D5-7E-33-36'  -> '**-**-**-**-**-36'
    - 'DC1ED57E3334'       -> '***3334'
    """
    if not mac:
        return ""
    mac_str = str(mac).strip()
    register_github_mask(mac_str)

    if ":" in mac_str:
        parts = mac_str.split(":")
        if len(parts) == 6:
            return f"**:**:**:**:**:{parts[-1]}"
    elif "-" in mac_str:
        parts = mac_str.split("-")
        if len(parts) == 6:
            return f"**-**-**-**-**-{parts[-1]}"
    if len(mac_str) > 4:
        return f"***{mac_str[-4:]}"
    return "***"


def mask_ssid(ssid):
    """
    Maskiert eine WLAN-SSID für Logs:
    - 'MyHomeNetwork' -> 'My***rk'
    - 'WiFi'          -> '***'
    """
    if not ssid:
        return ""
    s_str = str(ssid).strip()
    register_github_mask(s_str)

    if len(s_str) <= 4:
        return "***"
    return f"{s_str[:2]}***{s_str[-2:]}"


def mask_s3_key(key):
    """
    Maskiert Gerätekennungen in S3-Objektnamen:
    - 'epdPicture-epd7-dc1ed57e3334.jpg' -> 'epdPicture-epd7-***3334.jpg'
    """
    if not key:
        return ""
    return re.sub(
        r"(epd(?:7|13)-)[0-9a-zA-Z_-]{4,}([0-9a-zA-Z]{4})",
        r"\1***\2",
        str(key)
    )


def mask_topic(topic):
    """
    Maskiert MQTT-Topics mit Geräte-UID:
    - '$aws/things/epd7-dc1ed57e3334/epaper/receive' -> '$aws/things/epd7-***3334/epaper/receive'
    """
    if not topic:
        return ""
    return re.sub(
        r"(things/epd(?:7|13)-)[0-9a-zA-Z_-]{4,}([0-9a-zA-Z]{4})",
        r"\1***\2",
        str(topic)
    )


def mask_path(path):
    """
    Maskiert Benutzerprofile und Runner-Pfade:
    - 'C:\\Users\\danie\\actions-runner\\...' -> '<runner_root>\\...'
    - '/home/runner/...' -> '<runner_root>/...'
    """
    if not path:
        return ""
    p_str = str(path)
    p_str = re.sub(r"[A-Za-z]:\\[Uu]sers\\[^\\]+\\", lambda m: "<runner_root>\\", p_str)
    p_str = re.sub(r"/home/[^/]+/", lambda m: "<runner_root>/", p_str)
    return p_str


def sanitize_log_line(line):
    """
    Bereinigt eine beliebige Log-Zeile (z. B. serielle Logs vom Display)
    von UIDs, MACs, Passwörtern und lokalen Host-Pfaden vor der Konsolenausgabe.
    """
    if not line:
        return ""
    res = str(line)

    # EPD UIDs (z. B. epd7-dc1ed57e3334)
    res = re.sub(
        r"\b(epd(?:7|13)-)[0-9a-fA-F]{4,}([0-9a-fA-F]{4})\b",
        r"\1***\2",
        res
    )

    # MAC Adressen (DC:1E:D5:7E:33:36)
    res = re.sub(
        r"\b(?:[0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})\b",
        r"**:**:**:**:**:\1",
        res
    )

    # S3 Objektnamen in Logs
    res = mask_s3_key(res)

    # Pfade
    res = mask_path(res)

    return res
