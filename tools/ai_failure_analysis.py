#!/usr/bin/env python3
"""
tools/ai_failure_analysis.py
Sichere, automatisierte Fehlerursachenanalyse (Root Cause Analysis / RCA) für
fehlgeschlagene Hardware-in-the-Loop (HIL) Tests mittels Google Gemini API.

Enthält eine strikte Sanitizing-Engine zur zuverlässigen Maskierung von Secrets
(AWS Keys, Bearer/JWTs, Passwörter, SSIDs, Zertifikate und Runner-Env-Vars).
"""

import os
import re
import sys
import json
import time
import urllib.request
import urllib.error

# Windows CLI Encoding-Fix
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

GEMINI_MODEL = os.environ.get("GEMINI_MODEL", "gemini-3.8-flash")

# Keywords in Umgebungsvariablen-Namen, deren Werte redaktiert werden müssen
SENSITIVE_ENV_KEYWORDS = (
    "SECRET", "KEY", "PASS", "TOKEN", "CREDENTIAL",
    "AUTH", "SSID", "BUCKET", "ENDPOINT", "CLIENT_ID", "AUDIENCE"
)

SYSTEM_PROMPT = """Du bist ein hochqualifizierter Senior Embedded Systems & Testbench-Experte für das PaperlessPaper-Ökosystem (ESP32-C6, E-Paper Displays EPD7/EPD13, BLE-Provisionierung, AWS IoT MQTT/Shadows und FOTA-Updates).

Analysiere den folgenden Fehler aus einem Testbench-Lauf und erstelle eine präzise, lösungsorientierte Fehlerdiagnose auf Deutsch.
Konzentriere dich ausschließlich auf den technischen Kern:
- **Erkanntes Problem:** (Exakt 1 Satz: Was genau ist fehlgeschlagen oder in ein Timeout gelaufen?)
- **Wahrscheinliche Ursache (Root Cause):** (1-3 Sätze: Warum trat der Fehler auf? Z. B. schwaches WLAN-Signal, zu kurzer Timeout für 1.85 MB Firmware, nicht ansprechbares Bluetooth, falsche MQTT-State-Transition, Hardware nicht im Werkszustand?)
- **Empfohlene Lösung:** (1-2 konkrete, umsetzbare Handlungsschritte für Firmware-Code, Testbench-Konfiguration oder Hardware-Setup)

Halte die Antwort kompakt, professionell und ohne Floskeln."""


def sanitize_log(text, extra_secrets=None):
    """
    Maskiert sensitive Daten zuverlässig aus Fehlermeldungen und Logs:
    1. Dynamischer Abgleich mit gesetzten sensiblen Umgebungsvariablen
    2. Regex-Filter für AWS-Keys, Bearer/JWT-Tokens, Zertifikatsblöcke
    3. Erkennung von Passwörtern, SSIDs und URLs mit Credentials
    4. Kürzen auf maximal 4.000 Zeichen (Token- & Privacy-Schutz)
    """
    if not text:
        return ""

    sanitized = str(text)

    # 1. Dynamischer Abgleich mit Environment-Secrets
    secrets_to_redact = set()
    if extra_secrets:
        for sec in extra_secrets:
            if sec and len(str(sec).strip()) >= 4:
                secrets_to_redact.add(str(sec).strip())

    for k, v in os.environ.items():
        k_upper = k.upper()
        if any(keyword in k_upper for keyword in SENSITIVE_ENV_KEYWORDS):
            val_clean = str(v).strip()
            # Ignoriere Standard-Systemwerte wie "1", "true", "eu-central-1", "all"
            if len(val_clean) >= 5 and val_clean.lower() not in ("true", "false", "relay_hex", "eu-central-1", "all", "none"):
                secrets_to_redact.add(val_clean)

    for secret in sorted(secrets_to_redact, key=len, reverse=True):
        sanitized = sanitized.replace(secret, "[REDACTED_SECRET]")

    # 2. Regex-Muster für strukturierte sensitive Daten
    # AWS Access Key ID (AKIA...)
    sanitized = re.sub(r"\bAKIA[0-9A-Z]{16}\b", "[REDACTED_AWS_KEY]", sanitized)

    # PEM-Zertifikate & Private Keys
    sanitized = re.sub(r"-----BEGIN [A-Z ]+-----[^-]+-----END [A-Z ]+-----", "[REDACTED_CERTIFICATE]", sanitized)

    # JWT Tokens (ey...)
    sanitized = re.sub(r"\bey[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{10,}\b", "[REDACTED_JWT]", sanitized)

    # Bearer Token
    sanitized = re.sub(r"(?i)\bbearer\s+[A-Za-z0-9\-._~+/]+=*", "Bearer [REDACTED_TOKEN]", sanitized)

    # URLs mit Basic-Auth (https://user:pass@host)
    sanitized = re.sub(r"(https?://)([^:\s]+):([^@\s]+)@", r"\1[REDACTED_USER]:[REDACTED_PASS]@", sanitized)

    # MAC-Adressen (AA:BB:CC:DD:EE:FF oder AA-BB-CC-DD-EE-FF)
    sanitized = re.sub(r"\b(?:[0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}\b", "[REDACTED_MAC]", sanitized)

    # EPD Hardware-UIDs (z. B. epd7-4c7525df8abc, epd13-64e833...)
    sanitized = re.sub(r"\bepd(?:7|13)-[0-9a-zA-Z_-]{4,}\b", "[REDACTED_UID]", sanitized)

    # BLE Device-Namen (z. B. EPD7_DF8ABC, EPD13_123456)
    sanitized = re.sub(r"\b(?:EPD7|EPD13)_[0-9A-Fa-f]{4,}\b", "[REDACTED_DEVICE]", sanitized)

    # Dateipfade mit Benutzernamen (Windows & Linux Runner Privacy)
    sanitized = re.sub(r"[A-Za-z]:\\[Uu]sers\\[^\\]+\\", r"<runner_root>\\", sanitized)
    sanitized = re.sub(r"/home/[^/]+/", r"<runner_root>/", sanitized)

    # Typische Log-Zeilen für WLAN & Passwörter
    sanitized = re.sub(r"(?i)(passwort|password|passwd|secret|api_?key)\s*[:=]\s*[^\s,]+", r"\1: [REDACTED]", sanitized)
    sanitized = re.sub(r'(?i)"(passwort|password|passwd|secret|token|key|ssid)"\s*:\s*"[^"]*"', r'"\1": "[REDACTED]"', sanitized)
    sanitized = re.sub(r"(?i)\bssid\s*[:=]\s*[^\s,]+", "SSID: [REDACTED_SSID]", sanitized)

    # 3. Kontext-Eingrenzung (max. 4.000 Zeichen)
    max_len = 4000
    if len(sanitized) > max_len:
        sanitized = sanitized[:max_len] + "\n\n[... Log für KI-Analyse gekürzt ...]"

    return sanitized


def analyze_failure(test_name, phase_title, failure_message, api_key=None, model=None):
    """
    Führt die Fehlerursachenanalyse mit Google Gemini durch.
    Gibt ein strukturiertes Markdown-Snippet zurück oder einen Fallback,
    falls kein API-Key verfügbar ist.
    """
    sanitized_error = sanitize_log(failure_message)
    if not sanitized_error:
        sanitized_error = "Keine detaillierte Fehlermeldung vorhanden."

    api_key = api_key or os.environ.get("GEMINI_API_KEY")
    if not api_key:
        return (
            f"> ℹ️ *KI-Fehlerdiagnose nicht verfügbar (kein `GEMINI_API_KEY` gesetzt).*\n\n"
            f"**Fehlermeldung (bereinigt):**\n```text\n{sanitized_error}\n```"
        )

    user_prompt = (
        f"TESTCASE: {test_name}\n"
        f"PHASE: {phase_title}\n"
        f"FEHLER-LOG / TRACEBACK:\n```text\n{sanitized_error}\n```\n\n"
        f"Erstelle die Diagnose gemäß dem definierten Schema."
    )

    payload = {
        "contents": [
            {
                "parts": [
                    {"text": SYSTEM_PROMPT},
                    {"text": user_prompt}
                ]
            }
        ],
        "generationConfig": {
            "temperature": 0.2,
            "maxOutputTokens": 1024
        }
    }

    preferred_model = model or os.environ.get("GEMINI_MODEL", GEMINI_MODEL)
    models_to_try = [preferred_model]
    for fallback in ["gemini-3.8-flash", "gemini-3.7-flash", "gemini-3.5-flash", "gemini-flash-latest"]:
        if fallback not in models_to_try:
            models_to_try.append(fallback)

    last_error = None
    for m in models_to_try:
        url = f"https://generativelanguage.googleapis.com/v1beta/models/{m}:generateContent"
        req = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "x-goog-api-key": api_key
            }
        )

        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                text = data["candidates"][0]["content"]["parts"][0]["text"].strip()
                return f"{text}\n\n*Diagnose generiert mit `{m}`*"
        except urllib.error.HTTPError as e:
            error_body = e.read().decode("utf-8", errors="ignore")
            last_error = f"HTTP {e.code}: {error_body}"
            if e.code == 429:
                time.sleep(1)
                continue
            elif e.code in (404, 500, 502, 503):
                continue
            else:
                break
        except Exception as e:
            last_error = str(e)
            break

    return (
        f"> ⚠️ *KI-Diagnose fehlgeschlagen ({last_error})*\n\n"
        f"**Fehlermeldung (bereinigt):**\n```text\n{sanitized_error}\n```"
    )
