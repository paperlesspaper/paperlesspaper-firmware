#!/usr/bin/env python3
"""
tools/test_ai_failure_analysis.py
Unit-Tests für tools/ai_failure_analysis.py:
Validiert die zuverlässige Redaktierung von sensiblen Daten (AWS Keys, Passwörter, Tokens,
SSIDs, Zertifikate und Env-Variablen) sowie das Verhalten der Fehleranalyse.
"""

import os
import unittest
from tools.ai_failure_analysis import sanitize_log, analyze_failure


class TestAIFailureAnalysis(unittest.TestCase):

    def setUp(self):
        # Setze Test-Umgebungsvariablen
        os.environ["TEST_HIL_WIFI_PASSWORD"] = "SuperSecretWiFiPass123"
        os.environ["TEST_AWS_SECRET_ACCESS_KEY"] = "wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY"
        os.environ["TEST_AUTH_TOKEN"] = "my_super_secret_auth_token_value_999"

    def tearDown(self):
        os.environ.pop("TEST_HIL_WIFI_PASSWORD", None)
        os.environ.pop("TEST_AWS_SECRET_ACCESS_KEY", None)
        os.environ.pop("TEST_AUTH_TOKEN", None)

    def test_aws_access_key_redaction(self):
        sample = "Error connecting with AKIAIOSFODNN7EXAMPLE and secret."
        redacted = sanitize_log(sample)
        self.assertNotIn("AKIAIOSFODNN7EXAMPLE", redacted)
        self.assertIn("[REDACTED_AWS_KEY]", redacted)

    def test_env_var_secret_redaction(self):
        sample = "Failed with password SuperSecretWiFiPass123 on router."
        redacted = sanitize_log(sample)
        self.assertNotIn("SuperSecretWiFiPass123", redacted)
        self.assertIn("[REDACTED_SECRET]", redacted)

    def test_bearer_and_jwt_redaction(self):
        sample = "Authorization: Bearer mySecretToken123456789 and JWT eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiIxMjM0NTY3ODkwIn0.doNotLeakThisSignaturePart123"
        redacted = sanitize_log(sample)
        self.assertNotIn("mySecretToken123456789", redacted)
        self.assertNotIn("doNotLeakThisSignaturePart123", redacted)
        self.assertIn("Bearer [REDACTED_TOKEN]", redacted)
        self.assertIn("[REDACTED_JWT]", redacted)

    def test_wifi_and_credentials_log_redaction(self):
        sample = 'SSID: MyHomeNetwork, Passwort: secretpassword, "password": "hidden_pass", "ssid": "MyGuestWifi"'
        redacted = sanitize_log(sample)
        self.assertNotIn("secretpassword", redacted)
        self.assertNotIn("hidden_pass", redacted)
        self.assertNotIn("MyGuestWifi", redacted)

    def test_certificate_block_redaction(self):
        sample = "-----BEGIN RSA PRIVATE KEY-----\nMIIEowIBAAKCAQEA0...\n-----END RSA PRIVATE KEY-----"
        redacted = sanitize_log(sample)
        self.assertNotIn("MIIEowIBAAKCAQEA0", redacted)
        self.assertIn("[REDACTED_CERTIFICATE]", redacted)

    def test_basic_auth_url_redaction(self):
        sample = "Fetching from https://admin:supersecret@example.com/api"
        redacted = sanitize_log(sample)
        self.assertNotIn("admin:supersecret", redacted)
        self.assertIn("[REDACTED_USER]:[REDACTED_PASS]@", redacted)

    def test_mac_uid_and_path_redaction(self):
        sample = "Device epd7-4c7525df8abc (MAC: 4c:75:25:df:8a:bc, BLE: EPD7_DF8ABC) failed at C:\\Users\\runneradmin\\workspace\\test.py"
        redacted = sanitize_log(sample)
        self.assertNotIn("4c7525df8abc", redacted)
        self.assertNotIn("4c:75:25:df:8a:bc", redacted)
        self.assertNotIn("EPD7_DF8ABC", redacted)
        self.assertNotIn("runneradmin", redacted)
        self.assertIn("[REDACTED_UID]", redacted)
        self.assertIn("[REDACTED_MAC]", redacted)
        self.assertIn("[REDACTED_DEVICE]", redacted)
        self.assertIn("<runner_root>\\", redacted)

    def test_length_clipping(self):
        large_text = "A" * 6000
        redacted = sanitize_log(large_text)
        self.assertLessEqual(len(redacted), 4100)
        self.assertIn("[... Log für KI-Analyse gekürzt ...]", redacted)

    def test_analyze_failure_without_api_key(self):
        # Stellt sicher, dass ohne API-Key kein Fehler auftritt, sondern ein sauberer Fallback
        res = analyze_failure(
            test_name="test_03_candidate_firmware_ota",
            phase_title="Phase 3: Kandidaten-Firmware OTA",
            failure_message="TimeoutError: [EPD7] Timeout (90s) with secret SuperSecretWiFiPass123",
            api_key=""
        )
        self.assertIn("KI-Fehlerdiagnose nicht verfügbar", res)
        self.assertNotIn("SuperSecretWiFiPass123", res)
        self.assertIn("[REDACTED_SECRET]", res)

    def test_generate_markdown_with_failure(self):
        from tools.generate_test_protocol import generate_markdown
        epd7_data = {
            "file": "junit_epd7.xml",
            "total": 7,
            "passed": 6,
            "failed": 1,
            "skipped": 0,
            "duration": 120.0,
            "cases": [
                {
                    "name": "test_03_candidate_firmware_ota",
                    "phase": "Phase 3: Kandidaten-Firmware OTA",
                    "classname": "testbench.test_epd_lifecycle.TestEPD7Lifecycle",
                    "duration": 90.0,
                    "status": "FAILED",
                    "message": "TimeoutError: [EPD7] Timeout (90s) beim Warten auf Muster '\\\\[MAIN\\\\] INIT Device V:\\\\s*([^\\\\s]+)'! with password SuperSecretWiFiPass123"
                }
            ]
        }
        risk_data = {
            "available": True,
            "rating": "GERING (LOW)",
            "recommendation": "GENEHMIGT",
            "diff_summary": "- Erhöhung des Timeouts für OTA",
            "raw_text": "# Vollständiger Audit-Report\nAlles in Ordnung."
        }

        md = generate_markdown(
            fw_version="b3.0.65",
            commit_sha="abcdef123456",
            deploy_status="success",
            epd7_data=epd7_data,
            epd13_data=None,
            risk_data=risk_data
        )

        # Verifiziere Struktur
        self.assertIn("🚨 Fehlgeschlagene Tests & 🤖 KI-Fehlerursachenanalyse", md)
        self.assertIn("Phase 3: Kandidaten-Firmware OTA", md)
        self.assertIn("KI-Fehlerdiagnose", md)
        # Verifiziere Secret-Maskierung im Markdown
        self.assertNotIn("SuperSecretWiFiPass123", md)
        self.assertIn("[REDACTED_SECRET]", md)
        # Verifiziere kompakte Risikoanalyse mit ausklappbarem Vollbericht
        self.assertIn("Pre-Flight KI-Risikoanalyse (Zusammenfassung)", md)
        self.assertIn("Vollständigen Pre-Flight Audit-Bericht anzeigen", md)


if __name__ == "__main__":
    unittest.main()
