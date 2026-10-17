#!/usr/bin/env python3
"""Host regression tests. No device, cloud, credentials, or flashing required.
Run: ARDUINOJSON_INCLUDE=/path/to/ArduinoJson/src python3 tests/test_version_check.py
The default parser path is populated by `pio pkg install -e epd13`.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src/main.cpp").read_text()


def function(signature):
    start = SOURCE.index(signature + " {")
    opening = SOURCE.index("{", start)
    depth = 0
    for pos in range(opening, len(SOURCE)):
        if SOURCE[pos] == "{":
            depth += 1
        elif SOURCE[pos] == "}":
            depth -= 1
            if depth == 0:
                return SOURCE[start:pos + 1]
    raise AssertionError("Unterminated function " + signature)


class VersionCheckTests(unittest.TestCase):
    def test_host_scenarios(self):
        parser = Path(os.environ.get("ARDUINOJSON_INCLUDE", str(
            ROOT / ".pio/libdeps/epd13/ArduinoJson/src")))
        self.assertTrue((parser / "ArduinoJson.h").is_file(),
                        "Set ARDUINOJSON_INCLUDE to the ArduinoJson 7.x src directory")
        gate = SOURCE[SOURCE.index("   const auto versionResult = getVersionUpdate();"):
                      SOURCE.index("   if (versionResult == version_check::Result::NoUpdate) {")]
        integration = r'''
struct SerialStub {
   template <typename T> void print(T) {}
   template <typename T> void println(T) {}
   template <typename... T> void printf(const char*, T...) {}
} Serial;
struct Sleep { int seconds; bool showScreen; bool motionWake; };
struct { int timeout = 86400; } settings;
struct { int sleepPrediction = 86400; } systemData;
constexpr int DEFAULT_SLEEP = 3600;
bool getActivatedFromMem() { return true; }
void gotToDeepSleep(int seconds, bool showScreen = true, bool motionWake = true) {
   throw Sleep{seconds, showScreen, motionWake};
}
Harness* integrationHarness;
#define client (integrationHarness->client)
#define versionResponseState (integrationHarness->state)
#define newVersionSave (integrationHarness->version)
uint32_t millis() { return integrationHarness->clock.millis(); }
void delay(uint32_t ms) { integrationHarness->clock.advance(ms); }
int readIntFromFlash(int address) { CHECK(address == 150); return 41; }
'''
        integration += function("version_check::Result getVersionUpdate(void)")
        integration += "\n#undef client\n#undef versionResponseState\n#undef newVersionSave\n"
        integration += function("void timeoutFailsave(int time)")
        integration += r'''
Result injectedResult;
unsigned normalPathCalls;
Result injectedVersionCheck() { return injectedResult; }
#define getVersionUpdate injectedVersionCheck
void exerciseSetupVersionGate() {
'''
        integration += gate + "\n ++normalPathCalls;\n}\n#undef getVersionUpdate\n"
        with tempfile.TemporaryDirectory(prefix="mqtt-version-test-") as temp:
            temp = Path(temp)
            (temp / "version_check_integration.inc").write_text(integration)
            binary = temp / "version-check-test"
            subprocess.run([os.environ.get("CXX", "g++"), "-std=c++11", "-Wall", "-Wextra",
                            "-pedantic", "-I" + str(ROOT / "src"), "-I" + str(parser),
                            "-I" + str(temp), str(ROOT / "tests/host/version_check_test.cpp"),
                            "-o", str(binary)], check=True, timeout=60)
            subprocess.run([str(binary)], check=True, timeout=10)

    def test_failsafe_is_still_armed_and_not_touched_by_version_wait(self):
        setup = SOURCE[SOURCE.index("void setup() {"):]
        armed = "tickerFailsave.once_ms(FAILSAVE_TIMER * 1000, timeoutFailsave, 0);"
        self.assertIn(armed, setup)
        self.assertLess(setup.index(armed), setup.index("getVersionUpdate();"))
        self.assertNotIn("tickerFailsave", function("version_check::Result getVersionUpdate(void)"))
        self.assertNotIn("tickerFailsave", (ROOT / "src/version_check.h").read_text())
        self.assertIn("#define FAILSAVE_TIMER 180", SOURCE)

    def test_handler_validates_json_and_version(self):
        handler = function("void iotReceiveHandler(String& topic, String& payload)")
        self.assertIn("if (deserializeJson(doc, payload))", handler)
        error_branch = handler[handler.index("if (deserializeJson(doc, payload))"):
                               handler.index('if (doc["url"]')]
        self.assertNotIn("version_check::rejectResponse", error_branch)
        self.assertIn('version_check::receiveVersion(doc["t"], versionResponseState, newVersionSave);', handler)


if __name__ == "__main__":
    unittest.main(verbosity=2)
