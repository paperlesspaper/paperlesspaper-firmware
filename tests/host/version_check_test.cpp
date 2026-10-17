#include "version_check.h"
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

using version_check::ResponseState;
using version_check::Result;
#define CHECK(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (false)

struct Clock {
   uint64_t now = 0;
   uint64_t alarmAt = UINT64_MAX;
   std::function<void()> alarm;
   void advance(uint32_t ms) {
      now += ms;
      if (alarm && now >= alarmAt) { auto callback = alarm; alarm = nullptr; callback(); }
   }
   uint32_t millis() const { return static_cast<uint32_t>(now); }
};

struct Harness {
   Clock clock;
   ResponseState state = ResponseState::Pending;
   int version = 41;
   struct Client {
      Harness& owner;
      explicit Client(Harness& harness) : owner(harness) {}
      bool online = true;
      int timeout = 1000;
      unsigned calls = 0;
      uint32_t workMs = 0;
      std::function<bool()> onLoop;
      bool connected() const { return online; }
      void setTimeout(int ms) { timeout = ms; }
      bool loop() {
         ++calls;
         CHECK(timeout > 0 && timeout <= 1000);
         owner.clock.advance(workMs < static_cast<uint32_t>(timeout) ? workMs : timeout);
         return onLoop ? onLoop() : true;
      }
   } client{*this};
   void receive(const char* payload) {
      JsonDocument doc;
      if (!deserializeJson(doc, payload)) version_check::receiveVersion(doc["t"], state, version);
   }
   Result wait(int stored = 41) {
      const auto result = version_check::waitForVersion(client, state, version, stored,
          [this]() { return clock.millis(); }, [this](uint32_t ms) { clock.advance(ms); });
      CHECK(client.timeout == 1000);
      return result;
   }
};

// The Python runner extracts these functions/branches from the actual main.cpp.
// Do not copy their implementations into the test.
#include "version_check_integration.inc"

static unsigned passed = 0;
static void test(const char* name, const std::function<void()>& body) {
   body(); ++passed; std::cout << "PASS " << name << '\n';
}

int main() {
   test("immediate newer response, even if disconnected afterwards", [] {
      Harness h; h.receive("{\"t\":42}"); h.client.online = false;
      CHECK(h.wait() == Result::UpdateAvailable); CHECK(h.client.calls == 0);
      CHECK(h.clock.now == 0 && h.state == ResponseState::Pending);
   });
   test("delayed reply is pumped by MQTT loop", [] {
      Harness h;
      h.client.onLoop = [&] { if (h.clock.now >= 170) h.receive("{\"t\":42}"); return true; };
      CHECK(h.wait() == Result::UpdateAvailable); CHECK(h.clock.now == 170);
   });
   test("unchanged version is a valid NoUpdate", [] {
      Harness h; h.receive("{\"t\":41}"); CHECK(h.wait() == Result::NoUpdate);
   });
   test("older version preserves the existing comparison", [] {
      Harness h; h.receive("{\"t\":40}"); CHECK(h.wait() == Result::NoUpdate);
   });
   test("zero remains a valid no-picture version", [] {
      Harness h; h.receive("{\"t\":0}"); CHECK(h.wait() == Result::NoUpdate);
      CHECK(h.version == 0);
   });
   test("missing reply stops at five seconds without reconnecting", [] {
      Harness h; CHECK(h.wait() == Result::Timeout); CHECK(h.clock.now == 5000);
      CHECK(h.client.calls == 500 && h.version == 41);
   });
   test("missing t in settings is not an invalid version", [] {
      Harness h; h.receive("{\"act\":\"activated\"}");
      CHECK(h.wait() == Result::Timeout); CHECK(h.clock.now == 5000);
   });
   test("already disconnected exits immediately", [] {
      Harness h; h.client.online = false; CHECK(h.wait() == Result::Disconnected);
      CHECK(h.client.calls == 0 && h.clock.now == 0);
   });
   test("connection lost during polling exits without retry", [] {
      Harness h; h.client.onLoop = [&] {
         if (h.clock.now >= 100) { h.client.online = false; return false; } return true;
      };
      CHECK(h.wait() == Result::Disconnected); CHECK(h.clock.now == 100);
   });
   test("MQTT processing error while transport is connected", [] {
      Harness h; h.client.onLoop = [] { return false; };
      CHECK(h.wait() == Result::MqttError); CHECK(h.client.calls == 1);
   });
   test("valid response survives subsequent loop/keepalive failure", [] {
      Harness h; h.client.onLoop = [&] { h.receive("{\"t\":42}"); return false; };
      CHECK(h.wait() == Result::UpdateAvailable);
   });
   for (const auto* payload : {"{\"t\":null}", "{\"t\":true}", "{\"t\":\"42\"}",
            "{\"t\":1.5}", "{\"t\":-1}", "{\"t\":2147483648}", "{\"t\":{}}", "{\"t\":[]}"}) {
      test((std::string("reject invalid version: ") + payload).c_str(), [=] {
         Harness h; h.receive(payload); CHECK(h.wait() == Result::InvalidResponse);
         CHECK(h.version == 41 && h.clock.now == 5000);
      });
   }
   test("malformed JSON alone is not attributable to a version", [] {
      Harness h; h.receive("{bad"); CHECK(h.state == ResponseState::Pending);
      CHECK(h.wait() == Result::Timeout && h.clock.now == 5000);
   });
   test("maximum signed 32-bit version is valid", [] {
      Harness h; h.receive("{\"t\":2147483647}"); CHECK(h.wait() == Result::UpdateAvailable);
   });
   test("malformed message cannot discard an already valid version", [] {
      Harness h; h.receive("{\"t\":42}"); h.receive("{bad");
      CHECK(h.wait() == Result::UpdateAvailable && h.version == 42);
   });
   test("valid response replaces an earlier invalid response", [] {
      Harness h; h.receive("{\"t\":null}"); h.receive("{\"t\":42}");
      CHECK(h.wait() == Result::UpdateAvailable);
   });
   test("I/O budget and pause cannot extend polling beyond deadline", [] {
      Harness h; h.client.workMs = 2000;
      CHECK(h.wait() == Result::Timeout); CHECK(h.clock.now == 5000);
      CHECK(h.client.calls < 50);
   });
   test("millis wraparound keeps the five-second budget", [] {
      Harness h; h.clock.now = UINT32_MAX - 200;
      const auto start = h.clock.now;
      CHECK(h.wait() == Result::Timeout); CHECK(h.clock.now - start == 5000);
   });
   test("consumed reply is not reused by another check", [] {
      Harness h; h.receive("{\"t\":42}"); CHECK(h.wait() == Result::UpdateAvailable);
      CHECK(h.wait() == Result::Timeout);
   });
   test("actual getVersionUpdate wrapper uses the poller", [] {
      integrationHarness = new Harness;
      integrationHarness->client.onLoop = [] {
         if (integrationHarness->clock.now >= 80) integrationHarness->receive("{\"t\":42}");
         return true;
      };
      CHECK(getVersionUpdate() == Result::UpdateAvailable);
      CHECK(integrationHarness->clock.now == 80);
      delete integrationHarness;
   });
   for (const auto result : {Result::Timeout, Result::Disconnected, Result::MqttError, Result::InvalidResponse}) {
      test("actual setup error branch preserves display/file path", [=] {
         injectedResult = result; normalPathCalls = 0;
         try { exerciseSetupVersionGate(); CHECK(false); }
         catch (const Sleep& sleep) {
            CHECK(sleep.seconds == 86400 && !sleep.showScreen && sleep.motionWake);
         }
         CHECK(normalPathCalls == 0);
      });
   }
   for (const auto result : {Result::UpdateAvailable, Result::NoUpdate}) {
      test("actual setup accepts a confirmed version", [=] {
         injectedResult = result; normalPathCalls = 0;
         exerciseSetupVersionGate(); CHECK(normalPathCalls == 1);
      });
   }
   test("existing failsafe can still interrupt version polling", [] {
      Harness h; h.clock.alarmAt = 2000; h.clock.alarm = [] { timeoutFailsave(0); };
      try { h.wait(); CHECK(false); }
      catch (const Sleep& sleep) { CHECK(sleep.seconds == 86400); }
      CHECK(h.clock.now == 2000 && h.client.timeout == 1000);
   });
   test("malformed unrelated packet does not abort delayed valid reply", [] {
      Harness h; bool sentBad = false;
      h.client.onLoop = [&] {
         if (!sentBad) { h.receive("{bad"); sentBad = true; }
         else if (h.clock.now >= 170) h.receive("{\"t\":42}");
         return true;
      };
      CHECK(h.wait() == Result::UpdateAvailable && h.clock.now == 170);
   });
   test("invalid version does not abort a later valid reply", [] {
      Harness h; bool sentBad = false;
      h.client.onLoop = [&] {
         if (!sentBad) { h.receive("{\"t\":null}"); sentBad = true; }
         else if (h.clock.now >= 170) h.receive("{\"t\":42}");
         return true;
      };
      CHECK(h.wait() == Result::UpdateAvailable && h.clock.now == 170);
   });
   for (const auto* payload : {"{\"url\":\"image.bin\"}", "{\"ota\":\"https://example.invalid/fw.json\"}",
            "{\"act\":\"activated\",\"timeout\":86400,\"overlay\":true}",
            "{\"act\":\"not_started\"}", "{\"act\":\"reset\"}",
            "{\"cmd\":\"wipe\"}", "{\"cmd\":\"bootloader\"}",
            "[]", "null", "true", "42", "\"unrelated\""}) {
      test((std::string("non-version envelope leaves wait pending: ") + payload).c_str(), [=] {
         Harness h; h.receive(payload); CHECK(h.state == ResponseState::Pending);
         h.client.onLoop = [&] { if (h.clock.now >= 70) h.receive("{\"t\":42}"); return true; };
         CHECK(h.wait() == Result::UpdateAvailable && h.clock.now == 70);
      });
   }
   for (const auto* payload : {"{\"act\":\"activated\",\"t\":42}", "{\"url\":\"image.bin\",\"t\":42}"}) {
      test("combined envelope accepts its valid version", [=] {
         Harness h; h.receive(payload); CHECK(h.wait() == Result::UpdateAvailable);
      });
   }
   test("mixed envelopes in a single MQTT poll preserve valid response", [] {
      Harness h; h.client.onLoop = [&] {
         h.receive("{bad"); h.receive("{\"act\":\"activated\"}");
         h.receive("{\"t\":null}"); h.receive("{\"t\":42}");
         h.receive("{\"url\":\"image.bin\"}"); h.receive("{\"t\":false}");
         h.receive("{bad"); return true;
      };
      CHECK(h.wait() == Result::UpdateAvailable && h.version == 42);
   });
   test("continuous invalid versions do not restart the deadline", [] {
      Harness h; h.client.onLoop = [&] { h.receive("{\"t\":null}"); return true; };
      CHECK(h.wait() == Result::InvalidResponse && h.clock.now == 5000);
      CHECK(h.client.calls == 500 && h.state == ResponseState::Pending);
   });
   test("continuous malformed packets do not restart the deadline", [] {
      Harness h; h.client.onLoop = [&] { h.receive("{bad"); return true; };
      CHECK(h.wait() == Result::Timeout && h.clock.now == 5000);
      CHECK(h.client.calls == 500);
   });
   test("transport failure after invalid field still exits promptly", [] {
      Harness h; h.client.onLoop = [&] {
         h.receive("{\"t\":null}"); h.client.online = false; return false;
      };
      CHECK(h.wait() == Result::Disconnected && h.clock.now == 0);
      CHECK(h.state == ResponseState::Pending);
   });
   test("a reply beyond the deadline does not trigger an implicit retry", [] {
      Harness h; h.client.onLoop = [&] {
         if (h.clock.now >= 5100) h.receive("{\"t\":42}");
         return true;
      };
      CHECK(h.wait() == Result::Timeout && h.clock.now == 5000 && h.version == 41);
   });
   std::cout << passed << " host scenarios passed\n";
}
