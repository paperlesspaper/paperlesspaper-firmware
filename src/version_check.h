#pragma once

#include <ArduinoJson.h>
#include <stdint.h>

namespace version_check {

constexpr uint32_t WAIT_TIMEOUT_MS = 5000;
constexpr uint32_t POLL_DELAY_MS = 10;
// MQTTClient defaults to 1000 ms; no other caller changes it in this firmware.
constexpr int MQTT_DEFAULT_TIMEOUT_MS = 1000;

enum class ResponseState { Pending, Valid, Invalid };
enum class Result { UpdateAvailable, NoUpdate, Timeout, Disconnected, MqttError, InvalidResponse };

inline void rejectResponse(ResponseState& state) {
   // A malformed version field must not discard an already valid reply.
   if (state != ResponseState::Valid) state = ResponseState::Invalid;
}

inline void receiveVersion(JsonVariantConst value, ResponseState& state, int& version) {
   if (value.isUnbound()) return;  // Settings/commands need not include a version.
   if (!value.is<int32_t>() || value.as<int32_t>() < 0) {
      rejectResponse(state);
      return;
   }
   version = value.as<int32_t>();
   state = ResponseState::Valid;
}

// Clock and pause are millis()/delay() on-device and a virtual clock in host tests.
// MQTT's own I/O budget is capped as well. Blocking application callbacks or a
// stalled TLS/RTOS implementation still require the independent device failsafe.
template <typename Client, typename Clock, typename Pause>
Result waitForVersion(Client& client, ResponseState& state, const int& version,
                      int storedVersion, Clock clock, Pause pause) {
   struct RestoreTimeout {
      Client& client;
      ~RestoreTimeout() { client.setTimeout(MQTT_DEFAULT_TIMEOUT_MS); }
   } restore{client};

   const auto finish = [&state](Result result) {
      state = ResponseState::Pending;
      return result;
   };
   const uint32_t started = clock();
   while (true) {
      if (state == ResponseState::Valid) {
         return finish(version > storedVersion ? Result::UpdateAvailable : Result::NoUpdate);
      }
      // An invalid field may precede a valid reply on this shared topic. Keep
      // polling within the original deadline; invalid traffic cannot extend it.
      uint32_t elapsed = static_cast<uint32_t>(clock() - started);
      if (elapsed >= WAIT_TIMEOUT_MS) {
         return finish(state == ResponseState::Invalid ? Result::InvalidResponse : Result::Timeout);
      }
      if (!client.connected()) return finish(Result::Disconnected);

      uint32_t remaining = WAIT_TIMEOUT_MS - elapsed;
      client.setTimeout(static_cast<int>(remaining < MQTT_DEFAULT_TIMEOUT_MS
                                           ? remaining : MQTT_DEFAULT_TIMEOUT_MS));
      const bool ok = client.loop();
      // A valid reply remains usable even if keepalive fails after receiving it.
      if (state == ResponseState::Valid) continue;
      if (!ok) return finish(client.connected() ? Result::MqttError : Result::Disconnected);

      elapsed = static_cast<uint32_t>(clock() - started);
      if (elapsed < WAIT_TIMEOUT_MS) {
         remaining = WAIT_TIMEOUT_MS - elapsed;
         pause(remaining < POLL_DELAY_MS ? remaining : POLL_DELAY_MS);
      }
   }
}

}  // namespace version_check
