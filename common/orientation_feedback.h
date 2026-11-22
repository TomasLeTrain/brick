#pragma once
#include <cstdint>

// Application-task deduplication. Preserve across radio reconnects so a
// retained request doesn't flash again; a new sender session starts a new
// request stream.
class OrientationRequests {
public:
  bool accept(uint32_t session, uint8_t request, bool inverted) {
    const bool changed =
        !seen || session != last_session
            ? request != 0 || inverted != last_inverted
            : request != last_request || inverted != last_inverted;
    seen = true;
    last_session = session;
    last_request = request;
    last_inverted = inverted;
    return changed;
  }

private:
  bool seen = false, last_inverted = false;
  uint32_t last_session = 0;
  uint8_t last_request = 0;
};

// Motor-task-owned timer. New events replace the previous face's flash. Tokens
// are published atomically by Comms; bit 0 identifies the selected bottom face.
class OrientationFlash {
public:
  static constexpr uint32_t duration_ms = 300;
  void observe(uint32_t token, uint32_t now_ms) {
    if (!token || token == last_token)
      return;
    last_token = token;
    started_ms = now_ms;
  }
  bool active(bool bottom_face, uint32_t now_ms) const {
    return last_token && bool(last_token & 1u) == bottom_face &&
           uint32_t(now_ms - started_ms) < duration_ms;
  }

private:
  uint32_t last_token = 0, started_ms = 0;
};
