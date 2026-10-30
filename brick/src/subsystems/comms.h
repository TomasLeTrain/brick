#include "config.h"
#include "src/lib/NowPair.h"

struct TelemetryPacket {
  uint16_t a_x;
  uint16_t a_y;
  uint16_t a_z;

  // TODO: bit endianess?
  size_t serialize(uint8_t *buffer);
};

class Comms {
public:
  Comms();
  ~Comms();

  bool send(TelemetryPacket *packet);
  bool begin();
  void update();
  bool connected();

private:
  static void dataCb(const uint8_t *data, size_t length, void *arg);
  void on_data(const uint8_t *data, size_t length);
  void end();

  bool peerConnected = false;
  NowPair *peer_link = nullptr;
};
