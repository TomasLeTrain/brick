#include "../common/dump_command.h"
#include "../common/packets.h"
#include "../common/radio_config.h"
#include "DumpReceiver.h"
#include "IBusBM.h"
#include "NowPair.h"
#include "SenderConsole.h"
#include <esp_system.h>

#define HEARTBEAT_MS 50
#define TIMEOUT_MS 250

NowPair peer_link(NowPair::Role::Master, RADIO_CHANNEL, HEARTBEAT_MS,
                  TIMEOUT_MS);

IBusBM IBus;
DumpReceiver dump_receiver;
SenderConsole console;
uint32_t control_session = 0, control_sequence = 0;

void connectionChanged(bool connected, void *) {
  console.status(connected ? "Slave connected" : "Slave disconnected");
}

void received(const uint8_t *data, size_t length, int, uint32_t, void *) {
  if (dump_receiver.receive(data, length))
    return;
  TelemetryPacket packet;
  if (packet.deserialize(data, length)) {
    console.receive(packet);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  IBus.begin(Serial0, IBUSBM_NOTIMER);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // needed before any networking or else there is a slight disconnect at
  // beginning
  delay(1000);

  control_session = esp_random();
  if (!control_session)
    control_session = 1;
  peer_link.expectPeer(ROBOT_MAC);
  peer_link.wakeOnSend(xTaskGetCurrentTaskHandle());
  peer_link.onConnectionChange(connectionChanged, NULL);
  peer_link.onData(received, NULL);
  // Shorten unicast airtime so a frame can usually complete within one tick.
  // Discovery broadcasts continue at the default rate.
  const esp_now_rate_config_t tx_rate = {WIFI_PHY_MODE_11G, WIFI_PHY_RATE_6M,
                                         false, false};
  peer_link.setRate(&tx_rate);
  peer_link.setRssiStreaming(true);
  if (!peer_link.begin()) {
    Serial.println("ESP-NOW initialization failed");
    while (true)
      delay(1000);
  }
  Serial.println("Broadcasting discovery; waiting for slave...");
  Serial.println("# commands s/p/e/f/x; heading a=accel-only t=translation-on "
                 "n=off; press Enter");
}

uint32_t last_cnt_rec_update = 0;
uint8_t last_cnt_rec = 0;

void loop() {
  IBus.loop();
  sendThrottle();
  for (unsigned budget = 0; budget < 128 && Serial.available(); ++budget) {
    const char *line = console.readLine(Serial.read());
    if (!line)
      continue;
    if (line[0] == '@') {
      if (!console.companion.receive(line, millis()))
        console.status("invalid/stale companion frame");
    } else if (line[1] == '\0' && std::strchr("spefxatn", line[0])) {
      const char command = line[0];
      if (command == 'x')
        dump_receiver.cancel();
      uint8_t request[RunDump::prefix_size + 1];
      RunDump::prefix(request, RunDump::Message::Command, 0, 0);
      request[RunDump::prefix_size] = command;
      if (!peer_link.sendData(request, sizeof(request)))
        console.status("command queue full/disconnected; retry");
    }
  }
  peer_link.update();
  // Finish any console line before allowing CSV bytes onto the same stream.
  const bool console_busy = console.printing();
  dump_receiver.update(peer_link, !console_busy);
  console.update(peer_link,
                 dump_receiver.receiving() || dump_receiver.printing());

  // Send completion wakes this task promptly; avoid losing every other tick.
  // Still give the idle task time at least every 10 ms under continuous
  // traffic.
  static uint32_t last_yield_ms = 0;
  if (millis() - last_yield_ms >= 10) {
    last_yield_ms = millis();
    vTaskDelay(1);
  } else
    ulTaskNotifyTake(pdTRUE, 1);
}

void sendThrottle() {
  constexpr uint32_t throttle_interval_ms = 20;
  static uint32_t last_send_ms = 0;
  const uint32_t now_ms = millis();
  if (!peer_link.connected() || now_ms - last_send_ms < throttle_interval_ms)
    return;

  digitalWrite(LED_BUILTIN, HIGH);
  ThrottlePacket packet{};
  packet.session = control_session;
  packet.sequence = ++control_sequence;

  // 0 - right, left-right
  // 1 - right, up-down
  // 2 - left, left-right
  // 3 - left, up-down
  // 4 - vra
  // 5 - vrb
  packet.right_lr = std::clamp(int(IBus.readChannel(0)) - 1000, 0, 1000);
  packet.right_ud = std::clamp(int(IBus.readChannel(1)) - 1000, 0, 1000);

  packet.left_lr = std::clamp(int(IBus.readChannel(3)) - 1000, 0, 1000);
  packet.left_ud = std::clamp(int(IBus.readChannel(2)) - 1000, 0, 1000);

  packet.knob_left = std::clamp(int(IBus.readChannel(4)) - 1000, 0, 1000);
  packet.knob_right = std::clamp(int(IBus.readChannel(5)) - 1000, 0, 1000);

  // Serial.printf("swa:%d,swb:%d,swc:%d,swd:%d\n",IBus.readChannel(6),IBus.readChannel(7),IBus.readChannel(8),IBus.readChannel(9));

  packet.swa = (IBus.readChannel(6) - 1000) > 500;
  packet.swb = (IBus.readChannel(7) - 1000) > 500;
  const uint16_t swd_channel = IBus.readChannel(9);
  // packet.swc = IBus.readChannel(4) - 1000;
  // packet.swd = IBus.readChannel(5) - 1000;

  // Serial.printf("bruh: %d\n",packet.swa);

  // assume connected, updated later
  packet.connected = true;

  // TODO: on controller dc, sometimes toggles between frozen values / failsafe
  // values for a couple secs, then goes back to normal

  // two failsafe methods:
  // - check if no signal (through cnt_rec)
  // - check if controller is not connected (through failsafe values)

  // failsafe values
  // NOTE: MUST KEEP UPDATED AT ALL TIMES
  uint16_t fail_l_lr = 177;
  uint16_t fail_l_ud = 2;
  uint16_t fail_r_lr = 690;
  uint16_t fail_r_ud = 327;
  uint16_t fail_kl = 255;
  uint16_t fail_kr = 794;

  int fail_cnt = 0;
  if (fail_l_lr == packet.left_lr)
    fail_cnt++;
  if (fail_l_ud == packet.left_ud)
    fail_cnt++;
  if (fail_r_lr == packet.right_lr)
    fail_cnt++;
  if (fail_r_ud == packet.right_ud)
    fail_cnt++;
  if (fail_kl == packet.knob_left)
    fail_cnt++;
  if (fail_kr == packet.knob_right)
    fail_cnt++;

  if (fail_cnt >= 5) {
    // more than likely a disconnect, indicate such
    packet.connected = false;
  }

  if (IBus.cnt_rec != last_cnt_rec) {
    last_cnt_rec = IBus.cnt_rec;
    last_cnt_rec_update = millis();
  }

  // way too long without any new packets, disconnect
  if (millis() - last_cnt_rec_update > 100) {
    packet.connected = false;
  }

  console.companion.observe_radio_orientation(
      swd_channel > 1500,
      packet.connected && swd_channel >= 1000 && swd_channel <= 2000);
  console.companion.apply(packet, millis());

  uint8_t data_buffer[NowPair::MAX_PAYLOAD];
  const size_t length = packet.serialize(data_buffer);
  if (peer_link.sendLatestData(data_buffer, length))
    last_send_ms = now_ms;
}
