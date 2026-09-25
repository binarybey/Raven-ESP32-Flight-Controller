// command_console.h - line-based operator commands over any Arduino Stream.
//
// Transport-agnostic on purpose: today it runs on the USB serial port; an RF
// link is just another Stream. A transparent UART radio (SiK 433/915 MHz
// telemetry pair, or a LoRa module in transparent mode) plugs in by creating
// a second console on its HardwareSerial - see TaskConsole in main.cpp. The
// parser doesn't care where the bytes come from.
//
// Commands (case-insensitive, one per line):
//   arm      run pre-arm checks, arm if they pass (mission: starts TAKEOFF)
//   disarm   MOTORS OFF IMMEDIATELY - also in flight. This is the kill switch.
//   land     controlled landing right here (FAILSAFE COMMANDED)
//   status   one-shot status summary
//   help     list commands

#pragma once

#include <Arduino.h>

namespace raven {

enum class Command : uint8_t { NONE, ARM, DISARM, LAND, STATUS, HELP, UNKNOWN };

class CommandConsole {
  public:
    explicit CommandConsole(Stream &io) : io_(io) {}

    // Non-blocking. Consumes whatever bytes are available and returns a
    // command once a complete line has arrived (NONE otherwise).
    Command poll();

    Stream &io() { return io_; }

  private:
    Stream &io_;
    char    buf_[24];
    uint8_t n_ = 0;
    bool    overflow_ = false;
};

const char *commandHelpText();

}  // namespace raven
