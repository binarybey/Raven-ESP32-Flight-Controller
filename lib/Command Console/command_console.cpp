#include "command_console.h"

#include <string.h>
#include <ctype.h>

namespace raven {

static Command lookup(const char *s) {
    if (strcmp(s, "arm") == 0)    return Command::ARM;
    if (strcmp(s, "disarm") == 0) return Command::DISARM;
    if (strcmp(s, "land") == 0)   return Command::LAND;
    if (strcmp(s, "status") == 0) return Command::STATUS;
    if (strcmp(s, "help") == 0 || strcmp(s, "?") == 0) return Command::HELP;
    return Command::UNKNOWN;
}

Command CommandConsole::poll() {
    while (io_.available() > 0) {
        const int c = io_.read();
        if (c < 0) break;
        if (c == '\r' || c == '\n') {
            if (n_ == 0 && !overflow_) continue;   // blank line / second half of CRLF
            while (n_ > 0 && (buf_[n_-1] == ' ' || buf_[n_-1] == '\t')) --n_;
            buf_[n_] = '\0';
            const bool was_overflow = overflow_;
            n_ = 0;
            overflow_ = false;
            return was_overflow ? Command::UNKNOWN : lookup(buf_);
        }
        if (n_ < sizeof(buf_) - 1) {
            if (!(n_ == 0 && c == ' ')) buf_[n_++] = (char)tolower(c);
        } else {
            overflow_ = true;
        }
    }
    return Command::NONE;
}

const char *commandHelpText() {
    return "Commands:\n"
           "  arm     - pre-arm checks, then arm (with a mission: TAKEOFF)\n"
           "  disarm  - MOTORS OFF IMMEDIATELY (also in flight)\n"
           "  land    - controlled landing at the current position\n"
           "  status  - status summary\n"
           "  help    - this text\n";
}

}  // namespace raven
