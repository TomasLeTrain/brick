#pragma once

// Only a complete, single-letter command line is actionable. Text from startup,
// terminal echo, and stale partial input must not turn an embedded 'e' into an
// export request. No allocation and bounded state for arbitrarily long lines.
class DumpCommand {
public:
  char feed(char byte) {
    if (byte == '\r' || byte == '\n') {
      const char result = length == 1 && !invalid ? command : 0;
      reset();
      return result;
    }
    if (byte == ' ' || byte == '\t') {
      if (length) invalid = true;
      return 0;
    }
    if (length) invalid = true;
    else {
      command = byte;
      invalid = byte != 's' && byte != 'p' && byte != 'e' && byte != 'f' && byte != 'x' &&
                byte != 'a' && byte != 't' && byte != 'n';
      length = 1;
    }
    return 0;
  }
  void reset() { command = 0; length = 0; invalid = false; }
private:
  char command = 0;
  unsigned length = 0;
  bool invalid = false;
};
