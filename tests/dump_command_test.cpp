#include "../common/dump_command.h"
#include <cassert>
#include <string>
int main() {
  DumpCommand parser;
  for (const auto &line : {"Slave connected\n", "# dump filesystem ready\r\n", "export\n", "eeee\n"})
    for (char byte : std::string(line)) assert(parser.feed(byte) == 0);
  assert(parser.feed('e') == 0);
  assert(parser.feed('\n') == 'e');
  assert(parser.feed('\n') == 0);
  assert(parser.feed('s') == 0);
  assert(parser.feed('\r') == 's');
  assert(parser.feed('\n') == 0);
  parser.feed('e'); parser.reset();
  assert(parser.feed('\n') == 0);
  for (unsigned i = 0; i < 100000; ++i) assert(parser.feed('e') == 0);
  assert(parser.feed('\n') == 0);
}
