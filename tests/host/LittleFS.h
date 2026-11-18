#pragma once
#include "FS.h"
struct HostLittleFS {
  bool begin(bool) { flash_operation(); return true; }
  void end() { flash_operation(); }
  bool format() { flash_operation(); host_files.clear(); return true; }
  fs::File open(const char *path, const char *mode) {
    return fs::File(path, mode[0] == 'w');
  }
};
extern HostLittleFS LittleFS;
