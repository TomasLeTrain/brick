#pragma once
#include "Arduino.h"
#include <map>
#include <memory>

#define FILE_READ "r"
#define FILE_WRITE "w"
extern std::map<std::string, std::vector<uint8_t>> host_files;

namespace fs {
class File {
public:
  File() = default;
  File(const std::string &path, bool write) {
    flash_operation();
    if (write)
      host_files[path].clear();
    if (host_files.count(path))
      handle = std::make_shared<Handle>(Handle{path, 0});
  }
  explicit operator bool() const { return bool(handle); }
  size_t read(uint8_t *out, size_t length) {
    if (!handle) return 0;
    flash_operation();
    auto &data = host_files.at(handle->path);
    length = std::min(length, data.size() - handle->position);
    std::memcpy(out, data.data() + handle->position, length);
    handle->position += length;
    return length;
  }
  size_t write(const uint8_t *in, size_t length) {
    if (!handle) return 0;
    flash_operation();
    auto &data = host_files.at(handle->path);
    data.resize(std::max(data.size(), handle->position + length));
    std::memcpy(data.data() + handle->position, in, length);
    handle->position += length;
    return length;
  }
  size_t size() const {
    flash_operation();
    return host_files.at(handle->path).size();
  }
  size_t available() const { return handle ? size() - handle->position : 0; }
  bool seek(size_t position) {
    flash_operation();
    if (!handle) return false;
    handle->position = position;
    return true;
  }
  void flush() { if (handle) flash_operation(); }
  void close() { if (handle) flash_operation(); handle.reset(); }
private:
  struct Handle { std::string path; size_t position; };
  std::shared_ptr<Handle> handle;
};
}
