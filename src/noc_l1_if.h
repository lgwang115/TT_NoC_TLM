#ifndef NOC_L1_IF_H
#define NOC_L1_IF_H

#include <cstddef>
#include <cstdint>

class NocL1MemoryIf {
 public:
  virtual ~NocL1MemoryIf() = default;
  virtual size_t size() const = 0;
  virtual void read(uint64_t addr, uint8_t* dst, size_t len) const = 0;
  virtual void write(uint64_t addr, const uint8_t* src, size_t len) = 0;
};

#endif
