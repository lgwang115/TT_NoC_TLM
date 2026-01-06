#ifndef RISCV_CORE_H
#define RISCV_CORE_H

#include <systemc>

#include "noc_niu.h"

SC_MODULE(RiscvCore) {
  struct Config {
    bool enabled = true;
    unsigned mesh_width = 10;
    unsigned mesh_height = 12;
    int dst_dx = 3;
    int dst_dy = 2;
    uint32_t targ_base = 0x100;
    uint32_t ret_base = 0x200;
    uint32_t len_bytes = 16;
    uint32_t packet_tag = 0;
    uint32_t ctrl_read = 0;
    uint32_t ctrl_write = (2u << 0) | (1u << 3) | (1u << 4);
    uint32_t write_data = 0xA5A5A5A5u;
    uint32_t write_byte_enable = 0xFFFFu;
    bool do_write = true;
    bool do_read = true;
  };

  sc_core::sc_fifo_out<NiuMmioReq> mmio_out;
  sc_core::sc_fifo_in<NiuMmioResp> mmio_in;
  sc_core::sc_fifo_in<NiuResp> resp_in;

  sc_core::sc_time cycle_time;
  NocCoord coord;
  unsigned core_id = 0;

  explicit RiscvCore(sc_core::sc_module_name name,
                     sc_core::sc_time cycle,
                     NocCoord coord,
                     unsigned core_id);

  void configure(const Config &config);
  void run();

 private:
 static constexpr uint32_t kNocCmdBufOffset = 0x400;
  static constexpr uint32_t kNocTargAddrLo = 0x000;
  static constexpr uint32_t kNocTargAddrMid = 0x004;
  static constexpr uint32_t kNocRetAddrLo = 0x00C;
  static constexpr uint32_t kNocRetAddrMid = 0x010;
  static constexpr uint32_t kNocPacketTag = 0x018;
  static constexpr uint32_t kNocCtrl = 0x01C;
  static constexpr uint32_t kNocAtLenBe = 0x020;
  static constexpr uint32_t kNocAtData = 0x024;
  static constexpr uint32_t kNocCmdCtrl = 0x028;

  Config config_;
  void mmio_write(uint32_t addr, uint32_t data);
  uint32_t mmio_read(uint32_t addr);
  uint64_t pack_unicast(uint64_t addr, uint8_t x, uint8_t y) const;
};

#endif
