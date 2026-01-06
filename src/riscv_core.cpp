#include "riscv_core.h"

#include <iostream>

RiscvCore::RiscvCore(sc_core::sc_module_name name,
                     sc_core::sc_time cycle,
                     NocCoord coord_in,
                     unsigned core)
    : sc_module(name), cycle_time(cycle), coord(coord_in), core_id(core) {
  SC_THREAD(run);
}

void RiscvCore::configure(const Config &config) { config_ = config; }

void RiscvCore::mmio_write(uint32_t addr, uint32_t data) {
  NiuMmioReq req;
  req.write = true;
  req.addr = addr;
  req.data = data;
  mmio_out.write(req);
}

uint32_t RiscvCore::mmio_read(uint32_t addr) {
  NiuMmioReq req;
  req.write = false;
  req.addr = addr;
  req.data = 0;
  mmio_out.write(req);
  return mmio_in.read().data;
}

uint64_t RiscvCore::pack_unicast(uint64_t addr, uint8_t x, uint8_t y) const {
  uint64_t val = addr & ((1ULL << 36) - 1);
  val |= (static_cast<uint64_t>(x) & 0x3Fu) << 36;
  val |= (static_cast<uint64_t>(y) & 0x3Fu) << 42;
  return val;
}

void RiscvCore::run() {
  if (!config_.enabled) {
    return;
  }

  wait(cycle_time * 10);

  uint8_t dst_x = static_cast<uint8_t>(
      (coord.x + config_.dst_dx + config_.mesh_width) % config_.mesh_width);
  uint8_t dst_y = static_cast<uint8_t>(
      (coord.y + config_.dst_dy + config_.mesh_height) % config_.mesh_height);

  uint64_t targ = pack_unicast(config_.targ_base + core_id * 0x40, dst_x, dst_y);
  uint64_t ret = pack_unicast(config_.ret_base + core_id * 0x40, coord.x, coord.y);

  if (config_.do_write) {
    mmio_write(kNocTargAddrLo, static_cast<uint32_t>(targ & 0xFFFFFFFFu));
    mmio_write(kNocTargAddrMid, static_cast<uint32_t>(targ >> 32));
    mmio_write(kNocRetAddrLo, static_cast<uint32_t>(ret & 0xFFFFFFFFu));
    mmio_write(kNocRetAddrMid, static_cast<uint32_t>(ret >> 32));
    mmio_write(kNocPacketTag, config_.packet_tag);
    mmio_write(kNocAtData, config_.write_data);
    mmio_write(kNocAtLenBe, config_.write_byte_enable);
    mmio_write(kNocCtrl, config_.ctrl_write);
    mmio_write(kNocCmdCtrl, 1);

    if (config_.ctrl_write & (1u << 4)) {
      NiuResp ack = resp_in.read();
      std::cout << sc_core::sc_time_stamp() << " core" << core_id
                << " write ack pkt " << ack.packet_id << std::endl;
    }
  }

  if (config_.do_read) {
    mmio_write(kNocTargAddrLo, static_cast<uint32_t>(targ & 0xFFFFFFFFu));
    mmio_write(kNocTargAddrMid, static_cast<uint32_t>(targ >> 32));
    mmio_write(kNocRetAddrLo, static_cast<uint32_t>(ret & 0xFFFFFFFFu));
    mmio_write(kNocRetAddrMid, static_cast<uint32_t>(ret >> 32));
    mmio_write(kNocPacketTag, config_.packet_tag);
    mmio_write(kNocAtLenBe, config_.len_bytes);
    mmio_write(kNocCtrl, config_.ctrl_read);
    mmio_write(kNocCmdCtrl, 1);

    NiuResp resp = resp_in.read();
    std::cout << sc_core::sc_time_stamp() << " core" << core_id
              << " read resp len " << resp.data.size() << std::endl;
    if (!resp.data.empty()) {
      std::cout << "core" << core_id << " read data:";
      for (size_t i = 0; i < resp.data.size(); ++i) {
        std::cout << " " << std::hex << static_cast<int>(resp.data[i]) << std::dec;
      }
      std::cout << std::endl;
    }
  }
}
