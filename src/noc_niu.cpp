#include "noc_niu.h"

#include <algorithm>
#include <iostream>

namespace {
constexpr uint32_t kNocCmdBufOffset = 0x400;

constexpr uint32_t kNocTargAddrLo = 0x000;
constexpr uint32_t kNocTargAddrMid = 0x004;
constexpr uint32_t kNocRetAddrLo = 0x00C;
constexpr uint32_t kNocRetAddrMid = 0x010;
constexpr uint32_t kNocPacketTag = 0x018;
constexpr uint32_t kNocCtrl = 0x01C;
constexpr uint32_t kNocAtLenBe = 0x020;
constexpr uint32_t kNocAtData = 0x024;
constexpr uint32_t kNocCmdCtrl = 0x028;

constexpr uint32_t kNiuNodeId = 0x02C;
constexpr uint32_t kNiuEndpointId = 0x030;
constexpr uint32_t kNiuClearCounters = 0x050;
constexpr uint32_t kNiuCmdStatus = 0x054;

constexpr uint32_t kNiuCfg0 = 0x100;
constexpr uint32_t kRouterCfg0 = 0x104;
constexpr uint32_t kRouterCfg1 = 0x108;
constexpr uint32_t kRouterCfg2 = 0x10C;
constexpr uint32_t kRouterCfg3 = 0x110;
constexpr uint32_t kRouterCfg4 = 0x114;
constexpr uint32_t kNocXTranslate0 = 0x118;
constexpr uint32_t kNocYTranslate0 = 0x128;
constexpr uint32_t kNocIdLogical = 0x138;

constexpr uint32_t kCountersBase = 0x200;
constexpr uint32_t kCountersEnd = 0x2FF;
constexpr uint32_t kRouterDebugBase = 0x300;
constexpr uint32_t kRouterDebugEnd = 0x37F;
constexpr uint32_t kNiuDebugBase = 0x380;
constexpr uint32_t kNiuDebugEnd = 0x3FF;

constexpr size_t kCounterMstAtomicResp = 0;
constexpr size_t kCounterMstWrAck = 1;
constexpr size_t kCounterMstRdResp = 2;
constexpr size_t kCounterMstRdDataFlit = 3;
constexpr size_t kCounterMstCmdAccepted = 4;
constexpr size_t kCounterMstRdReqSent = 5;
constexpr size_t kCounterMstNonpostedAtomicSent = 6;
constexpr size_t kCounterMstPostedAtomicSent = 7;
constexpr size_t kCounterMstNonpostedWrDataFlitSent = 8;
constexpr size_t kCounterMstPostedWrDataFlitSent = 9;
constexpr size_t kCounterMstNonpostedWrReqSent = 10;
constexpr size_t kCounterMstPostedWrReqSent = 11;
constexpr size_t kCounterMstNonpostedWrReqStarted = 12;
constexpr size_t kCounterMstPostedWrReqStarted = 13;
constexpr size_t kCounterMstRdReqStarted = 14;
constexpr size_t kCounterMstNonpostedAtomicStarted = 15;
constexpr size_t kCounterMstReqsOutstandingBase = 16;
constexpr size_t kCounterMstWriteReqsOutgoingBase = 32;
constexpr size_t kCounterSlvAtomicRespSent = 48;
constexpr size_t kCounterSlvWrAckSent = 49;
constexpr size_t kCounterSlvRdRespSent = 50;
constexpr size_t kCounterSlvRdDataFlitSent = 51;
constexpr size_t kCounterSlvReqAccepted = 52;
constexpr size_t kCounterSlvRdReqReceived = 53;
constexpr size_t kCounterSlvNonpostedAtomicReceived = 54;
constexpr size_t kCounterSlvPostedAtomicReceived = 55;
constexpr size_t kCounterSlvNonpostedWrDataFlitReceived = 56;
constexpr size_t kCounterSlvPostedWrDataFlitReceived = 57;
constexpr size_t kCounterSlvNonpostedWrReqReceived = 58;
constexpr size_t kCounterSlvPostedWrReqReceived = 59;
constexpr size_t kCounterSlvNonpostedWrReqStarted = 60;
constexpr size_t kCounterSlvPostedWrReqStarted = 61;

struct AtomicDecoded {
  uint8_t opcode = 0;
  uint8_t ofs = 0;
  uint8_t int_width = 0;
  uint8_t cmp_val = 0;
  uint8_t set_val = 0;
  uint8_t mask = 0;
  uint8_t zaamo_op = 0;
};

AtomicDecoded decode_atomic(uint32_t raw) {
  AtomicDecoded out;
  out.opcode = static_cast<uint8_t>((raw >> 12) & 0xFu);
  out.ofs = static_cast<uint8_t>(raw & 0x3u);
  out.int_width = static_cast<uint8_t>((raw >> 2) & 0x1Fu);
  out.cmp_val = static_cast<uint8_t>((raw >> 2) & 0xFu);
  out.set_val = static_cast<uint8_t>((raw >> 6) & 0xFu);
  out.mask = static_cast<uint8_t>((raw >> 2) & 0xFFu);
  out.zaamo_op = static_cast<uint8_t>((raw >> 8) & 0x7u);
  return out;
}

// ============================================================================
// Address Alignment Validation per Spec
// ============================================================================

// Memory region types for alignment validation
enum class MemRegion {
  L1,      // L1 scratchpad (0x00000000 - 0x0016FFFF)
  MMIO,    // MMIO registers (0xFFB00000+)
  DRAM,    // External DRAM
  PCIe,    // Host PCIe
  Other    // Other memory types
};

// Determine memory region from address
MemRegion get_mem_region(uint64_t addr) {
  // L1 scratchpad: 0x00000000 - 0x0016FFFF (1464 KiB)
  if (addr < 0x00170000) {
    return MemRegion::L1;
  }
  // MMIO region: 0xFFB00000+
  if (addr >= 0xFFB00000) {
    return MemRegion::MMIO;
  }
  // For simplicity, treat other addresses as "Other"
  // In a full implementation, would check for DRAM/PCIe based on coordinates
  return MemRegion::Other;
}

// Check if address is aligned to given boundary
bool is_aligned(uint64_t addr, uint32_t alignment) {
  return (addr % alignment) == 0;
}

// Check if two addresses are congruent mod N (same offset within alignment)
bool is_congruent(uint64_t addr1, uint64_t addr2, uint32_t mod) {
  return (addr1 % mod) == (addr2 % mod);
}

// Alignment validation result
struct AlignmentResult {
  bool valid = true;
  const char* error_msg = nullptr;
};

// Validate alignment for length-based transfers (not byte-enable)
// Per spec alignment table
AlignmentResult validate_length_transfer(uint64_t src_addr, uint64_t dst_addr,
                                          uint32_t length, MemRegion src_type,
                                          MemRegion dst_type) {
  AlignmentResult result;

  // L1 -> L1: Addresses must be congruent mod 16
  if (src_type == MemRegion::L1 && dst_type == MemRegion::L1) {
    if (!is_congruent(src_addr, dst_addr, 16)) {
      result.valid = false;
      result.error_msg = "L1->L1: addresses not congruent mod 16";
    }
    return result;
  }

  // L1 -> MMIO: Dest must be 4-byte aligned, congruent mod 16
  if (src_type == MemRegion::L1 && dst_type == MemRegion::MMIO) {
    if (!is_aligned(dst_addr, 4)) {
      result.valid = false;
      result.error_msg = "L1->MMIO: dest not 4-byte aligned";
    } else if (!is_congruent(src_addr, dst_addr, 16)) {
      result.valid = false;
      result.error_msg = "L1->MMIO: addresses not congruent mod 16";
    }
    return result;
  }

  // MMIO -> L1: Length <= 4 - (addr % 4), congruent mod 4
  if (src_type == MemRegion::MMIO && dst_type == MemRegion::L1) {
    uint32_t max_len = 4 - (src_addr % 4);
    if (length > max_len) {
      result.valid = false;
      result.error_msg = "MMIO->L1: length exceeds alignment boundary";
    } else if (!is_congruent(src_addr, dst_addr, 4)) {
      result.valid = false;
      result.error_msg = "MMIO->L1: addresses not congruent mod 4";
    }
    return result;
  }

  // MMIO -> MMIO: Must be 4-byte aligned
  if (src_type == MemRegion::MMIO && dst_type == MemRegion::MMIO) {
    if (!is_aligned(src_addr, 4) || !is_aligned(dst_addr, 4)) {
      result.valid = false;
      result.error_msg = "MMIO->MMIO: addresses not 4-byte aligned";
    }
    return result;
  }

  // L1 -> Other (DRAM): Congruent mod 32
  if (src_type == MemRegion::L1 && dst_type == MemRegion::Other) {
    if (!is_congruent(src_addr, dst_addr, 32)) {
      result.valid = false;
      result.error_msg = "L1->Other: addresses not congruent mod 32";
    }
    return result;
  }

  // Other -> L1: Congruent mod 32
  if (src_type == MemRegion::Other && dst_type == MemRegion::L1) {
    if (!is_congruent(src_addr, dst_addr, 32)) {
      result.valid = false;
      result.error_msg = "Other->L1: addresses not congruent mod 32";
    }
    return result;
  }

  return result;  // Default: valid
}

// Validate alignment for byte-enable transfers
AlignmentResult validate_byte_enable_transfer(uint64_t src_addr, uint64_t dst_addr,
                                               bool is_inline, MemRegion src_type,
                                               MemRegion dst_type) {
  AlignmentResult result;

  // Inline -> L1: Dest must be 16-byte aligned
  if (is_inline && dst_type == MemRegion::L1) {
    if (!is_aligned(dst_addr, 16)) {
      result.valid = false;
      result.error_msg = "Inline->L1: dest not 16-byte aligned";
    }
    return result;
  }

  // Inline -> MMIO: Dest must be 4-byte aligned
  if (is_inline && dst_type == MemRegion::MMIO) {
    if (!is_aligned(dst_addr, 4)) {
      result.valid = false;
      result.error_msg = "Inline->MMIO: dest not 4-byte aligned";
    }
    return result;
  }

  // L1 -> L1 (byte-enable): Both must be 32-byte aligned
  if (src_type == MemRegion::L1 && dst_type == MemRegion::L1) {
    if (!is_aligned(dst_addr, 32) || !is_aligned(src_addr, 32)) {
      result.valid = false;
      result.error_msg = "L1->L1 (BE): addresses not 32-byte aligned";
    }
    return result;
  }

  // L1 -> MMIO (byte-enable): Dest 4-byte, congruent mod 16
  if (src_type == MemRegion::L1 && dst_type == MemRegion::MMIO) {
    if (!is_aligned(dst_addr, 4)) {
      result.valid = false;
      result.error_msg = "L1->MMIO (BE): dest not 4-byte aligned";
    } else if (!is_congruent(src_addr, dst_addr, 16)) {
      result.valid = false;
      result.error_msg = "L1->MMIO (BE): addresses not congruent mod 16";
    }
    return result;
  }

  // MMIO -> L1 (byte-enable): Dest 32-byte, src 4-byte aligned
  if (src_type == MemRegion::MMIO && dst_type == MemRegion::L1) {
    if (!is_aligned(dst_addr, 32)) {
      result.valid = false;
      result.error_msg = "MMIO->L1 (BE): dest not 32-byte aligned";
    } else if (!is_aligned(src_addr, 4)) {
      result.valid = false;
      result.error_msg = "MMIO->L1 (BE): src not 4-byte aligned";
    }
    return result;
  }

  // MMIO -> MMIO (byte-enable): Both 4-byte aligned
  if (src_type == MemRegion::MMIO && dst_type == MemRegion::MMIO) {
    if (!is_aligned(src_addr, 4) || !is_aligned(dst_addr, 4)) {
      result.valid = false;
      result.error_msg = "MMIO->MMIO (BE): addresses not 4-byte aligned";
    }
    return result;
  }

  return result;  // Default: valid
}

// Validate atomic operation alignment (must be 4-byte aligned for 32-bit atomics)
AlignmentResult validate_atomic_alignment(uint64_t addr) {
  AlignmentResult result;
  if (!is_aligned(addr, 4)) {
    result.valid = false;
    result.error_msg = "Atomic: address not 4-byte aligned";
  }
  return result;
}

}  // namespace

NocNiu::NocNiu(sc_core::sc_module_name name,
               NocCoord coord_in,
               NocId noc_id_in,
               sc_core::sc_time cycle,
               unsigned mesh_w,
               unsigned mesh_h,
               size_t mem_size_bytes)
    : sc_module(name),
      coord(coord_in),
      noc_id(noc_id_in),
      cycle_time(cycle),
      mesh_width(mesh_w),
      mesh_height(mesh_h),
      mem_(mem_size_bytes, 0),
      translation_(default_translation(noc_id_in)) {
  noc_id_logical_ = static_cast<uint32_t>((coord.y & 0x3Fu) << 6) |
                    static_cast<uint32_t>(coord.x & 0x3Fu);
  SC_THREAD(cmd_loop);
  SC_THREAD(rx_loop);
  SC_THREAD(mmio_loop);
  SC_THREAD(mmio_issue_loop);
}

size_t NocNiu::l1_size() const {
  return l1_mem_if_ ? l1_mem_if_->size() : mem_.size();
}

void NocNiu::l1_read(uint64_t addr, uint8_t* dst, size_t len) const {
  if (!dst || len == 0) {
    return;
  }
  if (l1_mem_if_) {
    l1_mem_if_->read(addr, dst, len);
    return;
  }
  std::copy(mem_.begin() + addr, mem_.begin() + addr + len, dst);
}

void NocNiu::l1_write(uint64_t addr, const uint8_t* src, size_t len) {
  if (!src || len == 0) {
    return;
  }
  if (l1_mem_if_) {
    l1_mem_if_->write(addr, src, len);
    return;
  }
  std::copy(src, src + len, mem_.begin() + addr);
}

void NocNiu::cmd_loop() {
  while (true) {
    NiuCmd cmd;
    bool have_cmd = false;
    if (cmd_in.nb_read(cmd)) {
      have_cmd = true;
    } else if (!pending_cmds_.empty()) {
      cmd = pending_cmds_.front();
      pending_cmds_.pop_front();
      have_cmd = true;
    }

    if (!have_cmd) {
      wait(cycle_time);
      continue;
    }

    NocPacket pkt;
    pkt.id = next_packet_id_++;
    pkt.src = coord;
    pkt.dst = cmd.coord_translate ? translate_coord(cmd.dst, translation_) : cmd.dst;
    pkt.class_bits = cmd.class_bits;
    pkt.buddy_bit = cmd.buddy_bit;
    pkt.vc_static = cmd.vc_static;
    pkt.vc_linked = cmd.vc_linked;
    pkt.linked_id = cmd.linked_id;
    pkt.coord_translate = cmd.coord_translate;
    pkt.is_response = false;
    pkt.posted = cmd.posted;
    pkt.req_type = cmd.type;
    pkt.addr = cmd.addr;
    pkt.ret_addr = cmd.addr;
    pkt.ret_coord = coord;
    pkt.atomic_imm = cmd.atomic_imm;
    pkt.is_broadcast = false;
    pkt.brcst_xy_major = false;

    if (cmd.type == NocReqType::Read) {
      pkt.length = cmd.length;
    } else {
      pkt.payload = cmd.data;
      pkt.length = static_cast<uint32_t>(pkt.payload.size());
    }

    send_packet(pkt);
  }
}

void NocNiu::rx_loop() {
  while (true) {
    NocFlit flit = net_in.read();
    if (flit.deliver_to_overlay && overlay_out.get_interface()) {
      overlay_out->write(flit);
    }
    auto &entry = inflight_[flit.packet_id];
    if (entry.received_flits == 0) {
      entry.packet.id = flit.packet_id;
      entry.packet.src = flit.src;
      entry.packet.dst = flit.dst;
      entry.packet.is_response = flit.is_response;
      entry.packet.posted = flit.posted;
      entry.packet.req_type = flit.req_type;
      entry.packet.addr = flit.addr;
      entry.packet.ret_addr = flit.ret_addr;
      entry.packet.ret_coord = flit.ret_coord;
      entry.packet.length = flit.length;
      entry.packet.atomic_imm = flit.atomic_imm;
      entry.packet.has_byte_enable = flit.has_byte_enable;
      entry.packet.byte_enable = flit.byte_enable;
      entry.packet.transaction_id = flit.transaction_id;
      entry.packet.packet_tag = flit.packet_tag;
      entry.packet.at_len_be_raw = flit.at_len_be_raw;
      entry.packet.stream_id = flit.stream_id;
      entry.packet.deliver_to_overlay = flit.deliver_to_overlay;
      entry.packet.msg_first = flit.msg_first;
      entry.packet.msg_last = flit.msg_last;
      entry.packet.header_store = flit.header_store;
      entry.packet.path_reserve = flit.path_reserve;
      entry.packet.arb_priority = flit.arb_priority;
      entry.packet.mem_rd_drop_ack = flit.mem_rd_drop_ack;
      if (flit.length > 0 &&
          (flit.req_type == NocReqType::Write || flit.is_response)) {
        entry.packet.payload.resize(flit.length);
      }
    }

    if (flit.is_header && flit.total_flits == 1 && !entry.packet.payload.empty()) {
      size_t copy_len = std::min<size_t>(flit.length, flit.data.size());
      for (size_t i = 0; i < copy_len; ++i) {
        entry.packet.payload[i] = flit.data[i];
      }
    } else if (!flit.is_header) {
      size_t offset = (flit.flit_index - 1) * 32;
      size_t copy_len = 0;
      if (offset < entry.packet.payload.size()) {
        copy_len = std::min<size_t>(32, entry.packet.payload.size() - offset);
      }
      for (size_t i = 0; i < copy_len; ++i) {
        entry.packet.payload[offset + i] = flit.data[i];
      }
    }

    entry.received_flits++;
    if (entry.received_flits == flit.total_flits) {
      if (entry.packet.is_response) {
        if (entry.packet.req_type == NocReqType::Read && !entry.packet.payload.empty()) {
          if (entry.packet.addr + entry.packet.payload.size() <= l1_size()) {
            l1_write(entry.packet.addr,
                     entry.packet.payload.data(),
                     entry.packet.payload.size());
          }
        }
        if (entry.packet.req_type == NocReqType::Read) {
          inc_counter(kCounterMstRdResp);
          inc_counter(kCounterMstRdDataFlit, static_cast<uint32_t>((entry.packet.payload.size() + 31) / 32));
          dec_counter(kCounterMstReqsOutstandingBase + entry.packet.transaction_id);
        } else if (entry.packet.req_type == NocReqType::Write) {
          inc_counter(kCounterMstWrAck);
          dec_counter(kCounterMstReqsOutstandingBase + entry.packet.transaction_id);
        } else if (entry.packet.req_type == NocReqType::Atomic) {
          inc_counter(kCounterMstAtomicResp);
          dec_counter(kCounterMstReqsOutstandingBase + entry.packet.transaction_id);
        }
        NiuResp resp;
        resp.packet_id = entry.packet.id;
        resp.type = entry.packet.req_type;
        resp.data = entry.packet.payload;
        resp_out.write(resp);
      } else {
        handle_request(entry.packet);
      }
      inflight_.erase(flit.packet_id);
    }
  }
}

void NocNiu::mmio_loop() {
  while (true) {
    NiuMmioReq req;
    if (!mmio_in.nb_read(req)) {
      wait(cycle_time);
      continue;
    }

    if (req.write) {
      mmio_write(req.addr, req.data);
    } else {
      NiuMmioResp resp;
      resp.data = mmio_read(req.addr);
      mmio_out.write(resp);
    }
  }
}

void NocNiu::mmio_issue_loop() {
  while (true) {
    if (pending_mmio_issues_.empty()) {
      wait(mmio_issue_event_);
      continue;
    }
    size_t idx = pending_mmio_issues_.front();
    pending_mmio_issues_.pop_front();
    issue_from_initiator(idx);
    if (idx < initiators_.size()) {
      initiators_[idx].cmd_ctrl &= ~0x1u;
    }
  }
}

void NocNiu::inc_counter(size_t idx, uint32_t value) {
  if (idx >= counters_.size()) {
    return;
  }
  counters_[idx] += value;
  // Per spec: counters 16-47 are 8-bit, so mask to 8 bits
  if (idx >= kCounterMstReqsOutstandingBase && idx < kCounterSlvAtomicRespSent) {
    counters_[idx] &= 0xFFu;
  }
}

void NocNiu::dec_counter(size_t idx, uint32_t value) {
  if (idx >= counters_.size()) {
    return;
  }
  counters_[idx] -= value;
  // Per spec: counters 16-47 are 8-bit, so mask to 8 bits
  if (idx >= kCounterMstReqsOutstandingBase && idx < kCounterSlvAtomicRespSent) {
    counters_[idx] &= 0xFFu;
  }
}

uint32_t NocNiu::mmio_read(uint32_t addr) {
  auto read_initiator = [&](size_t idx, uint32_t off) -> uint32_t {
    if (idx >= initiators_.size()) {
      return 0;
    }
    const InitiatorRegs &regs = initiators_[idx];
    switch (off) {
      case kNocTargAddrLo:
        return regs.targ_addr_lo;
      case kNocTargAddrMid:
        return regs.targ_addr_mid;
      case kNocRetAddrLo:
        return regs.ret_addr_lo;
      case kNocRetAddrMid:
        return regs.ret_addr_mid;
      case kNocPacketTag:
        return regs.packet_tag;
      case kNocCtrl:
        return regs.ctrl;
      case kNocAtLenBe:
        return regs.at_len_be;
      case kNocAtData:
        return regs.at_data;
      case kNocCmdCtrl:
        return regs.cmd_ctrl;
      default:
        return 0;
    }
  };

  auto read_ident = [&]() -> uint32_t {
    // Per spec: dateline flip indicates whether this node is at the dateline position
    // For NOC0: dateline at x=width-1 (East wrap) and y=height-1 (South wrap)
    // For NOC1: dateline at x=0 (West wrap) and y=0 (North wrap)
    uint32_t dateline_x = 0;
    uint32_t dateline_y = 0;
    if (noc_id == NocId::NOC0) {
      dateline_x = (mesh_width > 1 && coord.x == mesh_width - 1) ? 1u : 0u;
      dateline_y = (mesh_height > 1 && coord.y == mesh_height - 1) ? 1u : 0u;
    } else {
      dateline_x = (mesh_width > 1 && coord.x == 0) ? 1u : 0u;
      dateline_y = (mesh_height > 1 && coord.y == 0) ? 1u : 0u;
    }
    uint32_t x_first = (noc_id == NocId::NOC0) ? 1u : 0u;
    return (coord.x & 0x3Fu) |
           ((coord.y & 0x3Fu) << 6) |
           ((mesh_width & 0x7Fu) << 12) |
           ((mesh_height & 0x7Fu) << 19) |
           ((dateline_x & 0x1u) << 26) |
           ((dateline_y & 0x1u) << 27) |
           ((x_first & 0x1u) << 28);
  };

  auto read_endpoint = [&]() -> uint32_t {
    uint32_t tile_index = 0;
    uint32_t group_index = 0;
    uint32_t tile_type = 3;
    uint32_t noc_index = (noc_id == NocId::NOC0) ? 0u : 1u;
    return (tile_index & 0xFFu) |
           ((group_index & 0xFFu) << 8) |
           ((tile_type & 0xFFu) << 16) |
           ((noc_index & 0xFFu) << 24);
  };

  if (addr < kNocCmdBufOffset * 4) {
    size_t idx = addr / kNocCmdBufOffset;
    uint32_t off = addr % kNocCmdBufOffset;
    if (off <= kNocCmdCtrl) {
      return read_initiator(idx, off);
    }
    if (off == kNiuNodeId) {
      return read_ident();
    }
    if (off == kNiuEndpointId) {
      return read_endpoint();
    }
  }

  if ((addr >= 0x02C && addr <= 0x033) ||
      (addr >= 0x42C && addr <= 0x433) ||
      (addr >= 0x82C && addr <= 0x833) ||
      (addr >= 0xC2C && addr <= 0xC33)) {
    uint32_t off = addr & 0x3F;
    if (off == 0x2C) {
      return read_ident();
    }
    if (off == 0x30) {
      return read_endpoint();
    }
  }

  if (addr == kNiuCmdStatus) {
    uint32_t status = 0;
    for (size_t i = 0; i < initiators_.size(); ++i) {
      if (initiators_[i].cmd_ctrl & 0x1u) {
        status |= (1u << i);
      }
    }
    return status;
  }

  if (addr >= kCountersBase && addr <= kCountersEnd) {
    size_t idx = (addr - kCountersBase) / 4;
    if (idx < counters_.size()) {
      return counters_[idx];
    }
    return 0;
  }

  if (addr >= kRouterDebugBase && addr <= kRouterDebugEnd) {
    return 0;
  }
  if (addr >= kNiuDebugBase && addr <= kNiuDebugEnd) {
    return 0;
  }

  uint32_t off = addr;
  switch (off) {
    case kNiuCfg0:
      return niu_cfg_0_;
    case kRouterCfg0:
      return router_cfg_0_;
    case kRouterCfg1:
      return router_cfg_1_;
    case kRouterCfg2:
      return router_cfg_2_;
    case kRouterCfg3:
      return router_cfg_3_;
    case kRouterCfg4:
      return router_cfg_4_;
    case kNocIdLogical:
      return noc_id_logical_;
    default:
      if (off >= kNocXTranslate0 && off < kNocXTranslate0 + 16) {
        size_t word = (off - kNocXTranslate0) / 4;
        uint32_t value = 0;
        for (size_t i = 0; i < 8; ++i) {
          size_t entry = word * 8 + i;
          value |= (translation_.x[entry] & 0xFu) << (i * 4);
        }
        return value;
      }
      if (off >= kNocYTranslate0 && off < kNocYTranslate0 + 16) {
        size_t word = (off - kNocYTranslate0) / 4;
        uint32_t value = 0;
        for (size_t i = 0; i < 8; ++i) {
          size_t entry = word * 8 + i;
          value |= (translation_.y[entry] & 0xFu) << (i * 4);
        }
        return value;
      }
      return 0;
  }
}

void NocNiu::mmio_write(uint32_t addr, uint32_t data) {
  if (addr == kNiuClearCounters) {
    for (size_t i = 0; i < 16; ++i) {
      if (data & (1u << i)) {
        counters_[kCounterMstReqsOutstandingBase + i] = 0;
      }
    }
    return;
  }

  if (addr < kNocCmdBufOffset * 4) {
    size_t idx = addr / kNocCmdBufOffset;
    uint32_t off = addr % kNocCmdBufOffset;
    InitiatorRegs &regs = initiators_[idx];
    switch (off) {
      case kNocTargAddrLo:
        regs.targ_addr_lo = data;
        return;
      case kNocTargAddrMid:
        regs.targ_addr_mid = data;
        return;
      case kNocRetAddrLo:
        regs.ret_addr_lo = data;
        return;
      case kNocRetAddrMid:
        regs.ret_addr_mid = data;
        return;
      case kNocPacketTag:
        regs.packet_tag = data;
        return;
      case kNocCtrl:
        regs.ctrl = data;
        return;
      case kNocAtLenBe:
        regs.at_len_be = data;
        return;
      case kNocAtData:
        regs.at_data = data;
        return;
      case kNocCmdCtrl:
        regs.cmd_ctrl = data;
        if (data & 0x1u) {
          pending_mmio_issues_.push_back(idx);
          mmio_issue_event_.notify(sc_core::SC_ZERO_TIME);
        }
        return;
      default:
        return;
    }
  }

  uint32_t off = addr;
  switch (off) {
    case kNiuCfg0:
      niu_cfg_0_ = data;
      if ((niu_cfg_0_ >> 14) & 0x1u) {
        NocCoord logical = translate_coord(coord, translation_);
        noc_id_logical_ = static_cast<uint32_t>((logical.y & 0x3Fu) << 6) |
                          static_cast<uint32_t>(logical.x & 0x3Fu);
      } else {
        noc_id_logical_ = static_cast<uint32_t>((coord.y & 0x3Fu) << 6) |
                          static_cast<uint32_t>(coord.x & 0x3Fu);
      }
      return;
    case kRouterCfg0:
      router_cfg_0_ = data;
      return;
    case kRouterCfg1:
      router_cfg_1_ = data;
      return;
    case kRouterCfg2:
      router_cfg_2_ = data;
      return;
    case kRouterCfg3:
      router_cfg_3_ = data;
      return;
    case kRouterCfg4:
      router_cfg_4_ = data;
      return;
    case kNocIdLogical:
      noc_id_logical_ = data;
      return;
    default:
      if (off >= kNocXTranslate0 && off < kNocXTranslate0 + 16) {
        size_t word = (off - kNocXTranslate0) / 4;
        for (size_t i = 0; i < 8; ++i) {
          size_t entry = word * 8 + i;
          translation_.x[entry] = (data >> (i * 4)) & 0xFu;
        }
        return;
      }
      if (off >= kNocYTranslate0 && off < kNocYTranslate0 + 16) {
        size_t word = (off - kNocYTranslate0) / 4;
        for (size_t i = 0; i < 8; ++i) {
          size_t entry = word * 8 + i;
          translation_.y[entry] = (data >> (i * 4)) & 0xFu;
        }
        return;
      }
      return;
  }
}

void NocNiu::issue_from_initiator(size_t idx) {
  if (idx >= initiators_.size()) {
    return;
  }

  const InitiatorRegs &regs = initiators_[idx];
  uint64_t targ_full = (static_cast<uint64_t>(regs.targ_addr_mid) << 32) | regs.targ_addr_lo;
  uint64_t ret_full = (static_cast<uint64_t>(regs.ret_addr_mid) << 32) | regs.ret_addr_lo;

  bool is_broadcast = (regs.ctrl >> 5) & 0x1u;
  bool vc_linked = (regs.ctrl >> 6) & 0x1u;
  bool vc_static = (regs.ctrl >> 7) & 0x1u;
  bool brcst_xy = (regs.ctrl >> 16) & 0x1u;
  bool brcst_src_include = (regs.ctrl >> 17) & 0x1u;
  bool resp_marked = (regs.ctrl >> 4) & 0x1u;
  bool wr_inline = (regs.ctrl >> 3) & 0x1u;
  bool wr_be = (regs.ctrl >> 2) & 0x1u;
  bool path_reserve = (regs.ctrl >> 8) & 0x1u;
  bool mem_rd_drop_ack = (regs.ctrl >> 9) & 0x1u;
  // ARB_PRIORITY is bits 31:27 per spec (5 bits, values 0-31)
  uint8_t arb_priority = static_cast<uint8_t>((regs.ctrl >> 27) & 0x1Fu);

  uint8_t class_bits = 0;
  uint8_t buddy_bit = 0;
  if (vc_static) {
    buddy_bit = (regs.ctrl >> 13) & 0x1u;
    class_bits = (regs.ctrl >> 14) & 0x3u;
  }

  NocReqType req_type = NocReqType::Read;
  switch (regs.ctrl & 0x3u) {
    case 0:
      req_type = NocReqType::Read;
      break;
    case 1:
      req_type = NocReqType::Atomic;
      break;
    case 2:
      req_type = NocReqType::Write;
      break;
    default:
      req_type = NocReqType::Read;
      break;
  }

  auto decode_unicast = [](uint64_t full, NocCoord *coord_out, uint64_t *addr_out) {
    *addr_out = full & ((1ULL << 36) - 1);
    coord_out->x = static_cast<unsigned>((full >> 36) & 0x3Fu);
    coord_out->y = static_cast<unsigned>((full >> 42) & 0x3Fu);
  };

  auto decode_broadcast = [](uint64_t full, NocCoord *start, NocCoord *end, uint64_t *addr_out) {
    *addr_out = full & ((1ULL << 36) - 1);
    end->x = static_cast<unsigned>((full >> 36) & 0x3Fu);
    end->y = static_cast<unsigned>((full >> 42) & 0x3Fu);
    start->x = static_cast<unsigned>((full >> 48) & 0x3Fu);
    start->y = static_cast<unsigned>((full >> 54) & 0x3Fu);
  };

  NocCoord targ_coord{};
  NocCoord ret_coord{};
  NocCoord brcst_start{};
  NocCoord brcst_end{};
  uint64_t targ_addr = 0;
  uint64_t ret_addr = 0;

  if (is_broadcast) {
    decode_broadcast(targ_full, &brcst_start, &brcst_end, &targ_addr);
  } else {
    decode_unicast(targ_full, &targ_coord, &targ_addr);
  }
  decode_unicast(ret_full, &ret_coord, &ret_addr);

  bool coord_translate = (niu_cfg_0_ >> 14) & 0x1u;
  if (coord_translate) {
    if (is_broadcast) {
      brcst_start = translate_coord(brcst_start, translation_);
      brcst_end = translate_coord(brcst_end, translation_);
    } else {
      targ_coord = translate_coord(targ_coord, translation_);
    }
    ret_coord = translate_coord(ret_coord, translation_);
  }

  uint64_t linked_id = 0;
  if (vc_linked) {
    if (!linked_active_[idx]) {
      linked_active_[idx] = true;
      linked_id_[idx] = next_linked_id_++;
    }
    linked_id = linked_id_[idx];
  } else if (linked_active_[idx]) {
    linked_id = linked_id_[idx];
    linked_active_[idx] = false;
  }

  NocPacket pkt;
  pkt.id = next_packet_id_++;
  pkt.src = coord;
  pkt.dst = targ_coord;
  pkt.class_bits = class_bits;
  pkt.buddy_bit = buddy_bit;
  pkt.vc_static = vc_static;
  pkt.vc_linked = vc_linked;
  pkt.linked_id = linked_id;
  pkt.coord_translate = false;
  pkt.is_response = false;
  pkt.posted = (req_type == NocReqType::Read) ? false : !resp_marked;
  pkt.req_type = req_type;
  pkt.addr = targ_addr;
  pkt.ret_addr = ret_addr;
  pkt.ret_coord = ret_coord;
  pkt.atomic_imm = regs.at_data;
  pkt.transaction_id = static_cast<uint8_t>((regs.packet_tag >> 10) & 0xFu);
  pkt.packet_tag = regs.packet_tag;
  pkt.at_len_be_raw = regs.at_len_be;
  pkt.stream_id = static_cast<uint8_t>(regs.packet_tag & 0x3Fu);
  pkt.deliver_to_overlay = (regs.packet_tag >> 6) & 0x1u;
  pkt.msg_first = (regs.packet_tag >> 7) & 0x1u;
  pkt.msg_last = (regs.packet_tag >> 8) & 0x1u;
  pkt.header_store = (regs.packet_tag >> 9) & 0x1u;
  pkt.path_reserve = path_reserve;
  pkt.arb_priority = arb_priority;
  pkt.mem_rd_drop_ack = mem_rd_drop_ack;
  pkt.is_broadcast = is_broadcast;
  pkt.brcst_xy_major = brcst_xy;

  uint32_t total_length = regs.at_len_be;
  uint32_t max_payload = 8192;

  // Validate address alignment before issuing request
  bool is_atomic = (req_type == NocReqType::Atomic);
  bool is_byte_enable_req = (req_type == NocReqType::Write) && (wr_be || wr_inline);
  if (!validate_request_alignment(ret_addr, targ_addr, total_length,
                                   is_byte_enable_req, wr_inline, is_atomic)) {
    // Log error but continue (in real hardware this would cause undefined behavior)
    std::cerr << "  Request will proceed but may produce incorrect results" << std::endl;
  }

  auto send_read_chunk = [&](uint64_t addr, uint64_t ret, uint32_t len) {
    NocPacket copy = pkt;
    copy.addr = addr;
    copy.ret_addr = ret;
    copy.length = len;
    copy.force_single_flit = true;
    send_packet(copy);
    inc_counter(kCounterMstCmdAccepted);
    inc_counter(kCounterMstRdReqStarted);
    inc_counter(kCounterMstRdReqSent);
  };

  auto send_write_chunk = [&](NocPacket &copy, bool posted, uint32_t data_flits) {
    send_packet(copy);
    inc_counter(kCounterMstCmdAccepted);
    if (posted) {
      inc_counter(kCounterMstPostedWrReqStarted);
      inc_counter(kCounterMstPostedWrReqSent);
      inc_counter(kCounterMstPostedWrDataFlitSent, data_flits);
    } else {
      inc_counter(kCounterMstNonpostedWrReqStarted);
      inc_counter(kCounterMstNonpostedWrReqSent);
      inc_counter(kCounterMstNonpostedWrDataFlitSent, data_flits);
    }
  };

  if (req_type == NocReqType::Read) {
    uint32_t chunks = (total_length + max_payload - 1) / max_payload;
    inc_counter(kCounterMstReqsOutstandingBase + pkt.transaction_id, chunks);
    for (uint32_t i = 0; i < chunks; ++i) {
      uint32_t len = std::min(max_payload, total_length - i * max_payload);
      send_read_chunk(targ_addr + i * max_payload, ret_addr + i * max_payload, len);
    }
  } else if (req_type == NocReqType::Write) {
    if (wr_inline) {
      uint32_t be_mask = regs.at_len_be;
      uint32_t byte_enable = 0;
      for (size_t i = 0; i < 16; ++i) {
        bool enable = ((be_mask >> i) & 0x1u) || ((be_mask >> (16 + i)) & 0x1u);
        if (enable) {
          byte_enable |= (1u << i);
        }
      }
      pkt.has_byte_enable = true;
      pkt.byte_enable = byte_enable;
      uint32_t data = regs.at_data;
      pkt.payload.resize(16);
      for (size_t i = 0; i < 16; ++i) {
        pkt.payload[i] = static_cast<uint8_t>((data >> ((i & 0x3u) * 8)) & 0xFFu);
      }
      pkt.length = static_cast<uint32_t>(pkt.payload.size());
      pkt.force_single_flit = true;
      pkt.addr = targ_addr & ~0xFULL;
      pkt.ret_coord = coord;
      pkt.ret_addr = 0;
      if (resp_marked) {
        inc_counter(kCounterMstReqsOutstandingBase + pkt.transaction_id);
      }
      uint32_t data_flits = static_cast<uint32_t>((pkt.payload.size() + 31) / 32);
      send_write_chunk(pkt, !resp_marked, data_flits);
    } else {
      uint32_t length = wr_be ? 32u : regs.at_len_be;
      pkt.length = length;
      uint64_t read_base = wr_be ? (targ_addr & ~0xFULL) : targ_addr;
      if (wr_be) {
        pkt.has_byte_enable = true;
        pkt.byte_enable = regs.at_len_be;
      }
      pkt.dst = ret_coord;
      pkt.addr = ret_addr & ~0xFULL;
      pkt.ret_coord = targ_coord;
      pkt.ret_addr = targ_addr;
      uint32_t chunks = wr_be ? 1 : (total_length + max_payload - 1) / max_payload;
      inc_counter(kCounterMstWriteReqsOutgoingBase + pkt.transaction_id, chunks);
      if (resp_marked) {
        inc_counter(kCounterMstReqsOutstandingBase + pkt.transaction_id, chunks);
      }
      for (uint32_t i = 0; i < chunks; ++i) {
        uint32_t len = wr_be ? 32u : std::min(max_payload, total_length - i * max_payload);
        NocPacket copy = pkt;
        copy.length = len;
        copy.addr = (ret_addr & ~0xFULL) + i * max_payload;
        copy.ret_addr = (targ_addr & ~0xFULL) + i * max_payload;
        uint64_t base = wr_be ? (read_base) : (targ_addr + i * max_payload);
        if (base + len <= l1_size()) {
          copy.payload.resize(len);
          l1_read(base, copy.payload.data(), len);
        }
        uint32_t data_flits = static_cast<uint32_t>((copy.payload.size() + 31) / 32);
        send_write_chunk(copy, !resp_marked, data_flits);
        dec_counter(kCounterMstWriteReqsOutgoingBase + pkt.transaction_id);
      }
    }
  } else {
    pkt.length = 4;
    pkt.force_single_flit = true;
    inc_counter(kCounterMstCmdAccepted);
    if (resp_marked) {
      inc_counter(kCounterMstNonpostedAtomicStarted);
      inc_counter(kCounterMstNonpostedAtomicSent);
      inc_counter(kCounterMstReqsOutstandingBase + pkt.transaction_id);
    } else {
      inc_counter(kCounterMstPostedAtomicSent);
    }
  }

  if (is_broadcast) {
    if (path_reserve) {
      wait(cycle_time * 10);
    }
    // Broadcast is sent as a single packet with is_broadcast=true.
    // The router handles multicast routing via compute_out_mask().
    NocPacket copy = pkt;
    copy.brcst_start = brcst_start;
    copy.brcst_end = brcst_end;
    copy.is_broadcast = true;
    copy.brcst_xy_major = brcst_xy;
    copy.brcst_src_include = brcst_src_include;
    send_packet(copy);
  } else {
    send_packet(pkt);
  }
}

void NocNiu::send_packet(const NocPacket &packet) {
  unsigned total_flits = packet.total_flits();
  for (unsigned i = 0; i < total_flits; ++i) {
    NocFlit flit;
    flit.packet_id = packet.id;
    flit.src = packet.src;
    flit.dst = packet.dst;
    flit.flit_index = i;
    flit.total_flits = total_flits;
    flit.is_header = (i == 0);
    flit.is_response = packet.is_response;
    flit.posted = packet.posted;
    flit.req_type = packet.req_type;
    flit.addr = packet.addr;
    flit.ret_addr = packet.ret_addr;
    flit.ret_coord = packet.ret_coord;
    flit.length = packet.length;
    flit.atomic_imm = packet.atomic_imm;
    flit.has_byte_enable = packet.has_byte_enable;
    flit.byte_enable = packet.byte_enable;
    flit.transaction_id = packet.transaction_id;
    flit.packet_tag = packet.packet_tag;
    flit.at_len_be_raw = packet.at_len_be_raw;
    flit.stream_id = packet.stream_id;
    flit.deliver_to_overlay = packet.deliver_to_overlay;
    flit.msg_first = packet.msg_first;
    flit.msg_last = packet.msg_last;
    flit.header_store = packet.header_store;
    flit.path_reserve = packet.path_reserve;
    flit.arb_priority = packet.arb_priority;
    flit.mem_rd_drop_ack = packet.mem_rd_drop_ack;
    uint8_t vc_id = make_vc_id(0, packet.class_bits, packet.buddy_bit);
    if (packet.vc_linked && packet.linked_id != 0) {
      auto it = linked_vc_map_.find(packet.linked_id);
      if (it == linked_vc_map_.end()) {
        linked_vc_map_[packet.linked_id] = vc_id;
      } else {
        vc_id = it->second;
      }
    }
    flit.vc_id = vc_id;
    flit.vc_static = packet.vc_static;
    flit.vc_linked = packet.vc_linked;
    flit.linked_id = packet.linked_id;

    if (!packet.payload.empty() && (i > 0 || packet.force_single_flit)) {
      size_t offset = (i == 0) ? 0 : (i - 1) * 32;
      size_t remaining = packet.payload.size() - offset;
      size_t copy_len = remaining < 32 ? remaining : 32;
      for (size_t b = 0; b < copy_len; ++b) {
        flit.data[b] = packet.payload[offset + b];
      }
    }

    net_out.write(flit);
    wait(cycle_time);
  }
}

void NocNiu::handle_request(const NocPacket &packet) {
  inc_counter(kCounterSlvReqAccepted);
  if (packet.req_type == NocReqType::Write) {
    bool posted = packet.posted;
    bool inline_write = packet.force_single_flit && packet.payload.size() <= 16;
    if (posted) {
      inc_counter(kCounterSlvPostedWrReqStarted);
    } else {
      inc_counter(kCounterSlvNonpostedWrReqStarted);
    }
    if (!packet.payload.empty() && packet.addr + packet.payload.size() <= l1_size()) {
      if (packet.has_byte_enable && packet.payload.size() <= 32) {
        uint64_t base = packet.addr & ~0xFULL;
        std::vector<uint8_t> buf(packet.payload.size(), 0);
        if (base + buf.size() <= l1_size()) {
          l1_read(base, buf.data(), buf.size());
          for (size_t i = 0; i < packet.payload.size(); ++i) {
            if (packet.byte_enable & (1u << i)) {
              buf[i] = packet.payload[i];
            }
          }
          l1_write(base, buf.data(), buf.size());
        }
      } else {
        l1_write(packet.addr, packet.payload.data(), packet.payload.size());
      }
    }
    uint32_t data_flits = static_cast<uint32_t>((packet.payload.size() + 31) / 32);
    if (posted) {
      inc_counter(kCounterSlvPostedWrDataFlitReceived, data_flits);
      inc_counter(kCounterSlvPostedWrReqReceived);
    } else {
      inc_counter(kCounterSlvNonpostedWrDataFlitReceived, data_flits);
      inc_counter(kCounterSlvNonpostedWrReqReceived);
    }
    if (!packet.posted) {
      inc_counter(kCounterSlvWrAckSent);
      send_response(packet, {});
    }
    if (packet.posted && packet.header_store) {
      uint64_t store_addr = static_cast<uint64_t>(packet.atomic_imm) << 4;
      size_t copy_len = std::min<size_t>(16, packet.payload.size());
      if (store_addr + copy_len <= l1_size()) {
        l1_write(store_addr, packet.payload.data(), copy_len);
      }
    }
    return;
  }

  if (packet.req_type == NocReqType::Read) {
    inc_counter(kCounterSlvRdReqReceived);
    std::vector<uint8_t> data;
    if (packet.addr + packet.length <= l1_size()) {
      data.resize(packet.length);
      l1_read(packet.addr, data.data(), data.size());
    }
    inc_counter(kCounterSlvRdRespSent);
    inc_counter(kCounterSlvRdDataFlitSent, static_cast<uint32_t>((data.size() + 31) / 32));
    send_response(packet, data);
    return;
  }

  if (packet.req_type == NocReqType::Atomic) {
    if (packet.posted) {
      inc_counter(kCounterSlvPostedAtomicReceived);
    } else {
      inc_counter(kCounterSlvNonpostedAtomicReceived);
    }
    AtomicDecoded decoded = decode_atomic(packet.at_len_be_raw);
    uint64_t base = packet.addr & ~0xFULL;
    uint64_t addr = base + static_cast<uint64_t>(decoded.ofs) * 4;
    uint32_t old_val = load_u32(addr);
    uint32_t new_val = old_val;
    switch (decoded.opcode) {
      case 1: {
        uint32_t inc_val = old_val + packet.atomic_imm;
        uint32_t mask = (decoded.int_width >= 31) ? 0xFFFFFFFFu
                                                  : ((2u << decoded.int_width) - 1u);
        new_val = (inc_val & mask) | (old_val & ~mask);
        store_u32(addr, new_val);
        break;
      }
      case 3: {
        if (base + 16 > l1_size()) {
          break;
        }
        uint16_t to_write[2] = {static_cast<uint16_t>(packet.atomic_imm & 0xFFFFu),
                                static_cast<uint16_t>(packet.atomic_imm >> 16)};
        std::vector<uint8_t> buf(16, 0);
        l1_read(base, buf.data(), buf.size());
        for (unsigned i = 0; i < 8; ++i) {
          if (decoded.mask & (1u << i)) {
            uint16_t val = to_write[i & 1];
            size_t off = i * 2;
            buf[off] = static_cast<uint8_t>(val & 0xFFu);
            buf[off + 1] = static_cast<uint8_t>((val >> 8) & 0xFFu);
          }
        }
        l1_write(base, buf.data(), buf.size());
        break;
      }
      case 4: {
        uint32_t cmp_val = decoded.cmp_val;
        uint32_t set_val = decoded.set_val;
        if (old_val == cmp_val) {
          store_u32(addr, set_val);
        }
        break;
      }
      case 6:
      case 7:
      case 10: {
        store_u32(addr, packet.atomic_imm);
        break;
      }
      default: {
        new_val = old_val + packet.atomic_imm;
        store_u32(addr, new_val);
        break;
      }
    }
    std::vector<uint8_t> data(4);
    data[0] = static_cast<uint8_t>(old_val & 0xFFu);
    data[1] = static_cast<uint8_t>((old_val >> 8) & 0xFFu);
    data[2] = static_cast<uint8_t>((old_val >> 16) & 0xFFu);
    data[3] = static_cast<uint8_t>((old_val >> 24) & 0xFFu);
    inc_counter(kCounterSlvAtomicRespSent);
    send_response(packet, data);
  }
}

void NocNiu::send_response(const NocPacket &request, const std::vector<uint8_t> &payload) {
  NocPacket resp;
  resp.id = next_packet_id_++;
  resp.src = coord;
  resp.dst = request.ret_coord;
  resp.is_response = true;
  resp.req_type = request.req_type;
  resp.payload = payload;
  resp.length = static_cast<uint32_t>(payload.size());
  resp.addr = request.ret_addr;
  if (request.req_type == NocReqType::Atomic) {
    resp.force_single_flit = true;
  }
  resp.transaction_id = request.transaction_id;
  resp.class_bits = 0x3;
  resp.buddy_bit = 0;
  resp.vc_static = false;
  resp.vc_linked = false;
  resp.linked_id = 0;
  send_packet(resp);
}

uint32_t NocNiu::load_u32(uint64_t addr) const {
  if (addr + 4 > l1_size()) {
    return 0;
  }
  uint32_t value = 0;
  uint8_t buf[4] = {0, 0, 0, 0};
  l1_read(addr, buf, sizeof(buf));
  value |= static_cast<uint32_t>(buf[0]);
  value |= static_cast<uint32_t>(buf[1]) << 8;
  value |= static_cast<uint32_t>(buf[2]) << 16;
  value |= static_cast<uint32_t>(buf[3]) << 24;
  return value;
}

void NocNiu::store_u32(uint64_t addr, uint32_t value) {
  if (addr + 4 > l1_size()) {
    return;
  }
  uint8_t buf[4] = {
      static_cast<uint8_t>(value & 0xFFu),
      static_cast<uint8_t>((value >> 8) & 0xFFu),
      static_cast<uint8_t>((value >> 16) & 0xFFu),
      static_cast<uint8_t>((value >> 24) & 0xFFu),
  };
  l1_write(addr, buf, sizeof(buf));
}

bool NocNiu::validate_request_alignment(uint64_t src_addr, uint64_t dst_addr,
                                        uint32_t length, bool is_byte_enable,
                                        bool is_inline, bool is_atomic) {
  // Determine memory regions
  MemRegion src_type = get_mem_region(src_addr);
  MemRegion dst_type = get_mem_region(dst_addr);

  AlignmentResult result;

  if (is_atomic) {
    result = validate_atomic_alignment(dst_addr);
  } else if (is_byte_enable) {
    result = validate_byte_enable_transfer(src_addr, dst_addr, is_inline, src_type, dst_type);
  } else {
    result = validate_length_transfer(src_addr, dst_addr, length, src_type, dst_type);
  }

  if (!result.valid) {
    std::cerr << sc_core::sc_time_stamp() << " NIU " << coord_to_string(coord)
              << " ALIGNMENT ERROR: " << result.error_msg
              << " src=0x" << std::hex << src_addr
              << " dst=0x" << dst_addr << std::dec
              << " len=" << length << std::endl;
  }

  return result.valid;
}
