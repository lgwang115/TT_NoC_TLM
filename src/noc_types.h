#ifndef NOC_TYPES_H
#define NOC_TYPES_H

#include <array>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

struct NocCoord {
  unsigned x = 0;
  unsigned y = 0;

  bool operator==(const NocCoord &other) const {
    return x == other.x && y == other.y;
  }
};

enum class NocId {
  NOC0,
  NOC1,
};

enum class NocReqType : uint8_t {
  Read = 0,
  Write = 1,
  AtomicAdd = 2,
};

struct NocPacket {
  uint64_t id = 0;
  NocCoord src;
  NocCoord dst;
  NocCoord brcst_start;
  NocCoord brcst_end;
  bool is_broadcast = false;
  bool brcst_xy_major = false;
  bool brcst_src_include = true;
  uint8_t class_bits = 0;
  uint8_t buddy_bit = 0;
  bool vc_static = false;
  bool vc_linked = false;
  uint64_t linked_id = 0;
  bool coord_translate = false;
  bool is_response = false;
  bool posted = false;
  NocReqType req_type = NocReqType::Write;
  uint64_t addr = 0;
  uint64_t ret_addr = 0;
  NocCoord ret_coord;
  uint32_t length = 0;
  uint32_t atomic_imm = 0;
  bool has_byte_enable = false;
  uint32_t byte_enable = 0;
  bool force_single_flit = false;
  uint8_t transaction_id = 0;
  uint32_t packet_tag = 0;
  uint32_t at_len_be_raw = 0;
  uint8_t stream_id = 0;
  bool deliver_to_overlay = false;
  bool msg_first = false;
  bool msg_last = false;
  bool header_store = false;
  bool path_reserve = false;
  uint8_t arb_priority = 0;
  bool mem_rd_drop_ack = false;
  std::vector<uint8_t> payload;

  unsigned total_flits() const {
    if (force_single_flit) {
      return 1;
    }
    unsigned data_flits = (payload.size() + 31) / 32;
    return 1 + data_flits;
  }
};

struct NocFlit {
  uint64_t packet_id = 0;
  NocCoord src;
  NocCoord dst;
  NocCoord brcst_start;
  NocCoord brcst_end;
  bool is_broadcast = false;
  bool brcst_xy_major = false;
  bool brcst_src_include = true;
  unsigned flit_index = 0;
  unsigned total_flits = 0;
  bool is_header = false;
  uint8_t vc_id = 0;
  bool vc_static = false;
  bool vc_linked = false;
  uint64_t linked_id = 0;
  bool is_response = false;
  bool posted = false;
  NocReqType req_type = NocReqType::Write;
  uint64_t addr = 0;
  uint64_t ret_addr = 0;
  NocCoord ret_coord;
  uint32_t length = 0;
  uint32_t atomic_imm = 0;
  bool has_byte_enable = false;
  uint32_t byte_enable = 0;
  uint8_t transaction_id = 0;
  uint32_t packet_tag = 0;
  uint32_t at_len_be_raw = 0;
  uint8_t stream_id = 0;
  bool deliver_to_overlay = false;
  bool msg_first = false;
  bool msg_last = false;
  bool header_store = false;
  bool path_reserve = false;
  uint8_t arb_priority = 0;
  bool mem_rd_drop_ack = false;
  std::array<uint8_t, 32> data = {};

  bool is_tail() const { return (flit_index + 1) == total_flits; }
};

inline std::string coord_to_string(const NocCoord &coord) {
  return "(" + std::to_string(coord.x) + "," + std::to_string(coord.y) + ")";
}

inline std::ostream &operator<<(std::ostream &os, const NocFlit &flit) {
  os << "NocFlit{pkt=" << flit.packet_id << ",src=" << coord_to_string(flit.src)
     << ",dst=" << coord_to_string(flit.dst) << ",idx=" << flit.flit_index << "/"
     << flit.total_flits << "}";
  return os;
}

struct NocVcFields {
  uint8_t dateline = 0;
  uint8_t class_bits = 0;
  uint8_t buddy = 0;
};

inline uint8_t make_vc_id(uint8_t dateline, uint8_t class_bits, uint8_t buddy) {
  return static_cast<uint8_t>(((dateline & 0x1u) << 3) | ((class_bits & 0x3u) << 1) |
                              (buddy & 0x1u));
}

inline NocVcFields decode_vc_id(uint8_t vc_id) {
  NocVcFields fields;
  fields.dateline = (vc_id >> 3) & 0x1u;
  fields.class_bits = (vc_id >> 1) & 0x3u;
  fields.buddy = vc_id & 0x1u;
  return fields;
}

#endif
