#ifndef NOC_NIU_H
#define NOC_NIU_H

#include <array>
#include <cstdint>
#include <deque>
#include <ostream>
#include <unordered_map>
#include <vector>

#include "noc_types.h"
#include "noc_translation.h"
#include <systemc>

struct NiuCmd {
  NocReqType type = NocReqType::Write;
  NocCoord dst;
  uint64_t addr = 0;
  std::vector<uint8_t> data;
  uint32_t length = 0;
  uint32_t atomic_imm = 0;
  bool posted = false;
  uint8_t class_bits = 0;
  uint8_t buddy_bit = 0;
  bool vc_static = false;
  bool vc_linked = false;
  uint64_t linked_id = 0;
  bool coord_translate = false;
};

struct NiuResp {
  uint64_t packet_id = 0;
  NocReqType type = NocReqType::Read;
  std::vector<uint8_t> data;
};

struct NiuMmioReq {
  bool write = false;
  uint32_t addr = 0;
  uint32_t data = 0;
};

struct NiuMmioResp {
  uint32_t data = 0;
};

inline std::ostream &operator<<(std::ostream &os, const NiuCmd &cmd) {
  os << "NiuCmd{type=" << static_cast<int>(cmd.type) << ",dst=" << coord_to_string(cmd.dst)
     << ",addr=0x" << std::hex << cmd.addr << std::dec << "}";
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const NiuResp &resp) {
  os << "NiuResp{pkt=" << resp.packet_id << ",type=" << static_cast<int>(resp.type)
     << ",len=" << resp.data.size() << "}";
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const NiuMmioReq &req) {
  os << "NiuMmioReq{write=" << req.write << ",addr=0x" << std::hex << req.addr
     << ",data=0x" << req.data << std::dec << "}";
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const NiuMmioResp &resp) {
  os << "NiuMmioResp{data=0x" << std::hex << resp.data << std::dec << "}";
  return os;
}

SC_MODULE(NocNiu) {
  SC_HAS_PROCESS(NocNiu);

  sc_core::sc_fifo_in<NocFlit> net_in;
  sc_core::sc_fifo_out<NocFlit> net_out;

  sc_core::sc_fifo_in<NiuCmd> cmd_in;
  sc_core::sc_fifo_out<NiuResp> resp_out;
  sc_core::sc_fifo_in<NiuMmioReq> mmio_in;
  sc_core::sc_fifo_out<NiuMmioResp> mmio_out;

  NocCoord coord;
  NocId noc_id;
  sc_core::sc_time cycle_time;
  unsigned mesh_width;
  unsigned mesh_height;

  // Default L1 size per spec: 1464 KiB = 16 banks × 91.5 KiB
  static constexpr size_t kDefaultL1Size = 1464 * 1024;  // 1,499,136 bytes

  explicit NocNiu(sc_core::sc_module_name name,
                  NocCoord coord,
                  NocId noc_id,
                  sc_core::sc_time cycle_time,
                  unsigned mesh_width = 10,
                  unsigned mesh_height = 12,
                  size_t mem_size_bytes = kDefaultL1Size);

  void cmd_loop();
  void rx_loop();
  void mmio_loop();

  // Public access to L1 memory for overlay coprocessor
  std::vector<uint8_t>& get_l1_memory() { return mem_; }
  const std::vector<uint8_t>& get_l1_memory() const { return mem_; }

 private:
  struct AssembledPacket {
    NocPacket packet;
    unsigned received_flits = 0;
  };

  struct InitiatorRegs {
    uint32_t targ_addr_lo = 0;
    uint32_t targ_addr_mid = 0;
    uint32_t ret_addr_lo = 0;
    uint32_t ret_addr_mid = 0;
    uint32_t packet_tag = 0;
    uint32_t ctrl = 0;
    uint32_t at_len_be = 0;
    uint32_t at_data = 0;
    uint32_t cmd_ctrl = 0;
    bool busy = false;
  };

  std::vector<uint8_t> mem_;
  uint64_t next_packet_id_ = 1;
  std::unordered_map<uint64_t, AssembledPacket> inflight_;
  std::unordered_map<uint64_t, uint8_t> linked_vc_map_;
  NocTranslationTable translation_;
  std::array<InitiatorRegs, 4> initiators_ = {};
  std::deque<NiuCmd> pending_cmds_;
  uint32_t niu_cfg_0_ = 0;
  uint32_t router_cfg_0_ = 0;
  uint32_t router_cfg_1_ = 0;
  uint32_t router_cfg_2_ = 0;
  uint32_t router_cfg_3_ = 0;
  uint32_t router_cfg_4_ = 0;
  uint32_t noc_id_logical_ = 0;
  uint64_t next_linked_id_ = 1;
  std::array<bool, 4> linked_active_ = {};
  std::array<uint64_t, 4> linked_id_ = {};
  std::array<uint32_t, 64> counters_ = {};

  void send_packet(const NocPacket &packet);
  void handle_request(const NocPacket &packet);
  void send_response(const NocPacket &request, const std::vector<uint8_t> &payload);
  uint32_t load_u32(uint64_t addr) const;
  void store_u32(uint64_t addr, uint32_t value);
  uint32_t mmio_read(uint32_t addr);
  void mmio_write(uint32_t addr, uint32_t data);
  void issue_from_initiator(size_t idx);
  void inc_counter(size_t idx, uint32_t value = 1);
  void dec_counter(size_t idx, uint32_t value = 1);
  bool validate_request_alignment(uint64_t src_addr, uint64_t dst_addr,
                                  uint32_t length, bool is_byte_enable,
                                  bool is_inline, bool is_atomic);
};

#endif
