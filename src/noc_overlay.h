#ifndef NOC_OVERLAY_H
#define NOC_OVERLAY_H

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

#include "noc_types.h"
#include <systemc>

// NoC Overlay coprocessor per WormholeB0 spec
// 64 streams per Tensix tile, 32 streams per Ethernet tile
// Streams assist with NoC transactions by managing message flow between tiles

// Stream state machine states
enum class OverlayStreamState : uint8_t {
  Idle = 0,               // Waiting for software configuration
  LoadingConfig = 1,      // Loading configuration from L1
  WaitingToStart = 3,     // Config loaded, waiting for phase advance
  FlushingPrevPhase = 4,  // Waiting for previous phase L1 reads to complete
  Running = 5             // Normal operation: receiving and transmitting messages
};

// Stream capabilities differ by stream index (per spec)
struct StreamCapabilities {
  bool can_multicast = false;           // Streams 0-3 only
  bool can_receive_gather = false;      // Streams 0-5 only
  bool can_transmit_dram = false;       // Streams 0-3 and 8-11 only
  bool can_cause_irq = false;           // Streams 0-3 and 8-11 only
  bool metadata_includes_header = false; // Streams 4-5 only
  unsigned metadata_fifo_capacity = 0;
  unsigned metadata_fifo_group_size = 0;
  unsigned l1_read_complete_fifo_capacity = 0;
};

// Message metadata FIFO entry
struct OverlayMsgMetadata {
  uint32_t buf_ptr = 0;     // Pointer into receive buffer (in 16-byte units)
  uint32_t msg_length = 0;  // Message length (in 16-byte units)
  std::array<uint32_t, 4> header = {}; // Copy of message header (streams 4-5 only)
};

// Stream configuration registers
struct OverlayStreamRegs {
  // STREAM_MISC_CFG_REG_INDEX fields
  uint8_t outgoing_data_noc = 0;       // Which NoC for message contents
  uint8_t remote_src_update_noc = 0;   // Which NoC for flow control
  bool local_sources_connected = false; // Receive from gather
  bool source_endpoint = false;         // Receive from software
  bool remote_source = false;           // Receive from other stream
  bool receiver_endpoint = false;       // Transmit to software
  bool local_receiver = false;          // Transmit to gather
  bool remote_receiver = false;         // Transmit to other stream
  bool phase_auto_config = false;
  bool phase_auto_advance = false;
  bool next_phase_src_change = true;
  bool next_phase_dest_change = true;
  bool data_buf_no_flow_ctrl = false;
  bool dest_data_buf_no_flow_ctrl = false;
  bool remote_src_is_mcast = false;
  bool no_prev_phase_outgoing_data_flush = false;  // Bit 17: skip flush wait
  uint8_t unicast_vc_reg = 0;
  uint8_t reg_update_vc_reg = 0;

  // Buffer configuration
  uint32_t buf_start = 0;      // Receive buffer start (16-byte units)
  uint32_t buf_size = 0;       // Receive buffer size (16-byte units)
  uint32_t rd_ptr = 0;         // Read pointer
  uint32_t wr_ptr = 0;         // Write pointer

  // Message header array
  uint32_t msg_info_ptr = 0;      // Front pointer (16-byte units)
  uint32_t msg_info_wr_ptr = 0;   // Back pointer (16-byte units)

  // Remote source/destination
  uint8_t remote_src_x = 0;
  uint8_t remote_src_y = 0;
  uint8_t remote_src_stream_id = 0;
  uint8_t remote_src_dest_index = 0;
  uint32_t remote_src_phase = 0;

  uint8_t remote_dest_x = 0;
  uint8_t remote_dest_y = 0;
  uint8_t remote_dest_stream_id = 0;
  uint32_t remote_dest_buf_start = 0;
  uint32_t remote_dest_buf_size = 0;
  uint32_t remote_dest_msg_info_wr_ptr = 0;
  uint32_t remote_dest_wr_ptr = 0;

  // Multicast configuration (streams 0-3 only)
  uint8_t mcast_end_x = 0;
  uint8_t mcast_end_y = 0;
  bool mcast_en = false;
  bool mcast_linked = false;
  uint8_t mcast_vc = 0;
  bool mcast_no_path_res = false;
  bool mcast_xy = false;
  uint8_t mcast_dest_num = 1;

  // Phase control
  uint32_t phase_auto_cfg_header = 0;
  uint32_t curr_phase = 0;
  uint32_t curr_phase_base = 0;
  uint32_t num_msgs_to_receive = 0;
  uint32_t num_msgs_received = 0;

  // Auto-config from L1 registers
  uint32_t phase_auto_cfg_ptr = 0;       // Address of config blob in L1
  uint32_t phase_auto_cfg_ptr_base = 0;  // Base address for relative indexing
  uint8_t next_phase_num_cfg_reg_writes = 0;  // Config blob size (N-1)

  // Gather operation registers (for gather output streams)
  uint8_t gather_msg_arb_group_size = 1;  // 1, 2, or 4
  bool gather_msg_src_in_order_fwd = true;
  uint16_t gather_msg_local_stream_clear_num = 1;
  bool gather_msg_group_stream_clear_type = false;
  uint32_t local_src_mask[3] = {0, 0, 0};  // 64-bit mask for gather inputs

  // Gather input stream register
  uint16_t local_dest_msg_clear_num = 1;
  uint8_t local_dest_stream_id = 0;

  // Flow control threshold
  uint8_t buf_space_ack_threshold = 0;

  // Flow control state
  uint32_t last_ack_space_available = 0;  // Track when to send flow control
};

// Handshake packet types for flow control
enum class OverlayPacketType : uint8_t {
  Data = 0,
  HandshakeRequest = 1,
  HandshakeResponse = 2,
  FlowControlUpdate = 3
};

// Single overlay stream
struct OverlayStream {
  OverlayStreamState state = OverlayStreamState::Idle;
  OverlayStreamRegs regs;
  StreamCapabilities caps;

  // Message metadata FIFO (in stream hardware)
  std::deque<OverlayMsgMetadata> metadata_fifo;

  // L1 read complete FIFO (in stream hardware)
  std::deque<uint32_t> l1_read_complete_fifo;

  // Handshake state (for transmitter)
  bool handshake_sent = false;
  bool handshake_response_received = false;
  uint32_t handshake_phase = 0;

  // Handshake state (for receiver)
  bool handshake_request_received = false;
  bool sent_speculative_response = false;

  // Flow control state
  uint32_t remote_buf_space_available = 0;  // Transmitter's view of receiver space
  bool waiting_for_flow_ctrl = false;

  // Gather state (for gather output streams)
  uint8_t current_gather_group = 0;
  uint8_t current_gather_stream_in_group = 0;
  uint16_t msgs_received_from_current_stream = 0;
  std::array<bool, 64> gather_input_phase_started = {};  // Track which inputs started
};

// Overlay request to NIU for NoC transmission
struct OverlayNiuReq {
  uint8_t stream_id = 0;
  NocCoord dest = {};
  uint64_t dest_addr = 0;
  uint64_t src_addr = 0;
  uint32_t length = 0;
  bool is_posted = false;
  bool is_multicast = false;
  uint8_t mcast_end_x = 0;
  uint8_t mcast_end_y = 0;
  bool mcast_xy = false;
  uint8_t vc_class = 0;
  uint8_t vc_buddy = 0;
};

// Overlay response from NIU
struct OverlayNiuResp {
  uint8_t stream_id = 0;
  bool success = false;
  uint32_t bytes_transferred = 0;
};

// Stream operators for SystemC sc_fifo compatibility
inline std::ostream& operator<<(std::ostream& os, const OverlayNiuReq& req) {
  os << "OverlayNiuReq{stream=" << (int)req.stream_id
     << ",dest=(" << (int)req.dest.x << "," << (int)req.dest.y << ")"
     << ",len=" << req.length << "}";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const OverlayNiuResp& resp) {
  os << "OverlayNiuResp{stream=" << (int)resp.stream_id
     << ",ok=" << resp.success
     << ",bytes=" << resp.bytes_transferred << "}";
  return os;
}

// Message header format (global to overlay)
struct OverlayMsgHeaderFormat {
  uint8_t word_cnt_offset = 0;  // Bit offset of length field (multiple of 8)
  uint8_t word_cnt_bits = 16;   // Bit width of length field
};

SC_MODULE(NocOverlay) {
  SC_HAS_PROCESS(NocOverlay);

  // Interface to NIU for NoC transactions
  sc_core::sc_fifo_out<OverlayNiuReq> niu_req_out;
  sc_core::sc_fifo_in<OverlayNiuResp> niu_resp_in;

  // Interface for incoming overlay packets (from NIU)
  sc_core::sc_fifo_in<NocFlit> overlay_in;
  sc_core::sc_fifo_out<NocFlit> overlay_out;

  NocCoord coord;
  sc_core::sc_time cycle_time;
  unsigned num_streams;  // 64 for Tensix, 32 for Ethernet

  explicit NocOverlay(sc_core::sc_module_name name,
                      NocCoord coord,
                      sc_core::sc_time cycle_time,
                      unsigned num_streams = 64);

  // MMIO register access
  uint32_t read_reg(uint8_t stream_id, uint8_t reg_id);
  void write_reg(uint8_t stream_id, uint8_t reg_id, uint32_t value);

  // Pointer to shared L1 memory (set by TensixTile)
  void set_l1_memory(std::vector<uint8_t>* mem) { l1_mem_ = mem; }

 private:
  std::vector<OverlayStream> streams_;
  OverlayMsgHeaderFormat msg_header_format_;
  std::vector<uint8_t>* l1_mem_ = nullptr;

  // Global registers (accessible via stream 0)
  uint64_t blob_auto_cfg_done_ = 0;  // 64-bit bitmask for auto-config completion
  uint8_t next_auto_cfg_done_idx_ = 0;  // Fair iteration index

  // Initialize stream capabilities based on index
  StreamCapabilities get_stream_caps(unsigned stream_id) const;

  // Stream processing threads
  void stream_process_thread();
  void rx_thread();
  void tx_thread();

  // Stream state machine
  void process_stream(unsigned stream_id);
  void start_phase(unsigned stream_id);
  void complete_phase(unsigned stream_id);

  // Auto-config from L1
  void load_config_from_l1(unsigned stream_id);

  // Handshake protocol
  void send_handshake_request(unsigned stream_id);
  void send_handshake_response(unsigned stream_id);
  void handle_handshake_request(unsigned stream_id, uint32_t phase);
  void handle_handshake_response(unsigned stream_id, uint32_t phase);

  // Flow control
  void send_flow_control_update(unsigned stream_id);
  void handle_flow_control_update(unsigned stream_id, uint32_t space_available);
  bool should_send_flow_control(unsigned stream_id) const;
  uint32_t calculate_flow_control_threshold(unsigned stream_id) const;

  // Gather operations
  void process_gather_output(unsigned stream_id);
  bool is_gather_group_ready(unsigned output_stream, unsigned group_start) const;
  void receive_from_gather_input(unsigned output_stream, unsigned input_stream);

  // Message operations
  bool can_push_msg(unsigned stream_id) const;
  void push_msg_to_metadata_fifo(unsigned stream_id, uint32_t buf_ptr, uint32_t length);
  bool pop_msg_from_metadata_fifo(unsigned stream_id, OverlayMsgMetadata& meta);

  // L1 memory access
  uint32_t l1_read_u32(uint64_t addr) const;
  void l1_write_u32(uint64_t addr, uint32_t value);
  void l1_read_block(uint64_t addr, uint8_t* data, size_t len) const;
  void l1_write_block(uint64_t addr, const uint8_t* data, size_t len);

  // Extract message length from header
  uint32_t extract_msg_length(const uint8_t* header) const;
};

// Register indices per spec
namespace OverlayRegIndex {
  constexpr uint8_t STREAM_MSG_HEADER_FORMAT = 0;
  constexpr uint8_t STREAM_MISC_CFG = 1;
  constexpr uint8_t STREAM_BUF_START = 2;
  constexpr uint8_t STREAM_BUF_SIZE = 3;
  constexpr uint8_t STREAM_MSG_INFO_PTR = 4;
  constexpr uint8_t STREAM_MSG_INFO_WR_PTR = 5;
  constexpr uint8_t STREAM_RD_PTR = 6;
  constexpr uint8_t STREAM_WR_PTR = 7;
  constexpr uint8_t STREAM_REMOTE_SRC = 8;
  constexpr uint8_t STREAM_REMOTE_SRC_PHASE = 9;
  constexpr uint8_t STREAM_REMOTE_DEST = 10;
  constexpr uint8_t STREAM_REMOTE_DEST_BUF_START = 11;
  constexpr uint8_t STREAM_REMOTE_DEST_BUF_SIZE = 12;
  constexpr uint8_t STREAM_REMOTE_DEST_MSG_INFO_WR_PTR = 13;
  constexpr uint8_t STREAM_REMOTE_DEST_WR_PTR = 14;
  constexpr uint8_t STREAM_MCAST_DEST = 15;
  constexpr uint8_t STREAM_MCAST_DEST_NUM = 16;
  constexpr uint8_t STREAM_PHASE_AUTO_CFG_HEADER = 17;
  constexpr uint8_t STREAM_CURR_PHASE = 18;
  constexpr uint8_t STREAM_CURR_PHASE_BASE = 19;
  constexpr uint8_t STREAM_PHASE_ADVANCE = 20;  // Write-only
  constexpr uint8_t STREAM_WAIT_STATUS = 21;    // Read-only
  constexpr uint8_t STREAM_NUM_MSGS_RECEIVED = 22;
  constexpr uint8_t STREAM_NEXT_RECEIVED_MSG_ADDR = 23;
  constexpr uint8_t STREAM_NEXT_RECEIVED_MSG_SIZE = 24;
  constexpr uint8_t STREAM_MSG_INFO_CLEAR = 25;  // Write-only
  constexpr uint8_t STREAM_MSG_DATA_CLEAR = 26;  // Write-only
  constexpr uint8_t STREAM_BUF_SPACE_AVAILABLE = 27;  // Read-only
  constexpr uint8_t STREAM_MSG_INFO_CAN_PUSH_NEW_MSG = 28;  // Read-only
  constexpr uint8_t STREAM_SOURCE_ENDPOINT_NEW_MSG_INFO = 29;  // Write-only
  constexpr uint8_t STREAM_NUM_MSGS_RECEIVED_INC = 30;  // Write-only
  constexpr uint8_t STREAM_MEM_BUF_SPACE_AVAILABLE_ACK_THRESHOLD = 31;

  // Auto-config registers
  constexpr uint8_t STREAM_PHASE_AUTO_CFG_PTR = 32;
  constexpr uint8_t STREAM_PHASE_AUTO_CFG_PTR_BASE = 33;
  constexpr uint8_t STREAM_BLOB_AUTO_CFG_DONE = 34;      // Global (stream 0), 2 regs
  constexpr uint8_t STREAM_BLOB_NEXT_AUTO_CFG_DONE = 36; // Global (stream 0)

  // Gather operation registers
  constexpr uint8_t STREAM_GATHER = 37;
  constexpr uint8_t STREAM_GATHER_CLEAR = 38;
  constexpr uint8_t STREAM_LOCAL_SRC_MASK = 39;  // 3 registers (39, 40, 41)
  constexpr uint8_t STREAM_LOCAL_DEST = 42;
}

#endif