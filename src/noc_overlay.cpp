#include "noc_overlay.h"

#include <iostream>

// ============================================================================
// NoC Overlay Implementation per WormholeB0 Spec
// ============================================================================
//
// The NoC Overlay is a coprocessor that assists with NoC transactions by
// managing message flow between tiles. Each Tensix tile has 64 streams,
// each Ethernet tile has 32 streams.
//
// Streams operate in phases:
// 1. Software configures stream registers
// 2. Software starts the phase (or auto-advance from L1 config)
// 3. Stream receives/transmits configured number of messages
// 4. Phase completes, stream returns to idle or auto-loads next phase
//
// Messages flow through:
// - Receive buffer FIFO (circular, in L1)
// - Message header array (linear, in L1)
// - Message metadata FIFO (in stream hardware)
// - L1 read complete FIFO (for tracking transmitted data)

NocOverlay::NocOverlay(sc_core::sc_module_name name,
                       NocCoord coord_in,
                       sc_core::sc_time cycle_time_in,
                       unsigned num_streams_in)
    : sc_module(name),
      coord(coord_in),
      cycle_time(cycle_time_in),
      num_streams(num_streams_in) {
  streams_.resize(num_streams);

  // Initialize each stream with appropriate capabilities
  for (unsigned i = 0; i < num_streams; ++i) {
    streams_[i].caps = get_stream_caps(i);
    streams_[i].state = OverlayStreamState::Idle;
  }

  SC_THREAD(stream_process_thread);
  SC_THREAD(rx_thread);
  SC_THREAD(tx_thread);
}

StreamCapabilities NocOverlay::get_stream_caps(unsigned stream_id) const {
  StreamCapabilities caps;

  // Per spec Table in README.md
  if (num_streams == 64) {
    // Tensix tile capabilities
    if (stream_id <= 3) {
      caps.can_multicast = true;
      caps.can_receive_gather = true;
      caps.can_transmit_dram = true;
      caps.can_cause_irq = true;
      caps.metadata_includes_header = false;
      caps.metadata_fifo_capacity = 8;
      caps.metadata_fifo_group_size = 2;
      caps.l1_read_complete_fifo_capacity = 8;
    } else if (stream_id <= 5) {
      caps.can_multicast = false;
      caps.can_receive_gather = true;
      caps.can_transmit_dram = false;
      caps.can_cause_irq = false;
      caps.metadata_includes_header = true;  // Streams 4-5 include header copy
      caps.metadata_fifo_capacity = 8;
      caps.metadata_fifo_group_size = 4;
      caps.l1_read_complete_fifo_capacity = 8;
    } else if (stream_id <= 7) {
      caps.can_multicast = false;
      caps.can_receive_gather = false;
      caps.can_transmit_dram = false;
      caps.can_cause_irq = false;
      caps.metadata_includes_header = false;
      caps.metadata_fifo_capacity = 2;
      caps.metadata_fifo_group_size = 2;
      caps.l1_read_complete_fifo_capacity = 2;
    } else if (stream_id <= 11) {
      caps.can_multicast = false;
      caps.can_receive_gather = false;
      caps.can_transmit_dram = true;
      caps.can_cause_irq = true;
      caps.metadata_includes_header = false;
      caps.metadata_fifo_capacity = 8;
      caps.metadata_fifo_group_size = 2;
      caps.l1_read_complete_fifo_capacity = 8;
    } else {
      // Streams 12-63
      caps.can_multicast = false;
      caps.can_receive_gather = false;
      caps.can_transmit_dram = false;
      caps.can_cause_irq = false;
      caps.metadata_includes_header = false;
      caps.metadata_fifo_capacity = 2;
      caps.metadata_fifo_group_size = 2;
      caps.l1_read_complete_fifo_capacity = 2;
    }
  } else {
    // Ethernet tile (32 streams) - simplified for now
    caps.can_multicast = (stream_id <= 1);
    caps.can_receive_gather = (stream_id <= 1);
    caps.can_transmit_dram = (stream_id <= 3) || (stream_id >= 8 && stream_id <= 11);
    caps.can_cause_irq = (stream_id <= 3) || (stream_id >= 8 && stream_id <= 11);
    caps.metadata_includes_header = false;
    caps.metadata_fifo_capacity = (stream_id <= 3 || (stream_id >= 8 && stream_id <= 11)) ? 32 : 2;
    caps.metadata_fifo_group_size = 2;
    caps.l1_read_complete_fifo_capacity = (stream_id <= 3 || (stream_id >= 8 && stream_id <= 11)) ? 128 : 2;
  }

  return caps;
}

void NocOverlay::stream_process_thread() {
  while (true) {
    for (unsigned i = 0; i < num_streams; ++i) {
      process_stream(i);
    }
    wait(cycle_time);
  }
}

void NocOverlay::process_stream(unsigned stream_id) {
  OverlayStream& stream = streams_[stream_id];

  switch (stream.state) {
    case OverlayStreamState::Idle:
      // Waiting for software to configure and start
      break;

    case OverlayStreamState::LoadingConfig:
      // Auto-loading configuration from L1
      load_config_from_l1(stream_id);
      if (stream.regs.phase_auto_advance) {
        // Check if we need to flush previous phase first
        if (!stream.regs.no_prev_phase_outgoing_data_flush &&
            !stream.l1_read_complete_fifo.empty()) {
          stream.state = OverlayStreamState::FlushingPrevPhase;
        } else {
          stream.state = OverlayStreamState::Running;
          start_phase(stream_id);
        }
      } else {
        stream.state = OverlayStreamState::WaitingToStart;
      }
      break;

    case OverlayStreamState::WaitingToStart:
      // Waiting for software to write to STREAM_PHASE_ADVANCE
      break;

    case OverlayStreamState::FlushingPrevPhase:
      // Wait for all L1 reads from previous phase to complete
      // unless NO_PREV_PHASE_OUTGOING_DATA_FLUSH is set
      if (stream.regs.no_prev_phase_outgoing_data_flush ||
          stream.l1_read_complete_fifo.empty()) {
        stream.state = OverlayStreamState::Running;
        start_phase(stream_id);
      }
      break;

    case OverlayStreamState::Running:
      // Normal operation - receiving and transmitting messages
      // Handle handshake protocol for remote transmit
      if (stream.regs.remote_receiver && !stream.regs.dest_data_buf_no_flow_ctrl) {
        if (!stream.handshake_sent) {
          send_handshake_request(stream_id);
        }
      }

      // Handle gather operations for gather output streams
      if (stream.regs.local_sources_connected) {
        process_gather_output(stream_id);
      }

      // Check if phase is complete
      if (stream.regs.num_msgs_received >= stream.regs.num_msgs_to_receive &&
          stream.metadata_fifo.empty() &&
          stream.l1_read_complete_fifo.empty()) {
        complete_phase(stream_id);
      }
      break;
  }
}

void NocOverlay::start_phase(unsigned stream_id) {
  OverlayStream& stream = streams_[stream_id];

  // Extract number of messages from phase config header
  stream.regs.num_msgs_to_receive = (stream.regs.phase_auto_cfg_header >> 12) & 0xFFF;
  stream.regs.num_msgs_received = 0;

  // Reset pointers if source/dest changed
  if (stream.regs.next_phase_src_change) {
    stream.regs.rd_ptr = 0;
    stream.regs.wr_ptr = 0;
    // Reset receiver handshake state
    stream.handshake_request_received = false;
    stream.sent_speculative_response = false;
  }
  if (stream.regs.next_phase_dest_change) {
    stream.regs.remote_dest_wr_ptr = 0;
    // Reset transmitter handshake state - need to handshake with new destination
    stream.handshake_sent = false;
    stream.handshake_response_received = false;
    stream.remote_buf_space_available = 0;
    stream.waiting_for_flow_ctrl = false;
  }

  stream.regs.curr_phase++;
  stream.handshake_phase = stream.regs.curr_phase;

  std::cout << sc_core::sc_time_stamp() << " overlay (" << coord_to_string(coord)
            << ") stream " << stream_id << " starting phase " << stream.regs.curr_phase
            << " msgs=" << stream.regs.num_msgs_to_receive << std::endl;
}

void NocOverlay::complete_phase(unsigned stream_id) {
  OverlayStream& stream = streams_[stream_id];

  std::cout << sc_core::sc_time_stamp() << " overlay (" << coord_to_string(coord)
            << ") stream " << stream_id << " completed phase " << stream.regs.curr_phase
            << std::endl;

  if (stream.regs.phase_auto_config) {
    // Auto-load next phase configuration from L1
    stream.state = OverlayStreamState::LoadingConfig;
  } else {
    stream.state = OverlayStreamState::Idle;
  }
}

void NocOverlay::rx_thread() {
  while (true) {
    if (overlay_in.num_available() > 0) {
      NocFlit flit = overlay_in.read();

      // Route incoming packets to appropriate stream based on stream_id field
      // or from the destination address encoding
      uint8_t dest_stream = flit.stream_id;
      if (dest_stream == 0 && flit.is_header) {
        // Try to extract from address if stream_id not set
        dest_stream = (flit.addr >> 4) & 0x3F;
      }

      if (dest_stream < num_streams) {
        OverlayStream& stream = streams_[dest_stream];

        // Handle different packet types based on stream configuration
        if (stream.state == OverlayStreamState::Running && stream.regs.remote_source) {
          // This is a data packet for this stream
          // Write to receive buffer in L1
          if (l1_mem_ != nullptr) {
            uint64_t buf_addr = (stream.regs.buf_start + stream.regs.wr_ptr) << 4;
            size_t bytes_to_write = std::min(static_cast<size_t>(32), flit.data.size());
            l1_write_block(buf_addr, flit.data.data(), bytes_to_write);

            // If this is a header flit, also write to message header array
            if (flit.is_header) {
              uint64_t hdr_addr = stream.regs.msg_info_wr_ptr << 4;
              l1_write_block(hdr_addr, flit.data.data(), 16);
            }
          }

          // Update write pointer
          stream.regs.wr_ptr += 2;  // 32 bytes = 2 units of 16 bytes
          if (stream.regs.wr_ptr >= stream.regs.buf_size) {
            stream.regs.wr_ptr -= stream.regs.buf_size;  // Wrap around
          }

          // If this completes a message (last flit), update metadata FIFO
          bool is_last_flit = (flit.flit_index + 1 == flit.total_flits);
          if (is_last_flit) {
            uint32_t msg_len = extract_msg_length(flit.data.data());
            push_msg_to_metadata_fifo(dest_stream,
                                      stream.regs.wr_ptr - msg_len,
                                      msg_len);
            stream.regs.num_msgs_received++;
            stream.regs.msg_info_wr_ptr++;
          }
        }
      }
    }
    wait(cycle_time);
  }
}

void NocOverlay::tx_thread() {
  while (true) {
    // Check each stream for messages to transmit
    for (unsigned i = 0; i < num_streams; ++i) {
      OverlayStream& stream = streams_[i];

      if (stream.state == OverlayStreamState::Running &&
          stream.regs.remote_receiver &&
          !stream.metadata_fifo.empty()) {

        OverlayMsgMetadata meta;
        if (pop_msg_from_metadata_fifo(i, meta)) {
          // Create NIU request to transmit this message
          OverlayNiuReq req;
          req.stream_id = i;
          req.dest = {stream.regs.remote_dest_x, stream.regs.remote_dest_y};
          req.dest_addr = (stream.regs.remote_dest_buf_start + stream.regs.remote_dest_wr_ptr) << 4;
          req.dest_addr |= (static_cast<uint64_t>(stream.regs.remote_dest_stream_id) << 4);
          req.src_addr = (stream.regs.buf_start + meta.buf_ptr) << 4;
          req.length = meta.msg_length << 4;
          req.is_posted = stream.regs.dest_data_buf_no_flow_ctrl;
          req.is_multicast = stream.caps.can_multicast && stream.regs.mcast_en;
          req.mcast_end_x = stream.regs.mcast_end_x;
          req.mcast_end_y = stream.regs.mcast_end_y;
          req.mcast_xy = stream.regs.mcast_xy;
          req.vc_class = (stream.regs.unicast_vc_reg >> 1) & 0x3;
          req.vc_buddy = stream.regs.unicast_vc_reg & 0x1;

          if (niu_req_out.num_free() > 0) {
            niu_req_out.write(req);

            // Push length to L1 read complete FIFO
            stream.l1_read_complete_fifo.push_back(meta.msg_length);

            // Update remote destination write pointer
            stream.regs.remote_dest_wr_ptr += meta.msg_length;

            std::cout << sc_core::sc_time_stamp() << " overlay (" << coord_to_string(coord)
                      << ") stream " << i << " transmitting msg len=" << (meta.msg_length << 4)
                      << " to (" << (int)stream.regs.remote_dest_x << ","
                      << (int)stream.regs.remote_dest_y << ")" << std::endl;
          }
        }
      }
    }
    wait(cycle_time);
  }
}

bool NocOverlay::can_push_msg(unsigned stream_id) const {
  const OverlayStream& stream = streams_[stream_id];
  return stream.metadata_fifo.size() < stream.caps.metadata_fifo_capacity &&
         stream.regs.msg_info_ptr == stream.regs.msg_info_wr_ptr;
}

void NocOverlay::push_msg_to_metadata_fifo(unsigned stream_id, uint32_t buf_ptr, uint32_t length) {
  OverlayStream& stream = streams_[stream_id];

  if (stream.metadata_fifo.size() >= stream.caps.metadata_fifo_capacity) {
    return;  // FIFO full
  }

  OverlayMsgMetadata meta;
  meta.buf_ptr = buf_ptr;
  meta.msg_length = length;

  // If this stream includes header copies, read header from L1
  if (stream.caps.metadata_includes_header && l1_mem_ != nullptr) {
    uint64_t hdr_addr = (stream.regs.buf_start + buf_ptr) << 4;
    for (int i = 0; i < 4; ++i) {
      meta.header[i] = l1_read_u32(hdr_addr + i * 4);
    }
  }

  stream.metadata_fifo.push_back(meta);
}

bool NocOverlay::pop_msg_from_metadata_fifo(unsigned stream_id, OverlayMsgMetadata& meta) {
  OverlayStream& stream = streams_[stream_id];

  if (stream.metadata_fifo.empty()) {
    return false;
  }

  meta = stream.metadata_fifo.front();
  stream.metadata_fifo.pop_front();
  return true;
}

uint32_t NocOverlay::l1_read_u32(uint64_t addr) const {
  if (l1_mem_ == nullptr || addr + 4 > l1_mem_->size()) {
    return 0;
  }
  uint32_t value = 0;
  value |= (*l1_mem_)[addr];
  value |= static_cast<uint32_t>((*l1_mem_)[addr + 1]) << 8;
  value |= static_cast<uint32_t>((*l1_mem_)[addr + 2]) << 16;
  value |= static_cast<uint32_t>((*l1_mem_)[addr + 3]) << 24;
  return value;
}

void NocOverlay::l1_write_u32(uint64_t addr, uint32_t value) {
  if (l1_mem_ == nullptr || addr + 4 > l1_mem_->size()) {
    return;
  }
  (*l1_mem_)[addr] = value & 0xFF;
  (*l1_mem_)[addr + 1] = (value >> 8) & 0xFF;
  (*l1_mem_)[addr + 2] = (value >> 16) & 0xFF;
  (*l1_mem_)[addr + 3] = (value >> 24) & 0xFF;
}

void NocOverlay::l1_read_block(uint64_t addr, uint8_t* data, size_t len) const {
  if (l1_mem_ == nullptr) {
    return;
  }
  for (size_t i = 0; i < len && addr + i < l1_mem_->size(); ++i) {
    data[i] = (*l1_mem_)[addr + i];
  }
}

void NocOverlay::l1_write_block(uint64_t addr, const uint8_t* data, size_t len) {
  if (l1_mem_ == nullptr) {
    return;
  }
  for (size_t i = 0; i < len && addr + i < l1_mem_->size(); ++i) {
    (*l1_mem_)[addr + i] = data[i];
  }
}

uint32_t NocOverlay::extract_msg_length(const uint8_t* header) const {
  // Extract message length from header based on configured format
  // Length is in 16-byte units
  uint8_t byte_offset = msg_header_format_.word_cnt_offset / 8;
  uint8_t bit_offset = msg_header_format_.word_cnt_offset % 8;
  uint32_t mask = (1u << msg_header_format_.word_cnt_bits) - 1;

  uint32_t value = 0;
  for (int i = 0; i < 4 && byte_offset + i < 16; ++i) {
    value |= static_cast<uint32_t>(header[byte_offset + i]) << (i * 8);
  }
  return (value >> bit_offset) & mask;
}

// ============================================================================
// Register Access
// ============================================================================

uint32_t NocOverlay::read_reg(uint8_t stream_id, uint8_t reg_id) {
  if (stream_id >= num_streams) {
    return 0;
  }

  OverlayStream& stream = streams_[stream_id];
  using namespace OverlayRegIndex;

  switch (reg_id) {
    case STREAM_MSG_HEADER_FORMAT:
      // Global register, use stream 0
      return (msg_header_format_.word_cnt_offset) |
             (static_cast<uint32_t>(msg_header_format_.word_cnt_bits) << 7);

    case STREAM_MISC_CFG:
      return (stream.regs.outgoing_data_noc << 1) |
             (stream.regs.remote_src_update_noc << 2) |
             (stream.regs.local_sources_connected ? (1u << 3) : 0) |
             (stream.regs.source_endpoint ? (1u << 4) : 0) |
             (stream.regs.remote_source ? (1u << 5) : 0) |
             (stream.regs.receiver_endpoint ? (1u << 6) : 0) |
             (stream.regs.local_receiver ? (1u << 7) : 0) |
             (stream.regs.remote_receiver ? (1u << 8) : 0) |
             (stream.regs.phase_auto_config ? (1u << 9) : 0) |
             (stream.regs.phase_auto_advance ? (1u << 10) : 0) |
             (stream.regs.next_phase_src_change ? (1u << 12) : 0) |
             (stream.regs.next_phase_dest_change ? (1u << 13) : 0) |
             (stream.regs.data_buf_no_flow_ctrl ? (1u << 14) : 0) |
             (stream.regs.dest_data_buf_no_flow_ctrl ? (1u << 15) : 0) |
             (stream.regs.remote_src_is_mcast ? (1u << 16) : 0) |
             (stream.regs.no_prev_phase_outgoing_data_flush ? (1u << 17) : 0) |
             (static_cast<uint32_t>(stream.regs.unicast_vc_reg) << 18) |
             (static_cast<uint32_t>(stream.regs.reg_update_vc_reg) << 21);

    case STREAM_BUF_START:
      return stream.regs.buf_start;

    case STREAM_BUF_SIZE:
      return stream.regs.buf_size;

    case STREAM_MSG_INFO_PTR:
      return stream.regs.msg_info_ptr;

    case STREAM_MSG_INFO_WR_PTR:
      return stream.regs.msg_info_wr_ptr;

    case STREAM_RD_PTR:
      return stream.regs.rd_ptr;

    case STREAM_WR_PTR:
      return stream.regs.wr_ptr;

    case STREAM_REMOTE_SRC:
      return stream.regs.remote_src_x |
             (static_cast<uint32_t>(stream.regs.remote_src_y) << 6) |
             (static_cast<uint32_t>(stream.regs.remote_src_stream_id) << 12) |
             (static_cast<uint32_t>(stream.regs.remote_src_dest_index) << 18);

    case STREAM_REMOTE_SRC_PHASE:
      return stream.regs.remote_src_phase;

    case STREAM_REMOTE_DEST:
      return stream.regs.remote_dest_x |
             (static_cast<uint32_t>(stream.regs.remote_dest_y) << 6) |
             (static_cast<uint32_t>(stream.regs.remote_dest_stream_id) << 12);

    case STREAM_REMOTE_DEST_BUF_START:
      return stream.regs.remote_dest_buf_start;

    case STREAM_REMOTE_DEST_BUF_SIZE:
      return stream.regs.remote_dest_buf_size;

    case STREAM_REMOTE_DEST_MSG_INFO_WR_PTR:
      return stream.regs.remote_dest_msg_info_wr_ptr;

    case STREAM_REMOTE_DEST_WR_PTR:
      return stream.regs.remote_dest_wr_ptr;

    case STREAM_MCAST_DEST:
      return stream.regs.mcast_end_x |
             (static_cast<uint32_t>(stream.regs.mcast_end_y) << 6) |
             (stream.regs.mcast_en ? (1u << 12) : 0) |
             (stream.regs.mcast_linked ? (1u << 13) : 0) |
             (static_cast<uint32_t>(stream.regs.mcast_vc) << 14) |
             (stream.regs.mcast_no_path_res ? (1u << 15) : 0) |
             (stream.regs.mcast_xy ? (1u << 16) : 0);

    case STREAM_MCAST_DEST_NUM:
      return stream.regs.mcast_dest_num;

    case STREAM_PHASE_AUTO_CFG_HEADER:
      return stream.regs.phase_auto_cfg_header;

    case STREAM_CURR_PHASE:
      return stream.regs.curr_phase;

    case STREAM_CURR_PHASE_BASE:
      return stream.regs.curr_phase_base;

    case STREAM_WAIT_STATUS: {
      // Read-only status register
      uint32_t status = 0;
      bool wait_sw = (stream.state == OverlayStreamState::Idle ||
                      stream.state == OverlayStreamState::WaitingToStart);
      bool wait_flush = (stream.state == OverlayStreamState::FlushingPrevPhase);
      bool msg_fwd = (stream.state == OverlayStreamState::Running);
      status |= (wait_sw ? 1u : 0);
      status |= (wait_flush ? (1u << 1) : 0);
      status |= (msg_fwd ? (1u << 2) : 0);
      status |= (static_cast<uint32_t>(stream.state) << 3);
      return status;
    }

    case STREAM_NUM_MSGS_RECEIVED:
      return static_cast<uint32_t>(stream.metadata_fifo.size());

    case STREAM_NEXT_RECEIVED_MSG_ADDR:
      if (!stream.metadata_fifo.empty()) {
        return stream.metadata_fifo.front().buf_ptr;
      }
      return 0;

    case STREAM_NEXT_RECEIVED_MSG_SIZE:
      if (!stream.metadata_fifo.empty()) {
        return stream.metadata_fifo.front().msg_length;
      }
      return 0;

    case STREAM_BUF_SPACE_AVAILABLE:
      // Return available space in receive buffer
      if (stream.regs.rd_ptr == stream.regs.wr_ptr && stream.metadata_fifo.empty()) {
        return stream.regs.buf_size;
      }
      return (stream.regs.rd_ptr - stream.regs.wr_ptr + stream.regs.buf_size) % stream.regs.buf_size;

    case STREAM_MSG_INFO_CAN_PUSH_NEW_MSG:
      return can_push_msg(stream_id) ? 1 : 0;

    case STREAM_MEM_BUF_SPACE_AVAILABLE_ACK_THRESHOLD:
      return stream.regs.buf_space_ack_threshold;

    // Auto-config registers
    case STREAM_PHASE_AUTO_CFG_PTR:
      return stream.regs.phase_auto_cfg_ptr;

    case STREAM_PHASE_AUTO_CFG_PTR_BASE:
      return stream.regs.phase_auto_cfg_ptr_base;

    case STREAM_BLOB_AUTO_CFG_DONE:
      // Global register (stream 0): lower 32 bits of 64-bit bitmask
      return static_cast<uint32_t>(blob_auto_cfg_done_ & 0xFFFFFFFF);

    case STREAM_BLOB_AUTO_CFG_DONE + 1:
      // Global register (stream 0): upper 32 bits of 64-bit bitmask
      return static_cast<uint32_t>(blob_auto_cfg_done_ >> 32);

    case STREAM_BLOB_NEXT_AUTO_CFG_DONE: {
      // Global register: find next stream with auto-config done, round-robin
      uint8_t start_idx = next_auto_cfg_done_idx_;
      for (unsigned i = 0; i < num_streams; ++i) {
        uint8_t idx = (start_idx + i) % num_streams;
        if (blob_auto_cfg_done_ & (1ull << idx)) {
          next_auto_cfg_done_idx_ = (idx + 1) % num_streams;
          return idx;
        }
      }
      return 0x3F;  // No stream with auto-config done
    }

    // Gather operation registers
    case STREAM_GATHER:
      return (stream.regs.gather_msg_arb_group_size) |
             (stream.regs.gather_msg_src_in_order_fwd ? (1u << 2) : 0) |
             (static_cast<uint32_t>(stream.regs.gather_msg_local_stream_clear_num) << 3) |
             (stream.regs.gather_msg_group_stream_clear_type ? (1u << 19) : 0);

    case STREAM_LOCAL_SRC_MASK:
      return stream.regs.local_src_mask[0];

    case STREAM_LOCAL_SRC_MASK + 1:
      return stream.regs.local_src_mask[1];

    case STREAM_LOCAL_SRC_MASK + 2:
      return stream.regs.local_src_mask[2];

    case STREAM_LOCAL_DEST:
      return stream.regs.local_dest_stream_id |
             (static_cast<uint32_t>(stream.regs.local_dest_msg_clear_num) << 6);

    default:
      return 0;
  }
}

void NocOverlay::write_reg(uint8_t stream_id, uint8_t reg_id, uint32_t value) {
  if (stream_id >= num_streams) {
    return;
  }

  OverlayStream& stream = streams_[stream_id];
  using namespace OverlayRegIndex;

  switch (reg_id) {
    case STREAM_MSG_HEADER_FORMAT:
      // Global register
      msg_header_format_.word_cnt_offset = value & 0x7F;
      msg_header_format_.word_cnt_bits = (value >> 7) & 0x7F;
      break;

    case STREAM_MISC_CFG:
      stream.regs.outgoing_data_noc = (value >> 1) & 0x1;
      stream.regs.remote_src_update_noc = (value >> 2) & 0x1;
      stream.regs.local_sources_connected = (value >> 3) & 0x1;
      stream.regs.source_endpoint = (value >> 4) & 0x1;
      stream.regs.remote_source = (value >> 5) & 0x1;
      stream.regs.receiver_endpoint = (value >> 6) & 0x1;
      stream.regs.local_receiver = (value >> 7) & 0x1;
      stream.regs.remote_receiver = (value >> 8) & 0x1;
      stream.regs.phase_auto_config = (value >> 9) & 0x1;
      stream.regs.phase_auto_advance = (value >> 10) & 0x1;
      stream.regs.next_phase_src_change = (value >> 12) & 0x1;
      stream.regs.next_phase_dest_change = (value >> 13) & 0x1;
      stream.regs.data_buf_no_flow_ctrl = (value >> 14) & 0x1;
      stream.regs.dest_data_buf_no_flow_ctrl = (value >> 15) & 0x1;
      stream.regs.remote_src_is_mcast = (value >> 16) & 0x1;
      stream.regs.no_prev_phase_outgoing_data_flush = (value >> 17) & 0x1;
      stream.regs.unicast_vc_reg = (value >> 18) & 0x7;
      stream.regs.reg_update_vc_reg = (value >> 21) & 0x7;
      break;

    case STREAM_BUF_START:
      stream.regs.buf_start = value;
      break;

    case STREAM_BUF_SIZE:
      stream.regs.buf_size = value;
      break;

    case STREAM_MSG_INFO_PTR:
      stream.regs.msg_info_ptr = value;
      break;

    case STREAM_MSG_INFO_WR_PTR:
      stream.regs.msg_info_wr_ptr = value;
      break;

    case STREAM_RD_PTR:
      stream.regs.rd_ptr = value;
      break;

    case STREAM_WR_PTR:
      stream.regs.wr_ptr = value;
      break;

    case STREAM_REMOTE_SRC:
      stream.regs.remote_src_x = value & 0x3F;
      stream.regs.remote_src_y = (value >> 6) & 0x3F;
      stream.regs.remote_src_stream_id = (value >> 12) & 0x3F;
      stream.regs.remote_src_dest_index = (value >> 18) & 0x3F;
      break;

    case STREAM_REMOTE_SRC_PHASE:
      stream.regs.remote_src_phase = value;
      break;

    case STREAM_REMOTE_DEST:
      stream.regs.remote_dest_x = value & 0x3F;
      stream.regs.remote_dest_y = (value >> 6) & 0x3F;
      stream.regs.remote_dest_stream_id = (value >> 12) & 0x3F;
      break;

    case STREAM_REMOTE_DEST_BUF_START:
      stream.regs.remote_dest_buf_start = value;
      break;

    case STREAM_REMOTE_DEST_BUF_SIZE:
      stream.regs.remote_dest_buf_size = value;
      break;

    case STREAM_REMOTE_DEST_MSG_INFO_WR_PTR:
      stream.regs.remote_dest_msg_info_wr_ptr = value;
      break;

    case STREAM_REMOTE_DEST_WR_PTR:
      stream.regs.remote_dest_wr_ptr = value;
      break;

    case STREAM_MCAST_DEST:
      if (stream.caps.can_multicast) {
        stream.regs.mcast_end_x = value & 0x3F;
        stream.regs.mcast_end_y = (value >> 6) & 0x3F;
        stream.regs.mcast_en = (value >> 12) & 0x1;
        stream.regs.mcast_linked = (value >> 13) & 0x1;
        stream.regs.mcast_vc = (value >> 14) & 0x1;
        stream.regs.mcast_no_path_res = (value >> 15) & 0x1;
        stream.regs.mcast_xy = (value >> 16) & 0x1;
      }
      break;

    case STREAM_MCAST_DEST_NUM:
      if (stream.caps.can_multicast) {
        stream.regs.mcast_dest_num = value & 0x1F;
      }
      break;

    case STREAM_PHASE_AUTO_CFG_HEADER:
      stream.regs.phase_auto_cfg_header = value;
      stream.regs.num_msgs_to_receive = (value >> 12) & 0xFFF;
      break;

    case STREAM_CURR_PHASE_BASE:
      stream.regs.curr_phase_base = value;
      break;

    case STREAM_PHASE_ADVANCE:
      // Write-only: start the phase
      if (stream.state == OverlayStreamState::Idle ||
          stream.state == OverlayStreamState::WaitingToStart) {
        if (!stream.l1_read_complete_fifo.empty()) {
          stream.state = OverlayStreamState::FlushingPrevPhase;
        } else {
          stream.state = OverlayStreamState::Running;
          start_phase(stream_id);
        }
      }
      break;

    case STREAM_MSG_INFO_CLEAR: {
      // Pop entries from metadata FIFO and push lengths to L1 read complete FIFO
      unsigned num_to_pop = value & 0xFFF;
      if (num_to_pop > stream.caps.metadata_fifo_group_size) {
        num_to_pop = stream.caps.metadata_fifo_group_size;
      }
      uint32_t total_length = 0;
      for (unsigned i = 0; i < num_to_pop && !stream.metadata_fifo.empty(); ++i) {
        total_length += stream.metadata_fifo.front().msg_length;
        stream.metadata_fifo.pop_front();
      }
      if (total_length > 0) {
        stream.l1_read_complete_fifo.push_back(total_length);
      }
      break;
    }

    case STREAM_MSG_DATA_CLEAR:
      // Pop from L1 read complete FIFO and advance read pointer
      if (!stream.l1_read_complete_fifo.empty()) {
        uint32_t length = stream.l1_read_complete_fifo.front();
        stream.l1_read_complete_fifo.pop_front();
        stream.regs.rd_ptr += length;
        if (stream.regs.rd_ptr >= stream.regs.buf_size) {
          stream.regs.rd_ptr -= stream.regs.buf_size;
        }
      }
      break;

    case STREAM_SOURCE_ENDPOINT_NEW_MSG_INFO: {
      // Push message from software to metadata FIFO
      uint32_t buf_ptr = value & 0x1FFFF;
      uint32_t msg_len = value >> 17;
      push_msg_to_metadata_fifo(stream_id, buf_ptr, msg_len);
      stream.regs.msg_info_ptr++;
      stream.regs.msg_info_wr_ptr++;
      stream.regs.wr_ptr += msg_len;
      if (stream.regs.wr_ptr >= stream.regs.buf_size) {
        stream.regs.wr_ptr -= stream.regs.buf_size;
      }
      break;
    }

    case STREAM_NUM_MSGS_RECEIVED_INC: {
      // Increment counters for received messages
      unsigned num_msgs = value & 0xFFF;
      uint32_t total_length = value >> 12;
      stream.regs.msg_info_wr_ptr += num_msgs;
      stream.regs.wr_ptr += total_length;
      if (stream.regs.wr_ptr >= stream.regs.buf_size) {
        stream.regs.wr_ptr -= stream.regs.buf_size;
      }
      break;
    }

    case STREAM_MEM_BUF_SPACE_AVAILABLE_ACK_THRESHOLD:
      stream.regs.buf_space_ack_threshold = value & 0xF;
      break;

    // Auto-config registers
    case STREAM_PHASE_AUTO_CFG_PTR:
      stream.regs.phase_auto_cfg_ptr = value;
      break;

    case STREAM_PHASE_AUTO_CFG_PTR_BASE:
      stream.regs.phase_auto_cfg_ptr_base = value;
      break;

    case STREAM_BLOB_AUTO_CFG_DONE:
      // Global register (stream 0): lower 32 bits - write clears bits
      blob_auto_cfg_done_ &= ~static_cast<uint64_t>(value);
      break;

    case STREAM_BLOB_AUTO_CFG_DONE + 1:
      // Global register (stream 0): upper 32 bits - write clears bits
      blob_auto_cfg_done_ &= ~(static_cast<uint64_t>(value) << 32);
      break;

    // Gather operation registers
    case STREAM_GATHER:
      if (stream.caps.can_receive_gather) {
        stream.regs.gather_msg_arb_group_size = value & 0x3;
        stream.regs.gather_msg_src_in_order_fwd = (value >> 2) & 0x1;
        stream.regs.gather_msg_local_stream_clear_num = (value >> 3) & 0xFFFF;
        stream.regs.gather_msg_group_stream_clear_type = (value >> 19) & 0x1;
      }
      break;

    case STREAM_GATHER_CLEAR:
      // Clear gather input state for specified streams
      if (stream.caps.can_receive_gather) {
        // Reset gather group tracking
        stream.current_gather_group = 0;
        stream.current_gather_stream_in_group = 0;
        stream.msgs_received_from_current_stream = 0;
      }
      break;

    case STREAM_LOCAL_SRC_MASK:
      if (stream.caps.can_receive_gather) {
        stream.regs.local_src_mask[0] = value;
      }
      break;

    case STREAM_LOCAL_SRC_MASK + 1:
      if (stream.caps.can_receive_gather) {
        stream.regs.local_src_mask[1] = value;
      }
      break;

    case STREAM_LOCAL_SRC_MASK + 2:
      if (stream.caps.can_receive_gather) {
        stream.regs.local_src_mask[2] = value;
      }
      break;

    case STREAM_LOCAL_DEST:
      stream.regs.local_dest_stream_id = value & 0x3F;
      stream.regs.local_dest_msg_clear_num = (value >> 6) & 0xFFFF;
      break;

    default:
      break;
  }
}

// ============================================================================
// Auto-Config from L1
// ============================================================================

void NocOverlay::load_config_from_l1(unsigned stream_id) {
  if (l1_mem_ == nullptr) {
    return;
  }

  OverlayStream& stream = streams_[stream_id];

  // Read configuration blob from L1 at phase_auto_cfg_ptr
  uint64_t cfg_addr = stream.regs.phase_auto_cfg_ptr << 4;

  // First word is the phase auto-config header
  stream.regs.phase_auto_cfg_header = l1_read_u32(cfg_addr);
  cfg_addr += 4;

  // Extract number of register writes from header (bits 0-3, stored as N-1)
  uint8_t num_reg_writes = (stream.regs.phase_auto_cfg_header & 0xF) + 1;
  stream.regs.next_phase_num_cfg_reg_writes = num_reg_writes;

  // Each register write is a pair of words: (reg_index << 24 | value)
  // or two separate words depending on format
  for (uint8_t i = 0; i < num_reg_writes && i < 32; ++i) {
    uint32_t cfg_word = l1_read_u32(cfg_addr);
    cfg_addr += 4;

    // Extract register index and value
    // Format: upper 8 bits = register index, lower 24 bits = value (or full next word)
    uint8_t reg_idx = (cfg_word >> 24) & 0xFF;
    uint32_t reg_value = cfg_word & 0x00FFFFFF;

    // For full 32-bit values, read next word
    if (reg_idx >= 128) {
      reg_idx -= 128;
      reg_value = l1_read_u32(cfg_addr);
      cfg_addr += 4;
    }

    // Write the register (don't recurse into auto-config registers)
    if (reg_idx < OverlayRegIndex::STREAM_PHASE_AUTO_CFG_PTR) {
      write_reg(stream_id, reg_idx, reg_value);
    }
  }

  // Update auto-config pointer for next phase
  stream.regs.phase_auto_cfg_ptr = static_cast<uint32_t>(cfg_addr >> 4);

  // Mark this stream's auto-config as done
  blob_auto_cfg_done_ |= (1ull << stream_id);

  std::cout << sc_core::sc_time_stamp() << " overlay (" << coord_to_string(coord)
            << ") stream " << stream_id << " loaded config from L1, "
            << (int)num_reg_writes << " registers" << std::endl;
}

// ============================================================================
// Handshake Protocol
// ============================================================================

void NocOverlay::send_handshake_request(unsigned stream_id) {
  OverlayStream& stream = streams_[stream_id];

  if (stream.handshake_sent) {
    return;  // Already sent
  }

  // Create handshake request packet
  // This goes to the remote destination stream
  NocFlit flit;
  flit.is_header = true;
  flit.addr = (static_cast<uint64_t>(stream.regs.remote_dest_stream_id) << 4);
  flit.src = coord;
  flit.dst = {stream.regs.remote_dest_x, stream.regs.remote_dest_y};
  flit.stream_id = stream.regs.remote_dest_stream_id;
  flit.flit_index = 0;
  flit.total_flits = 1;

  // Encode handshake packet type and phase in data
  flit.data.fill(0);
  flit.data[0] = static_cast<uint8_t>(OverlayPacketType::HandshakeRequest);
  flit.data[1] = stream_id;  // Source stream ID
  // Encode current phase (little-endian)
  flit.data[4] = stream.regs.curr_phase & 0xFF;
  flit.data[5] = (stream.regs.curr_phase >> 8) & 0xFF;
  flit.data[6] = (stream.regs.curr_phase >> 16) & 0xFF;
  flit.data[7] = (stream.regs.curr_phase >> 24) & 0xFF;

  if (overlay_out.num_free() > 0) {
    overlay_out.write(flit);
    stream.handshake_sent = true;
    stream.handshake_phase = stream.regs.curr_phase;

    std::cout << sc_core::sc_time_stamp() << " overlay (" << coord_to_string(coord)
              << ") stream " << stream_id << " sent handshake request for phase "
              << stream.regs.curr_phase << std::endl;
  }
}

void NocOverlay::send_handshake_response(unsigned stream_id) {
  OverlayStream& stream = streams_[stream_id];

  // Create handshake response packet
  NocFlit flit;
  flit.is_header = true;
  flit.addr = (static_cast<uint64_t>(stream.regs.remote_src_stream_id) << 4);
  flit.src = coord;
  flit.dst = {stream.regs.remote_src_x, stream.regs.remote_src_y};
  flit.stream_id = stream.regs.remote_src_stream_id;
  flit.flit_index = 0;
  flit.total_flits = 1;

  // Encode handshake response with buffer space available
  flit.data.fill(0);
  flit.data[0] = static_cast<uint8_t>(OverlayPacketType::HandshakeResponse);
  flit.data[1] = stream_id;  // This stream's ID

  // Calculate available buffer space
  uint32_t space = calculate_flow_control_threshold(stream_id);
  flit.data[4] = space & 0xFF;
  flit.data[5] = (space >> 8) & 0xFF;
  flit.data[6] = (space >> 16) & 0xFF;
  flit.data[7] = (space >> 24) & 0xFF;

  // Include current phase
  flit.data[8] = stream.regs.curr_phase & 0xFF;
  flit.data[9] = (stream.regs.curr_phase >> 8) & 0xFF;
  flit.data[10] = (stream.regs.curr_phase >> 16) & 0xFF;
  flit.data[11] = (stream.regs.curr_phase >> 24) & 0xFF;

  if (overlay_out.num_free() > 0) {
    overlay_out.write(flit);
    stream.sent_speculative_response = true;

    std::cout << sc_core::sc_time_stamp() << " overlay (" << coord_to_string(coord)
              << ") stream " << stream_id << " sent handshake response, space="
              << space << std::endl;
  }
}

void NocOverlay::handle_handshake_request(unsigned stream_id, uint32_t phase) {
  OverlayStream& stream = streams_[stream_id];

  // Receiver got a handshake request from transmitter
  stream.handshake_request_received = true;

  // If phases match and we're ready, send response
  if (stream.state == OverlayStreamState::Running &&
      stream.regs.curr_phase == phase) {
    send_handshake_response(stream_id);
  }
  // Otherwise, queue speculative response when we reach that phase
}

void NocOverlay::handle_handshake_response(unsigned stream_id, uint32_t phase) {
  OverlayStream& stream = streams_[stream_id];

  // Transmitter got response from receiver
  if (stream.handshake_phase == phase) {
    stream.handshake_response_received = true;
    // Extract available buffer space from response
    // (This would be in the packet data, simplified here)
    stream.remote_buf_space_available = stream.regs.remote_dest_buf_size;
    stream.waiting_for_flow_ctrl = false;

    std::cout << sc_core::sc_time_stamp() << " overlay (" << coord_to_string(coord)
              << ") stream " << stream_id << " received handshake response for phase "
              << phase << std::endl;
  }
}

// ============================================================================
// Flow Control
// ============================================================================

void NocOverlay::send_flow_control_update(unsigned stream_id) {
  OverlayStream& stream = streams_[stream_id];

  // Create flow control update packet
  NocFlit flit;
  flit.is_header = true;
  flit.addr = (static_cast<uint64_t>(stream.regs.remote_src_stream_id) << 4);
  flit.src = coord;
  flit.dst = {stream.regs.remote_src_x, stream.regs.remote_src_y};
  flit.stream_id = stream.regs.remote_src_stream_id;
  flit.flit_index = 0;
  flit.total_flits = 1;

  flit.data.fill(0);
  flit.data[0] = static_cast<uint8_t>(OverlayPacketType::FlowControlUpdate);
  flit.data[1] = stream_id;

  uint32_t space = calculate_flow_control_threshold(stream_id);
  flit.data[4] = space & 0xFF;
  flit.data[5] = (space >> 8) & 0xFF;
  flit.data[6] = (space >> 16) & 0xFF;
  flit.data[7] = (space >> 24) & 0xFF;

  if (overlay_out.num_free() > 0) {
    overlay_out.write(flit);
    stream.regs.last_ack_space_available = space;
  }
}

void NocOverlay::handle_flow_control_update(unsigned stream_id, uint32_t space_available) {
  OverlayStream& stream = streams_[stream_id];

  stream.remote_buf_space_available = space_available;
  stream.waiting_for_flow_ctrl = false;
}

bool NocOverlay::should_send_flow_control(unsigned stream_id) const {
  const OverlayStream& stream = streams_[stream_id];

  if (stream.regs.data_buf_no_flow_ctrl) {
    return false;
  }

  // Send flow control when available space crosses threshold
  uint32_t current_space = calculate_flow_control_threshold(stream_id);
  uint32_t threshold = (1u << stream.regs.buf_space_ack_threshold) << 4;

  return (current_space >= stream.regs.last_ack_space_available + threshold);
}

uint32_t NocOverlay::calculate_flow_control_threshold(unsigned stream_id) const {
  const OverlayStream& stream = streams_[stream_id];

  // Calculate available space in receive buffer
  if (stream.regs.buf_size == 0) {
    return 0;
  }

  uint32_t used = (stream.regs.wr_ptr - stream.regs.rd_ptr + stream.regs.buf_size) %
                  stream.regs.buf_size;
  return stream.regs.buf_size - used;
}

// ============================================================================
// Gather Operations
// ============================================================================

void NocOverlay::process_gather_output(unsigned stream_id) {
  OverlayStream& stream = streams_[stream_id];

  if (!stream.caps.can_receive_gather) {
    return;
  }

  // Get the local source mask (64 bits spread across 3 registers)
  uint64_t src_mask = stream.regs.local_src_mask[0] |
                      (static_cast<uint64_t>(stream.regs.local_src_mask[1]) << 32);

  if (src_mask == 0) {
    return;
  }

  // Find active input streams in current group
  uint8_t group_size = stream.regs.gather_msg_arb_group_size;
  if (group_size == 0) group_size = 1;

  uint8_t group_start = stream.current_gather_group * group_size;

  // Check if current group is ready
  if (is_gather_group_ready(stream_id, group_start)) {
    // Receive from current stream in group
    uint8_t input_idx = group_start + stream.current_gather_stream_in_group;

    // Find the actual input stream ID from the mask
    uint8_t actual_input = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < 64 && count <= input_idx; ++i) {
      if (src_mask & (1ull << i)) {
        if (count == input_idx) {
          actual_input = i;
          break;
        }
        count++;
      }
    }

    if (actual_input < num_streams) {
      receive_from_gather_input(stream_id, actual_input);
    }
  }
}

bool NocOverlay::is_gather_group_ready(unsigned output_stream, unsigned group_start) const {
  const OverlayStream& stream = streams_[output_stream];

  uint8_t group_size = stream.regs.gather_msg_arb_group_size;
  if (group_size == 0) group_size = 1;

  uint64_t src_mask = stream.regs.local_src_mask[0] |
                      (static_cast<uint64_t>(stream.regs.local_src_mask[1]) << 32);

  // Check if all streams in group have messages ready
  uint8_t streams_checked = 0;
  uint8_t mask_idx = 0;
  for (uint8_t i = 0; i < 64 && streams_checked < group_size; ++i) {
    if (src_mask & (1ull << i)) {
      if (mask_idx >= group_start && mask_idx < group_start + group_size) {
        // This stream is in our group - check if it has messages
        if (i < num_streams && !streams_[i].metadata_fifo.empty()) {
          streams_checked++;
        } else {
          return false;  // Stream not ready
        }
      }
      mask_idx++;
    }
  }

  return streams_checked == group_size;
}

void NocOverlay::receive_from_gather_input(unsigned output_stream, unsigned input_stream) {
  if (input_stream >= num_streams || output_stream >= num_streams) {
    return;
  }

  OverlayStream& out = streams_[output_stream];
  OverlayStream& in = streams_[input_stream];

  if (in.metadata_fifo.empty()) {
    return;
  }

  // Pop message from input stream's metadata FIFO
  OverlayMsgMetadata meta;
  if (!pop_msg_from_metadata_fifo(input_stream, meta)) {
    return;
  }

  // Push to output stream's metadata FIFO
  push_msg_to_metadata_fifo(output_stream, meta.buf_ptr, meta.msg_length);

  out.msgs_received_from_current_stream++;

  // Check if we should move to next stream in group
  if (out.msgs_received_from_current_stream >= out.regs.gather_msg_local_stream_clear_num) {
    out.msgs_received_from_current_stream = 0;
    out.current_gather_stream_in_group++;

    uint8_t group_size = out.regs.gather_msg_arb_group_size;
    if (group_size == 0) group_size = 1;

    if (out.current_gather_stream_in_group >= group_size) {
      // Move to next group
      out.current_gather_stream_in_group = 0;
      out.current_gather_group++;

      // Count total groups
      uint64_t src_mask = out.regs.local_src_mask[0] |
                          (static_cast<uint64_t>(out.regs.local_src_mask[1]) << 32);
      uint8_t total_inputs = 0;
      for (uint8_t i = 0; i < 64; ++i) {
        if (src_mask & (1ull << i)) total_inputs++;
      }
      uint8_t num_groups = (total_inputs + group_size - 1) / group_size;

      if (out.current_gather_group >= num_groups) {
        out.current_gather_group = 0;  // Wrap around
      }
    }
  }

  std::cout << sc_core::sc_time_stamp() << " overlay (" << coord_to_string(coord)
            << ") gather: stream " << input_stream << " -> stream " << output_stream
            << " len=" << meta.msg_length << std::endl;
}
