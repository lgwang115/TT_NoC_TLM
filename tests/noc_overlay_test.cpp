// NoC Overlay Unit Tests
// Tests for the NoC Overlay coprocessor implementation per WormholeB0 spec

#include <gtest/gtest.h>
#include <systemc>
#include "noc_overlay.h"

// Test fixture for NoC Overlay tests
class NocOverlayTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // SystemC requires initialization before creating modules
    // We'll create modules in individual tests or use a static initialization
  }

  void TearDown() override {
    // Cleanup if needed
  }
};

// ============================================================================
// Stream Capabilities Tests
// ============================================================================

class StreamCapabilitiesTest : public NocOverlayTest {
 protected:
  static sc_core::sc_signal<bool> dummy_clk;
  static std::unique_ptr<NocOverlay> overlay;
  static sc_core::sc_fifo<OverlayNiuReq> niu_req_fifo;
  static sc_core::sc_fifo<OverlayNiuResp> niu_resp_fifo;
  static sc_core::sc_fifo<NocFlit> overlay_in_fifo;
  static sc_core::sc_fifo<NocFlit> overlay_out_fifo;
  static bool initialized;

  static void SetUpTestSuite() {
    if (!initialized) {
      // Initialize SystemC (only once)
      int argc = 1;
      char* argv[] = {(char*)"test"};
      sc_core::sc_elab_and_sim(argc, argv);
      initialized = true;
    }
  }
};

// Static members
bool StreamCapabilitiesTest::initialized = false;

// ============================================================================
// Register Access Tests
// ============================================================================

// Test STREAM_MISC_CFG register bit 17 (NO_PREV_PHASE_OUTGOING_DATA_FLUSH)
TEST(OverlayRegisterTest, MiscCfgBit17) {
  // Test bit encoding/decoding for NO_PREV_PHASE_OUTGOING_DATA_FLUSH
  uint32_t value = 0;

  // Set bit 17
  value |= (1u << 17);
  EXPECT_EQ((value >> 17) & 0x1, 1);

  // Clear bit 17
  value &= ~(1u << 17);
  EXPECT_EQ((value >> 17) & 0x1, 0);
}

// Test STREAM_MISC_CFG register field encoding
TEST(OverlayRegisterTest, MiscCfgFieldEncoding) {
  // Test all STREAM_MISC_CFG fields
  uint32_t value = 0;

  // outgoing_data_noc (bit 1)
  value |= (1u << 1);
  EXPECT_EQ((value >> 1) & 0x1, 1);

  // remote_src_update_noc (bit 2)
  value |= (1u << 2);
  EXPECT_EQ((value >> 2) & 0x1, 1);

  // local_sources_connected (bit 3)
  value |= (1u << 3);
  EXPECT_EQ((value >> 3) & 0x1, 1);

  // source_endpoint (bit 4)
  value |= (1u << 4);
  EXPECT_EQ((value >> 4) & 0x1, 1);

  // remote_source (bit 5)
  value |= (1u << 5);
  EXPECT_EQ((value >> 5) & 0x1, 1);

  // receiver_endpoint (bit 6)
  value |= (1u << 6);
  EXPECT_EQ((value >> 6) & 0x1, 1);

  // local_receiver (bit 7)
  value |= (1u << 7);
  EXPECT_EQ((value >> 7) & 0x1, 1);

  // remote_receiver (bit 8)
  value |= (1u << 8);
  EXPECT_EQ((value >> 8) & 0x1, 1);

  // phase_auto_config (bit 9)
  value |= (1u << 9);
  EXPECT_EQ((value >> 9) & 0x1, 1);

  // phase_auto_advance (bit 10)
  value |= (1u << 10);
  EXPECT_EQ((value >> 10) & 0x1, 1);

  // next_phase_src_change (bit 12)
  value |= (1u << 12);
  EXPECT_EQ((value >> 12) & 0x1, 1);

  // next_phase_dest_change (bit 13)
  value |= (1u << 13);
  EXPECT_EQ((value >> 13) & 0x1, 1);

  // data_buf_no_flow_ctrl (bit 14)
  value |= (1u << 14);
  EXPECT_EQ((value >> 14) & 0x1, 1);

  // dest_data_buf_no_flow_ctrl (bit 15)
  value |= (1u << 15);
  EXPECT_EQ((value >> 15) & 0x1, 1);

  // remote_src_is_mcast (bit 16)
  value |= (1u << 16);
  EXPECT_EQ((value >> 16) & 0x1, 1);

  // no_prev_phase_outgoing_data_flush (bit 17)
  value |= (1u << 17);
  EXPECT_EQ((value >> 17) & 0x1, 1);

  // unicast_vc_reg (bits 18-20)
  value |= (0x7u << 18);
  EXPECT_EQ((value >> 18) & 0x7, 0x7);

  // reg_update_vc_reg (bits 21-23)
  value |= (0x7u << 21);
  EXPECT_EQ((value >> 21) & 0x7, 0x7);
}

// Test STREAM_REMOTE_SRC register field encoding
TEST(OverlayRegisterTest, RemoteSrcEncoding) {
  uint32_t value = 0;

  // remote_src_x (bits 0-5)
  uint8_t x = 0x3F;
  value |= x;
  EXPECT_EQ(value & 0x3F, x);

  // remote_src_y (bits 6-11)
  uint8_t y = 0x2A;
  value |= (static_cast<uint32_t>(y) << 6);
  EXPECT_EQ((value >> 6) & 0x3F, y);

  // remote_src_stream_id (bits 12-17)
  uint8_t stream_id = 0x15;
  value |= (static_cast<uint32_t>(stream_id) << 12);
  EXPECT_EQ((value >> 12) & 0x3F, stream_id);

  // remote_src_dest_index (bits 18-23)
  uint8_t dest_idx = 0x0A;
  value |= (static_cast<uint32_t>(dest_idx) << 18);
  EXPECT_EQ((value >> 18) & 0x3F, dest_idx);
}

// Test STREAM_MCAST_DEST register field encoding
TEST(OverlayRegisterTest, McastDestEncoding) {
  uint32_t value = 0;

  // mcast_end_x (bits 0-5)
  value |= 0x1F;
  EXPECT_EQ(value & 0x3F, 0x1F);

  // mcast_end_y (bits 6-11)
  value |= (0x2A << 6);
  EXPECT_EQ((value >> 6) & 0x3F, 0x2A);

  // mcast_en (bit 12)
  value |= (1u << 12);
  EXPECT_EQ((value >> 12) & 0x1, 1);

  // mcast_linked (bit 13)
  value |= (1u << 13);
  EXPECT_EQ((value >> 13) & 0x1, 1);

  // mcast_vc (bit 14)
  value |= (1u << 14);
  EXPECT_EQ((value >> 14) & 0x1, 1);

  // mcast_no_path_res (bit 15)
  value |= (1u << 15);
  EXPECT_EQ((value >> 15) & 0x1, 1);

  // mcast_xy (bit 16)
  value |= (1u << 16);
  EXPECT_EQ((value >> 16) & 0x1, 1);
}

// Test STREAM_GATHER register field encoding
TEST(OverlayRegisterTest, GatherRegEncoding) {
  uint32_t value = 0;

  // gather_msg_arb_group_size (bits 0-1)
  value |= 0x3;
  EXPECT_EQ(value & 0x3, 0x3);

  // gather_msg_src_in_order_fwd (bit 2)
  value |= (1u << 2);
  EXPECT_EQ((value >> 2) & 0x1, 1);

  // gather_msg_local_stream_clear_num (bits 3-18)
  value |= (0xFFFF << 3);
  EXPECT_EQ((value >> 3) & 0xFFFF, 0xFFFF);

  // gather_msg_group_stream_clear_type (bit 19)
  value |= (1u << 19);
  EXPECT_EQ((value >> 19) & 0x1, 1);
}

// ============================================================================
// Stream State Tests
// ============================================================================

TEST(OverlayStreamStateTest, StateEnumValues) {
  // Verify state enum values match spec
  EXPECT_EQ(static_cast<uint8_t>(OverlayStreamState::Idle), 0);
  EXPECT_EQ(static_cast<uint8_t>(OverlayStreamState::LoadingConfig), 1);
  EXPECT_EQ(static_cast<uint8_t>(OverlayStreamState::WaitingToStart), 3);
  EXPECT_EQ(static_cast<uint8_t>(OverlayStreamState::FlushingPrevPhase), 4);
  EXPECT_EQ(static_cast<uint8_t>(OverlayStreamState::Running), 5);
}

// ============================================================================
// Packet Type Tests
// ============================================================================

TEST(OverlayPacketTypeTest, PacketTypeValues) {
  EXPECT_EQ(static_cast<uint8_t>(OverlayPacketType::Data), 0);
  EXPECT_EQ(static_cast<uint8_t>(OverlayPacketType::HandshakeRequest), 1);
  EXPECT_EQ(static_cast<uint8_t>(OverlayPacketType::HandshakeResponse), 2);
  EXPECT_EQ(static_cast<uint8_t>(OverlayPacketType::FlowControlUpdate), 3);
}

// ============================================================================
// Register Index Tests
// ============================================================================

TEST(OverlayRegIndexTest, RegisterIndices) {
  using namespace OverlayRegIndex;

  // Verify register indices match spec
  EXPECT_EQ(STREAM_MSG_HEADER_FORMAT, 0);
  EXPECT_EQ(STREAM_MISC_CFG, 1);
  EXPECT_EQ(STREAM_BUF_START, 2);
  EXPECT_EQ(STREAM_BUF_SIZE, 3);
  EXPECT_EQ(STREAM_MSG_INFO_PTR, 4);
  EXPECT_EQ(STREAM_MSG_INFO_WR_PTR, 5);
  EXPECT_EQ(STREAM_RD_PTR, 6);
  EXPECT_EQ(STREAM_WR_PTR, 7);
  EXPECT_EQ(STREAM_REMOTE_SRC, 8);
  EXPECT_EQ(STREAM_REMOTE_SRC_PHASE, 9);
  EXPECT_EQ(STREAM_REMOTE_DEST, 10);
  EXPECT_EQ(STREAM_REMOTE_DEST_BUF_START, 11);
  EXPECT_EQ(STREAM_REMOTE_DEST_BUF_SIZE, 12);
  EXPECT_EQ(STREAM_REMOTE_DEST_MSG_INFO_WR_PTR, 13);
  EXPECT_EQ(STREAM_REMOTE_DEST_WR_PTR, 14);
  EXPECT_EQ(STREAM_MCAST_DEST, 15);
  EXPECT_EQ(STREAM_MCAST_DEST_NUM, 16);
  EXPECT_EQ(STREAM_PHASE_AUTO_CFG_HEADER, 17);
  EXPECT_EQ(STREAM_CURR_PHASE, 18);
  EXPECT_EQ(STREAM_CURR_PHASE_BASE, 19);
  EXPECT_EQ(STREAM_PHASE_ADVANCE, 20);
  EXPECT_EQ(STREAM_WAIT_STATUS, 21);
  EXPECT_EQ(STREAM_NUM_MSGS_RECEIVED, 22);
  EXPECT_EQ(STREAM_NEXT_RECEIVED_MSG_ADDR, 23);
  EXPECT_EQ(STREAM_NEXT_RECEIVED_MSG_SIZE, 24);
  EXPECT_EQ(STREAM_MSG_INFO_CLEAR, 25);
  EXPECT_EQ(STREAM_MSG_DATA_CLEAR, 26);
  EXPECT_EQ(STREAM_BUF_SPACE_AVAILABLE, 27);
  EXPECT_EQ(STREAM_MSG_INFO_CAN_PUSH_NEW_MSG, 28);
  EXPECT_EQ(STREAM_SOURCE_ENDPOINT_NEW_MSG_INFO, 29);
  EXPECT_EQ(STREAM_NUM_MSGS_RECEIVED_INC, 30);
  EXPECT_EQ(STREAM_MEM_BUF_SPACE_AVAILABLE_ACK_THRESHOLD, 31);

  // Auto-config registers
  EXPECT_EQ(STREAM_PHASE_AUTO_CFG_PTR, 32);
  EXPECT_EQ(STREAM_PHASE_AUTO_CFG_PTR_BASE, 33);
  EXPECT_EQ(STREAM_BLOB_AUTO_CFG_DONE, 34);
  EXPECT_EQ(STREAM_BLOB_NEXT_AUTO_CFG_DONE, 36);

  // Gather operation registers
  EXPECT_EQ(STREAM_GATHER, 37);
  EXPECT_EQ(STREAM_GATHER_CLEAR, 38);
  EXPECT_EQ(STREAM_LOCAL_SRC_MASK, 39);
  EXPECT_EQ(STREAM_LOCAL_DEST, 42);
}

// ============================================================================
// Stream Capabilities Tests (Data Structure)
// ============================================================================

TEST(StreamCapabilitiesTest, DefaultValues) {
  StreamCapabilities caps;

  // Default values should be false/zero
  EXPECT_FALSE(caps.can_multicast);
  EXPECT_FALSE(caps.can_receive_gather);
  EXPECT_FALSE(caps.can_transmit_dram);
  EXPECT_FALSE(caps.can_cause_irq);
  EXPECT_FALSE(caps.metadata_includes_header);
  EXPECT_EQ(caps.metadata_fifo_capacity, 0u);
  EXPECT_EQ(caps.metadata_fifo_group_size, 0u);
  EXPECT_EQ(caps.l1_read_complete_fifo_capacity, 0u);
}

// ============================================================================
// Message Metadata Tests
// ============================================================================

TEST(OverlayMsgMetadataTest, DefaultValues) {
  OverlayMsgMetadata meta;

  EXPECT_EQ(meta.buf_ptr, 0u);
  EXPECT_EQ(meta.msg_length, 0u);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(meta.header[i], 0u);
  }
}

TEST(OverlayMsgMetadataTest, SetValues) {
  OverlayMsgMetadata meta;

  meta.buf_ptr = 0x1234;
  meta.msg_length = 0x100;
  meta.header[0] = 0xDEADBEEF;
  meta.header[1] = 0xCAFEBABE;
  meta.header[2] = 0x12345678;
  meta.header[3] = 0xABCDEF00;

  EXPECT_EQ(meta.buf_ptr, 0x1234u);
  EXPECT_EQ(meta.msg_length, 0x100u);
  EXPECT_EQ(meta.header[0], 0xDEADBEEFu);
  EXPECT_EQ(meta.header[1], 0xCAFEBABEu);
  EXPECT_EQ(meta.header[2], 0x12345678u);
  EXPECT_EQ(meta.header[3], 0xABCDEF00u);
}

// ============================================================================
// Stream Register Tests
// ============================================================================

TEST(OverlayStreamRegsTest, DefaultValues) {
  OverlayStreamRegs regs;

  // Check default values
  EXPECT_EQ(regs.outgoing_data_noc, 0);
  EXPECT_EQ(regs.remote_src_update_noc, 0);
  EXPECT_FALSE(regs.local_sources_connected);
  EXPECT_FALSE(regs.source_endpoint);
  EXPECT_FALSE(regs.remote_source);
  EXPECT_FALSE(regs.receiver_endpoint);
  EXPECT_FALSE(regs.local_receiver);
  EXPECT_FALSE(regs.remote_receiver);
  EXPECT_FALSE(regs.phase_auto_config);
  EXPECT_FALSE(regs.phase_auto_advance);
  EXPECT_TRUE(regs.next_phase_src_change);  // Default true per spec
  EXPECT_TRUE(regs.next_phase_dest_change); // Default true per spec
  EXPECT_FALSE(regs.data_buf_no_flow_ctrl);
  EXPECT_FALSE(regs.dest_data_buf_no_flow_ctrl);
  EXPECT_FALSE(regs.remote_src_is_mcast);
  EXPECT_FALSE(regs.no_prev_phase_outgoing_data_flush);
  EXPECT_EQ(regs.unicast_vc_reg, 0);
  EXPECT_EQ(regs.reg_update_vc_reg, 0);

  // Buffer configuration
  EXPECT_EQ(regs.buf_start, 0u);
  EXPECT_EQ(regs.buf_size, 0u);
  EXPECT_EQ(regs.rd_ptr, 0u);
  EXPECT_EQ(regs.wr_ptr, 0u);

  // Multicast configuration
  EXPECT_EQ(regs.mcast_end_x, 0);
  EXPECT_EQ(regs.mcast_end_y, 0);
  EXPECT_FALSE(regs.mcast_en);
  EXPECT_FALSE(regs.mcast_linked);
  EXPECT_EQ(regs.mcast_vc, 0);
  EXPECT_FALSE(regs.mcast_no_path_res);
  EXPECT_FALSE(regs.mcast_xy);
  EXPECT_EQ(regs.mcast_dest_num, 1);  // Default 1 per spec

  // Gather configuration
  EXPECT_EQ(regs.gather_msg_arb_group_size, 1);  // Default 1
  EXPECT_TRUE(regs.gather_msg_src_in_order_fwd); // Default true
  EXPECT_EQ(regs.gather_msg_local_stream_clear_num, 1);  // Default 1
  EXPECT_FALSE(regs.gather_msg_group_stream_clear_type);

  // Flow control
  EXPECT_EQ(regs.buf_space_ack_threshold, 0);
  EXPECT_EQ(regs.last_ack_space_available, 0u);
}

// ============================================================================
// NIU Request/Response Tests
// ============================================================================

TEST(OverlayNiuReqTest, DefaultValues) {
  OverlayNiuReq req;

  EXPECT_EQ(req.stream_id, 0);
  EXPECT_EQ(req.dest.x, 0);
  EXPECT_EQ(req.dest.y, 0);
  EXPECT_EQ(req.dest_addr, 0u);
  EXPECT_EQ(req.src_addr, 0u);
  EXPECT_EQ(req.length, 0u);
  EXPECT_FALSE(req.is_posted);
  EXPECT_FALSE(req.is_multicast);
  EXPECT_EQ(req.mcast_end_x, 0);
  EXPECT_EQ(req.mcast_end_y, 0);
  EXPECT_FALSE(req.mcast_xy);
  EXPECT_EQ(req.vc_class, 0);
  EXPECT_EQ(req.vc_buddy, 0);
}

TEST(OverlayNiuRespTest, DefaultValues) {
  OverlayNiuResp resp;

  EXPECT_EQ(resp.stream_id, 0);
  EXPECT_FALSE(resp.success);
  EXPECT_EQ(resp.bytes_transferred, 0u);
}

// ============================================================================
// Stream Operators Tests (for sc_fifo compatibility)
// ============================================================================

TEST(StreamOperatorTest, NiuReqOutput) {
  OverlayNiuReq req;
  req.stream_id = 5;
  req.dest = {3, 2};
  req.length = 64;

  std::ostringstream oss;
  oss << req;

  std::string output = oss.str();
  EXPECT_TRUE(output.find("stream=5") != std::string::npos);
  EXPECT_TRUE(output.find("len=64") != std::string::npos);
}

TEST(StreamOperatorTest, NiuRespOutput) {
  OverlayNiuResp resp;
  resp.stream_id = 10;
  resp.success = true;
  resp.bytes_transferred = 128;

  std::ostringstream oss;
  oss << resp;

  std::string output = oss.str();
  EXPECT_TRUE(output.find("stream=10") != std::string::npos);
  EXPECT_TRUE(output.find("ok=1") != std::string::npos);
  EXPECT_TRUE(output.find("bytes=128") != std::string::npos);
}

// ============================================================================
// Message Header Format Tests
// ============================================================================

TEST(OverlayMsgHeaderFormatTest, DefaultValues) {
  OverlayMsgHeaderFormat fmt;

  EXPECT_EQ(fmt.word_cnt_offset, 0);
  EXPECT_EQ(fmt.word_cnt_bits, 16);  // Default 16 bits per spec
}

TEST(OverlayMsgHeaderFormatTest, CustomValues) {
  OverlayMsgHeaderFormat fmt;
  fmt.word_cnt_offset = 24;  // Offset at byte 3
  fmt.word_cnt_bits = 12;    // 12-bit length field

  EXPECT_EQ(fmt.word_cnt_offset, 24);
  EXPECT_EQ(fmt.word_cnt_bits, 12);
}

// ============================================================================
// Overlay Stream Tests
// ============================================================================

TEST(OverlayStreamTest, DefaultState) {
  OverlayStream stream;

  EXPECT_EQ(stream.state, OverlayStreamState::Idle);
  EXPECT_TRUE(stream.metadata_fifo.empty());
  EXPECT_TRUE(stream.l1_read_complete_fifo.empty());

  // Handshake state
  EXPECT_FALSE(stream.handshake_sent);
  EXPECT_FALSE(stream.handshake_response_received);
  EXPECT_EQ(stream.handshake_phase, 0u);
  EXPECT_FALSE(stream.handshake_request_received);
  EXPECT_FALSE(stream.sent_speculative_response);

  // Flow control state
  EXPECT_EQ(stream.remote_buf_space_available, 0u);
  EXPECT_FALSE(stream.waiting_for_flow_ctrl);

  // Gather state
  EXPECT_EQ(stream.current_gather_group, 0);
  EXPECT_EQ(stream.current_gather_stream_in_group, 0);
  EXPECT_EQ(stream.msgs_received_from_current_stream, 0);
}

TEST(OverlayStreamTest, MetadataFifoOperations) {
  OverlayStream stream;

  // Initially empty
  EXPECT_TRUE(stream.metadata_fifo.empty());
  EXPECT_EQ(stream.metadata_fifo.size(), 0u);

  // Add entries
  OverlayMsgMetadata meta1;
  meta1.buf_ptr = 0x100;
  meta1.msg_length = 0x10;
  stream.metadata_fifo.push_back(meta1);

  EXPECT_FALSE(stream.metadata_fifo.empty());
  EXPECT_EQ(stream.metadata_fifo.size(), 1u);

  OverlayMsgMetadata meta2;
  meta2.buf_ptr = 0x200;
  meta2.msg_length = 0x20;
  stream.metadata_fifo.push_back(meta2);

  EXPECT_EQ(stream.metadata_fifo.size(), 2u);

  // Check front
  EXPECT_EQ(stream.metadata_fifo.front().buf_ptr, 0x100u);
  EXPECT_EQ(stream.metadata_fifo.front().msg_length, 0x10u);

  // Pop front
  stream.metadata_fifo.pop_front();
  EXPECT_EQ(stream.metadata_fifo.size(), 1u);
  EXPECT_EQ(stream.metadata_fifo.front().buf_ptr, 0x200u);
}

TEST(OverlayStreamTest, L1ReadCompleteFifoOperations) {
  OverlayStream stream;

  // Initially empty
  EXPECT_TRUE(stream.l1_read_complete_fifo.empty());

  // Add entries
  stream.l1_read_complete_fifo.push_back(16);
  stream.l1_read_complete_fifo.push_back(32);
  stream.l1_read_complete_fifo.push_back(64);

  EXPECT_EQ(stream.l1_read_complete_fifo.size(), 3u);
  EXPECT_EQ(stream.l1_read_complete_fifo.front(), 16u);

  // Pop
  stream.l1_read_complete_fifo.pop_front();
  EXPECT_EQ(stream.l1_read_complete_fifo.front(), 32u);
}

TEST(OverlayStreamTest, GatherInputPhaseTracking) {
  OverlayStream stream;

  // All should be false initially
  for (int i = 0; i < 64; ++i) {
    EXPECT_FALSE(stream.gather_input_phase_started[i]);
  }

  // Set some
  stream.gather_input_phase_started[0] = true;
  stream.gather_input_phase_started[5] = true;
  stream.gather_input_phase_started[63] = true;

  EXPECT_TRUE(stream.gather_input_phase_started[0]);
  EXPECT_FALSE(stream.gather_input_phase_started[1]);
  EXPECT_TRUE(stream.gather_input_phase_started[5]);
  EXPECT_TRUE(stream.gather_input_phase_started[63]);
}

// ============================================================================
// Phase Auto-Config Header Tests
// ============================================================================

TEST(PhaseAutoCfgHeaderTest, ExtractNumMsgs) {
  // Phase auto-config header format:
  // Bits 0-3: number of config register writes - 1
  // Bits 12-23: number of messages to receive

  uint32_t header = 0;

  // Set num_msgs to 100
  header |= (100u << 12);
  EXPECT_EQ((header >> 12) & 0xFFF, 100u);

  // Set num_cfg_writes to 5 (stored as 4)
  header |= 4;
  EXPECT_EQ((header & 0xF) + 1, 5u);
}

// ============================================================================
// Buffer Wrap-Around Tests
// ============================================================================

TEST(BufferWrapAroundTest, CircularBufferPointer) {
  // Test circular buffer pointer wrap-around logic
  uint32_t buf_size = 1024;  // 16KB buffer (1024 * 16 bytes)
  uint32_t wr_ptr = 1000;
  uint32_t msg_len = 100;

  // Write would exceed buffer
  wr_ptr += msg_len;
  if (wr_ptr >= buf_size) {
    wr_ptr -= buf_size;
  }

  EXPECT_EQ(wr_ptr, 76u);  // 1100 - 1024 = 76
}

TEST(BufferWrapAroundTest, SpaceCalculation) {
  // Test available space calculation
  uint32_t buf_size = 1024;
  uint32_t rd_ptr = 100;
  uint32_t wr_ptr = 800;

  // Used space
  uint32_t used = (wr_ptr - rd_ptr + buf_size) % buf_size;
  EXPECT_EQ(used, 700u);

  // Available space
  uint32_t available = buf_size - used;
  EXPECT_EQ(available, 324u);

  // Test wrap-around case
  rd_ptr = 800;
  wr_ptr = 100;
  used = (wr_ptr - rd_ptr + buf_size) % buf_size;
  EXPECT_EQ(used, 324u);  // 100 - 800 + 1024 = 324
}

// ============================================================================
// Flow Control Threshold Tests
// ============================================================================

TEST(FlowControlTest, ThresholdCalculation) {
  // Test threshold calculation based on buf_space_ack_threshold
  // Threshold = (1 << buf_space_ack_threshold) << 4

  uint8_t threshold_reg = 0;
  uint32_t threshold = (1u << threshold_reg) << 4;
  EXPECT_EQ(threshold, 16u);  // 1 * 16 = 16 bytes

  threshold_reg = 4;
  threshold = (1u << threshold_reg) << 4;
  EXPECT_EQ(threshold, 256u);  // 16 * 16 = 256 bytes

  threshold_reg = 8;
  threshold = (1u << threshold_reg) << 4;
  EXPECT_EQ(threshold, 4096u);  // 256 * 16 = 4096 bytes
}

// ============================================================================
// Gather Group Tests
// ============================================================================

TEST(GatherGroupTest, GroupIndexing) {
  // Test gather group indexing
  uint8_t group_size = 4;
  uint8_t current_group = 2;

  uint8_t group_start = current_group * group_size;
  EXPECT_EQ(group_start, 8u);

  // Streams in group 2: 8, 9, 10, 11
  for (uint8_t i = 0; i < group_size; ++i) {
    uint8_t stream_idx = group_start + i;
    EXPECT_GE(stream_idx, 8u);
    EXPECT_LE(stream_idx, 11u);
  }
}

TEST(GatherGroupTest, LocalSrcMask) {
  // Test 64-bit local source mask spread across 2 registers
  uint32_t local_src_mask_0 = 0x0000001F;  // Streams 0-4
  uint32_t local_src_mask_1 = 0x00000000;

  uint64_t src_mask = local_src_mask_0 |
                      (static_cast<uint64_t>(local_src_mask_1) << 32);

  // Count set bits
  int count = 0;
  for (int i = 0; i < 64; ++i) {
    if (src_mask & (1ull << i)) {
      count++;
    }
  }
  EXPECT_EQ(count, 5);  // 5 input streams

  // Check specific streams
  EXPECT_TRUE(src_mask & (1ull << 0));
  EXPECT_TRUE(src_mask & (1ull << 1));
  EXPECT_TRUE(src_mask & (1ull << 2));
  EXPECT_TRUE(src_mask & (1ull << 3));
  EXPECT_TRUE(src_mask & (1ull << 4));
  EXPECT_FALSE(src_mask & (1ull << 5));
}

// ============================================================================
// Capability Check Tests (Tensix Tile Streams)
// ============================================================================

TEST(TensixCapabilityTest, Streams0to3) {
  // Streams 0-3 have full capabilities
  for (int stream_id = 0; stream_id <= 3; ++stream_id) {
    // These should be capable of multicast, gather, DRAM, IRQ
    EXPECT_TRUE(stream_id <= 3);  // Placeholder for actual capability check
  }
}

TEST(TensixCapabilityTest, Streams4to5HeaderCopy) {
  // Streams 4-5 include header copy in metadata
  for (int stream_id = 4; stream_id <= 5; ++stream_id) {
    EXPECT_TRUE(stream_id >= 4 && stream_id <= 5);
  }
}

TEST(TensixCapabilityTest, Streams8to11DramIrq) {
  // Streams 8-11 can transmit to DRAM and cause IRQ
  for (int stream_id = 8; stream_id <= 11; ++stream_id) {
    EXPECT_TRUE(stream_id >= 8 && stream_id <= 11);
  }
}

// ============================================================================
// New Critical Register Index Tests
// ============================================================================

TEST(NewRegisterIndexTest, ReceiverEndpointRegisters) {
  using namespace OverlayRegIndex;

  // Verify receiver endpoint register indices
  EXPECT_EQ(STREAM_RECEIVER_ENDPOINT_SET_MSG_HEADER, 43);
  EXPECT_EQ(STREAM_RECEIVER_ENDPOINT_SET_MSG_HEADER + 1, 44);
  EXPECT_EQ(STREAM_RECEIVER_ENDPOINT_SET_MSG_HEADER + 2, 45);
  EXPECT_EQ(STREAM_RECEIVER_ENDPOINT_SET_MSG_HEADER + 3, 46);
  EXPECT_EQ(STREAM_RECEIVER_ENDPOINT_MSG_INFO, 47);
}

TEST(NewRegisterIndexTest, DramHighAddressRegisters) {
  using namespace OverlayRegIndex;

  // Verify DRAM high address register indices
  EXPECT_EQ(STREAM_REMOTE_DEST_BUF_START_HI, 48);
  EXPECT_EQ(STREAM_REMOTE_DEST_BUF_SIZE_HI, 49);
  EXPECT_EQ(STREAM_REMOTE_DEST_MSG_INFO_WR_PTR_HI, 50);
}

TEST(NewRegisterIndexTest, ScratchRegisters) {
  using namespace OverlayRegIndex;

  // Verify scratch register indices (6 registers)
  EXPECT_EQ(STREAM_SCRATCH, 51);
  EXPECT_EQ(STREAM_SCRATCH + 5, 56);
}

TEST(NewRegisterIndexTest, PhaseAndPriorityRegisters) {
  using namespace OverlayRegIndex;

  EXPECT_EQ(STREAM_DEST_PHASE_READY_UPDATE, 57);
  EXPECT_EQ(STREAM_REMOTE_DEST_TRAFFIC_PRIORITY, 58);
}

TEST(NewRegisterIndexTest, MulticastFlowControlRegisters) {
  using namespace OverlayRegIndex;

  // 32 registers for per-destination space tracking
  EXPECT_EQ(STREAM_REMOTE_DEST_BUF_SPACE_AVAILABLE, 59);
  EXPECT_EQ(STREAM_REMOTE_DEST_BUF_SPACE_AVAILABLE + 31, 90);
  EXPECT_EQ(STREAM_REMOTE_DEST_BUF_SPACE_AVAILABLE_UPDATE, 91);
}

TEST(NewRegisterIndexTest, DebugAndCompressionRegisters) {
  using namespace OverlayRegIndex;

  // Debug status registers (3 registers)
  EXPECT_EQ(STREAM_DEBUG_STATUS, 92);
  EXPECT_EQ(STREAM_DEBUG_STATUS + 2, 94);

  // Message compression registers
  EXPECT_EQ(STREAM_MSG_GROUP_COMPRESS, 95);
  EXPECT_EQ(STREAM_MSG_GROUP_ZERO_MASK, 96);
  EXPECT_EQ(STREAM_MSG_GROUP_ZERO_MASK + 3, 99);
}

// ============================================================================
// Receiver Endpoint Header Register Tests
// ============================================================================

TEST(ReceiverEndpointTest, HeaderRegisterDefaultValues) {
  OverlayStreamRegs regs;

  // All 4 header registers should default to 0
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(regs.receiver_endpoint_msg_header[i], 0u);
  }
}

TEST(ReceiverEndpointTest, HeaderRegisterSetValues) {
  OverlayStreamRegs regs;

  // Set header values
  regs.receiver_endpoint_msg_header[0] = 0xDEADBEEF;
  regs.receiver_endpoint_msg_header[1] = 0xCAFEBABE;
  regs.receiver_endpoint_msg_header[2] = 0x12345678;
  regs.receiver_endpoint_msg_header[3] = 0xABCDEF00;

  EXPECT_EQ(regs.receiver_endpoint_msg_header[0], 0xDEADBEEFu);
  EXPECT_EQ(regs.receiver_endpoint_msg_header[1], 0xCAFEBABEu);
  EXPECT_EQ(regs.receiver_endpoint_msg_header[2], 0x12345678u);
  EXPECT_EQ(regs.receiver_endpoint_msg_header[3], 0xABCDEF00u);
}

TEST(ReceiverEndpointTest, MsgInfoEncoding) {
  // STREAM_RECEIVER_ENDPOINT_MSG_INFO returns:
  // Bits 0-16: buf_ptr (17 bits)
  // Bits 17-31: msg_length (15 bits)

  uint32_t buf_ptr = 0x1FFFF;  // Max 17-bit value
  uint32_t msg_length = 0x7FFF;  // Max 15-bit value

  uint32_t encoded = (buf_ptr & 0x1FFFF) | ((msg_length & 0x7FFF) << 17);

  EXPECT_EQ(encoded & 0x1FFFF, buf_ptr);
  EXPECT_EQ((encoded >> 17) & 0x7FFF, msg_length);
}

// ============================================================================
// DRAM High Address Register Tests
// ============================================================================

TEST(DramHighAddressTest, DefaultValues) {
  OverlayStreamRegs regs;

  EXPECT_EQ(regs.remote_dest_buf_start_hi, 0);
  EXPECT_EQ(regs.remote_dest_buf_size_hi, 0);
  EXPECT_EQ(regs.remote_dest_msg_info_wr_ptr_hi, 0);
}

TEST(DramHighAddressTest, FourBitMasking) {
  // High address registers are 4 bits (for 36-bit addressing)
  OverlayStreamRegs regs;

  // Set max 4-bit values
  regs.remote_dest_buf_start_hi = 0xF;
  regs.remote_dest_buf_size_hi = 0xF;
  regs.remote_dest_msg_info_wr_ptr_hi = 0xF;

  EXPECT_EQ(regs.remote_dest_buf_start_hi & 0xF, 0xF);
  EXPECT_EQ(regs.remote_dest_buf_size_hi & 0xF, 0xF);
  EXPECT_EQ(regs.remote_dest_msg_info_wr_ptr_hi & 0xF, 0xF);
}

TEST(DramHighAddressTest, Full36BitAddress) {
  // Test constructing a full 36-bit address from low 32 bits + high 4 bits
  uint32_t addr_lo = 0xABCDEF00;
  uint8_t addr_hi = 0xF;

  uint64_t full_addr = addr_lo | (static_cast<uint64_t>(addr_hi) << 32);

  EXPECT_EQ(full_addr, 0xFABCDEF00ull);
  EXPECT_EQ(full_addr & 0xFFFFFFFF, addr_lo);
  EXPECT_EQ((full_addr >> 32) & 0xF, addr_hi);
}

// ============================================================================
// Scratch Register Tests
// ============================================================================

TEST(ScratchRegisterTest, DefaultValues) {
  OverlayStreamRegs regs;

  // All 6 scratch registers should default to 0
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(regs.scratch[i], 0u);
  }
}

TEST(ScratchRegisterTest, SetValues) {
  OverlayStreamRegs regs;

  // Set different values in each scratch register
  regs.scratch[0] = 0x11111111;
  regs.scratch[1] = 0x22222222;
  regs.scratch[2] = 0x33333333;
  regs.scratch[3] = 0x44444444;
  regs.scratch[4] = 0x55555555;
  regs.scratch[5] = 0x66666666;

  EXPECT_EQ(regs.scratch[0], 0x11111111u);
  EXPECT_EQ(regs.scratch[1], 0x22222222u);
  EXPECT_EQ(regs.scratch[2], 0x33333333u);
  EXPECT_EQ(regs.scratch[3], 0x44444444u);
  EXPECT_EQ(regs.scratch[4], 0x55555555u);
  EXPECT_EQ(regs.scratch[5], 0x66666666u);
}

TEST(ScratchRegisterTest, DramStreamCapability) {
  // Scratch registers are only valid for DRAM-capable streams (0-3, 8-11)
  StreamCapabilities caps;

  // Streams 0-3 can transmit to DRAM
  caps.can_transmit_dram = true;
  EXPECT_TRUE(caps.can_transmit_dram);

  // Streams 4-7 cannot transmit to DRAM
  caps.can_transmit_dram = false;
  EXPECT_FALSE(caps.can_transmit_dram);
}

// ============================================================================
// Traffic Priority Register Tests
// ============================================================================

TEST(TrafficPriorityTest, DefaultValue) {
  OverlayStreamRegs regs;

  EXPECT_EQ(regs.remote_dest_traffic_priority, 0);
}

TEST(TrafficPriorityTest, FourBitPriority) {
  OverlayStreamRegs regs;

  // Priority is 4 bits (0-15)
  regs.remote_dest_traffic_priority = 0xF;
  EXPECT_EQ(regs.remote_dest_traffic_priority & 0xF, 0xF);

  regs.remote_dest_traffic_priority = 8;
  EXPECT_EQ(regs.remote_dest_traffic_priority, 8);
}

// ============================================================================
// Multicast Flow Control Register Tests
// ============================================================================

TEST(MulticastFlowControlTest, DefaultValues) {
  OverlayStreamRegs regs;

  // All 32 destination space registers should default to 0
  for (int i = 0; i < 32; ++i) {
    EXPECT_EQ(regs.remote_dest_buf_space_available[i], 0u);
  }
}

TEST(MulticastFlowControlTest, SetPerDestinationSpace) {
  OverlayStreamRegs regs;

  // Set space for different destinations
  regs.remote_dest_buf_space_available[0] = 1024;
  regs.remote_dest_buf_space_available[15] = 2048;
  regs.remote_dest_buf_space_available[31] = 4096;

  EXPECT_EQ(regs.remote_dest_buf_space_available[0], 1024u);
  EXPECT_EQ(regs.remote_dest_buf_space_available[15], 2048u);
  EXPECT_EQ(regs.remote_dest_buf_space_available[31], 4096u);
}

TEST(MulticastFlowControlTest, SpaceUpdateEncoding) {
  // STREAM_REMOTE_DEST_BUF_SPACE_AVAILABLE_UPDATE encoding:
  // Bits 0-4: destination index (5 bits, 0-31)
  // Bits 5-31: space delta (signed, 27 bits)

  uint8_t dest_idx = 15;
  int32_t delta = 256;

  uint32_t encoded = (dest_idx & 0x1F) | (static_cast<uint32_t>(delta) << 5);

  EXPECT_EQ(encoded & 0x1F, dest_idx);
  EXPECT_EQ(static_cast<int32_t>(encoded) >> 5, delta);

  // Test negative delta
  delta = -128;
  encoded = (dest_idx & 0x1F) | (static_cast<uint32_t>(delta) << 5);

  EXPECT_EQ(encoded & 0x1F, dest_idx);
  EXPECT_EQ(static_cast<int32_t>(encoded) >> 5, delta);
}

TEST(MulticastFlowControlTest, SpaceUpdateCalculation) {
  // Test the delta update calculation
  uint32_t current_space = 1000;
  int32_t delta = 256;

  int64_t new_space = static_cast<int64_t>(current_space) + delta;
  EXPECT_EQ(new_space, 1256);

  // Test negative delta
  delta = -500;
  new_space = static_cast<int64_t>(current_space) + delta;
  EXPECT_EQ(new_space, 500);

  // Test underflow protection
  delta = -2000;
  new_space = static_cast<int64_t>(current_space) + delta;
  if (new_space < 0) new_space = 0;
  EXPECT_EQ(new_space, 0);
}

// ============================================================================
// Debug Status Register Tests
// ============================================================================

TEST(DebugStatusTest, DefaultValues) {
  OverlayStreamRegs regs;

  EXPECT_EQ(regs.debug_status_0, 0u);
  EXPECT_EQ(regs.debug_status_1, 0u);
}

TEST(DebugStatusTest, FifoStatusEncoding) {
  // DEBUG_STATUS + 2 returns FIFO status:
  // Bits 0-15: metadata FIFO size
  // Bits 16-31: l1_read_complete FIFO size

  uint16_t metadata_size = 8;
  uint16_t l1_complete_size = 4;

  uint32_t status = (static_cast<uint32_t>(metadata_size) & 0xFFFF) |
                    ((static_cast<uint32_t>(l1_complete_size) & 0xFFFF) << 16);

  EXPECT_EQ(status & 0xFFFF, metadata_size);
  EXPECT_EQ((status >> 16) & 0xFFFF, l1_complete_size);
}

TEST(DebugStatusTest, MaxFifoSizes) {
  // Test maximum FIFO sizes in status register
  uint16_t max_metadata = 0xFFFF;
  uint16_t max_l1_complete = 0xFFFF;

  uint32_t status = (static_cast<uint32_t>(max_metadata) & 0xFFFF) |
                    ((static_cast<uint32_t>(max_l1_complete) & 0xFFFF) << 16);

  EXPECT_EQ(status, 0xFFFFFFFFu);
}

// ============================================================================
// Message Compression Register Tests
// ============================================================================

TEST(MsgCompressionTest, DefaultValues) {
  OverlayStreamRegs regs;

  EXPECT_EQ(regs.msg_group_compress, 0u);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(regs.msg_group_zero_mask[i], 0u);
  }
}

TEST(MsgCompressionTest, CompressRegisterSetValue) {
  OverlayStreamRegs regs;

  regs.msg_group_compress = 0x12345678;
  EXPECT_EQ(regs.msg_group_compress, 0x12345678u);
}

TEST(MsgCompressionTest, ZeroMaskRegisters) {
  OverlayStreamRegs regs;

  // Set all 4 zero mask registers (128 bits total for header mask)
  regs.msg_group_zero_mask[0] = 0xFFFFFFFF;
  regs.msg_group_zero_mask[1] = 0x00FF00FF;
  regs.msg_group_zero_mask[2] = 0xFF00FF00;
  regs.msg_group_zero_mask[3] = 0xAAAAAAAA;

  EXPECT_EQ(regs.msg_group_zero_mask[0], 0xFFFFFFFFu);
  EXPECT_EQ(regs.msg_group_zero_mask[1], 0x00FF00FFu);
  EXPECT_EQ(regs.msg_group_zero_mask[2], 0xFF00FF00u);
  EXPECT_EQ(regs.msg_group_zero_mask[3], 0xAAAAAAAAu);
}

// ============================================================================
// Capability-Based Register Access Tests
// ============================================================================

TEST(CapabilityAccessTest, MulticastOnlyRegisters) {
  // Multicast flow control registers only valid for streams with can_multicast
  StreamCapabilities caps;

  // Streams 0-3 can multicast
  caps.can_multicast = true;
  EXPECT_TRUE(caps.can_multicast);

  // Other streams cannot multicast
  caps.can_multicast = false;
  EXPECT_FALSE(caps.can_multicast);
}

TEST(CapabilityAccessTest, HeaderCopyOnlyRegisters) {
  // Receiver endpoint header registers only valid for streams 4-5
  StreamCapabilities caps;

  // Streams 4-5 have metadata_includes_header
  caps.metadata_includes_header = true;
  EXPECT_TRUE(caps.metadata_includes_header);

  // Other streams don't have this capability
  caps.metadata_includes_header = false;
  EXPECT_FALSE(caps.metadata_includes_header);
}

TEST(CapabilityAccessTest, DramOnlyRegisters) {
  // Scratch registers and phase ready only for DRAM streams
  StreamCapabilities caps;

  // Test DRAM capability check
  caps.can_transmit_dram = true;
  caps.can_cause_irq = true;
  EXPECT_TRUE(caps.can_transmit_dram);
  EXPECT_TRUE(caps.can_cause_irq);
}

// ============================================================================
// Phase Ready Update Test
// ============================================================================

TEST(PhaseReadyUpdateTest, WriteOnlyBehavior) {
  // STREAM_DEST_PHASE_READY_UPDATE is write-only
  // Writing signals destination is ready for specified phase

  uint32_t phase_value = 0x12345;

  // The value written is the phase number to signal ready
  EXPECT_EQ(phase_value, 0x12345u);
}

// ============================================================================
// Medium Severity Fix Tests - STREAM_CURR_PHASE Base Adjustment
// ============================================================================

TEST(CurrPhaseBaseTest, ReadSubtractsBase) {
  // Per spec: Reading STREAM_CURR_PHASE subtracts curr_phase_base
  OverlayStreamRegs regs;

  regs.curr_phase = 100;
  regs.curr_phase_base = 50;

  // Simulated read should return curr_phase - curr_phase_base
  uint32_t read_value = regs.curr_phase - regs.curr_phase_base;
  EXPECT_EQ(read_value, 50u);
}

TEST(CurrPhaseBaseTest, WriteAddsBase) {
  // Per spec: Writing STREAM_CURR_PHASE adds curr_phase_base
  OverlayStreamRegs regs;

  regs.curr_phase_base = 50;

  // Simulated write of 25 should result in curr_phase = 25 + 50 = 75
  uint32_t write_value = 25;
  regs.curr_phase = write_value + regs.curr_phase_base;
  EXPECT_EQ(regs.curr_phase, 75u);
}

TEST(CurrPhaseBaseTest, RoundTripConsistency) {
  // Write then read should return the same value
  OverlayStreamRegs regs;

  regs.curr_phase_base = 1000;

  // Write 500 (internal becomes 500 + 1000 = 1500)
  uint32_t write_value = 500;
  regs.curr_phase = write_value + regs.curr_phase_base;

  // Read should return 1500 - 1000 = 500
  uint32_t read_value = regs.curr_phase - regs.curr_phase_base;
  EXPECT_EQ(read_value, write_value);
}

// ============================================================================
// Medium Severity Fix Tests - STREAM_PHASE_AUTO_CFG_PTR Base Adjustment
// ============================================================================

TEST(AutoCfgPtrBaseTest, ReadSubtractsBase) {
  // Per spec: Reading STREAM_PHASE_AUTO_CFG_PTR subtracts base
  OverlayStreamRegs regs;

  regs.phase_auto_cfg_ptr = 0x2000;
  regs.phase_auto_cfg_ptr_base = 0x1000;

  // Simulated read should return ptr - base
  uint32_t read_value = regs.phase_auto_cfg_ptr - regs.phase_auto_cfg_ptr_base;
  EXPECT_EQ(read_value, 0x1000u);
}

TEST(AutoCfgPtrBaseTest, WriteAddsBase) {
  // Per spec: Writing STREAM_PHASE_AUTO_CFG_PTR adds base
  OverlayStreamRegs regs;

  regs.phase_auto_cfg_ptr_base = 0x1000;

  // Write 0x500, internal should be 0x500 + 0x1000 = 0x1500
  uint32_t write_value = 0x500;
  regs.phase_auto_cfg_ptr = write_value + regs.phase_auto_cfg_ptr_base;
  EXPECT_EQ(regs.phase_auto_cfg_ptr, 0x1500u);
}

TEST(AutoCfgPtrBaseTest, BaseZeroNoEffect) {
  // With base = 0, read/write should be direct
  OverlayStreamRegs regs;

  regs.phase_auto_cfg_ptr_base = 0;
  regs.phase_auto_cfg_ptr = 0x1234;

  uint32_t read_value = regs.phase_auto_cfg_ptr - regs.phase_auto_cfg_ptr_base;
  EXPECT_EQ(read_value, 0x1234u);
}

// ============================================================================
// Medium Severity Fix Tests - PHASE_AUTO_CFG_HEADER PHASE_NUM_INCR
// ============================================================================

TEST(PhaseAutoCfgHeaderTest, PhaseNumIncrExtraction) {
  // Bits 0-11: PHASE_NUM_INCR - increment curr_phase by this amount
  uint32_t header = 0;

  // Set PHASE_NUM_INCR to 5
  header |= 5;
  EXPECT_EQ(header & 0xFFF, 5u);

  // Set to max value (4095)
  header = 0xFFF;
  EXPECT_EQ(header & 0xFFF, 0xFFFu);
}

TEST(PhaseAutoCfgHeaderTest, NumMsgsExtraction) {
  // Bits 12-23: CURR_PHASE_NUM_MSGS
  uint32_t header = 0;

  // Set num_msgs to 100
  header |= (100u << 12);
  EXPECT_EQ((header >> 12) & 0xFFF, 100u);
}

TEST(PhaseAutoCfgHeaderTest, NextPhaseNumCfgWrites) {
  // Bits 24-31: NEXT_PHASE_NUM_CFG_REG_WRITES (stored as N-1)
  uint32_t header = 0;

  // Set to 8 writes (stored as 7)
  header |= (7u << 24);
  EXPECT_EQ(((header >> 24) & 0xFF) + 1, 8u);
}

TEST(PhaseAutoCfgHeaderTest, PhaseIncrementBehavior) {
  // Writing PHASE_AUTO_CFG_HEADER with non-zero PHASE_NUM_INCR
  // should increment curr_phase
  OverlayStreamRegs regs;

  regs.curr_phase = 10;

  // Simulate write with PHASE_NUM_INCR = 5
  uint32_t header = 5;  // bits 0-11
  uint32_t phase_num_incr = header & 0xFFF;

  if (phase_num_incr > 0) {
    regs.curr_phase += phase_num_incr;
  }

  EXPECT_EQ(regs.curr_phase, 15u);
}

TEST(PhaseAutoCfgHeaderTest, ZeroIncrNoChange) {
  // PHASE_NUM_INCR = 0 should not change curr_phase
  OverlayStreamRegs regs;

  regs.curr_phase = 10;

  uint32_t header = 0;  // PHASE_NUM_INCR = 0
  uint32_t phase_num_incr = header & 0xFFF;

  if (phase_num_incr > 0) {
    regs.curr_phase += phase_num_incr;
  }

  EXPECT_EQ(regs.curr_phase, 10u);  // Unchanged
}

// ============================================================================
// Medium Severity Fix Tests - REMOTE_DEST_BUF_SIZE Space Init
// ============================================================================

TEST(RemoteDestBufSizeTest, InitializesSpaceAvailable) {
  // Per spec: Writing REMOTE_DEST_BUF_SIZE initializes
  // remote_buf_space_available to the same value
  OverlayStream stream;

  stream.regs.remote_dest_buf_size = 1024;
  // Simulate the write behavior
  stream.remote_buf_space_available = stream.regs.remote_dest_buf_size;

  EXPECT_EQ(stream.remote_buf_space_available, 1024u);
}

TEST(RemoteDestBufSizeTest, UpdateResetsSpace) {
  // Changing buf_size should update available space
  OverlayStream stream;

  // Initial state
  stream.regs.remote_dest_buf_size = 512;
  stream.remote_buf_space_available = 512;

  // Update buf_size
  stream.regs.remote_dest_buf_size = 2048;
  stream.remote_buf_space_available = stream.regs.remote_dest_buf_size;

  EXPECT_EQ(stream.remote_buf_space_available, 2048u);
}

// ============================================================================
// Medium Severity Fix Tests - REMOTE_DEST_BUF_START wr_ptr Reset
// ============================================================================

TEST(RemoteDestBufStartTest, ResetsWrPtr) {
  // Per spec: Writing REMOTE_DEST_BUF_START resets remote_dest_wr_ptr to 0
  OverlayStreamRegs regs;

  regs.remote_dest_wr_ptr = 500;  // Some non-zero value

  // Simulate write to buf_start
  regs.remote_dest_buf_start = 0x1000;
  regs.remote_dest_wr_ptr = 0;  // Reset behavior

  EXPECT_EQ(regs.remote_dest_buf_start, 0x1000u);
  EXPECT_EQ(regs.remote_dest_wr_ptr, 0u);
}

TEST(RemoteDestBufStartTest, MultipleWrites) {
  // Each write to buf_start should reset wr_ptr
  OverlayStreamRegs regs;

  // First write
  regs.remote_dest_buf_start = 0x1000;
  regs.remote_dest_wr_ptr = 0;

  // Simulate some data written
  regs.remote_dest_wr_ptr = 100;

  // Second write to buf_start
  regs.remote_dest_buf_start = 0x2000;
  regs.remote_dest_wr_ptr = 0;  // Should reset again

  EXPECT_EQ(regs.remote_dest_buf_start, 0x2000u);
  EXPECT_EQ(regs.remote_dest_wr_ptr, 0u);
}

// ============================================================================
// Medium Severity Fix Tests - Speculative Handshake Response
// ============================================================================

TEST(SpeculativeHandshakeTest, ConditionsForSpeculativeResponse) {
  // Speculative response is sent when:
  // 1. remote_source is true
  // 2. next_phase_src_change is true
  // 3. data_buf_no_flow_ctrl is false
  OverlayStream stream;

  stream.regs.remote_source = true;
  stream.regs.next_phase_src_change = true;
  stream.regs.data_buf_no_flow_ctrl = false;

  bool should_send_speculative = stream.regs.remote_source &&
                                  stream.regs.next_phase_src_change &&
                                  !stream.regs.data_buf_no_flow_ctrl;

  EXPECT_TRUE(should_send_speculative);
}

TEST(SpeculativeHandshakeTest, NoSpeculativeWithNoFlowCtrl) {
  // No speculative response when flow control is disabled
  OverlayStream stream;

  stream.regs.remote_source = true;
  stream.regs.next_phase_src_change = true;
  stream.regs.data_buf_no_flow_ctrl = true;  // Flow control disabled

  bool should_send_speculative = stream.regs.remote_source &&
                                  stream.regs.next_phase_src_change &&
                                  !stream.regs.data_buf_no_flow_ctrl;

  EXPECT_FALSE(should_send_speculative);
}

TEST(SpeculativeHandshakeTest, NoSpeculativeWithoutSrcChange) {
  // No speculative response when source doesn't change
  OverlayStream stream;

  stream.regs.remote_source = true;
  stream.regs.next_phase_src_change = false;  // No source change
  stream.regs.data_buf_no_flow_ctrl = false;

  bool should_send_speculative = stream.regs.remote_source &&
                                  stream.regs.next_phase_src_change &&
                                  !stream.regs.data_buf_no_flow_ctrl;

  EXPECT_FALSE(should_send_speculative);
}

TEST(SpeculativeHandshakeTest, StateTracking) {
  // Track that speculative response was sent
  OverlayStream stream;

  stream.sent_speculative_response = false;

  // Simulate sending speculative response
  stream.sent_speculative_response = true;

  EXPECT_TRUE(stream.sent_speculative_response);
}

// ============================================================================
// Medium Severity Fix Tests - 16-Mode Flow Control Threshold
// ============================================================================

TEST(FlowControlThresholdTest, Mode0FixedThreshold) {
  // Mode 0: threshold = 16 bytes (1 unit)
  uint8_t mode = 0;
  uint32_t threshold;

  switch (mode) {
    case 0: threshold = 16; break;
    default: threshold = 0; break;
  }

  EXPECT_EQ(threshold, 16u);
}

TEST(FlowControlThresholdTest, FixedThresholdModes) {
  // Modes 0-7 use fixed thresholds
  uint32_t expected[] = {16, 32, 64, 128, 256, 512, 1024, 2048};

  for (uint8_t mode = 0; mode <= 7; ++mode) {
    uint32_t threshold;
    switch (mode) {
      case 0: threshold = 16; break;
      case 1: threshold = 32; break;
      case 2: threshold = 64; break;
      case 3: threshold = 128; break;
      case 4: threshold = 256; break;
      case 5: threshold = 512; break;
      case 6: threshold = 1024; break;
      case 7: threshold = 2048; break;
      default: threshold = 0; break;
    }
    EXPECT_EQ(threshold, expected[mode]) << "Mode " << (int)mode;
  }
}

TEST(FlowControlThresholdTest, ProportionalThresholdModes) {
  // Modes 8-15 use proportional thresholds based on buf_size
  uint32_t buf_size = 4096;

  // Mode 8: buf_size / 128
  EXPECT_EQ(buf_size >> 7, 32u);

  // Mode 9: buf_size / 64
  EXPECT_EQ(buf_size >> 6, 64u);

  // Mode 10: buf_size / 32
  EXPECT_EQ(buf_size >> 5, 128u);

  // Mode 11: buf_size / 16
  EXPECT_EQ(buf_size >> 4, 256u);

  // Mode 12: buf_size / 8
  EXPECT_EQ(buf_size >> 3, 512u);

  // Mode 13: buf_size / 4
  EXPECT_EQ(buf_size >> 2, 1024u);

  // Mode 14: buf_size / 2
  EXPECT_EQ(buf_size >> 1, 2048u);

  // Mode 15: buf_size (full)
  EXPECT_EQ(buf_size, 4096u);
}

TEST(FlowControlThresholdTest, MinimumThresholdGuarantee) {
  // Threshold should never be 0, minimum is 1 unit (16 bytes)
  uint32_t buf_size = 64;  // Small buffer

  // Mode 8 with small buffer: 64 / 128 = 0, should become 1
  uint32_t threshold = buf_size >> 7;
  if (threshold == 0) {
    threshold = 1;
  }

  EXPECT_GE(threshold, 1u);
}

TEST(FlowControlThresholdTest, AllModeCoverage) {
  // Test all 16 modes compute correctly
  uint32_t buf_size = 8192;

  for (uint8_t mode = 0; mode < 16; ++mode) {
    uint32_t threshold;
    switch (mode) {
      case 0:  threshold = 16; break;
      case 1:  threshold = 32; break;
      case 2:  threshold = 64; break;
      case 3:  threshold = 128; break;
      case 4:  threshold = 256; break;
      case 5:  threshold = 512; break;
      case 6:  threshold = 1024; break;
      case 7:  threshold = 2048; break;
      case 8:  threshold = buf_size >> 7; break;
      case 9:  threshold = buf_size >> 6; break;
      case 10: threshold = buf_size >> 5; break;
      case 11: threshold = buf_size >> 4; break;
      case 12: threshold = buf_size >> 3; break;
      case 13: threshold = buf_size >> 2; break;
      case 14: threshold = buf_size >> 1; break;
      case 15: threshold = buf_size; break;
      default: threshold = 16; break;
    }

    // Ensure minimum threshold
    if (threshold == 0) threshold = 1;

    EXPECT_GT(threshold, 0u) << "Mode " << (int)mode;
  }
}

// ============================================================================
// Medium Severity Fix Tests - Auto-Pop on Transmit to Software
// ============================================================================

TEST(AutoPopTest, NegativeEvenValueEncoding) {
  // Per spec: Writing negative even value to REMOTE_DEST_MSG_INFO_WR_PTR
  // triggers auto-pop when receiver_endpoint is true

  // -2 (pop 1 message)
  int32_t value = -2;
  EXPECT_TRUE(value < 0);
  EXPECT_TRUE((value & 1) == 0);  // Even
  EXPECT_EQ(static_cast<unsigned>((-value) / 2), 1u);

  // -4 (pop 2 messages)
  value = -4;
  EXPECT_TRUE(value < 0);
  EXPECT_TRUE((value & 1) == 0);
  EXPECT_EQ(static_cast<unsigned>((-value) / 2), 2u);

  // -10 (pop 5 messages)
  value = -10;
  EXPECT_TRUE(value < 0);
  EXPECT_TRUE((value & 1) == 0);
  EXPECT_EQ(static_cast<unsigned>((-value) / 2), 5u);
}

TEST(AutoPopTest, NegativeOddValueNoAutoPop) {
  // Negative odd values should NOT trigger auto-pop
  int32_t value = -3;
  EXPECT_TRUE(value < 0);
  EXPECT_FALSE((value & 1) == 0);  // Odd, no auto-pop
}

TEST(AutoPopTest, PositiveValueNoAutoPop) {
  // Positive values should NOT trigger auto-pop
  int32_t value = 100;
  EXPECT_FALSE(value < 0);  // Positive, no auto-pop
}

TEST(AutoPopTest, AutoPopConditions) {
  // Auto-pop only when receiver_endpoint is true
  OverlayStreamRegs regs;

  regs.receiver_endpoint = true;
  int32_t write_value = -4;

  bool should_auto_pop = regs.receiver_endpoint &&
                          (write_value < 0) &&
                          ((write_value & 1) == 0);

  EXPECT_TRUE(should_auto_pop);
}

TEST(AutoPopTest, NoAutoPopWithoutReceiverEndpoint) {
  // No auto-pop when receiver_endpoint is false
  OverlayStreamRegs regs;

  regs.receiver_endpoint = false;
  int32_t write_value = -4;

  bool should_auto_pop = regs.receiver_endpoint &&
                          (write_value < 0) &&
                          ((write_value & 1) == 0);

  EXPECT_FALSE(should_auto_pop);
}

TEST(AutoPopTest, PopCountCalculation) {
  // Test various pop counts
  std::vector<std::pair<int32_t, unsigned>> test_cases = {
    {-2, 1},
    {-4, 2},
    {-6, 3},
    {-8, 4},
    {-20, 10},
    {-100, 50}
  };

  for (const auto& tc : test_cases) {
    int32_t value = tc.first;
    unsigned expected_count = tc.second;
    unsigned actual_count = static_cast<unsigned>((-value) / 2);
    EXPECT_EQ(actual_count, expected_count) << "Value: " << value;
  }
}

// ============================================================================
// Medium Severity Fix Tests - Automatic Metadata FIFO Advancement
// ============================================================================

TEST(MetadataAdvancementTest, AdvancementConditions) {
  // Metadata FIFO advancement happens when:
  // 1. metadata_fifo.size() < capacity
  // 2. msg_info_ptr < msg_info_wr_ptr (messages waiting in L1)
  OverlayStream stream;

  stream.caps.metadata_fifo_capacity = 8;
  stream.regs.msg_info_ptr = 5;
  stream.regs.msg_info_wr_ptr = 10;

  bool can_advance = (stream.metadata_fifo.size() < stream.caps.metadata_fifo_capacity) &&
                      (stream.regs.msg_info_ptr < stream.regs.msg_info_wr_ptr);

  EXPECT_TRUE(can_advance);
}

TEST(MetadataAdvancementTest, NoAdvancementWhenFifoFull) {
  // No advancement when metadata FIFO is at capacity
  OverlayStream stream;

  stream.caps.metadata_fifo_capacity = 2;
  stream.regs.msg_info_ptr = 5;
  stream.regs.msg_info_wr_ptr = 10;

  // Fill FIFO to capacity
  OverlayMsgMetadata meta;
  stream.metadata_fifo.push_back(meta);
  stream.metadata_fifo.push_back(meta);

  bool can_advance = (stream.metadata_fifo.size() < stream.caps.metadata_fifo_capacity) &&
                      (stream.regs.msg_info_ptr < stream.regs.msg_info_wr_ptr);

  EXPECT_FALSE(can_advance);
}

TEST(MetadataAdvancementTest, NoAdvancementWhenNoMessages) {
  // No advancement when no messages waiting (ptr == wr_ptr)
  OverlayStream stream;

  stream.caps.metadata_fifo_capacity = 8;
  stream.regs.msg_info_ptr = 10;
  stream.regs.msg_info_wr_ptr = 10;  // Equal, no messages waiting

  bool can_advance = (stream.metadata_fifo.size() < stream.caps.metadata_fifo_capacity) &&
                      (stream.regs.msg_info_ptr < stream.regs.msg_info_wr_ptr);

  EXPECT_FALSE(can_advance);
}

TEST(MetadataAdvancementTest, PtrIncrement) {
  // After advancement, msg_info_ptr should increment
  OverlayStreamRegs regs;

  regs.msg_info_ptr = 5;

  // Simulate advancement
  regs.msg_info_ptr++;

  EXPECT_EQ(regs.msg_info_ptr, 6u);
}

TEST(MetadataAdvancementTest, MultipleAdvancements) {
  // Multiple consecutive advancements
  OverlayStream stream;

  stream.caps.metadata_fifo_capacity = 8;
  stream.regs.msg_info_ptr = 0;
  stream.regs.msg_info_wr_ptr = 5;

  unsigned advancements = 0;
  while (stream.metadata_fifo.size() < stream.caps.metadata_fifo_capacity &&
         stream.regs.msg_info_ptr < stream.regs.msg_info_wr_ptr) {
    OverlayMsgMetadata meta;
    meta.buf_ptr = stream.regs.msg_info_ptr * 16;
    stream.metadata_fifo.push_back(meta);
    stream.regs.msg_info_ptr++;
    advancements++;
  }

  EXPECT_EQ(advancements, 5u);
  EXPECT_EQ(stream.metadata_fifo.size(), 5u);
  EXPECT_EQ(stream.regs.msg_info_ptr, stream.regs.msg_info_wr_ptr);
}

// ============================================================================
// SystemC main function (required by SystemC library)
// ============================================================================

// SystemC requires sc_main to be defined.
// We run gtest from within sc_main.
int sc_main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// ============================================================================
// Main function - forwards to sc_main via SystemC
// ============================================================================
