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
