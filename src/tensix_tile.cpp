#include "tensix_tile.h"

TensixTile::TensixTile(sc_core::sc_module_name name,
                       NocCoord coord_in,
                       sc_core::sc_time cycle_time_in,
                       unsigned mesh_w,
                       unsigned mesh_h)
    : sc_module(name),
      coord(coord_in),
      cycle_time(cycle_time_in),
      mesh_width(mesh_w),
      mesh_height(mesh_h),
      niu0_("niu0", coord_in, NocId::NOC0, cycle_time_in, mesh_w, mesh_h),
      niu1_("niu1", coord_in, NocId::NOC1, cycle_time_in, mesh_w, mesh_h),
      riscv_b_("riscv_b", cycle_time_in, coord_in, 0),
      riscv_t0_("riscv_t0", cycle_time_in, coord_in, 1),
      riscv_t1_("riscv_t1", cycle_time_in, coord_in, 2),
      riscv_t2_("riscv_t2", cycle_time_in, coord_in, 3),
      riscv_nc_("riscv_nc", cycle_time_in, coord_in, 4),
      cmd_fifo_0_(4),
      resp_fifo_0_(4),
      mmio_req_fifo_0_(4),
      mmio_resp_fifo_0_(4),
      cmd_fifo_1_(4),
      resp_fifo_1_(4),
      mmio_req_fifo_1_(4),
      mmio_resp_fifo_1_(4),
      mmio_req_fifo_t0_(4),
      mmio_resp_fifo_t0_(4),
      resp_fifo_t0_(4),
      mmio_req_fifo_t1_(4),
      mmio_resp_fifo_t1_(4),
      resp_fifo_t1_(4),
      mmio_req_fifo_t2_(4),
      mmio_resp_fifo_t2_(4),
      resp_fifo_t2_(4),
      mmio_req_mux_fifo_(16),
      mmio_resp_mux_fifo_(16) {
  // Connect NIUs to external NoC ports
  niu0_.net_in.bind(noc0_in);
  niu0_.net_out.bind(noc0_out);
  niu1_.net_in.bind(noc1_in);
  niu1_.net_out.bind(noc1_out);

  // NIU0 command/response (shared by all cores that access NoC0)
  niu0_.cmd_in.bind(cmd_fifo_0_);
  niu0_.resp_out.bind(resp_fifo_0_);
  niu0_.mmio_in.bind(mmio_req_mux_fifo_);
  niu0_.mmio_out.bind(mmio_resp_mux_fifo_);

  // NIU1 command/response (used by NC core)
  niu1_.cmd_in.bind(cmd_fifo_1_);
  niu1_.resp_out.bind(resp_fifo_1_);
  niu1_.mmio_in.bind(mmio_req_fifo_1_);
  niu1_.mmio_out.bind(mmio_resp_fifo_1_);

  // B-core connects to NIU0 initiator 0
  riscv_b_.mmio_out.bind(mmio_req_fifo_0_);
  riscv_b_.mmio_in.bind(mmio_resp_fifo_0_);
  riscv_b_.resp_in.bind(resp_fifo_0_);

  // T0 core connects to NIU0 initiator 1
  riscv_t0_.mmio_out.bind(mmio_req_fifo_t0_);
  riscv_t0_.mmio_in.bind(mmio_resp_fifo_t0_);
  riscv_t0_.resp_in.bind(resp_fifo_t0_);

  // T1 core connects to NIU0 initiator 2
  riscv_t1_.mmio_out.bind(mmio_req_fifo_t1_);
  riscv_t1_.mmio_in.bind(mmio_resp_fifo_t1_);
  riscv_t1_.resp_in.bind(resp_fifo_t1_);

  // T2 core connects to NIU0 initiator 3
  riscv_t2_.mmio_out.bind(mmio_req_fifo_t2_);
  riscv_t2_.mmio_in.bind(mmio_resp_fifo_t2_);
  riscv_t2_.resp_in.bind(resp_fifo_t2_);

  // NC-core connects to NIU1 initiator 0
  riscv_nc_.mmio_out.bind(mmio_req_fifo_1_);
  riscv_nc_.mmio_in.bind(mmio_resp_fifo_1_);
  riscv_nc_.resp_in.bind(resp_fifo_1_);

  // Disable T-cores by default (they drive Tensix coprocessor which is modeled elsewhere)
  RiscvCore::Config disabled_config;
  disabled_config.enabled = false;
  riscv_t0_.configure(disabled_config);
  riscv_t1_.configure(disabled_config);
  riscv_t2_.configure(disabled_config);

  SC_THREAD(t_cores_mux_thread);
}

void TensixTile::t_cores_mux_thread() {
  // Multiplexer to route MMIO requests from B and T cores to NIU0
  // Each core uses a different initiator offset:
  // - B-core: initiator 0 (offset 0x000)
  // - T0: initiator 1 (offset 0x400)
  // - T1: initiator 2 (offset 0x800)
  // - T2: initiator 3 (offset 0xC00)
  constexpr uint32_t kInitiatorOffset = 0x400;

  while (true) {
    bool did_work = false;

    // Check B-core requests (initiator 0, no offset adjustment)
    NiuMmioReq req;
    if (mmio_req_fifo_0_.nb_read(req)) {
      mmio_req_mux_fifo_.write(req);
      did_work = true;
    }

    // Check T0 requests (initiator 1)
    if (mmio_req_fifo_t0_.nb_read(req)) {
      req.addr = (req.addr % kInitiatorOffset) + kInitiatorOffset * 1;
      mmio_req_mux_fifo_.write(req);
      did_work = true;
    }

    // Check T1 requests (initiator 2)
    if (mmio_req_fifo_t1_.nb_read(req)) {
      req.addr = (req.addr % kInitiatorOffset) + kInitiatorOffset * 2;
      mmio_req_mux_fifo_.write(req);
      did_work = true;
    }

    // Check T2 requests (initiator 3)
    if (mmio_req_fifo_t2_.nb_read(req)) {
      req.addr = (req.addr % kInitiatorOffset) + kInitiatorOffset * 3;
      mmio_req_mux_fifo_.write(req);
      did_work = true;
    }

    // Route responses back (simplified: responses go to B-core by default)
    // In real hardware, responses would be tagged with initiator ID
    NiuMmioResp resp;
    if (mmio_resp_mux_fifo_.nb_read(resp)) {
      mmio_resp_fifo_0_.write(resp);
      did_work = true;
    }

    if (!did_work) {
      wait(cycle_time);
    } else {
      wait(sc_core::SC_ZERO_TIME);
    }
  }
}

void TensixTile::configure_riscv_b(const RiscvCore::Config &config) {
  riscv_b_.configure(config);
}

void TensixTile::configure_riscv_t0(const RiscvCore::Config &config) {
  riscv_t0_.configure(config);
}

void TensixTile::configure_riscv_t1(const RiscvCore::Config &config) {
  riscv_t1_.configure(config);
}

void TensixTile::configure_riscv_t2(const RiscvCore::Config &config) {
  riscv_t2_.configure(config);
}

void TensixTile::configure_riscv_nc(const RiscvCore::Config &config) {
  riscv_nc_.configure(config);
}
