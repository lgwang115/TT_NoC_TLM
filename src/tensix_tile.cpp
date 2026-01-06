#include "tensix_tile.h"

TensixTile::TensixTile(sc_core::sc_module_name name,
                       NocCoord coord_in,
                       sc_core::sc_time cycle_time_in)
    : sc_module(name),
      coord(coord_in),
      cycle_time(cycle_time_in),
      niu0_("niu0", coord_in, NocId::NOC0, cycle_time_in),
      niu1_("niu1", coord_in, NocId::NOC1, cycle_time_in),
      riscv_b_("riscv_b", cycle_time_in, coord_in, 0),
      riscv_nc_("riscv_nc", cycle_time_in, coord_in, 4),
      cmd_fifo_0_(4),
      resp_fifo_0_(4),
      mmio_req_fifo_0_(4),
      mmio_resp_fifo_0_(4),
      cmd_fifo_1_(4),
      resp_fifo_1_(4),
      mmio_req_fifo_1_(4),
      mmio_resp_fifo_1_(4) {
  niu0_.net_in.bind(noc0_in);
  niu0_.net_out.bind(noc0_out);
  niu1_.net_in.bind(noc1_in);
  niu1_.net_out.bind(noc1_out);

  niu0_.cmd_in.bind(cmd_fifo_0_);
  niu0_.resp_out.bind(resp_fifo_0_);
  niu0_.mmio_in.bind(mmio_req_fifo_0_);
  niu0_.mmio_out.bind(mmio_resp_fifo_0_);

  niu1_.cmd_in.bind(cmd_fifo_1_);
  niu1_.resp_out.bind(resp_fifo_1_);
  niu1_.mmio_in.bind(mmio_req_fifo_1_);
  niu1_.mmio_out.bind(mmio_resp_fifo_1_);

  riscv_b_.mmio_out.bind(mmio_req_fifo_0_);
  riscv_b_.mmio_in.bind(mmio_resp_fifo_0_);
  riscv_b_.resp_in.bind(resp_fifo_0_);

  riscv_nc_.mmio_out.bind(mmio_req_fifo_1_);
  riscv_nc_.mmio_in.bind(mmio_resp_fifo_1_);
      riscv_nc_.resp_in.bind(resp_fifo_1_);
}

void TensixTile::configure_riscv_b(const RiscvCore::Config &config) {
  riscv_b_.configure(config);
}

void TensixTile::configure_riscv_nc(const RiscvCore::Config &config) {
  riscv_nc_.configure(config);
}
