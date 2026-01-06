#ifndef TENSIX_TILE_H
#define TENSIX_TILE_H

#include <systemc>

#include "noc_niu.h"
#include "riscv_core.h"

SC_MODULE(TensixTile) {
  sc_core::sc_fifo_in<NocFlit> noc0_in;
  sc_core::sc_fifo_out<NocFlit> noc0_out;
  sc_core::sc_fifo_in<NocFlit> noc1_in;
  sc_core::sc_fifo_out<NocFlit> noc1_out;

  NocCoord coord;
  sc_core::sc_time cycle_time;

  explicit TensixTile(sc_core::sc_module_name name,
                      NocCoord coord,
                      sc_core::sc_time cycle_time);

  void configure_riscv_b(const RiscvCore::Config &config);
  void configure_riscv_nc(const RiscvCore::Config &config);

 private:
  NocNiu niu0_;
  NocNiu niu1_;
  RiscvCore riscv_b_;
  RiscvCore riscv_nc_;

  sc_core::sc_fifo<NiuCmd> cmd_fifo_0_;
  sc_core::sc_fifo<NiuResp> resp_fifo_0_;
  sc_core::sc_fifo<NiuMmioReq> mmio_req_fifo_0_;
  sc_core::sc_fifo<NiuMmioResp> mmio_resp_fifo_0_;

  sc_core::sc_fifo<NiuCmd> cmd_fifo_1_;
  sc_core::sc_fifo<NiuResp> resp_fifo_1_;
  sc_core::sc_fifo<NiuMmioReq> mmio_req_fifo_1_;
  sc_core::sc_fifo<NiuMmioResp> mmio_resp_fifo_1_;
};

#endif
