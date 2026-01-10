#ifndef TENSIX_TILE_H
#define TENSIX_TILE_H

#include <systemc>

#include "noc_niu.h"
#include "riscv_core.h"

SC_MODULE(TensixTile) {
  SC_HAS_PROCESS(TensixTile);

  sc_core::sc_fifo_in<NocFlit> noc0_in;
  sc_core::sc_fifo_out<NocFlit> noc0_out;
  sc_core::sc_fifo_in<NocFlit> noc1_in;
  sc_core::sc_fifo_out<NocFlit> noc1_out;

  NocCoord coord;
  sc_core::sc_time cycle_time;
  unsigned mesh_width;
  unsigned mesh_height;

  explicit TensixTile(sc_core::sc_module_name name,
                      NocCoord coord,
                      sc_core::sc_time cycle_time,
                      unsigned mesh_width = 10,
                      unsigned mesh_height = 12);

  // Configuration for all 5 Baby RISC-V cores per spec
  void configure_riscv_b(const RiscvCore::Config &config);
  void configure_riscv_t0(const RiscvCore::Config &config);
  void configure_riscv_t1(const RiscvCore::Config &config);
  void configure_riscv_t2(const RiscvCore::Config &config);
  void configure_riscv_nc(const RiscvCore::Config &config);

 private:
  NocNiu niu0_;
  NocNiu niu1_;

  // 5 Baby RISC-V cores per spec:
  // - B (Brisc): General control, core_id=0
  // - T0, T1, T2 (Trisc): Tensix coprocessor drivers, core_id=1,2,3
  // - NC (NCrisc): NoC control, core_id=4
  RiscvCore riscv_b_;
  RiscvCore riscv_t0_;
  RiscvCore riscv_t1_;
  RiscvCore riscv_t2_;
  RiscvCore riscv_nc_;

  // FIFOs for B-core (uses NIU0 initiator 0)
  sc_core::sc_fifo<NiuCmd> cmd_fifo_0_;
  sc_core::sc_fifo<NiuResp> resp_fifo_0_;
  sc_core::sc_fifo<NiuMmioReq> mmio_req_fifo_0_;
  sc_core::sc_fifo<NiuMmioResp> mmio_resp_fifo_0_;

  // FIFOs for NC-core (uses NIU1 initiator 0)
  sc_core::sc_fifo<NiuCmd> cmd_fifo_1_;
  sc_core::sc_fifo<NiuResp> resp_fifo_1_;
  sc_core::sc_fifo<NiuMmioReq> mmio_req_fifo_1_;
  sc_core::sc_fifo<NiuMmioResp> mmio_resp_fifo_1_;

  // FIFOs for T0, T1, T2 cores (share NIU0 initiators 1, 2, 3)
  sc_core::sc_fifo<NiuMmioReq> mmio_req_fifo_t0_;
  sc_core::sc_fifo<NiuMmioResp> mmio_resp_fifo_t0_;
  sc_core::sc_fifo<NiuResp> resp_fifo_t0_;

  sc_core::sc_fifo<NiuMmioReq> mmio_req_fifo_t1_;
  sc_core::sc_fifo<NiuMmioResp> mmio_resp_fifo_t1_;
  sc_core::sc_fifo<NiuResp> resp_fifo_t1_;

  sc_core::sc_fifo<NiuMmioReq> mmio_req_fifo_t2_;
  sc_core::sc_fifo<NiuMmioResp> mmio_resp_fifo_t2_;
  sc_core::sc_fifo<NiuResp> resp_fifo_t2_;

  // Multiplexer thread to route T-core MMIO requests to NIU0
  void t_cores_mux_thread();
  sc_core::sc_fifo<NiuMmioReq> mmio_req_mux_fifo_;
  sc_core::sc_fifo<NiuMmioResp> mmio_resp_mux_fifo_;
};

#endif
