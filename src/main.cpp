#include "noc_mesh.h"
#include "tensix_tile.h"

#include <systemc>

int sc_main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  const unsigned width = 10;
  const unsigned height = 12;
  const sc_core::sc_time cycle_time(1, sc_core::SC_NS);

  NocMesh mesh0("mesh0", width, height, NocId::NOC0, cycle_time);
  NocMesh mesh1("mesh1", width, height, NocId::NOC1, cycle_time);

  TensixTile tile00("tile00", NocCoord{0, 0}, cycle_time);
  TensixTile tile32("tile32", NocCoord{3, 2}, cycle_time);

  tile00.noc0_out.bind(mesh0.local_in(0, 0));
  tile00.noc0_in.bind(mesh0.local_out(0, 0));
  tile32.noc0_out.bind(mesh0.local_in(3, 2));
  tile32.noc0_in.bind(mesh0.local_out(3, 2));

  tile00.noc1_out.bind(mesh1.local_in(0, 0));
  tile00.noc1_in.bind(mesh1.local_out(0, 0));
  tile32.noc1_out.bind(mesh1.local_in(3, 2));
  tile32.noc1_in.bind(mesh1.local_out(3, 2));

  RiscvCore::Config cfg_to_tile32;
  cfg_to_tile32.mesh_width = width;
  cfg_to_tile32.mesh_height = height;
  cfg_to_tile32.dst_dx = 3;
  cfg_to_tile32.dst_dy = 2;
  cfg_to_tile32.do_write = true;
  cfg_to_tile32.do_read = true;

  RiscvCore::Config cfg_to_tile00;
  cfg_to_tile00.mesh_width = width;
  cfg_to_tile00.mesh_height = height;
  cfg_to_tile00.dst_dx = -3;
  cfg_to_tile00.dst_dy = -2;
  cfg_to_tile00.do_write = true;
  cfg_to_tile00.do_read = true;

  tile00.configure_riscv_b(cfg_to_tile32);
  tile00.configure_riscv_nc(cfg_to_tile32);
  tile32.configure_riscv_b(cfg_to_tile00);
  tile32.configure_riscv_nc(cfg_to_tile00);

  sc_core::sc_start(sc_core::sc_time(5000, sc_core::SC_NS));
  return 0;
}
