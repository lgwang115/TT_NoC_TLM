#ifndef NOC_MESH_H
#define NOC_MESH_H

#include <memory>
#include <vector>

#include "noc_link.h"
#include "noc_router.h"
#include <systemc>

class NocMesh : public sc_core::sc_module {
 public:
  NocMesh(sc_core::sc_module_name name,
          unsigned width,
          unsigned height,
          NocId noc_id,
          sc_core::sc_time cycle_time);

  sc_core::sc_fifo<NocFlit> &local_in(unsigned x, unsigned y);
  sc_core::sc_fifo<NocFlit> &local_out(unsigned x, unsigned y);

 private:
  struct TileChannels {
    std::array<std::unique_ptr<sc_core::sc_fifo<NocFlit>>, kNocPortCount> in;
    std::array<std::unique_ptr<sc_core::sc_fifo<NocFlit>>, kNocPortCount> out;
  };

  unsigned width_ = 0;
  unsigned height_ = 0;
  NocId noc_id_ = NocId::NOC0;
  sc_core::sc_time cycle_time_;

  std::vector<std::unique_ptr<NocRouter>> routers_;
  std::vector<std::unique_ptr<TileChannels>> channels_;

  std::vector<std::unique_ptr<sc_core::sc_fifo<NocFlit>>> local_in_;
  std::vector<std::unique_ptr<sc_core::sc_fifo<NocFlit>>> local_out_;

  std::vector<std::unique_ptr<NocLink>> links_;

  size_t index(unsigned x, unsigned y) const { return y * width_ + x; }
  void build_links();
};

#endif
