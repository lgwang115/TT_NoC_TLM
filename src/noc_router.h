#ifndef NOC_ROUTER_H
#define NOC_ROUTER_H

#include <array>
#include <deque>
#include <unordered_map>

#include "noc_types.h"
#include <systemc>

enum class NocPort {
  Local = 0,
  North = 1,
  South = 2,
  East = 3,
  West = 4,
  Count = 5,
};

inline constexpr size_t kNocPortCount = static_cast<size_t>(NocPort::Count);

SC_MODULE(NocRouter) {
  SC_HAS_PROCESS(NocRouter);

  std::array<sc_core::sc_fifo_in<NocFlit>, kNocPortCount> in;
  std::array<sc_core::sc_fifo_out<NocFlit>, kNocPortCount> out;

  NocCoord coord;
  NocId noc_id;
  unsigned width;
  unsigned height;
  sc_core::sc_time cycle_time;

  explicit NocRouter(sc_core::sc_module_name name,
                     NocCoord coord,
                     NocId noc_id,
                     unsigned width,
                     unsigned height,
                     sc_core::sc_time cycle);

  void route();

 private:
  static constexpr size_t kVcs = 16;
  struct QueuedFlit {
    NocFlit flit;
    sc_core::sc_time enqueue_time;
    uint32_t pending_mask = 0;
  };

  static constexpr size_t kClasses = 4;
  static constexpr std::array<unsigned, kClasses> kClassWeights = {1, 1, 1, 2};
  static constexpr unsigned kBuddyFlipCycles = 3;

  std::array<std::array<std::deque<QueuedFlit>, kVcs>, kNocPortCount> input_vcs_;
  std::array<size_t, kNocPortCount> rr_index_ = {};
  std::array<size_t, kNocPortCount> class_cursor_ = {};
  std::array<unsigned, kNocPortCount> class_remaining_ = {};
  std::unordered_map<uint64_t, bool> packet_buddy_flip_;
  std::unordered_map<uint64_t, bool> linked_buddy_flip_;
  std::unordered_map<uint32_t, uint64_t> reserved_vc_;

  struct RouteDecision {
    NocPort port;
    bool dateline_flip;
  };

  RouteDecision route_port(const NocFlit &flit) const;
  uint8_t next_vc_id(const NocFlit &flit, bool dateline_flip, bool buddy_flip) const;
  bool try_send_on_port(size_t out_idx);
  bool try_send_on_port_for_class(size_t out_idx, uint8_t class_bits);
  void ensure_class_state(size_t out_idx);
  void advance_class_cursor(size_t out_idx);
  uint32_t compute_out_mask(const NocFlit &flit) const;
  bool span_contains(unsigned start, unsigned end, unsigned value, unsigned size) const;
  bool span_has_ahead(unsigned curr,
                      unsigned start,
                      unsigned end,
                      unsigned size,
                      int dir) const;
  bool dateline_flip_for_port(NocPort port) const;
};

#endif
