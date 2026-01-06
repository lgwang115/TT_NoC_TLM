#ifndef NOC_LINK_H
#define NOC_LINK_H

#include <deque>

#include "noc_types.h"
#include <systemc>

struct TimedFlit {
  NocFlit flit;
  sc_core::sc_time ready_time;
};

SC_MODULE(NocLink) {
  sc_core::sc_fifo_in<NocFlit> in;
  sc_core::sc_fifo_out<NocFlit> out;

  sc_core::sc_time latency;
  sc_core::sc_time cycle_time;

  explicit NocLink(sc_core::sc_module_name name,
                   sc_core::sc_time latency_time,
                   sc_core::sc_time cycle);

  void run();

 private:
  std::deque<TimedFlit> pending_;
};

#endif
