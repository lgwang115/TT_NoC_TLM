#include "noc_link.h"

#include <iostream>

NocLink::NocLink(sc_core::sc_module_name name,
                 sc_core::sc_time latency_time,
                 sc_core::sc_time cycle)
    : sc_module(name), latency(latency_time), cycle_time(cycle) {
  SC_THREAD(run);
}

void NocLink::run() {
  while (true) {
    bool progressed = false;

    NocFlit flit;
    if (in.nb_read(flit)) {
      pending_.push_back(TimedFlit{flit, sc_core::sc_time_stamp() + latency});
      std::cout << sc_core::sc_time_stamp() << " link " << name() << " in pkt "
                << flit.packet_id << " flit " << flit.flit_index << "/" << flit.total_flits
                << std::endl;
      progressed = true;
    }

    if (!pending_.empty() && pending_.front().ready_time <= sc_core::sc_time_stamp()) {
      if (out.nb_write(pending_.front().flit)) {
        const NocFlit &out_flit = pending_.front().flit;
        std::cout << sc_core::sc_time_stamp() << " link " << name() << " out pkt "
                  << out_flit.packet_id << " flit " << out_flit.flit_index << "/"
                  << out_flit.total_flits << std::endl;
        pending_.pop_front();
        progressed = true;
      }
    }

    if (!progressed) {
      wait(cycle_time);
    } else {
      wait(sc_core::SC_ZERO_TIME);
    }
  }
}
