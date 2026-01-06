#ifndef TRAFFIC_GEN_H
#define TRAFFIC_GEN_H

#include "noc_types.h"
#include "noc_translation.h"
#include <systemc>

SC_MODULE(TrafficGen) {
  sc_core::sc_fifo_out<NocFlit> inject;
  sc_core::sc_fifo_in<NocFlit> egress;

  NocCoord coord;
  NocId noc_id;
  sc_core::sc_time start_time;
  sc_core::sc_time cycle_time;
  NocPacket packet;
  bool enable_send = true;

  explicit TrafficGen(sc_core::sc_module_name name,
                      NocCoord coord,
                      NocId noc_id,
                      NocPacket packet,
                      sc_core::sc_time start_time,
                      sc_core::sc_time cycle_time,
                      bool enable_send);

  void send();
  void recv();

 private:
  unsigned received_flits_ = 0;
  NocTranslationTable translation_;
  void send_packet_to(const NocCoord &dst);
};

#endif
