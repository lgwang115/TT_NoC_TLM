#include "traffic_gen.h"

#include <iostream>
#include <unordered_map>
#include <utility>

TrafficGen::TrafficGen(sc_core::sc_module_name name,
                       NocCoord coord_in,
                       NocId noc_id_in,
                       NocPacket pkt,
                       sc_core::sc_time start,
                       sc_core::sc_time cycle,
                       bool enable_send_in)
    : sc_module(name),
      coord(coord_in),
      noc_id(noc_id_in),
      start_time(start),
      cycle_time(cycle),
      packet(std::move(pkt)),
      enable_send(enable_send_in),
      translation_(default_translation(noc_id_in)) {
  if (enable_send) {
    SC_THREAD(send);
  }
  SC_THREAD(recv);
}

void TrafficGen::send() {
  wait(start_time);

  NocCoord translated_dst = packet.coord_translate ? translate_coord(packet.dst, translation_)
                                                   : packet.dst;
  NocCoord translated_start = packet.coord_translate ? translate_coord(packet.brcst_start, translation_)
                                                     : packet.brcst_start;
  NocCoord translated_end = packet.coord_translate ? translate_coord(packet.brcst_end, translation_)
                                                   : packet.brcst_end;

  if (translated_start.x > translated_end.x) {
    std::swap(translated_start.x, translated_end.x);
  }
  if (translated_start.y > translated_end.y) {
    std::swap(translated_start.y, translated_end.y);
  }

  if (packet.is_broadcast) {
    if (translated_start.x <= translated_end.x && translated_start.y <= translated_end.y) {
      for (unsigned y = translated_start.y; y <= translated_end.y; ++y) {
        for (unsigned x = translated_start.x; x <= translated_end.x; ++x) {
          send_packet_to(NocCoord{x, y});
        }
      }
    }
  } else {
    send_packet_to(translated_dst);
  }
}

void TrafficGen::send_packet_to(const NocCoord &dst) {
  static std::unordered_map<uint64_t, uint8_t> linked_vc_map;
  unsigned total_flits = packet.total_flits();
  uint8_t vc_id = make_vc_id(0, packet.class_bits, packet.buddy_bit);
  if (packet.vc_linked && packet.linked_id != 0) {
    auto it = linked_vc_map.find(packet.linked_id);
    if (it == linked_vc_map.end()) {
      linked_vc_map[packet.linked_id] = vc_id;
    } else {
      vc_id = it->second;
    }
  }

  for (unsigned i = 0; i < total_flits; ++i) {
    NocFlit flit;
    flit.packet_id = packet.id;
    flit.src = packet.src;
    flit.dst = dst;
    flit.brcst_start = packet.brcst_start;
    flit.brcst_end = packet.brcst_end;
    flit.is_broadcast = packet.is_broadcast;
    flit.brcst_xy_major = packet.brcst_xy_major;
    flit.brcst_src_include = packet.brcst_src_include;
    flit.flit_index = i;
    flit.total_flits = total_flits;
    flit.is_header = (i == 0);
    flit.is_response = packet.is_response;
    flit.posted = packet.posted;
    flit.req_type = packet.req_type;
    flit.addr = packet.addr;
    flit.ret_addr = packet.ret_addr;
    flit.ret_coord = packet.ret_coord;
    flit.length = packet.length;
    flit.atomic_imm = packet.atomic_imm;
    flit.has_byte_enable = packet.has_byte_enable;
    flit.byte_enable = packet.byte_enable;
    flit.transaction_id = packet.transaction_id;
    flit.packet_tag = packet.packet_tag;
    flit.at_len_be_raw = packet.at_len_be_raw;
    flit.stream_id = packet.stream_id;
    flit.deliver_to_overlay = packet.deliver_to_overlay;
    flit.msg_first = packet.msg_first;
    flit.msg_last = packet.msg_last;
    flit.header_store = packet.header_store;
    flit.path_reserve = packet.path_reserve;
    flit.arb_priority = packet.arb_priority;
    flit.mem_rd_drop_ack = packet.mem_rd_drop_ack;
    flit.vc_id = vc_id;
    flit.vc_static = packet.vc_static;
    flit.vc_linked = packet.vc_linked;
    flit.linked_id = packet.linked_id;

    if (!packet.payload.empty() && (i > 0 || packet.force_single_flit)) {
      size_t offset = (i == 0) ? 0 : (i - 1) * 32;
      size_t remaining = packet.payload.size() - offset;
      size_t copy_len = remaining < 32 ? remaining : 32;
      for (size_t b = 0; b < copy_len; ++b) {
        flit.data[b] = packet.payload[offset + b];
      }
    }

    inject.write(flit);
    wait(cycle_time);
  }
}

void TrafficGen::recv() {
  while (true) {
    NocFlit flit = egress.read();
    ++received_flits_;
    if (flit.is_tail()) {
      std::cout << sc_core::sc_time_stamp() << " received packet " << flit.packet_id
                << " at " << coord_to_string(coord) << " with " << received_flits_
                << " flits" << std::endl;
      received_flits_ = 0;
    }
  }
}
