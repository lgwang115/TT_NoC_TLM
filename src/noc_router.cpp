#include "noc_router.h"

#include <iostream>

namespace {
const char *port_name(NocPort port) {
  switch (port) {
    case NocPort::Local:
      return "Local";
    case NocPort::North:
      return "North";
    case NocPort::South:
      return "South";
    case NocPort::East:
      return "East";
    case NocPort::West:
      return "West";
    default:
      return "Unknown";
  }
}
}  // namespace

NocRouter::NocRouter(sc_core::sc_module_name name,
                     NocCoord coord_in,
                     NocId noc,
                     unsigned w,
                     unsigned h,
                     sc_core::sc_time cycle)
    : sc_module(name),
      coord(coord_in),
      noc_id(noc),
      width(w),
      height(h),
      cycle_time(cycle) {
  SC_THREAD(route);
}

NocRouter::RouteDecision NocRouter::route_port(const NocFlit &flit) const {
  if (flit.dst == coord) {
    return {NocPort::Local, false};
  }

  if (noc_id == NocId::NOC0) {
    if (coord.x != flit.dst.x) {
      bool wrap = (width > 1) && (coord.x == width - 1) && (flit.dst.x != width - 1);
      return {NocPort::East, wrap};
    }
    bool wrap = (height > 1) && (coord.y == height - 1) && (flit.dst.y != height - 1);
    return {NocPort::South, wrap};
  }

  if (coord.y != flit.dst.y) {
    bool wrap = (height > 1) && (coord.y == 0) && (flit.dst.y != 0);
    return {NocPort::North, wrap};
  }
  bool wrap = (width > 1) && (coord.x == 0) && (flit.dst.x != 0);
  return {NocPort::West, wrap};
}

bool NocRouter::span_contains(unsigned start, unsigned end, unsigned value, unsigned size) const {
  if (size == 0) {
    return false;
  }
  if (start <= end) {
    return value >= start && value <= end;
  }
  return value >= start || value <= end;
}

bool NocRouter::span_has_ahead(unsigned curr,
                               unsigned start,
                               unsigned end,
                               unsigned size,
                               int dir) const {
  if (size == 0) {
    return false;
  }
  for (unsigned step = 1; step < size; ++step) {
    unsigned pos = (curr + size + static_cast<unsigned>(dir) * step) % size;
    if (span_contains(start, end, pos, size)) {
      return true;
    }
  }
  return false;
}

bool NocRouter::dateline_flip_for_port(NocPort port) const {
  if (noc_id == NocId::NOC0) {
    if (port == NocPort::East) {
      return width > 1 && coord.x == width - 1;
    }
    if (port == NocPort::South) {
      return height > 1 && coord.y == height - 1;
    }
    return false;
  }
  if (port == NocPort::West) {
    return width > 1 && coord.x == 0;
  }
  if (port == NocPort::North) {
    return height > 1 && coord.y == 0;
  }
  return false;
}

uint32_t NocRouter::compute_out_mask(const NocFlit &flit) const {
  if (!flit.is_broadcast) {
    NocPort port = route_port(flit).port;
    return 1u << static_cast<uint32_t>(port);
  }

  unsigned max_x = width;
  unsigned max_y = height;
  bool in_x = span_contains(flit.brcst_start.x, flit.brcst_end.x, coord.x, max_x);
  bool in_y = span_contains(flit.brcst_start.y, flit.brcst_end.y, coord.y, max_y);

  uint32_t mask = 0;
  if (in_x && in_y) {
    if (flit.brcst_src_include || !(coord == flit.src)) {
      mask |= 1u << static_cast<uint32_t>(NocPort::Local);
    }
  }

  int dir_x = (noc_id == NocId::NOC0) ? 1 : -1;
  int dir_y = (noc_id == NocId::NOC0) ? 1 : -1;
  bool x_major = !flit.brcst_xy_major;

  if (x_major) {
    bool send_x = (coord.y == flit.src.y) || in_y;
    if (send_x && span_has_ahead(coord.x, flit.brcst_start.x, flit.brcst_end.x, max_x, dir_x)) {
      mask |= 1u << static_cast<uint32_t>((noc_id == NocId::NOC0) ? NocPort::East : NocPort::West);
    }
    if (in_x && span_has_ahead(coord.y, flit.brcst_start.y, flit.brcst_end.y, max_y, dir_y)) {
      mask |= 1u << static_cast<uint32_t>((noc_id == NocId::NOC0) ? NocPort::South : NocPort::North);
    }
  } else {
    bool send_y = (coord.x == flit.src.x) || in_x;
    if (send_y && span_has_ahead(coord.y, flit.brcst_start.y, flit.brcst_end.y, max_y, dir_y)) {
      mask |= 1u << static_cast<uint32_t>((noc_id == NocId::NOC0) ? NocPort::South : NocPort::North);
    }
    if (in_y && span_has_ahead(coord.x, flit.brcst_start.x, flit.brcst_end.x, max_x, dir_x)) {
      mask |= 1u << static_cast<uint32_t>((noc_id == NocId::NOC0) ? NocPort::East : NocPort::West);
    }
  }

  return mask;
}

uint8_t NocRouter::next_vc_id(const NocFlit &flit, bool dateline_flip, bool buddy_flip) const {
  NocVcFields fields = decode_vc_id(flit.vc_id);
  if (dateline_flip) {
    fields.dateline ^= 0x1u;
  }
  if (buddy_flip) {
    fields.buddy ^= 0x1u;
  }
  return make_vc_id(fields.dateline, fields.class_bits, fields.buddy);
}

void NocRouter::ensure_class_state(size_t out_idx) {
  if (class_remaining_[out_idx] == 0) {
    class_cursor_[out_idx] %= kClasses;
    class_remaining_[out_idx] = kClassWeights[class_cursor_[out_idx]];
  }
}

void NocRouter::advance_class_cursor(size_t out_idx) {
  class_cursor_[out_idx] = (class_cursor_[out_idx] + 1) % kClasses;
  class_remaining_[out_idx] = kClassWeights[class_cursor_[out_idx]];
}

bool NocRouter::try_send_on_port_for_class(size_t out_idx, uint8_t class_bits) {
  size_t start = rr_index_[out_idx] % (kNocPortCount * kVcs);
  bool found = false;
  size_t best_idx = 0;
  uint8_t best_prio = 0;
  for (size_t i = 0; i < kNocPortCount * kVcs; ++i) {
    size_t idx = (start + i) % (kNocPortCount * kVcs);
    size_t in_port = idx / kVcs;
    size_t vc = idx % kVcs;

    auto &q = input_vcs_[in_port][vc];
    if (q.empty()) {
      continue;
    }

    const QueuedFlit &queued = q.front();
    const NocFlit &flit = queued.flit;
    if ((queued.pending_mask & (1u << out_idx)) == 0) {
      continue;
    }
    if (decode_vc_id(flit.vc_id).class_bits != (class_bits & 0x3u)) {
      continue;
    }

    uint8_t prio = flit.arb_priority;
    if (!found || prio > best_prio) {
      found = true;
      best_prio = prio;
      best_idx = idx;
    }
  }

  if (!found) {
    return false;
  }

  size_t in_port = best_idx / kVcs;
  size_t vc = best_idx % kVcs;
  auto &q = input_vcs_[in_port][vc];
  const QueuedFlit &queued = q.front();
  const NocFlit &flit = queued.flit;
  bool dateline_flip = dateline_flip_for_port(static_cast<NocPort>(out_idx));
  NocFlit out_flit = flit;
  bool buddy_flip = false;
  bool allow_buddy_flip = !flit.vc_static;
  if (allow_buddy_flip) {
    if (flit.vc_linked && flit.linked_id != 0) {
      auto it = linked_buddy_flip_.find(flit.linked_id);
      if (it != linked_buddy_flip_.end()) {
        buddy_flip = it->second;
      } else {
        buddy_flip = (sc_core::sc_time_stamp() - queued.enqueue_time) >=
                     (cycle_time * kBuddyFlipCycles);
        linked_buddy_flip_[flit.linked_id] = buddy_flip;
      }
    } else {
      auto it = packet_buddy_flip_.find(flit.packet_id);
      if (it != packet_buddy_flip_.end()) {
        buddy_flip = it->second;
      } else {
        buddy_flip = (sc_core::sc_time_stamp() - queued.enqueue_time) >=
                     (cycle_time * kBuddyFlipCycles);
        packet_buddy_flip_[flit.packet_id] = buddy_flip;
      }
    }
  }

  out_flit.vc_id = next_vc_id(flit, dateline_flip, buddy_flip);
  uint32_t vc_key = static_cast<uint32_t>(out_idx) * kVcs + (out_flit.vc_id % kVcs);
  if (out_flit.linked_id != 0) {
    auto it = reserved_vc_.find(vc_key);
    if (it != reserved_vc_.end() && it->second != out_flit.linked_id) {
        return false;
    }
  }
  if (out[out_idx].nb_write(out_flit)) {
    std::cout << sc_core::sc_time_stamp() << " router " << coord_to_string(coord)
              << " in " << port_name(static_cast<NocPort>(in_port)) << " -> out "
              << port_name(static_cast<NocPort>(out_idx)) << " pkt " << out_flit.packet_id
              << " flit " << out_flit.flit_index << "/" << out_flit.total_flits
              << " vc " << static_cast<int>(out_flit.vc_id) << std::endl;
      q.front().pending_mask &= ~(1u << out_idx);
      if (q.front().pending_mask == 0) {
        q.pop_front();
      }
      if (out_flit.linked_id != 0) {
        reserved_vc_[vc_key] = out_flit.linked_id;
        if (out_flit.is_tail() && !out_flit.vc_linked) {
          reserved_vc_.erase(vc_key);
        }
      }
      if (flit.is_tail()) {
        packet_buddy_flip_.erase(flit.packet_id);
        if (flit.vc_linked && flit.linked_id != 0) {
          linked_buddy_flip_.erase(flit.linked_id);
        }
      }
      rr_index_[out_idx] = (best_idx + 1) % (kNocPortCount * kVcs);
      return true;
    }
  return false;
}

bool NocRouter::try_send_on_port(size_t out_idx) {
  ensure_class_state(out_idx);
  size_t attempts = 0;
  while (attempts < kClasses) {
    uint8_t class_bits = static_cast<uint8_t>(class_cursor_[out_idx]);
    if (try_send_on_port_for_class(out_idx, class_bits)) {
      if (class_remaining_[out_idx] > 0) {
        --class_remaining_[out_idx];
      }
      if (class_remaining_[out_idx] == 0) {
        advance_class_cursor(out_idx);
      }
      return true;
    }
    advance_class_cursor(out_idx);
    ++attempts;
  }
  return false;
}

void NocRouter::route() {
  while (true) {
    bool progressed = false;

    for (size_t in_port = 0; in_port < kNocPortCount; ++in_port) {
      NocFlit flit;
      if (in[in_port].nb_read(flit)) {
        size_t vc = flit.vc_id % kVcs;
        uint32_t mask = compute_out_mask(flit);
        input_vcs_[in_port][vc].push_back(QueuedFlit{flit, sc_core::sc_time_stamp(), mask});
        progressed = true;
      }
    }

    for (size_t out_port = 0; out_port < kNocPortCount; ++out_port) {
      if (try_send_on_port(out_port)) {
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
