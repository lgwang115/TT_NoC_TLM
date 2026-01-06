#include "noc_mesh.h"

#include <string>

NocMesh::NocMesh(sc_core::sc_module_name name,
                 unsigned width,
                 unsigned height,
                 NocId noc_id,
                 sc_core::sc_time cycle_time)
    : sc_module(name),
      width_(width),
      height_(height),
      noc_id_(noc_id),
      cycle_time_(cycle_time) {
  routers_.reserve(width_ * height_);
  channels_.reserve(width_ * height_);
  local_in_.reserve(width_ * height_);
  local_out_.reserve(width_ * height_);

  for (unsigned y = 0; y < height_; ++y) {
    for (unsigned x = 0; x < width_; ++x) {
      auto channels = std::make_unique<TileChannels>();
      for (size_t p = 0; p < kNocPortCount; ++p) {
        channels->in[p] = std::make_unique<sc_core::sc_fifo<NocFlit>>(8);
        channels->out[p] = std::make_unique<sc_core::sc_fifo<NocFlit>>(8);
      }

      auto router_name = std::string("router_") + std::to_string(x) + "_" + std::to_string(y);
      auto router = std::make_unique<NocRouter>(
          router_name.c_str(), NocCoord{x, y}, noc_id_, width_, height_, cycle_time_);

      for (size_t p = 0; p < kNocPortCount; ++p) {
        router->in[p].bind(*channels->in[p]);
        router->out[p].bind(*channels->out[p]);
      }

      routers_.push_back(std::move(router));
      channels_.push_back(std::move(channels));

      local_in_.push_back(std::make_unique<sc_core::sc_fifo<NocFlit>>(8));
      local_out_.push_back(std::make_unique<sc_core::sc_fifo<NocFlit>>(8));
    }
  }

  build_links();
}

sc_core::sc_fifo<NocFlit> &NocMesh::local_in(unsigned x, unsigned y) {
  return *local_in_.at(index(x, y));
}

sc_core::sc_fifo<NocFlit> &NocMesh::local_out(unsigned x, unsigned y) {
  return *local_out_.at(index(x, y));
}

void NocMesh::build_links() {
  sc_core::sc_time niu_latency = 5 * cycle_time_;
  sc_core::sc_time router_latency = 9 * cycle_time_;

  for (unsigned y = 0; y < height_; ++y) {
    for (unsigned x = 0; x < width_; ++x) {
      size_t idx = index(x, y);

      auto in_link_name = std::string("niu_in_link_") + std::to_string(x) + "_" + std::to_string(y);
      auto out_link_name = std::string("niu_out_link_") + std::to_string(x) + "_" + std::to_string(y);

      auto niu_in_link = std::make_unique<NocLink>(in_link_name.c_str(), niu_latency, cycle_time_);
      auto niu_out_link = std::make_unique<NocLink>(out_link_name.c_str(), niu_latency, cycle_time_);

      niu_in_link->in.bind(*local_in_[idx]);
      niu_in_link->out.bind(*channels_[idx]->in[static_cast<size_t>(NocPort::Local)]);

      niu_out_link->in.bind(*channels_[idx]->out[static_cast<size_t>(NocPort::Local)]);
      niu_out_link->out.bind(*local_out_[idx]);

      links_.push_back(std::move(niu_in_link));
      links_.push_back(std::move(niu_out_link));
    }
  }

  for (unsigned y = 0; y < height_; ++y) {
    for (unsigned x = 0; x < width_; ++x) {
      size_t idx = index(x, y);
      if (noc_id_ == NocId::NOC0) {
        unsigned east_x = (x + 1) % width_;
        unsigned south_y = (y + 1) % height_;
        size_t east_idx = index(east_x, y);
        size_t south_idx = index(x, south_y);

        auto east_name = std::string("link_east_") + std::to_string(x) + "_" + std::to_string(y);
        auto south_name = std::string("link_south_") + std::to_string(x) + "_" + std::to_string(y);

        auto east_link = std::make_unique<NocLink>(east_name.c_str(), router_latency, cycle_time_);
        auto south_link = std::make_unique<NocLink>(south_name.c_str(), router_latency, cycle_time_);

        east_link->in.bind(*channels_[idx]->out[static_cast<size_t>(NocPort::East)]);
        east_link->out.bind(*channels_[east_idx]->in[static_cast<size_t>(NocPort::West)]);

        south_link->in.bind(*channels_[idx]->out[static_cast<size_t>(NocPort::South)]);
        south_link->out.bind(*channels_[south_idx]->in[static_cast<size_t>(NocPort::North)]);

        links_.push_back(std::move(east_link));
        links_.push_back(std::move(south_link));
      } else {
        unsigned west_x = (x + width_ - 1) % width_;
        unsigned north_y = (y + height_ - 1) % height_;
        size_t west_idx = index(west_x, y);
        size_t north_idx = index(x, north_y);

        auto west_name = std::string("link_west_") + std::to_string(x) + "_" + std::to_string(y);
        auto north_name = std::string("link_north_") + std::to_string(x) + "_" + std::to_string(y);

        auto west_link = std::make_unique<NocLink>(west_name.c_str(), router_latency, cycle_time_);
        auto north_link = std::make_unique<NocLink>(north_name.c_str(), router_latency, cycle_time_);

        west_link->in.bind(*channels_[idx]->out[static_cast<size_t>(NocPort::West)]);
        west_link->out.bind(*channels_[west_idx]->in[static_cast<size_t>(NocPort::East)]);

        north_link->in.bind(*channels_[idx]->out[static_cast<size_t>(NocPort::North)]);
        north_link->out.bind(*channels_[north_idx]->in[static_cast<size_t>(NocPort::South)]);

        links_.push_back(std::move(west_link));
        links_.push_back(std::move(north_link));
      }
    }
  }
}
