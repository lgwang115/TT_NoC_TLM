# WormholeB0 NoC SystemC/TLM Model (step-by-step)

This folder contains a simple, transaction-level SystemC model for the WormholeB0 NoC based on the documentation in `WormholeB0/NoC`. The model focuses on:
- 2D torus topology
- Deterministic routing (NoC0: right then down, NoC1: up then left)
- Per-link throughput of 1 flit/cycle with configurable latency
- Cut-through style forwarding (flit-by-flit)
- Virtual channels with per-output arbitration (16 VCs, simple round-robin)
- VC id is 4 bits: dateline, class, buddy; dateline flips on torus wrap hops
- Ordering support: VC_STATIC disables buddy adaptation; VC_LINKED reuses the same initial VC id and preserves buddy flips per router for a linked group
- Coordinate translation (default table) can be enabled per packet before injection
- Broadcast modeled as source replication over unicasts
- NIU model with command interface and local memory (read/write/atomic add)
- NIU MMIO register model for request initiators (partial `MemoryMap.md`)
- NIU counters and full MMIO address map coverage with debug stubs
- Atomic opcode decoding for `NOC_AT_LEN_BE` (increment, CAS, swap mask/index)
- `NOC_PACKET_TAG` fields propagated; header store supported for posted writes
- `NOC_CMD_PATH_RESERVE` modeled as a broadcast launch delay; `ARB_PRIORITY` used in router arbitration
- TensixTile wrapper with RISCV B/NC cores driving two NIUs (NoC0/NoC1)

It intentionally simplifies or omits advanced features (broadcast trees, full ordering semantics, path reservation). Those are listed as extension steps below.

## Step-by-step structure

1) **Define packet/flit types**
   - See `src/noc_types.h` for `NocCoord`, `NocPacket`, and `NocFlit`.
   - `NocFlit` is fixed at 32 bytes (256 bits) to match the spec.

2) **Model links with latency + 1 flit/cycle throughput**
   - See `src/noc_link.*`.
   - Each link holds incoming flits and releases them after a delay.

3) **Build a router with deterministic routing and VCs**
   - See `src/noc_router.*`.
   - One flit per output per cycle, with per-VC input queues.
   - Routing logic matches `RoutingPaths.md` for unicast packets.

4) **Assemble a torus mesh**
   - See `src/noc_mesh.*`.
   - Links are wired according to NoC0 (east/south) or NoC1 (north/west).
   - Local injection/ejection links model NIU-to-router and router-to-NIU latency.

5) **Generate traffic and observe delivery**
   - See `src/traffic_gen.*` and `src/main.cpp`.
   - The traffic generator injects packets and logs when they are fully received.
   - Broadcast is implemented by source replication over unicasts.
   - The NIU demo uses `src/noc_niu.*` and `src/main.cpp`.

6) **Use NIU MMIO registers**
   - See `src/noc_niu.*` for the partial register file.
   - Implemented registers: `NOC_TARG_ADDR_{LO,MID}`, `NOC_RET_ADDR_{LO,MID}`, `NOC_PACKET_TAG`, `NOC_CTRL`, `NOC_AT_LEN_BE`, `NOC_AT_DATA`, `NOC_CMD_CTRL`, `NIU_CFG_0`, `ROUTER_CFG_{0..4}`, `NOC_ID_LOGICAL`, translation tables, `NIU_MST/SRV` counters, ID registers, status, and debug ranges (read as zero).

## Build and run

Prereq: a SystemC installation (tested with SystemC 2.3+).

```bash
mkdir -p NoC_TLM/build
cmake -S NoC_TLM -B NoC_TLM/build -DSYSTEMC_HOME=/path/to/systemc
cmake --build NoC_TLM/build
./NoC_TLM/build/noc_sim
```

If your SystemC install is configured with `SYSTEMC_HOME` and `SYSTEMC_INCLUDE`/`SYSTEMC_LIBDIR`, CMake should pick it up automatically.

## Configuration knobs

- Mesh size and NoC ID live in `NoC_TLM/src/main.cpp` (default 10x12, NoC0).
- Link latencies are set in `NoC_TLM/src/noc_mesh.cpp` (NIU 5 cycles, router 9 cycles).

## How to extend toward full Wormhole behavior

- Parameterize coordinate translation tables for actual fused-row layout.
- Implement broadcast routing (major/minor/major axis as described in `RoutingPaths.md`) instead of source replication.
- Add counters and timing stats aligned with `Counters.md` and `Performance` in `README.md`.

## Files

- `NoC_TLM/CMakeLists.txt`
- `NoC_TLM/src/noc_types.h`
- `NoC_TLM/src/noc_link.h`, `NoC_TLM/src/noc_link.cpp`
- `NoC_TLM/src/noc_router.h`, `NoC_TLM/src/noc_router.cpp`
- `NoC_TLM/src/noc_mesh.h`, `NoC_TLM/src/noc_mesh.cpp`
- `NoC_TLM/src/noc_niu.h`, `NoC_TLM/src/noc_niu.cpp`
- `NoC_TLM/src/traffic_gen.h`, `NoC_TLM/src/traffic_gen.cpp`
- `NoC_TLM/src/riscv_core.h`, `NoC_TLM/src/riscv_core.cpp`
- `NoC_TLM/src/tensix_tile.h`, `NoC_TLM/src/tensix_tile.cpp`
- `NoC_TLM/src/main.cpp`
