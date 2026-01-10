# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

WormholeB0 NoC SystemC/TLM Model - A transaction-level SystemC simulation of a Network-on-Chip for the TensixTile architecture. Models a 2D torus mesh topology with deterministic routing, virtual channels, and packet-based communication.

## Build Commands

```bash
# Build (requires SystemC 2.3+ installation)
mkdir -p build
cmake -S . -B build -DSYSTEMC_HOME=/path/to/systemc
cmake --build build

# Run simulation
./build/noc_sim
```

SystemC can be auto-detected if `SYSTEMC_HOME` environment variable is set.

## Architecture

**Simulation Flow:** RISC-V cores → NIU (packet assembly) → Router mesh → NIU (packet delivery) → Local memory

**Key Components:**

- **NocMesh** (`src/noc_mesh.*`): Assembles 2D torus topology, wires routers and links. Default 10x12 grid.
- **NocRouter** (`src/noc_router.*`): Deterministic XY routing with 16 virtual channels. NoC0 routes right-then-down, NoC1 routes up-then-left.
- **NocLink** (`src/noc_link.*`): Latency-modeled links (NIU: 5 cycles, router: 9 cycles).
- **NocNiu** (`src/noc_niu.*`): Network Interface Unit - largest component (~1055 LOC). Handles MMIO registers, local memory, packet assembly/disassembly, atomic operations.
- **NocOverlay** (`src/noc_overlay.*`): NoC Overlay coprocessor with 64 streams per Tensix tile. Assists with message-based data movement between tiles. Supports phase-based operation, receive buffer FIFOs, and message metadata tracking.
- **TensixTile** (`src/tensix_tile.*`): Tile wrapper containing 5 RISC-V cores (B, T0, T1, T2, NC) and one NoC Overlay per spec. T-cores disabled by default (they drive Tensix coprocessor modeled elsewhere).
- **RiscvCore** (`src/riscv_core.*`): Simple traffic generator that issues MMIO operations to NIUs.

**Data Types** (`src/noc_types.h`):
- `NocCoord`: Grid coordinates
- `NocPacket`: Full packet with header fields (destination, VC mode, request type)
- `NocFlit`: 32-byte (256-bit) transmission unit

**Virtual Channel Encoding (4-bit VC ID):**
- Bit 3: Dateline (flips on torus wrap)
- Bits 2-1: Traffic class
- Bit 0: Buddy bit (adaptive routing)

## Configuration

- **Mesh dimensions**: `src/main.cpp` (ROWS, COLS constants)
- **NoC ID selection**: `src/main.cpp` (NOC0 or NOC1)
- **Link latencies**: `src/noc_mesh.cpp` (NIU_DELAY, ROUTER_DELAY)
- **Simulation duration**: `src/main.cpp` (sc_start call)

## Key Implementation Details

- Broadcast uses router-level multicast routing (not source replication or routing trees)
- Atomic operations supported: increment, CAS, swap mask/index
- VC modes: STATIC (strict ordering) and LINKED (transaction ordering across packets)
- Coordinate translation tables available for fused-row layout
- Inter-module communication via SystemC `sc_fifo`

## TODO: Spec Misalignments to Address

### Lower Priority

- **Broadcast Routing Trees**: Spec describes major/minor axis routing trees for broadcasts. Current implementation uses router-level multicast (functionally similar but different mechanism). See spec `NoC/RoutingPaths.md`.

- **Path Reservation**: Currently modeled as simple broadcast launch delay (`wait(cycle_time * 10)`). Spec defines more complex path reservation mechanism.

- **Full Ordering Semantics**: Currently simplified. Would need implementation of complete VC ordering rules per spec.

- **Mover (XMOV) Model**: L1-to-L1, L1-to-L0, L0-to-L1 data movement accelerator. Part of Tensix coprocessor (modeled elsewhere).
