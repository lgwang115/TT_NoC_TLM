#include "noc_translation.h"

// ============================================================================
// Coordinate Translation Tables per WormholeB0 Spec
// ============================================================================
//
// The 10x12 NoC grid contains different tile types:
//   - Row 0, 11: Ethernet tiles
//   - Column 0: PCIe/ARC tiles
//   - Column 5: DRAM controllers
//   - Remaining: Tensix compute tiles (8 columns x 10 rows = 80 tiles)
//
// Translation table structure (32 entries x 4 bits each):
//   - Entries 0-15:  Identity mapping (raw physical coordinates)
//   - Entries 16-25: Logical tile addressing
//       X: 16=PCIe, 17=DRAM, 18-25=Tensix columns
//       Y: 16=Eth0, 17-24=Tensix rows, 25=Eth1
//   - Entries 26-27: Harvested (fused-off) row remapping
//   - Entries 28-31: Reserved (map to 0)
//
// NOC0: Origin at top-left (0,0), flows right (East) then down (South)
// NOC1: Origin at bottom-right (9,11), flows left (West) then up (North)
//       Coordinates are effectively "mirrored" compared to NOC0

NocTranslationTable default_translation(NocId noc_id) {
  NocTranslationTable table;

  // Initialize all entries to 0 (reserved entries stay at 0)
  for (size_t i = 0; i < 32; ++i) {
    table.x[i] = 0;
    table.y[i] = 0;
  }

  // Entries 0-15: Identity mapping for raw physical coordinates
  // Allows direct access when translation is enabled
  for (size_t i = 0; i <= 15; ++i) {
    table.x[i] = static_cast<uint8_t>(i);
    table.y[i] = static_cast<uint8_t>(i);
  }

  if (noc_id == NocId::NOC0) {
    // NOC0: Origin at top-left, flows right-then-down

    // X translation (columns):
    //   16 = PCIe/ARC column (physical column 0)
    //   17 = DRAM column (physical column 5)
    //   18-25 = Tensix columns (physical columns 1,2,3,4,6,7,8,9)
    table.x[16] = 0;   // PCIe/ARC
    table.x[17] = 5;   // DRAM
    table.x[18] = 1;   // Tensix col 0
    table.x[19] = 2;   // Tensix col 1
    table.x[20] = 3;   // Tensix col 2
    table.x[21] = 4;   // Tensix col 3
    table.x[22] = 6;   // Tensix col 4
    table.x[23] = 7;   // Tensix col 5
    table.x[24] = 8;   // Tensix col 6
    table.x[25] = 9;   // Tensix col 7

    // Y translation (rows):
    //   16 = Ethernet row 0 (physical row 0)
    //   17-24 = Tensix rows (physical rows 1-10, skipping harvested)
    //   25 = Ethernet row 1 (physical row 11)
    //   26-27 = Harvested row remapping
    table.y[16] = 0;   // Ethernet row 0
    table.y[17] = 1;   // Tensix row 0
    table.y[18] = 2;   // Tensix row 1
    table.y[19] = 3;   // Tensix row 2
    table.y[20] = 4;   // Tensix row 3
    table.y[21] = 5;   // Tensix row 4
    table.y[22] = 6;   // Tensix row 5
    table.y[23] = 7;   // Tensix row 6
    table.y[24] = 8;   // Tensix row 7
    table.y[25] = 11;  // Ethernet row 1

    // Harvested row remapping (example: rows 9,10 could be fused off)
    // These map logical "harvested" indices to actual working rows
    // Default: map to rows 9 and 10 (non-harvested chip)
    table.y[26] = 9;   // Tensix row 8 (or harvested remap)
    table.y[27] = 10;  // Tensix row 9 (or harvested remap)

  } else {
    // NOC1: Origin at bottom-right, flows left-then-up
    // Coordinates are mirrored relative to NOC0

    // X translation (columns) - reversed order for NOC1
    table.x[16] = 9;   // PCIe/ARC (from NOC1 perspective)
    table.x[17] = 4;   // DRAM (from NOC1 perspective: 9-5=4)
    table.x[18] = 8;   // Tensix col 0
    table.x[19] = 7;   // Tensix col 1
    table.x[20] = 6;   // Tensix col 2
    table.x[21] = 5;   // Tensix col 3 (note: this is DRAM col in NOC0)
    table.x[22] = 3;   // Tensix col 4
    table.x[23] = 2;   // Tensix col 5
    table.x[24] = 1;   // Tensix col 6
    table.x[25] = 0;   // Tensix col 7

    // Y translation (rows) - reversed order for NOC1
    table.y[16] = 11;  // Ethernet row 0 (from NOC1 perspective)
    table.y[17] = 10;  // Tensix row 0
    table.y[18] = 9;   // Tensix row 1
    table.y[19] = 8;   // Tensix row 2
    table.y[20] = 7;   // Tensix row 3
    table.y[21] = 6;   // Tensix row 4
    table.y[22] = 5;   // Tensix row 5
    table.y[23] = 4;   // Tensix row 6
    table.y[24] = 3;   // Tensix row 7
    table.y[25] = 0;   // Ethernet row 1

    // Harvested row remapping for NOC1
    table.y[26] = 2;   // Tensix row 8 (or harvested remap)
    table.y[27] = 1;   // Tensix row 9 (or harvested remap)
  }

  // Entries 28-31 remain at 0 (reserved)

  return table;
}

NocCoord translate_coord(const NocCoord &coord, const NocTranslationTable &table) {
  NocCoord out = coord;
  if (coord.x < 32) {
    out.x = table.x[coord.x];
  }
  if (coord.y < 32) {
    out.y = table.y[coord.y];
  }
  return out;
}

// Helper function to create translation table for harvested chip
// harvest_mask: bitmask of fused-off rows (e.g., 0x300 = rows 8,9 harvested)
NocTranslationTable harvested_translation(NocId noc_id, uint32_t harvest_mask) {
  NocTranslationTable table = default_translation(noc_id);

  // Count harvested rows and remap remaining Tensix rows
  // This is a simplified implementation - real hardware would have
  // more complex remapping based on which specific rows are fused off

  if (harvest_mask == 0) {
    return table;  // No harvesting, use default
  }

  // Build list of working Tensix rows (rows 1-10, excluding harvested)
  uint8_t working_rows[10];
  size_t num_working = 0;

  for (unsigned row = 1; row <= 10; ++row) {
    if (!(harvest_mask & (1u << row))) {
      working_rows[num_working++] = static_cast<uint8_t>(row);
    }
  }

  // Remap Y translation entries 17-26 to working rows
  if (noc_id == NocId::NOC0) {
    for (size_t i = 0; i < num_working && i < 10; ++i) {
      table.y[17 + i] = working_rows[i];
    }
  } else {
    // NOC1: reversed order
    for (size_t i = 0; i < num_working && i < 10; ++i) {
      table.y[17 + i] = working_rows[num_working - 1 - i];
    }
  }

  return table;
}
