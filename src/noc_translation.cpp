#include "noc_translation.h"

NocTranslationTable default_translation(NocId noc_id) {
  NocTranslationTable table;
  for (size_t i = 0; i < 32; ++i) {
    table.x[i] = 0;
    table.y[i] = 0;
  }

  for (size_t i = 0; i <= 15; ++i) {
    table.x[i] = static_cast<uint8_t>(i);
    table.y[i] = static_cast<uint8_t>(i);
  }

  if (noc_id == NocId::NOC0) {
    table.x[16] = 0;
    table.x[17] = 5;
    table.x[18] = 1;
    table.x[19] = 2;
    table.x[20] = 3;
    table.x[21] = 4;
    table.x[22] = 6;
    table.x[23] = 7;
    table.x[24] = 8;
    table.x[25] = 9;

    table.y[16] = 0;
    table.y[17] = 6;
    table.y[18] = 1;
    table.y[19] = 2;
    table.y[20] = 3;
    table.y[21] = 4;
    table.y[22] = 5;
    table.y[23] = 8;
    table.y[24] = 9;
    table.y[25] = 11;
    table.y[26] = 7;
    table.y[27] = 10;
  } else {
    table.x[16] = 9;
    table.x[17] = 4;
    table.x[18] = 8;
    table.x[19] = 7;
    table.x[20] = 6;
    table.x[21] = 5;
    table.x[22] = 3;
    table.x[23] = 2;
    table.x[24] = 1;
    table.x[25] = 0;

    table.y[16] = 11;
    table.y[17] = 5;
    table.y[18] = 10;
    table.y[19] = 9;
    table.y[20] = 8;
    table.y[21] = 7;
    table.y[22] = 6;
    table.y[23] = 3;
    table.y[24] = 2;
    table.y[25] = 0;
    table.y[26] = 4;
    table.y[27] = 1;
  }

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
