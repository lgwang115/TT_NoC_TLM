#ifndef NOC_TRANSLATION_H
#define NOC_TRANSLATION_H

#include <array>

#include "noc_types.h"

struct NocTranslationTable {
  std::array<uint8_t, 32> x = {};
  std::array<uint8_t, 32> y = {};
};

NocTranslationTable default_translation(NocId noc_id);

NocCoord translate_coord(const NocCoord &coord, const NocTranslationTable &table);

#endif
