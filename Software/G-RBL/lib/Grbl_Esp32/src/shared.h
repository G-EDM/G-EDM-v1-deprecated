#pragma once

#include <stdint.h>

enum class SDState : uint8_t {
    Idle          = 0,
    NotPresent    = 1,
    Busy          = 2,
    BusyPrinting  = 2,
    BusyUploading = 3,
    BusyParsing   = 4,
    BusyCustom    = 5,
};

const int PLAN_OK          = true;
const int PLAN_EMPTY_BLOCK = false;