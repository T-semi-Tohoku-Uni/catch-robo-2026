#pragma once

#include <cstdint>
#include <endian.h>

// MCU output numbers, independent of the configurable L/C/R mapping.
// Bit-field layout follows the target ABI; union access uses the GCC extension.
union PumpCommand {
    struct {
#if __BYTE_ORDER == __LITTLE_ENDIAN
        uint32_t pump1 : 1;
        uint32_t pump2 : 1;
        uint32_t pump3 : 1;
        uint32_t valve1 : 1;
        uint32_t valve2 : 1;
        uint32_t valve3 : 1;
        uint32_t reserved : 26;
#elif __BYTE_ORDER == __BIG_ENDIAN
        uint32_t reserved : 26;
        uint32_t valve3 : 1;
        uint32_t valve2 : 1;
        uint32_t valve1 : 1;
        uint32_t pump3 : 1;
        uint32_t pump2 : 1;
        uint32_t pump1 : 1;
#else
#error "Unsupported byte order"
#endif
    } bits;
    uint32_t raw;
};

static_assert(sizeof(PumpCommand) == sizeof(uint32_t), "Pump command must be 32 bits");
