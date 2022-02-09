#include "tms9918.h"

extern "C" {

uint8_t TMS9918AComputeRow(const uint8_t* registers, const uint8_t* memory, int row, uint8_t* row_colors)
{
    using namespace TMS9918A;

    uint8_t flags_set = 0;

    if(ActiveDisplayAreaIsBlanked(registers)) {

        uint8_t color_index = GetBackdropColor(registers);
        for(int col = 0; col < SCREEN_X; col++) {
            row_colors[col] = color_index;
        }

    } else {

        FillRowFromPattern(row, row_colors, registers, memory);

        if(SpritesVisible(registers)) {
            flags_set |= AddSpritesToRowReturnFlags(row, row_colors, registers, memory);
        }
    }

    return flags_set;
}

};
