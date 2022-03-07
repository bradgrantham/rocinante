#ifndef _TMS9918_H_
#define _TMS9918_H_

#include <cstdint>
#include <cstdio>
#include <array>

namespace TMS9918A
{

constexpr int SCREEN_X = 256;
constexpr int SCREEN_Y = 192;

constexpr int REG_A0_A5_MASK = 0x3F;
constexpr int CMD_MASK = 0xC0;
constexpr int CMD_SET_REGISTER = 0x80;
constexpr int CMD_SET_WRITE_ADDRESS = 0x40;
constexpr int CMD_SET_READ_ADDRESS = 0x00;

constexpr int VR0_M3_MASK = 0x02;
[[maybe_unused]] constexpr int VR0_EXTVID_MASK = 0x01;

[[maybe_unused]] constexpr int VR1_16K_MASK = 0x80; 
constexpr int VR1_BLANK_MASK = 0x40; /* and BLANK is active low */
constexpr int VR1_INT_MASK = 0x20;
constexpr int VR1_M2_MASK = 0x10;
constexpr int VR1_M1_MASK = 0x08;
constexpr int VR1_SIZE4_MASK = 0x02;
constexpr int VR1_MAG2X_MASK = 0x01;

constexpr int VR2_NAME_TABLE_MASK = 0x0F;
constexpr int VR2_NAME_TABLE_SHIFT = 10;

constexpr int VR3_COLORTABLE_MASK_STANDARD = 0xFF;
constexpr int VR3_COLORTABLE_SHIFT_STANDARD = 6;

constexpr int VR3_COLORTABLE_MASK_BITMAP = 0x80;
constexpr int VR3_COLORTABLE_SHIFT_BITMAP = 6;

constexpr int VR3_ADDRESS_MASK_BITMAP = 0x7F;
constexpr int VR3_ADDRESS_MASK_SHIFT = 6;

constexpr int VR4_PATTERN_MASK_STANDARD = 0x07;
constexpr int VR4_PATTERN_SHIFT_STANDARD = 11;

constexpr int VR4_PATTERN_MASK_BITMAP = 0x04;
constexpr int VR4_PATTERN_SHIFT_BITMAP = 11;

constexpr int VR5_SPRITE_ATTR_MASK = 0x7F;
constexpr int VR5_SPRITE_ATTR_SHIFT = 7;

constexpr int VR6_SPRITE_PATTERN_MASK = 0x07;
constexpr int VR6_SPRITE_PATTERN_SHIFT = 11;

constexpr int VR7_BD_MASK = 0x0F;
constexpr int VR7_BD_SHIFT = 0;

constexpr int VDP_STATUS_F_BIT = 0x80;
constexpr int VDP_STATUS_5S_BIT = 0x40;
constexpr int VDP_STATUS_C_BIT = 0x20;

constexpr int ROW_SHIFT = 5;
constexpr int THIRD_SHIFT = 11;
constexpr int CHARACTER_PATTERN_SHIFT = 3;
constexpr int CHARACTER_COLOR_SHIFT = 3;
constexpr int ADDRESS_MASK_FILL = 0x3F;

constexpr int SPRITE_EARLY_CLOCK_MASK = 0x80;
constexpr int SPRITE_COLOR_MASK = 0x0F;
constexpr int SPRITE_NAME_SHIFT = 3;
constexpr int SPRITE_NAME_MASK_SIZE4 = 0xFC;

constexpr int TRANSPARENT_COLOR_INDEX = 0;

constexpr int REGISTER_COUNT = 8;

enum GraphicsMode { GRAPHICS_I, GRAPHICS_II, TEXT, MULTICOLOR, UNDEFINED };

inline bool SpritesAreSize4(const uint8_t* registers)
{
    return registers[1] & VR1_SIZE4_MASK;
}

inline bool SpritesAreMagnified2X(const uint8_t* registers)
{
    return registers[1] & VR1_MAG2X_MASK;
}

inline bool ActiveDisplayAreaIsBlanked(const uint8_t* registers)
{
    return (registers[1] & VR1_BLANK_MASK) == 0;
}

inline uint8_t GetBackdropColor(const uint8_t* registers)
{
    return (registers[7] & VR7_BD_MASK) >> VR7_BD_SHIFT;
}

inline bool InterruptsAreEnabled(const uint8_t* registers)
{
    return registers[1] & VR1_INT_MASK;
}

inline bool VSyncInterruptHasOccurred(uint8_t status_register)
{
    return status_register & VDP_STATUS_F_BIT;
}

inline GraphicsMode GetGraphicsMode(const uint8_t* registers)
{
    bool M1 = registers[1] & VR1_M1_MASK;
    bool M2 = registers[1] & VR1_M2_MASK;
    bool M3 = registers[0] & VR0_M3_MASK;

    if(!M1 && !M2 && !M3) {
        return GraphicsMode::GRAPHICS_I;
    } else if(!M1 && !M2 && M3) {
        return GraphicsMode::GRAPHICS_II;
    } else if(!M1 && M2 && !M3) {
        return GraphicsMode::MULTICOLOR;
    } else if(M1 && !M2 && !M3) {
        return GraphicsMode::TEXT;
    }
    return GraphicsMode::UNDEFINED;
}

inline bool SpritesVisible(const uint8_t* registers)
{
    if(ActiveDisplayAreaIsBlanked(registers)) {
        return false;
    }

    switch(GetGraphicsMode(registers)) {
        case GRAPHICS_I:
            return true;
            break;
        case GRAPHICS_II:
            return true;
            break;
        case TEXT:
            return false;
            break;
        case MULTICOLOR:
            return true;
            break;
        case UNDEFINED:
            return true;
            break;
    }
    return false;
}

inline uint16_t GetNameTableBase(const uint8_t* registers)
{
    return (registers[2] & VR2_NAME_TABLE_MASK) << VR2_NAME_TABLE_SHIFT;
}

inline uint16_t GetSpriteAttributeTableBase(const uint8_t* registers)
{
    return (registers[5] & VR5_SPRITE_ATTR_MASK) << VR5_SPRITE_ATTR_SHIFT;
}

inline uint16_t GetSpritePatternTableBase(const uint8_t* registers)
{
    return (registers[6] & VR6_SPRITE_PATTERN_MASK) << VR6_SPRITE_PATTERN_SHIFT;
}

inline void CopyColor(uint8_t* dst, uint8_t* src)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

inline void SetColor(uint8_t *color, uint8_t r, uint8_t g, uint8_t b)
{
    color[0] = r;
    color[1] = g;
    color[2] = b;
}

static uint8_t Colors[16][3] = {
    {0, 0, 0}, /* if BACKDROP is 0, supply black */
    {0, 0, 0},
    {62, 184, 73},
    {116, 208, 125},
    {89, 85, 224},
    {128, 118, 241},
    {185, 94, 81},
    {100, 220, 239},
    {219, 101, 89},
    {255, 137, 125},
    {204, 195, 94},
    {222, 208, 135},
    {58, 162, 65},
    {183, 102, 181},
    {204, 204, 204},
    {255, 255, 255},
};

static void FillRowFromGraphicsI(int y, uint8_t row_colors[TMS9918A::SCREEN_X], const uint8_t* registers, const uint8_t* memory)
{
    using namespace TMS9918A;

    uint16_t row = y / 8;
    uint16_t pattern_row = y % 8;

    uint16_t name_row_base = GetNameTableBase(registers) | (row << ROW_SHIFT);
    uint16_t pattern_row_base = (registers[4] & VR4_PATTERN_MASK_STANDARD) << VR4_PATTERN_SHIFT_STANDARD | pattern_row;
    uint16_t color_base = (registers[3] & VR3_COLORTABLE_MASK_STANDARD) << VR3_COLORTABLE_SHIFT_STANDARD;

    uint8_t *rowp = row_colors;
    uint8_t backdrop = GetBackdropColor(registers);
    for(int x = 0; x < SCREEN_X; x += 8) {
        uint16_t col = x / 8;

        uint16_t name_table_address = name_row_base | col;
        uint8_t pattern_name = memory[name_table_address];
        uint16_t pattern_address = pattern_row_base | (pattern_name << CHARACTER_PATTERN_SHIFT);
        uint16_t color_address = color_base | (pattern_name >> CHARACTER_COLOR_SHIFT);

        uint8_t pattern_byte = memory[pattern_address];
        uint8_t colortable = memory[color_address];
        uint8_t color0 = colortable & 0xf;
        uint8_t color1 = (colortable >> 4) & 0xf;

        if(color0 == TRANSPARENT_COLOR_INDEX) {
            color0 = backdrop;
        }
        if(color1 == TRANSPARENT_COLOR_INDEX) {
            color1 = backdrop;
        }

        for(int pattern_col = 0; pattern_col < 8; pattern_col++) {
            bool bit = pattern_byte & (0x80 >> pattern_col);
            *rowp++ = bit ? color1 : color0;
        }
    }
}

static void FillRowFromGraphicsII(int y, uint8_t row_colors[TMS9918A::SCREEN_X], const uint8_t* registers, const uint8_t* memory)
{
    using namespace TMS9918A;

    uint16_t row = y / 8;
    uint16_t pattern_row = y % 8;
    int third = (row / 8) << THIRD_SHIFT;

    uint16_t name_table_row_base = GetNameTableBase(registers) | (row << ROW_SHIFT);
    uint16_t address_mask = ((registers[3] & VR3_ADDRESS_MASK_BITMAP) << VR3_ADDRESS_MASK_SHIFT) | ADDRESS_MASK_FILL;
    uint16_t pattern_address_row_base = ((registers[4] & VR4_PATTERN_MASK_BITMAP) << VR4_PATTERN_SHIFT_BITMAP) | pattern_row | (third & address_mask);
    uint16_t color_address_row_base = ((registers[3] & VR3_COLORTABLE_MASK_BITMAP) << VR3_COLORTABLE_SHIFT_BITMAP) | pattern_row | (third & address_mask);

    uint8_t *rowp = row_colors;
    uint8_t backdrop = GetBackdropColor(registers);
    for(int x = 0; x < SCREEN_X; x += 8) {
        uint16_t col = x / 8;

        uint16_t name_table_address = name_table_row_base | col;
        uint16_t pattern_name = memory[name_table_address];

        uint16_t pattern_address = pattern_address_row_base | ((pattern_name << CHARACTER_PATTERN_SHIFT) & address_mask);
        uint16_t color_address = color_address_row_base | ((pattern_name << CHARACTER_PATTERN_SHIFT) & address_mask);
        uint8_t pattern_byte = memory[pattern_address];
        uint8_t colortable = memory[color_address];
        uint8_t color0 = colortable & 0xf;
        uint8_t color1 = (colortable >> 4) & 0xf;

        if(color0 == TRANSPARENT_COLOR_INDEX) {
            color0 = backdrop;
        }
        if(color1 == TRANSPARENT_COLOR_INDEX) {
            color1 = backdrop;
        }

        for(int pattern_col = 0; pattern_col < 8; pattern_col++) {
            bool bit = pattern_byte & (0x80 >> pattern_col);
            *rowp++ = bit ? color1 : color0;
        }
    }
}

static void FillRowFromPattern(int y, uint8_t row_colors[TMS9918A::SCREEN_X], const uint8_t* registers, const uint8_t* memory)
{
    using namespace TMS9918A;

    GraphicsMode mode = GetGraphicsMode(registers);

    if(mode == GraphicsMode::GRAPHICS_I) {

        FillRowFromGraphicsI(y, row_colors, registers, memory);

    } else if(mode == GraphicsMode::GRAPHICS_II) {

        FillRowFromGraphicsII(y, row_colors, registers, memory);

    } else {

        bool M1 = registers[1] & VR1_M1_MASK;
        bool M2 = registers[1] & VR1_M2_MASK;
        bool M3 = registers[0] & VR0_M3_MASK;
        printf("unhandled video mode M1 = %d M2 = %d M3 = %d\n", M1, M2, M3);

        for(int x = 0; x < SCREEN_X; x++) {
            row_colors[x] = 8; // RED
        }
        // abort();
    }
}

static uint8_t AddSpritesToRowReturnFlags(int row, uint8_t row_colors[TMS9918A::SCREEN_X], const uint8_t* registers, const uint8_t* memory)
{
    using namespace TMS9918A;

    bool sprite_touched[SCREEN_X]{};

    uint8_t flags_set = 0;

    // XXX do per row here because will do this per row on Rosa
    int sprite_table_address = GetSpriteAttributeTableBase(registers);
    bool mag2x = SpritesAreMagnified2X(registers);
    bool size4 = SpritesAreSize4(registers);
    int sprite_count = 32;
    for(int i = 0; i < 32; i++) {
        auto sprite = memory + sprite_table_address + i * 4;
        if(sprite[0] == 0xD0) {
            sprite_count = i;
            break;
        }
    }

    int size_pixels = 8;
    if(mag2x) {
        size_pixels *= 2;
    }

    if(size4) {
        size_pixels *= 2;
    }

    int sprites_in_row = 0;
    for(int i = sprite_count - 1; i >= 0; i--) {
        auto sprite = memory + sprite_table_address + i * 4;

        int sprite_y = sprite[0] + 1;
        int sprite_x = sprite[1];
        int sprite_name = sprite[2];
        bool sprite_earlyclock = sprite[3] & SPRITE_EARLY_CLOCK_MASK;
        int sprite_color = sprite[3] & SPRITE_COLOR_MASK;

        int masked_sprite = sprite_name & SPRITE_NAME_MASK_SIZE4;

        // printf("sprite %d: %d %d %d %d\n", i, sprite_x, sprite_y, sprite_name, sprite_color);

        if(sprite_earlyclock) {
            sprite_x -= 32;
        }

        int start_x = std::max(0, sprite_x);
        int start_y = std::max(0, sprite_y);
        int end_x = std::min(sprite_x + size_pixels, SCREEN_X) - 1;
        int end_y = std::min(sprite_y + size_pixels, SCREEN_Y) - 1;

        if(start_y <= row && row <= end_y) {

            int within_sprite_y = mag2x ? ((row - sprite_y) / 2) : (row - sprite_y);

            sprites_in_row ++;
            if(sprites_in_row > 5) {
                flags_set |= VDP_STATUS_5S_BIT;
                break; // XXX
            }

            if(size4) {
                int within_quadrant_y = within_sprite_y % 8;
                int quadrant_y = within_sprite_y / 8;

                for(int x = start_x; x <= end_x; x++) {

                    int within_sprite_x = mag2x ? ((x - sprite_x) / 2) : (x - sprite_x);

                    int bit = 0;

                    int quadrant = quadrant_y + (within_sprite_x / 8) * 2;
                    int within_quadrant_x = within_sprite_x % 8;
                    int sprite_pattern_address = GetSpritePatternTableBase(registers) | (masked_sprite << SPRITE_NAME_SHIFT) | (quadrant << 3) | within_quadrant_y;
                    bit = memory[sprite_pattern_address] & (0x80 >> within_quadrant_x);

                    if(bit) {
                        if(sprite_touched[x]) {
                            flags_set |= VDP_STATUS_C_BIT;
                        }
                        sprite_touched[x] = true;
                        // XXX I don't think this next bit is necessarily
                        // right - I think a transparent sprite pixel may 
                        // make the sprite pixels under it invisible
                        if(sprite_color != TRANSPARENT_COLOR_INDEX) {
                            row_colors[x] = sprite_color;
                        }
                    }
                }

            } else {

                int sprite_pattern_address = GetSpritePatternTableBase(registers) | (sprite_name << SPRITE_NAME_SHIFT) | within_sprite_y;
                int bitpattern = memory[sprite_pattern_address];

                for(int x = start_x; x <= end_x; x++) {

                    int within_sprite_x = mag2x ? ((x - sprite_x) / 2) : (x - sprite_x);

                    int bit = 0;

                    bit = bitpattern & (0x80 >> within_sprite_x);

                    if(bit) {
                        if(sprite_touched[x]) {
                            flags_set |= VDP_STATUS_C_BIT;
                        }
                        sprite_touched[x] = true;
                        // XXX I don't think this next bit is necessarily
                        // right - I think a transparent sprite pixel may 
                        // make the sprite pixels under it invisible
                        if(sprite_color != TRANSPARENT_COLOR_INDEX) {
                            row_colors[x] = sprite_color;
                        }
                    }
                }
            }

        }
    }
    return flags_set;
}

template <typename SetPixelFunc>
static uint8_t CreateImageAndReturnFlags(const uint8_t* registers, const uint8_t* memory, SetPixelFunc SetPixel)
{
    using namespace TMS9918A;

    uint8_t flags_set = 0;

    if(ActiveDisplayAreaIsBlanked(registers)) {
        uint8_t color_index = GetBackdropColor(registers);
        uint8_t rgb[3];
        CopyColor(rgb, Colors[color_index]);
        for(int row = 0; row < SCREEN_Y; row++) {
            for(int col = 0; col < SCREEN_X; col++) {
                SetPixel(col, row, rgb[0], rgb[1], rgb[2]);
            }
        }
        return flags_set;
    }

    static uint8_t row_colors[SCREEN_X];

    for(int row = 0; row < SCREEN_Y; row++) {

        FillRowFromPattern(row, row_colors, registers, memory);

        if(SpritesVisible(registers)) {
            flags_set |= AddSpritesToRowReturnFlags(row, row_colors, registers, memory);
        }

        for(int col = 0; col < SCREEN_X; col++) {
            uint8_t rgb[3];
            CopyColor(rgb, Colors[row_colors[col]]);
            SetPixel(col, row, rgb[0], rgb[1], rgb[2]);
        }
    }

    return flags_set;
}

};

#endif /* _TMS9918_H_ */
