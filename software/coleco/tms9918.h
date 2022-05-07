#ifndef _TMS9918_H_
#define _TMS9918_H_

#include <cstdint>
#include <cstdio>
#include <array>

// On desktop: call CreateImageAndReturnFlags as done previously to an RGB8 image
// On Rosa: call CreateImageAndReturnFlags to 4BitPixmap on scanout?
//     Maybe later optimization to keep track of sprites startrow-stoprow
// AddSpritesToRow should operate on a 4BitPixmap

namespace TMS9918A
{

static constexpr int SCREEN_X = 256;
static constexpr int SCREEN_Y = 192;

static constexpr int REG_A0_A5_MASK = 0x3F;
static constexpr int CMD_MASK = 0xC0;
static constexpr int CMD_SET_REGISTER = 0x80;
static constexpr int CMD_SET_WRITE_ADDRESS = 0x40;
static constexpr int CMD_SET_READ_ADDRESS = 0x00;

static constexpr int VR0_M3_MASK = 0x02;
[[maybe_unused]] static constexpr int VR0_EXTVID_MASK = 0x01;

[[maybe_unused]] static constexpr int VR1_16K_MASK = 0x80; 
static constexpr int VR1_BLANK_MASK = 0x40; /* and BLANK is active low */
static constexpr int VR1_INT_MASK = 0x20;
static constexpr int VR1_M2_MASK = 0x10;
static constexpr int VR1_M1_MASK = 0x08;
static constexpr int VR1_SIZE4_MASK = 0x02;
static constexpr int VR1_MAG2X_MASK = 0x01;

static constexpr int VR2_NAME_TABLE_MASK = 0x0F;
static constexpr int VR2_NAME_TABLE_SHIFT = 10;

static constexpr int VR3_COLORTABLE_MASK_STANDARD = 0xFF;
static constexpr int VR3_COLORTABLE_SHIFT_STANDARD = 6;

static constexpr int VR3_COLORTABLE_MASK_BITMAP = 0x80;
static constexpr int VR3_COLORTABLE_SHIFT_BITMAP = 6;

static constexpr int VR3_ADDRESS_MASK_BITMAP = 0x7F;
static constexpr int VR3_ADDRESS_MASK_SHIFT = 6;

static constexpr int VR4_PATTERN_MASK_STANDARD = 0x07;
static constexpr int VR4_PATTERN_SHIFT_STANDARD = 11;

static constexpr int VR4_PATTERN_MASK_BITMAP = 0x04;
static constexpr int VR4_PATTERN_SHIFT_BITMAP = 11;

static constexpr int VR5_SPRITE_ATTR_MASK = 0x7F;
static constexpr int VR5_SPRITE_ATTR_SHIFT = 7;

static constexpr int VR6_SPRITE_PATTERN_MASK = 0x07;
static constexpr int VR6_SPRITE_PATTERN_SHIFT = 11;

static constexpr int VR7_BD_MASK = 0x0F;
static constexpr int VR7_BD_SHIFT = 0;

static constexpr int VDP_STATUS_F_BIT = 0x80;
static constexpr int VDP_STATUS_5S_BIT = 0x40;
static constexpr int VDP_STATUS_C_BIT = 0x20;
static constexpr int VDP_STATUS_5S_MASK = 0x1F;

static constexpr int ROW_SHIFT = 5;
static constexpr int THIRD_SHIFT = 11;
static constexpr int CHARACTER_PATTERN_SHIFT = 3;
static constexpr int CHARACTER_COLOR_SHIFT = 3;
static constexpr int ADDRESS_MASK_FILL = 0x3F;

static constexpr int SPRITE_EARLY_CLOCK_MASK = 0x80;
static constexpr int SPRITE_COLOR_MASK = 0x0F;
static constexpr int SPRITE_NAME_SHIFT = 3;
static constexpr int SPRITE_NAME_MASK_SIZE4 = 0xFC;

static constexpr int TRANSPARENT_COLOR_INDEX = 0;

static constexpr int REGISTER_COUNT = 8;

enum GraphicsMode { GRAPHICS_I, GRAPHICS_II, TEXT, MULTICOLOR, UNDEFINED };

inline bool SpritesAreSize4(const uint8_t* registers)
{
    return registers[1] & VR1_SIZE4_MASK;
}

inline bool SpritesCollided(const uint8_t status_register)
{
    return status_register & VDP_STATUS_5S_BIT;
}

inline bool FifthSprite(const uint8_t status_register)
{
    return status_register & VDP_STATUS_5S_MASK;
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

inline uint16_t GetPatternNameTableBase(const uint8_t* registers)
{
    return (registers[2] & VR2_NAME_TABLE_MASK) << VR2_NAME_TABLE_SHIFT;
}

inline uint8_t PatternName(const uint8_t* registers, const uint8_t* memory, uint16_t x, uint16_t y)
{
    return memory[GetPatternNameTableBase(registers) | x | (y << ROW_SHIFT)];
}

inline uint16_t GetBitmapPatternGeneratorTableBase(const uint8_t* registers)
{
    return (registers[4] & VR4_PATTERN_MASK_BITMAP) << VR4_PATTERN_SHIFT_BITMAP;
}

inline const uint8_t* GetBitmapPatternRows(const uint8_t* registers, const uint8_t* memory, uint16_t sector, uint8_t pattern_name)
{
    uint16_t table_offset = (pattern_name * 8) + sector;
    const uint8_t *bitmap_pattern_generator_table = memory + GetBitmapPatternGeneratorTableBase(registers);
    return bitmap_pattern_generator_table + table_offset;
}

inline uint16_t GetStandardPatternGeneratorTableBase(const uint8_t* registers)
{
    return (registers[4] & VR4_PATTERN_MASK_STANDARD) << VR4_PATTERN_SHIFT_STANDARD;
}

inline const uint8_t* GetStandardPatternRows(const uint8_t* registers, const uint8_t* memory, uint8_t pattern_name)
{
    uint16_t table_offset = (pattern_name * 8);
    const uint8_t *standard_pattern_generator_table = memory + GetStandardPatternGeneratorTableBase(registers);
    return standard_pattern_generator_table + table_offset;
}

inline uint16_t GetBitmapColorTableBase(const uint8_t* registers)
{
    return (registers[3] & VR3_COLORTABLE_MASK_BITMAP) << VR3_COLORTABLE_SHIFT_BITMAP;
}

inline const uint8_t* GetBitmapColorRows(const uint8_t* registers, const uint8_t* memory, uint16_t sector, uint8_t pattern_name)
{
    uint16_t table_offset = (pattern_name * 8) + sector;
    const uint8_t *color_table = memory + GetBitmapColorTableBase(registers);
    return color_table + table_offset;
}

inline uint16_t GetStandardColorTableBase(const uint8_t* registers)
{
    return (registers[3] & VR3_COLORTABLE_MASK_STANDARD) << VR3_COLORTABLE_SHIFT_STANDARD;
}

inline uint8_t GetStandardColorPair(const uint8_t* registers, const uint8_t* memory, uint8_t pattern_name)
{
    uint16_t table_offset = pattern_name / 8;
    const uint8_t *color_table = memory + GetStandardColorTableBase(registers);
    return color_table[table_offset];
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

// Assumes X is increasing from 0.
static void Set4BitPixmapColorIncrementingX(uint8_t pixmap[128 * 192], int x, int y, uint8_t color)
{
    if((x & 0b1) == 0) {
        pixmap[x / 2 + y * 128] = color;
    } else {
        pixmap[x / 2 + y * 128] |= color << 4;
    }
}

static void Set4BitPixmapColor(uint8_t pixmap[128 * 192], int x, int y, uint8_t color)
{
    uint8_t& pixelpair = pixmap[x / 2 + y * 128];
    uint8_t shift = (x % 2) * 4;
    uint8_t nybble_cleared = pixelpair & ~(0xF << shift);
    pixelpair = nybble_cleared | (color << shift);
}

static void Clear4BitPixmap(uint8_t pixmap[128 * 192], uint8_t color)
{
    for(int index = 0; index < 128 * 192; index++) {
        pixmap[index] = (color << 4) | color;
    }
}

template <typename SetPixelFunc>
static void DrawPatternFromGraphicsI(const uint8_t* registers, const uint8_t* memory, SetPixelFunc SetPixel)
{
    uint8_t backdrop = GetBackdropColor(registers);

    for(uint16_t name_y = 0; name_y < 24; name_y++) {

        for(uint16_t name_x = 0; name_x < 32; name_x++) {

            uint8_t pattern_name = PatternName(registers, memory, name_x, name_y);

            const uint8_t *pattern_rows = GetStandardPatternRows(registers, memory, pattern_name);

            uint8_t color_pair = GetStandardColorPair(registers, memory, pattern_name);
            uint8_t color0 = color_pair & 0xf;
            uint8_t color1 = (color_pair >> 4) & 0xf;

            if(color0 == TRANSPARENT_COLOR_INDEX) {
                color0 = backdrop;
            }
            if(color1 == TRANSPARENT_COLOR_INDEX) {
                color1 = backdrop;
            }

            for(uint16_t pattern_row_index = 0; pattern_row_index < 8; pattern_row_index++) {

                uint8_t pattern_row_byte = pattern_rows[pattern_row_index];

#pragma GCC unroll 8
                for(int pattern_col = 0; pattern_col < 8; pattern_col++) {

                    bool bit = pattern_row_byte & (0x80 >> pattern_col);
                    uint8_t color = bit ? color1 : color0;

                    SetPixel(name_x * 8 + pattern_col, name_y * 8 + pattern_row_index, color);
                }
            }
        }
    }
}

template <typename SetPixelFunc>
static void DrawPatternFromGraphicsII(const uint8_t* registers, const uint8_t* memory, SetPixelFunc SetPixel)
{
    uint8_t backdrop = GetBackdropColor(registers);

    // uint16_t address_mask = ((registers[3] & VR3_ADDRESS_MASK_BITMAP) << VR3_ADDRESS_MASK_SHIFT) | ADDRESS_MASK_FILL;

    for(uint16_t name_y = 0; name_y < 24; name_y++) {

        uint16_t sector = (name_y / 8) << THIRD_SHIFT;

        for(uint16_t name_x = 0; name_x < 32; name_x++) {

            uint8_t pattern_name = PatternName(registers, memory, name_x, name_y);

            const uint8_t *pattern_rows = GetBitmapPatternRows(registers, memory, sector, pattern_name);

            const uint8_t *color_pair_rows = GetBitmapColorRows(registers, memory, sector, pattern_name);

            for(uint16_t pattern_row_index = 0; pattern_row_index < 8; pattern_row_index++) {

                uint8_t pattern_row_byte = pattern_rows[pattern_row_index];

                uint8_t color_pair = color_pair_rows[pattern_row_index];
                uint8_t color0 = color_pair & 0xf;
                uint8_t color1 = (color_pair >> 4) & 0xf;

                if(color0 == TRANSPARENT_COLOR_INDEX) {
                    color0 = backdrop;
                }
                if(color1 == TRANSPARENT_COLOR_INDEX) {
                    color1 = backdrop;
                }

#pragma GCC unroll 8
                for(int pattern_col = 0; pattern_col < 8; pattern_col++) {

                    bool bit = pattern_row_byte & (0x80 >> pattern_col);
                    uint8_t color = bit ? color1 : color0;

                    SetPixel(name_x * 8 + pattern_col, name_y * 8 + pattern_row_index, color);
                }
            }
        }
    }
}

template <typename SetPixelFunc>
static void DrawPatternColors(const uint8_t* registers, const uint8_t* memory, SetPixelFunc SetPixel)
{
    using namespace TMS9918A;

    GraphicsMode mode = GetGraphicsMode(registers);

    if(mode == GraphicsMode::GRAPHICS_I) {

        DrawPatternFromGraphicsI(registers, memory, SetPixel);

    } else if(mode == GraphicsMode::GRAPHICS_II) {
        
        DrawPatternFromGraphicsII(registers, memory, SetPixel);

    } else {

        bool M1 = registers[1] & VR1_M1_MASK;
        bool M2 = registers[1] & VR1_M2_MASK;
        bool M3 = registers[0] & VR0_M3_MASK;
        printf("unhandled video mode M1 = %d M2 = %d M3 = %d\n", M1, M2, M3);

        for(int y = 0; y < SCREEN_Y; y++) {
            for(int x = 0; x < SCREEN_X; x++) {
                SetPixel(x, y, 8);
            }
        }
    }
}

template <typename SetPixelFunc>
static void DrawSprites(int row, const uint8_t* registers, const uint8_t* memory, uint8_t& flags_set, SetPixelFunc SetPixel)
{
    using namespace TMS9918A;

    static bool sprite_touched[SCREEN_X];

    std::fill(sprite_touched, sprite_touched + SCREEN_X, false);

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
    for(int i = 0; i < sprite_count; i++) {
        auto sprite = memory + sprite_table_address + i * 4;

        int sprite_y_byte = (sprite[0] + 1) & 0xFF;
        int sprite_y = (sprite_y_byte > 209) ? (sprite_y_byte - 256) : sprite_y_byte;
        int sprite_x = sprite[1];
        int sprite_name = sprite[2];
        bool sprite_earlyclock = sprite[3] & SPRITE_EARLY_CLOCK_MASK;
        int sprite_color = sprite[3] & SPRITE_COLOR_MASK;

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
            if(!SpritesCollided(flags_set) && sprites_in_row > 4) {
                flags_set |= VDP_STATUS_5S_BIT;
                flags_set |= i & VDP_STATUS_5S_MASK;
                break;
            }

            auto set_color_from_bit = [&flags_set, sprite_color, &SetPixel, row](int bit, int x) {
                if(bit) {
                    if(!sprite_touched[x]) {
                        sprite_touched[x] = true;
                        if(sprite_color != TRANSPARENT_COLOR_INDEX) {
                            SetPixel(x, row, sprite_color);
                        }
                    } else {
                        flags_set |= VDP_STATUS_C_BIT;
                    }
                }
            };

            if(size4) {
                int within_quadrant_y = within_sprite_y % 8;
                int quadrant_y = within_sprite_y / 8;
                int masked_sprite_name = sprite_name & SPRITE_NAME_MASK_SIZE4;

#pragma GCC unroll 8
                for(int x = start_x; x <= end_x; x++) {

                    int within_sprite_x = mag2x ? ((x - sprite_x) / 2) : (x - sprite_x);

                    int quadrant = quadrant_y + (within_sprite_x / 8) * 2;
                    int within_quadrant_x = within_sprite_x % 8;
                    int sprite_pattern_address = GetSpritePatternTableBase(registers) | (masked_sprite_name << SPRITE_NAME_SHIFT) | (quadrant << 3) | within_quadrant_y;
                    int bit = memory[sprite_pattern_address] & (0x80 >> within_quadrant_x);
                    set_color_from_bit(bit, x);
                }

            } else {

                int sprite_pattern_address = GetSpritePatternTableBase(registers) | (sprite_name << SPRITE_NAME_SHIFT) | within_sprite_y;
                int bitpattern = memory[sprite_pattern_address];

#pragma GCC unroll 8
                for(int x = start_x; x <= end_x; x++) {

                    int within_sprite_x = mag2x ? ((x - sprite_x) / 2) : (x - sprite_x);

                    int bit = bitpattern & (0x80 >> within_sprite_x);
                    set_color_from_bit(bit, x);
                }
            }

        }
    }
}

static void AddSpritesToRow(int row, uint8_t row_colors[TMS9918A::SCREEN_X], const uint8_t* registers, const uint8_t* memory, uint8_t& flags_set)
{
    auto RowSetPixel = [row_colors, row](int x, int y, int color) {
        // We know row is constant so ignore it.
        if(row_colors) {
            row_colors[x] = color;
        }
    };
    DrawSprites(row, registers, memory, flags_set, RowSetPixel);
}

template <typename SetPixelFunc>
static uint8_t CreateImageAndReturnFlags(const uint8_t* registers, const uint8_t* memory, SetPixelFunc SetPixel)
{
    using namespace TMS9918A;

    uint8_t flags_set = 0;

    if(ActiveDisplayAreaIsBlanked(registers)) {
        uint8_t backdrop = GetBackdropColor(registers);
        for(int row = 0; row < SCREEN_Y; row++) {
            for(int col = 0; col < SCREEN_X; col++) {
                SetPixel(col, row, backdrop);
            }
        }
        return flags_set;
    }

    DrawPatternColors(registers, memory, SetPixel);
    if(SpritesVisible(registers)) {
        for(int row = 0; row < SCREEN_Y; row++) {
            DrawSprites(row, registers, memory, flags_set, SetPixel);
        }
    }

    return flags_set;
}

[[maybe_unused]] static uint8_t GetStatusFromSpriteConfiguration(const uint8_t* registers, const uint8_t* memory)
{
    using namespace TMS9918A;

    uint8_t flags_set = 0;

    for(int row = 0; row < SCREEN_Y; row++) {
        if(SpritesVisible(registers)) {
            AddSpritesToRow(row, nullptr, registers, memory, flags_set);
        }
    }

    return flags_set;
}

[[maybe_unused]] static uint8_t Create4BitPixmap(const uint8_t* registers, const uint8_t* memory, uint8_t fb[128 * 192])
{
    auto pixel_setter = [fb](int x, int y, uint8_t color) {
        TMS9918A::Set4BitPixmapColor(fb, x, y, color);
    };

    return TMS9918A::CreateImageAndReturnFlags(registers, memory, pixel_setter);
}

};

#endif /* _TMS9918_H_ */
