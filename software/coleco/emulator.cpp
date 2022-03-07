#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <cerrno>

#include <string>
#include <chrono>
#include <map>
#include <set>
#include <deque>
#include <array>
#include <thread>
#include <memory>

#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>

#define ENABLE_AUTOMATION

#undef PROVIDE_DEBUGGER

#ifdef PROVIDE_DEBUGGER
#include <signal.h>
#include <readline/readline.h>
#include <readline/history.h>
#include "readhex.h"
#endif /* PROVIDE_DEBUGGER */

#include "z80user.h"
#include "z80emu.h"

#include "coleco_platform.h"
#include "tms9918.h"

#if defined(ROSA)
#include "rocinante.h"
#endif

// XXX *** MUST do sprite signal at scanout time *** XXX

// Wrap the entire app in a namespace to protect from other apps if linked together.
namespace ColecovisionEmulator
{

Z80_STATE z80state;

bool Z80IsInNMI(Z80_STATE& z80state)
{
    return z80state.in_nmi;
}

typedef long long clk_t;

inline void write_rgb8_image_as_P6(uint8_t *imageRGB, int width, int height, FILE *fp)
{
    fprintf(fp, "P6 %d %d 255\n", width, height);
    for(int row = 0; row < height; row++) {
        for(int col = 0; col < width; col++) {
            fwrite(imageRGB + (row * width + col) * 3, 3, 1, fp);
        }
    }
}

inline void write_rgba8_image_as_P6(uint8_t *imageRGBA, int width, int height, FILE *fp)
{
    fprintf(fp, "P6 %d %d 255\n", width, height);
    for(int row = 0; row < height; row++) {
        for(int col = 0; col < width; col++) {
            fwrite(imageRGBA + (row * width + col) * 4, 3, 1, fp);
        }
    }
}

struct RAMboard
{
    uint16_t base;
    uint16_t mirroredLength;
    uint16_t addressMask;
    std::vector<uint8_t> bytes;
    RAMboard(uint16_t base_, uint16_t actualLength_, uint16_t mirroredLength_) :
        base(base_),
        mirroredLength(mirroredLength_),
        addressMask(actualLength_ - 1),
        bytes(actualLength_, 0)
    { }
};

struct ROMboard
{
    uint16_t base;
    uint16_t length;
    std::vector<uint8_t> bytes;
    ROMboard(uint16_t base_, uint16_t length_, uint8_t *bytes_) : 
        base(base_),
        length(length_),
        bytes(bytes_, bytes_ + length)
    {
    }
};

bool quit_requested = false;
bool enter_debugger = false; 

static constexpr clk_t machine_clock_rate = 3579545;
static constexpr uint32_t slice_frequency_times_1000 = 59940;
static constexpr uint32_t clocks_per_retrace = machine_clock_rate * 1000 / slice_frequency_times_1000;

static constexpr uint32_t DEBUG_NONE = 0x00;
[[maybe_unused]] static constexpr uint32_t DEBUG_ROM = 0x01; // Needs to be passed over to z80emu
[[maybe_unused]] static constexpr uint32_t DEBUG_RAM = 0x02; // Needs to be passed over to z80emu
static constexpr uint32_t DEBUG_IO = 0x04;
static constexpr uint32_t DEBUG_SCANOUT = 0x08;
static constexpr uint32_t DEBUG_VDP_OPERATIONS = 0x10;
uint32_t debug = DEBUG_NONE;
bool abort_on_exception = false;
bool do_save_images_on_vdp_write = false;
int dump_some_audio = 0;
static constexpr bool break_on_unknown_address = true;

void print_state(Z80_STATE* state)
{
    printf("BC :%04X  DE :%04X  HL :%04X  AF :%04X  IX : %04X  IY :%04X  SP :%04X\n",
        state->registers.word[Z80_BC], state->registers.word[Z80_DE],
        state->registers.word[Z80_HL], state->registers.word[Z80_AF],
        state->registers.word[Z80_IX], state->registers.word[Z80_IY],
        state->registers.word[Z80_SP]);
    printf("BC':%04X  DE':%04X  HL':%04X  AF':%04X\n",
        state->alternates[Z80_BC], state->alternates[Z80_DE],
        state->alternates[Z80_HL], state->alternates[Z80_AF]);
    printf("PC :%04X\n",
        state->pc);
}

typedef std::function<uint8_t (const uint8_t *registers, const uint8_t *memory)> tms9918_scanout_func;
typedef std::function<void (uint8_t *audiobuffer, size_t dist)> audio_flush_func;

struct SN76489A
{
    bool debug{false};
    uint32_t clock_rate{0};

    uint8_t cmd_latched = 0;
    static constexpr uint8_t CMD_BIT = 0x80;
    static constexpr uint8_t CMD_REG_MASK = 0x70;
    static constexpr uint8_t DATA_MASK = 0x0F;
    static constexpr int CMD_REG_SHIFT = 4;
    static constexpr int FREQ_HIGH_SHIFT = 4;
    static constexpr uint8_t FREQ_HIGH_MASK = 0x3F;
    static constexpr uint8_t CMD_NOISE_CONFIG_MASK = 0x04;
    static constexpr int CMD_NOISE_CONFIG_SHIFT = 2;
    static constexpr uint8_t CMD_NOISE_FREQ_MASK = 0x03;

    uint32_t sample_rate;

    uint32_t tone_lengths[3] = {0, 0, 0};
    uint32_t tone_attenuation[3] = {0, 0, 0};

    uint32_t noise_config{0};
    uint32_t noise_length{0};
    uint32_t noise_length_id{0};
    uint32_t noise_attenuation{0};

    uint32_t tone_counters[3] = {0, 0, 0};
    uint32_t tone_bit[3] = {0, 0, 0};
    uint32_t noise_counter{0};

    uint16_t noise_register = 0x8000;
    uint32_t noise_flipflop = 0;

    clk_t previous_clock{0};

    clk_t max_audio_forward;
    size_t audio_buffer_size;
    std::vector<uint8_t> audio_buffer;
    clk_t audio_buffer_next_sample{0};

    void reset()
    {
        cmd_latched = 0;
        std::fill(tone_lengths, tone_lengths + 3, 0);
        std::fill(tone_attenuation, tone_attenuation + 3, 0);

        noise_config = 0;
        noise_length = 0;
        noise_length_id = 0;
        noise_attenuation = 0;

        std::fill(tone_bit, tone_bit + 3, 0);

        noise_register = 0x8000;
        noise_flipflop = 0;
    }

    SN76489A(uint32_t clock_rate, uint32_t sample_rate, size_t audio_buffer_size) :
        clock_rate(clock_rate),
        sample_rate(sample_rate),
        audio_buffer_size(audio_buffer_size)
    {
        max_audio_forward = machine_clock_rate / sample_rate - 1;
        audio_buffer.resize(audio_buffer_size);

        for(int i = 0; i < 3; i++) {
            tone_counters[i] = 0;
            tone_bit[i] = 0;
        }
        noise_counter = 0;
        previous_clock = 0;
    }

    void write(uint8_t data)
    {
        if(debug) printf("sound write 0x%02X\n", data);
        if(data & CMD_BIT) {

            cmd_latched = data;

            uint32_t reg = (data & CMD_REG_MASK) >> CMD_REG_SHIFT;

            if(reg == 1 || reg == 3 || reg == 5) {
                tone_attenuation[(reg - 1) / 2] = data & DATA_MASK;
            } else if(reg == 7) {
                noise_attenuation = data & DATA_MASK;
            } else if(reg == 6) {
                noise_config = (data & CMD_NOISE_CONFIG_MASK) >> CMD_NOISE_CONFIG_SHIFT;
                noise_length_id = data & CMD_NOISE_FREQ_MASK;
                if(noise_length_id == 0) {
                    noise_length = 512;
                } else if(noise_length_id == 1) {
                    noise_length = 1024;
                } else if(noise_length_id == 2) {
                    noise_length = 2048;
                }
                /* if noise_length_id == 3 then noise counter is tone_counters[2]*/

                noise_register = 0x8000;
            }

        } else {

            uint32_t reg = (cmd_latched & CMD_REG_MASK) >> CMD_REG_SHIFT;

            if(reg == 0 || reg == 2 || reg == 4) {
                tone_lengths[reg / 2] = 16 * (((data & FREQ_HIGH_MASK) << FREQ_HIGH_SHIFT) | (cmd_latched & DATA_MASK));
                if(tone_counters[reg / 2] >= tone_lengths[reg / 2])
                    tone_counters[reg / 2] = 0;
            }
        }
    }

    clk_t calc_flip_count(clk_t previous_clock, clk_t current_clock, uint32_t previous_counter, uint32_t length)
    {
        if(length < 1) { 
            return 0;
        }
        clk_t clocks = current_clock - previous_clock;
        clk_t flips = (previous_counter + clocks) / length;
        return flips;
    }

    void advance_noise_to_clock(clk_t flips)
    {
        for(int i = 0; i < flips; i++) {
            noise_flipflop ^= 1;

            if(noise_flipflop) {
                uint8_t noise_bit = noise_register & 0x1;
                uint8_t new_bit;

                if(noise_config == 1) {
                    new_bit = (noise_register & 0x1) ^ ((noise_register & 0x8) >> 3);
                } else {
                    new_bit = noise_bit;
                }

                noise_register = (noise_register >> 1) | (new_bit << 15);
            }
        }
    }

    void advance_to_clock(clk_t clk)
    {
        clk_t tone_flips[3];
        
        for(int i = 0; i < 3; i++) {
            tone_flips[i] = calc_flip_count(previous_clock, clk, tone_counters[i], tone_lengths[i]);
        }

        int flips;
        if(noise_length_id == 3)  {
            flips = tone_flips[2];
        } else {
            flips = calc_flip_count(previous_clock, clk, noise_counter, noise_length);
        }
        advance_noise_to_clock(flips);

        for(int i = 0; i < 3; i++) {
            tone_bit[i] = tone_bit[i] ^ (tone_flips[i] & 0x1);
            if(tone_lengths[i] > 0) {
                tone_counters[i] = (tone_counters[i] + (clk - previous_clock)) % tone_lengths[i];
            }
        }
        if(noise_length > 0) {
            noise_counter = (noise_counter + (clk - previous_clock)) % noise_length;
        }

        previous_clock = clk;
    }

    static uint8_t scale_by_attenuation_flags(uint32_t att, uint8_t value)
    {
        const static uint16_t att_table[] = {
            256, 203, 161, 128, 101, 80, 64, 51, 40, 32, 25, 20, 16, 12, 10, 0,
        };

        return value * att_table[att] / 256;
    }

    uint8_t get_level()
    {
        uint8_t v = 128 + 
            scale_by_attenuation_flags(tone_attenuation[0], tone_bit[0] ? 0 : 64) 
            - scale_by_attenuation_flags(tone_attenuation[1], tone_bit[1] ? 0 : 64)
            + scale_by_attenuation_flags(tone_attenuation[2], tone_bit[2] ? 0 : 64)
            - scale_by_attenuation_flags(noise_attenuation, (noise_register & 0x1) ? 0 : 64);
            ;

        return v;
    }

    void generate_audio(clk_t clk, audio_flush_func audio_flush)
    {
	clk_t current_audio_sample = previous_clock * sample_rate / clock_rate;
        for(clk_t c = previous_clock + 1; c < clk; c += max_audio_forward) {

            clk_t next_audio_sample = (c + 1) * sample_rate / clock_rate;

            if(next_audio_sample > current_audio_sample) {
                advance_to_clock(c);

                audio_buffer[audio_buffer_next_sample] = get_level();
                if(dump_some_audio-- > 0) {
                    printf("audio: %d\n", audio_buffer[audio_buffer_next_sample]);
                }
                audio_buffer_next_sample++;

                if(audio_buffer_next_sample == audio_buffer_size) {
                    audio_flush(audio_buffer.data(), audio_buffer_size);
                    audio_buffer_next_sample = 0;
                }
            }
	    current_audio_sample = next_audio_sample;
        }

        previous_clock = clk;
    }
};

struct TMS9918AEmulator
{
    bool cmd_started_in_nmi{false};
    uint32_t frame_number{0};
    uint32_t write_number{0};

    static constexpr uint16_t MEMORY_SIZE = 16384;
    std::array<uint8_t, MEMORY_SIZE> memory{};
    std::array<uint8_t, 8> registers{};
    uint8_t status_register{0};

    enum {CMD_PHASE_FIRST, CMD_PHASE_SECOND} cmd_phase = CMD_PHASE_FIRST;
    uint8_t cmd_data = 0x0;
    uint16_t read_address = 0x0;
    uint16_t write_address = 0x0;

    TMS9918AEmulator()
    {
    }

    void reset()
    {
        std::fill(registers.begin(), registers.end(), 0);
        std::fill(memory.begin(), memory.end(), 0);
    }

    void vsync()
    {
        using namespace TMS9918A;
        status_register |= VDP_STATUS_F_BIT;
    }

    void write(uint8_t cmd, uint8_t data)
    {
        using namespace TMS9918A;
        if(debug & DEBUG_VDP_OPERATIONS) printf("VDP write %lu cmd==%d, in_nmi = %d\n", write_number, cmd, Z80IsInNMI(z80state) ? 1 : 0);
        if(do_save_images_on_vdp_write) { /* debug */

            uint8_t framebuffer[SCREEN_X * SCREEN_Y * 3];
            auto pixel_setter = [&framebuffer](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
                uint8_t *pixel = framebuffer + 3 * (x + y * SCREEN_X) + 0;
                SetColor(pixel, r, g, b);
            };

            CreateImageAndReturnFlags(registers.data(), memory.data(), pixel_setter);
            char name[512];
            sprintf(name, "frame_%04lu_%05lu_%d_%02X.ppm", frame_number, write_number, cmd, data);
            FILE *fp = fopen(name, "wb");
            write_rgb8_image_as_P6(framebuffer, SCREEN_X, SCREEN_Y, fp);
            fclose(fp);
        }
        write_number++;
        if(cmd) {

            if(cmd_phase == CMD_PHASE_FIRST) {

                if(debug & DEBUG_VDP_OPERATIONS) printf("VDP command write, first byte 0x%02X\n", data);
                cmd_data = data;
                cmd_phase = CMD_PHASE_SECOND;
                cmd_started_in_nmi = Z80IsInNMI(z80state);

            } else {

                if(Z80IsInNMI(z80state) != cmd_started_in_nmi) {
                    if(cmd_started_in_nmi) {
                        printf("VDP cmd was started in NMI but finished outside NMI; likely corruption\n");
                    } else {
                        printf("VDP cmd was started outside NMI but finished inside NMI; likely corruption\n");
                    }
                    if(abort_on_exception) abort();
                }

                uint8_t cmd = data & CMD_MASK;
                if(cmd == CMD_SET_REGISTER) {
                    uint8_t which_register = data & REG_A0_A5_MASK;
                    if(debug & DEBUG_VDP_OPERATIONS) printf("VDP command write to register 0x%02X, value 0x%02X\n", which_register, cmd_data);
                    registers[which_register] = cmd_data;
                } else if(cmd == CMD_SET_WRITE_ADDRESS) {
                    write_address = ((data & REG_A0_A5_MASK) << 8) | cmd_data;
                    if(debug & DEBUG_VDP_OPERATIONS) printf("VDP write address set to 0x%04X\n", write_address);
                } else if(cmd == CMD_SET_READ_ADDRESS) {
                    read_address = ((data & REG_A0_A5_MASK) << 8) | cmd_data;
                    if(debug & DEBUG_VDP_OPERATIONS) printf("VDP read address set to 0x%04X\n", write_address);
                } else {
                    if((debug & DEBUG_VDP_OPERATIONS) && !abort_on_exception) printf("VDP cmd was unknown 0x%02X!\n", cmd);
                    if(abort_on_exception) {
                        printf("VDP cmd was unknown 0x%02X, aborting\n", cmd);
                        abort();
                    }
                }
                cmd_phase = CMD_PHASE_FIRST;
            }

        } else {

            if(debug & DEBUG_VDP_OPERATIONS) {
                static char bitfield[9];
                for(int i = 0; i < 8; i++) bitfield[i] = (data & (0x80 >> i)) ? '*' : ' ';
                bitfield[8] = '\0';
                if(isprint(data)) {
                    printf("VDP data write 0x%02X, '%s' ('%c')\n", data, bitfield, data);
                } else {
                    printf("VDP data write 0x%02X, '%s'\n", data, bitfield);
                }
            }
            memory[write_address++] = data;
            write_address = write_address % MEMORY_SIZE;
        }

    }

    uint8_t read(uint8_t cmd)
    {
        using namespace TMS9918A;
        if(cmd) {
            if(cmd_phase == CMD_PHASE_SECOND) {
                if(Z80IsInNMI(z80state)) {
                    printf("cmd_phase was reset in ISR\n");
                } else {
                    printf("cmd_phase was reset outside ISR\n");
                }
                if(abort_on_exception) abort();
            }
            cmd_phase = CMD_PHASE_FIRST;
            uint8_t data = status_register;
            status_register = 0;  
            return data;
        } else {
            uint8_t data = memory[read_address++];
            read_address = read_address % MEMORY_SIZE;
            return data;
        }
    }

    void perform_scanout(tms9918_scanout_func scanout)
    {
        frame_number++;
        write_number = 0;
        if(debug & DEBUG_SCANOUT) {
            printf("scanout frame %lu\n", frame_number);
        }

        status_register |= scanout(registers.data(), memory.data());
    }

    bool nmi_required()
    {
        using namespace TMS9918A;
        return InterruptsAreEnabled(registers.data()) && VSyncInterruptHasOccurred(status_register);
    }
};

typedef std::function<uint8_t (int index, bool JoystickNotKeypad)> GetControllerStateFunc;

struct ColecoHW
{
    TMS9918AEmulator vdp;
    SN76489A sound;

    bool reading_joystick = true;

    static constexpr uint8_t VDP_DATA_PORT = 0xBE;
    static constexpr uint8_t VDP_CMD_PORT = 0xBF;

    static constexpr uint8_t SN76489A_PORT = 0xFF;

    static constexpr uint8_t SWITCH_TO_KEYPAD_PORT = 0x80;
    static constexpr uint8_t SWITCH_TO_JOYSTICK_PORT = 0xC0;
    static constexpr uint8_t CONTROLLER1_PORT = 0xFC;
    static constexpr uint8_t CONTROLLER2_PORT = 0xFF;
    GetControllerStateFunc get_controller_state;

    // XXX eventually template on a bool which determines whether
    // these are populated in io_read and io_write
    std::set<uint16_t> io_reads;
    std::map<uint16_t, uint8_t> io_writes;

    ColecoHW(uint32_t sample_rate, size_t audio_buffer_size, GetControllerStateFunc get_controller_state) :
        sound(machine_clock_rate, sample_rate, audio_buffer_size),
        get_controller_state(get_controller_state)
    {
    }

    bool io_write(uint8_t addr, uint8_t data)
    {
        if(false) {
            if(addr == ColecoHW::VDP_CMD_PORT) {
                vdp.write(1, data);
#ifdef PROVIDE_DEBUGGER
                io_writes.insert({addr, data});
#endif
                return true;
            }

            if(addr == ColecoHW::VDP_DATA_PORT) {
                vdp.write(0, data);
#ifdef PROVIDE_DEBUGGER
                io_writes.insert({addr, data});
#endif
                return true;
            }
        } else {
            if((addr >= 0xA0) && (addr <= 0xBF)) {
                vdp.write(addr & 0x1, data);
#ifdef PROVIDE_DEBUGGER
                io_writes.insert({addr, data});
#endif
                return true;
            }
        }

        /* if(addr == ColecoHW::SN76489A_PORT) */
        if((addr >= 0xE0) && (addr <= 0xFF)) {
            if(debug & DEBUG_IO) printf("audio write 0x%02X\n", data);
            sound.write(data);
#ifdef PROVIDE_DEBUGGER
            io_writes.insert({addr, data});
#endif
            return true;
        }

        if(addr == ColecoHW::SWITCH_TO_KEYPAD_PORT) {
            if(debug & DEBUG_IO) printf("switch to keypad\n");
            reading_joystick = false;
#ifdef PROVIDE_DEBUGGER
            io_writes.insert({addr, data});
#endif
            return true;
        }

        if(addr == ColecoHW::SWITCH_TO_JOYSTICK_PORT) {
            if(debug & DEBUG_IO) printf("switch to keypad\n");
            reading_joystick = true;
#ifdef PROVIDE_DEBUGGER
            io_writes.insert({addr, data});
#endif
            return true;
        }

        return false;
    }

    bool io_read(uint8_t addr, uint8_t &data)
    {
        if(false) {
            if(addr == ColecoHW::VDP_CMD_PORT) {
                if(debug & DEBUG_IO) printf("read VDP command port\n");
                data = vdp.read(1);
#ifdef PROVIDE_DEBUGGER
                io_reads.insert(addr);
#endif
                return true;
            }

            if(addr == ColecoHW::VDP_DATA_PORT) {
                if(debug & DEBUG_IO) printf("read VDP command port\n");
                data = vdp.read(0);
#ifdef PROVIDE_DEBUGGER
                io_reads.insert(addr);
#endif
                return true;
            }
        } else {
            if((addr >= 0xA0) && (addr <= 0xBF)) {
                if(debug & DEBUG_IO) printf("read VDP 0x%02X\n", addr);
                data = vdp.read(addr & 0x1);
#ifdef PROVIDE_DEBUGGER
                io_reads.insert(addr);
#endif
                return true;
            }
        }

        if((addr >= 0xE0) && (addr <= 0xFF)) {
            uint8_t index = (addr & 0x02) >> 1;
            data = get_controller_state(index, reading_joystick);
            if(debug & DEBUG_IO) printf("read controller%d port 0x%02X, read 0x%02X\n", index + 1, addr, data);
#ifdef PROVIDE_DEBUGGER
            io_reads.insert(addr);
#endif
            return true;
        }

        if(debug & DEBUG_IO) printf("read unknown address 0x%02X\n", addr);

        if(break_on_unknown_address) {
            abort();
        }

        return false;
    }

    void reset(void)
    {
        vdp.reset();
        sound.reset();
    }

    bool nmi_required()
    {
        return vdp.nmi_required();
    }

    void fill_flush_audio(clk_t clk, audio_flush_func audio_flush)
    {
        sound.generate_audio(clk, audio_flush);
    }
};

#if USE_BG80D
#include "bg80d.h"
#endif /* USE_BG80D */

struct reader_context
{
    uint16_t address;
    ColecovisionContext* colecovision_context;
};

uint8_t reader(void *p)
{
    auto* context = reinterpret_cast<reader_context*>(p);
    uint8_t data = cv_read_byte(context->colecovision_context, context->address);
    context->address++;
    return data;
}

int disassemble(uint16_t address, std::function<const std::string& (uint16_t address, uint16_t& symbol_offset)> get_symbol, int bytecount, ColecovisionContext* colecovision_context)
{
    int total_bytes = 0;

#if USE_BG80D

    while(bytecount > 0) {

        uint16_t symbol_offset;
        const std::string& sym = get_symbol(address, symbol_offset);

        reader_context context{.address = address, .colecovision_context = colecovision_context};
        bg80d::opcode_spec_t *opcode = bg80d::decode(reader, &context, address);
        if(opcode == 0) {
            break;
        }

        printf("%04X %s+0x%04X%*s", address, sym.c_str(), symbol_offset, 16 - (int)sym.size() - 5, "");

        int opcode_length = (opcode->pc_after - address);
        int opcode_bytes_pad = 1 + 3 + 3 + 3 - opcode_length * 3;
        for(int i = 0; i < opcode_length; i++) {
            uint8_t byte;
            byte = cv_read_byte(colecovision_context, address + i);
            printf("%.2hhX ", byte);
        }

        printf("%*s", opcode_bytes_pad, "");
        printf("%5s %s\n", opcode->prefix, opcode->description);

        bytecount -= opcode_length;
	total_bytes += opcode_length;
    }
    // FB5C conin+0x0002         e6     AND A, n        ff          ;  AND of ff to reg

#else

    uint16_t symbol_offset;
    uint8_t buffer[3];
    for(int i = 0; i < 3; i++) {
        buffer[i] = cv_read_byte(colecovision_context, address + i);
    }
    const std::string& sym = get_symbol(address, symbol_offset);
    printf("%04X %s+0x%04X%*s : %02X %02X %02X\n", address, sym.c_str(), symbol_offset, 16 - (int)sym.size() - 5, "", buffer[0], buffer[1], buffer[2]);

#endif

    return total_bytes;
}

#ifdef PROVIDE_DEBUGGER

struct BreakPoint
{
    enum Type {INSTRUCTION, DATA} type;
    uint16_t address;
    uint8_t old_value;
    bool enabled;
    BreakPoint(uint16_t address_, uint8_t old_value_) :
        type(DATA),
        address(address_),
        old_value(old_value_),
        enabled(true)
    {
    }
    BreakPoint(uint16_t address_) :
        type(INSTRUCTION),
        address(address_),
        old_value(0),
        enabled(true)
    {
    }
    void enable() { enabled = true; }
    void disable() { enabled = false; }
};

void clear_breakpoints(std::vector<BreakPoint>& breakpoints, Z80_STATE* state, ColecovisionContext* colecovision_context)
{
    for(auto i = breakpoints.begin(); i != breakpoints.end(); i++) {
        BreakPoint& bp = (*i);
        switch(bp.type) {
            case BreakPoint::DATA:
                bp.old_value = cv_read_byte(colecovision_context, bp.address);
                break;
            case BreakPoint::INSTRUCTION:
                break;
        }
    }
}

bool is_breakpoint_triggered(std::vector<BreakPoint>& breakpoints, Z80_STATE* state, int& which, ColecovisionContext* colecovision_context)
{
    for(auto i = breakpoints.begin(); i != breakpoints.end(); i++) {
        BreakPoint& bp = (*i);
        if(bp.enabled)
            switch(bp.type) {
                case BreakPoint::INSTRUCTION:
                    if(state->pc == bp.address) {
                        which = i - breakpoints.begin();
                        return true;
                    }
                    break;
                case BreakPoint::DATA:
                    uint8_t data;
                    data = cv_read_byte(colecovision_context, bp.address);
                    if(data != bp.old_value) {
                        which = i - breakpoints.begin();
                        return true;
                    }
                    break;
            }
    }
    return false;
}

struct Debugger
{
    ColecoHW* colecohw{nullptr};
    ColecovisionContext *colecovision_context;
    clk_t& clk;
    std::vector<BreakPoint> breakpoints;
    std::unordered_set<uint16_t> io_watch;
    std::string address_to_symbol[65536]; // XXX excessive memory?
    std::unordered_map<std::string, uint16_t> symbol_to_address; // XXX excessive memory?
    sig_t previous_sigint;
    bool state_may_have_changed;
    bool last_was_step;
    bool last_was_jump;
    const std::string& get_symbol(uint16_t address, uint16_t& offset)
    {
        static std::string no_symbol = "";
        offset = 0;
        while(address >= 0 && address_to_symbol[address].empty()) {
            address--;
            offset++;
        }
        if(address < 0) {
            return no_symbol;
        }
        return address_to_symbol[address];
    }
    bool load_symbols(char *filename)
    {
        FILE *fp = fopen(filename, "ra");
        if(fp == NULL) {
            fprintf(stderr, "couldn't open %s to read symbols\n", filename);
            return false;
        }
        fseek(fp, 0, SEEK_END);
        ssize_t size = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        char* buffer = new char[size];
        fread(buffer, size, 1, fp);
        fclose(fp);
        char *symbol_part = buffer;
        while((symbol_part - buffer) < size && (*symbol_part != '')) {
            symbol_part++;
        }
        if(symbol_part - buffer >= size) {
            fprintf(stderr, "couldn't find symbol section in %s\n", filename);
            delete[] buffer;
            return false;
        }
        int address, consumed;
        char symbol[512];
        while(sscanf(symbol_part, "%x %s%n", &address, symbol, &consumed) == 2) {
            address_to_symbol[address] = symbol;
            symbol_to_address[symbol] = address;
            symbol_part += consumed;
        }

        delete[] buffer;
        return true;
    }
    void ctor()
    {
        state_may_have_changed = true;
        last_was_step = false;
        last_was_jump = false;
    }
    Debugger(ColecoHW *colecohw, ColecovisionContext *colecovision_context, clk_t& clk) :
        colecohw(colecohw),
        colecovision_context(colecovision_context),
        clk(clk)
    {
        ctor();
    }
    bool process_line(Z80_STATE* state, char *line);
    bool process_command(Z80_STATE* state, char *command);
    void go(FILE *fp, Z80_STATE* state);
    bool should_debug(Z80_STATE* state);
};


// XXX make this pointers-to-members
typedef bool (*command_handler)(Debugger* d, Z80_STATE* state, int argc, char **argv);

std::unordered_map<std::string, command_handler> command_handlers;

bool lookup_or_parse(std::unordered_map<std::string, uint16_t>& symbol_to_address, char *s, uint16_t& a)
{
    auto found = symbol_to_address.find(s);
    if(found != symbol_to_address.end()) {
        a = found->second;
    } else {
        char *endptr;
        a = strtol(s, &endptr, 0);
        if(*endptr != '\0') {
            printf("number parsing failed for %s; forgot to lead with 0x?\n", s);
            return false;
        }
    }
    return true;
}

struct storer_context
{
    uint16_t min_address;
    uint16_t max_address;
    int write_count;
    ColecovisionContext* colecovision_context;
};

void store_memory(void *arg, uint16_t address, uint8_t p)
{
    auto* context = reinterpret_cast<storer_context*>(arg);
    cv_write_byte(context->colecovision_context, address, p);
    context->min_address = std::min(context->min_address, address);
    context->max_address = std::max(context->max_address, address);
    context->write_count++; // XXX Could be overwrites...
}

bool debugger_readhex(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "readhex: expected filename argument\n");
        return false;
    }
    FILE *fp = fopen(argv[1], "ra");
    if(fp == NULL) {
        fprintf(stderr, "failed to open %s for reading\n", argv[1]);
        return false;
    }

    storer_context context{.min_address = 0xFFFF, .max_address = 0, .write_count = 0, .colecovision_context = d->colecovision_context};

    int success = read_hex(fp, store_memory, &context, 0);
    if (!success) {
        fprintf(stderr, "error reading hex file %s\n", argv[1]);
        fclose(fp);
        return false;
    }
    printf("Read %d (0x%04X) bytes from %s into 0x%04X..0x%04X (might be sparse)\n",
        context->write_count, context->write_count, argv[1], context->min_address, context->max_address);
    fclose(fp);
    return false;
}

bool debugger_readbin(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 3) {
        fprintf(stderr, "readbin: expected filename and address\n");
        return false;
    }
    uint8_t buffer[128];

    uint16_t address;
    if(!lookup_or_parse(d->symbol_to_address, argv[2], address))
        return false;

    uint16_t a = address;
    FILE *fp = fopen(argv[1], "rb");
    if(fp == NULL) {
        fprintf(stderr, "failed to open %s for reading\n", argv[1]);
        return false;
    }
    size_t size;
    while((size = fread(buffer, 1, sizeof(buffer), fp)) > 0) {
        for(size_t i = 0; i <= size; i++, a++) {
            cv_write_byte(d->colecovision_context, a, buffer[i]);
        }
    }
    printf("Read %d (0x%04X) bytes from %s into 0x%04X..0x%04X\n",
        a - address, a - address, argv[1], address, a - 1);
    fclose(fp);
    return false;
}

void dump_buffer_hex(int indent, uint16_t actual_address, uint8_t *data, int size)
{
    uint16_t address = 0;
    int screen_lines = 0;

    while(size > 0) {
        if(screen_lines >= 24) { 
            printf(":");
            char line[512];
            fgets(line, sizeof(line), stdin);
            if(strcmp(line, "q") == 0)
                return;
            screen_lines = 0;
        }
        int howmany = std::min(size, 16);

        printf("%*s0x%04X: ", indent, "", actual_address + address);
        for(int i = 0; i < howmany; i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");

        printf("%*s        ", indent, "");
        for(int i = 0; i < howmany; i++) {
            printf(" %c ", isprint(data[i]) ? data[i] : '.');
        }
        printf("\n");
        screen_lines++;

        size -= howmany;
        data += howmany;
        address += howmany;
    }
}

void disassemble_instructions(uint16_t address, Debugger *d, int insncount)
{
    for(int i = 0; i < insncount; i++) {
	address += disassemble(address, [d](uint16_t address, uint16_t& symbol_offset) -> const std::string& {return d->get_symbol(address, symbol_offset);}, 1, d->colecovision_context);
    }
}

bool debugger_dis(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 3) {
        fprintf(stderr, "dis: expected address and count\n");
        return false;
    }
    uint16_t address;
    if(!lookup_or_parse(d->symbol_to_address, argv[1], address)) {
        return false;
    }

    char *endptr;
    int count = strtol(argv[2], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[2]);
        return false;
    }
    disassemble_instructions(address, d, count, d->colecovision_context);
    return false;
}

bool debugger_dump(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 3) {
        fprintf(stderr, "dump: expected address and length\n");
        return false;
    }
    uint16_t address;
    if(!lookup_or_parse(d->symbol_to_address, argv[1], address)) {
        return false;
    }

    char *endptr;
    int length = strtol(argv[2], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[2]);
        return false;
    }
    uint8_t buffer[65536];
    for(int i = 0; i < length; i++) {
        buffer[i] = cv_read_byte(d->colecovision_context, address + i);
    }
    dump_buffer_hex(4, address, buffer, length);
    return false;
}

bool debugger_symbols(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "symbols: expected filename argument\n");
        return false;
    }
    d->load_symbols(argv[1]);
    return false;
}

bool debugger_fill(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 4) {
        fprintf(stderr, "fill: expected address, length, and value\n");
        return false;
    }
    uint16_t address;
    if(!lookup_or_parse(d->symbol_to_address, argv[1], address)) {
        return false;
    }

    char *endptr;
    int length = strtol(argv[2], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[2]);
        return false;
    }
    uint8_t value = strtol(argv[3], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[3]);
        return false;
    }
    printf("fill %d for %d with %d\n", address, length, value);
    for(int i = 0; i < length; i++) {
        cv_write_byte(d->colecovision_context, address + i, value);
    }
    return false;
}

bool debugger_image(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    using namespace TMS9918A;
    FILE *fp = fopen("output.ppm", "wb");

    auto& vdp = d->colecohw->vdp;

    // XXX
    uint8_t framebuffer[SCREEN_X * SCREEN_Y * 3];
    auto pixel_setter = [&framebuffer](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        uint8_t *pixel = framebuffer + 3 * (x + y * SCREEN_X) + 0;
        SetColor(pixel, r, g, b);
    };
    std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
    CreateImageAndReturnFlags(vdp.registers.data(), vdp.memory.data(), pixel_setter);
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = now - start_time;
    if(false) printf("dump time %f seconds\n", elapsed.count());
    write_rgb8_image_as_P6(framebuffer, SCREEN_X, SCREEN_Y, fp);
    fclose(fp);

    fp = fopen("vdp_memory.txt", "w");
    fprintf(fp, "%02X %02X %02X %02X %02X %02X %02X %02X\n",
        vdp.registers[0], vdp.registers[1], vdp.registers[2], vdp.registers[3],
        vdp.registers[4], vdp.registers[5], vdp.registers[6], vdp.registers[7]);
    for(int j = 0; j < 64; j++) {
        for(int i = 0; i < 256; i++) {
            fprintf(fp, "%02X ", vdp.memory[j * 256 + i]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);

    return false;
}

bool debugger_in(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "out: expected port number\n");
        return false;
    }
    char *endptr;
    uint8_t port = strtol(argv[1], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[1]);
        return false;
    }
    uint8_t byte = cv_input_byte(d->colecovision_context, port, byte);
    printf("received byte 0x%02X from port %d (0x%02X)\n", byte, port, port);
    return false;
}

bool debugger_out(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 3) {
        fprintf(stderr, "out: expected port number and byte\n");
        return false;
    }
    char *endptr;
    uint8_t port = strtol(argv[1], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[1]);
        return false;
    }
    uint8_t value = strtol(argv[2], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[2]);
        return false;
    }
    cv_output_byte(d->colecovision_context, port, value);
    return false;
}

bool debugger_help(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    printf("Debugger commands:\n");
    printf("    go                    - continue normally\n");
    printf("    dump addr count       - dump count bytes at addr\n");
    printf("    fill addr count byte  - fill count bytes with byte at addr\n");
    printf("    readhex file.hex      - read file.hex into memory\n");
    printf("    readbin addr file.bin - read file.bin into memory at addr\n");
    printf("    symbols file.prn      - read symbols from file\n");
    printf("    step [N]              - step [for N instructions]\n");
    printf("    watch addr            - break out of step if addr changes\n");
    printf("    watchio addr          - break out of step if addr is IO read or write\n");
    printf("    break addr            - break into debugger at addr\n");
    printf("    disable N             - disable breakpoint N\n");
    printf("    enable N              - disable breakpoint N\n");
    printf("    remove N              - remove breakpoint N\n");
    printf("    list                  - list breakpoints and catchpoints\n");
    printf("    jump addr             - jump to addr \n");
    printf("    pc addr               - set PC to addr (in anticipation of \"step\")\n");
    printf("    in port               - input byte from port and print it\n");
    printf("    out port byte         - output byte to port\n");
    printf("    help                  - print this help message\n");
    printf("    ?                     - print this help message\n");
    printf("    dis addr count        - disassemble count instructions at addr\n");
    printf("    quit, exit, ^D        - exit the debugger\n");
    return false;
}

bool debugger_continue(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    d->state_may_have_changed = true;
    return true;
}

bool brads_zero_check = true;

bool debugger_step(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    int count = 1;
    bool verbose = false;

    if((argc > 1) && (strcmp(argv[1], "-v") == 0)) {
        verbose = true;
        argc--;
        argv++;
    }
    if(argc > 1) {
        char *endptr;
        count = strtol(argv[1], &endptr, 0);
        if(*endptr != '\0') {
            printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[1]);
            return false;
        }
    }
    for(int i = 0; i < count; i++) {
        d->clk += Z80Emulate(state, 1, d->colecovision_context);
        if(i < count - 1) {
            if(verbose) {
                print_state(state);
                disassemble(state->pc, d, 1, d->colecovision_context);
            }
        }
        if(d->should_debug(state)) {
            break;
        }
    }
    printf("%llu actual cycles emulated\n", d->clk);
    d->state_may_have_changed = true;
    d->last_was_step = true;
    return false;
}

bool debugger_jump(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "jump: expected address\n");
        return false;
    }

    uint16_t address;
    if(!lookup_or_parse(d->symbol_to_address, argv[1], address)) {
        return false;
    }
    state->pc = address;

    d->state_may_have_changed = true;
    d->last_was_jump = true;
    return true;
}

bool debugger_pc(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "pc: expected address\n");
        return false;
    }
    uint16_t address;
    if(!lookup_or_parse(d->symbol_to_address, argv[1], address)) {
        return false;
    }
    state->pc = address;

    d->state_may_have_changed = true;
    return false;
}

bool debugger_quit(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    quit_requested = true;
    return true;
}

bool debugger_break(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "break: expected address\n");
        return false;
    }

    uint16_t address;
    if(!lookup_or_parse(d->symbol_to_address, argv[1], address)) {
        return false;
    }

    d->breakpoints.push_back(BreakPoint(address));
    return false;
}

bool debugger_watch(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "watch: expected address\n");
        return false;
    }

    uint16_t address;
    if(!lookup_or_parse(d->symbol_to_address, argv[1], address)) {
        return false;
    }

    uint8_t old_value;
    old_value = cv_read_byte(d->colecovision_context, address);
    d->breakpoints.push_back(BreakPoint(address, old_value));
    return false;
}

bool debugger_watchio(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "watchio: expected address\n");
        return false;
    }

    uint16_t address;
    if(!lookup_or_parse(d->symbol_to_address, argv[1], address)) {
        return false;
    }

    if(d->io_watch.count(address) > 0) {
        fprintf(stderr, "watchio: removing watch on 0x%X\n", address);
        d->io_watch.erase(address);
    } else {
        fprintf(stderr, "watchio: adding watch on 0x%X\n", address);
        d->io_watch.insert(address);
    }
    return false;
}

bool debugger_disable(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "break: expected address\n");
        return false;
    }
    char *endptr;
    int i = strtol(argv[1], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[1]);
        return false;
    }
    if(i < 0 || (size_t)i >= d->breakpoints.size()) {
        fprintf(stderr, "breakpoint %d is out of range\n", i);
        return false;
    }
    d->breakpoints[i].disable();
    return false;
}

bool debugger_enable(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "break: expected address\n");
        return false;
    }
    char *endptr;
    int i = strtol(argv[1], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[1]);
        return false;
    }
    if(i < 0 || (size_t)i >= d->breakpoints.size()) {
        fprintf(stderr, "breakpoint %d is out of range\n", i);
        return false;
    }
    d->breakpoints[i].enable();
    return false;
}

bool debugger_remove(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    if(argc != 2) {
        fprintf(stderr, "break: expected address\n");
        return false;
    }
    char *endptr;
    int i = strtol(argv[1], &endptr, 0);
    if(*endptr != '\0') {
        printf("number parsing failed for %s; forgot to lead with 0x?\n", argv[1]);
        return false;
    }
    if(i < 0 || (size_t)i >= d->breakpoints.size()) {
        fprintf(stderr, "breakpoint %d is out of range\n", i);
        return false;
    }
    d->breakpoints.erase(d->breakpoints.begin() + i);
    return false;
}

bool debugger_list(Debugger *d, Z80_STATE* state, int argc, char **argv)
{
    printf("breakpoints:\n");
    for(auto i = d->breakpoints.begin(); i != d->breakpoints.end(); i++) {
        BreakPoint& bp = (*i);
        printf("%ld : ", i - d->breakpoints.begin());
        printf("%s ", bp.enabled ? " enabled" : "disabled");
        printf("%s ", (bp.type == BreakPoint::INSTRUCTION) ? " ins" : "data");
        if(bp.type == BreakPoint::INSTRUCTION) {
            uint16_t symbol_offset;
            const std::string& sym = d->get_symbol(bp.address, symbol_offset);
            printf("break at 0x%04x (%s+%d)\n", bp.address, sym.c_str(), symbol_offset);
        } else {
            printf("change at 0x%04X from 0x%02X\n", bp.address, bp.old_value);
        }
    }
    return false;
}

void populate_command_handlers()
{
    command_handlers["image"] = debugger_image;
    command_handlers["?"] = debugger_help;
    command_handlers["help"] = debugger_help;
    command_handlers["readhex"] = debugger_readhex;
    command_handlers["readbin"] = debugger_readbin;
    command_handlers["dump"] = debugger_dump;
    command_handlers["fill"] = debugger_fill;
    command_handlers["symbols"] = debugger_symbols;
    command_handlers["in"] = debugger_in;
    command_handlers["out"] = debugger_out;
    command_handlers["go"] = debugger_continue;
    command_handlers["g"] = debugger_continue;
    command_handlers["step"] = debugger_step;
    command_handlers["jump"] = debugger_jump;
    command_handlers["pc"] = debugger_pc;
    command_handlers["break"] = debugger_break;
    command_handlers["watch"] = debugger_watch;
    command_handlers["watchio"] = debugger_watchio;
    command_handlers["enable"] = debugger_enable;
    command_handlers["disable"] = debugger_disable;
    command_handlers["remove"] = debugger_remove;
    command_handlers["list"] = debugger_list;
    command_handlers["quit"] = debugger_quit;
    command_handlers["exit"] = debugger_quit;
    command_handlers["dis"] = debugger_dis;
        // reset
}

bool Debugger::process_command(Z80_STATE* state, char *command)
{
    // process commands
    char **ap, *argv[10];

    for (ap = argv; (*ap = strsep(&command, " \t")) != NULL;) {
        if (**ap != '\0') {
            if (++ap >= &argv[10]) {
                break;
            }
        }
    }
    int argc = ap - argv;

    if(argc == 0) {
        if(last_was_step) {
            return debugger_step(this, state, argc, argv);
        } else {
            return false;
        }
    }

    last_was_step = false;
    auto it = command_handlers.find(argv[0]);
    if(it == command_handlers.end()) {
        fprintf(stderr, "debugger command not defined: \"%s\"\n", argv[0]);
        return false;
    }
    
    return (*it).second(this, state, argc, argv);
}

bool Debugger::process_line(Z80_STATE* state, char *line)
{
    char *command;

    while((command = strsep(&line, ";")) != NULL) {
        bool run = process_command(state, command);
        if(run) {
            return true;
        }
    }
    return false;
}

bool Debugger::should_debug(Z80_STATE* state)
{
    int which;

    // XXX move?
    auto io_reads_tmp = d->colecohw->io_reads;
    auto io_writes_tmp = d->colecohw->io_writes;
    d->colecohw->io_reads.clear();
    d->colecohw->io_writes.clear();

    for(auto io_read: io_reads_tmp) {
        if(io_watch.count(io_read) > 0) {
            return true;
        }
    }
    for(auto [io_write_address, io_write_value]: io_writes_tmp) {
        if(io_watch.count(io_write_address) > 0) {
            return true;
        }
    }

    bool should = !last_was_jump && is_breakpoint_triggered(breakpoints, state, which, d->colecovision_context);

    last_was_jump = false;

    return should;
}

void mark_enter_debugger(int signal)
{
    enter_debugger = true;
}

void Debugger::go(FILE *fp, Z80_STATE* state)
{
    signal(SIGINT, previous_sigint);

    if(!feof(fp)) {
        bool run = false;
        do {
            if(state_may_have_changed) {
                state_may_have_changed = false;
                print_state(state);
                disassemble(state->pc, [this](uint16_t address, uint16_t& symbol_offset) -> const std::string& {return this->get_symbol(address, symbol_offset);}, 1, d->colecovision_context);
            }
            int which;
            if(is_breakpoint_triggered(breakpoints, state, which, d->colecovision_context))
            {
                printf("breakpoint %d: ", which);
                BreakPoint& bp = breakpoints[which];
                if(bp.type == BreakPoint::INSTRUCTION) {
                    uint16_t symbol_offset;
                    const std::string& sym = get_symbol(state->pc, symbol_offset);
                    printf("break at 0x%04x (%s+%d)\n", bp.address, sym.c_str(), symbol_offset);
                } else {
                    uint8_t new_value;
                    new_value = cv_read_byte(d->colecovision_context, bp.address);
                    printf("change at 0x%04X from 0x%02X to 0x%02X\n", bp.address, bp.old_value, new_value);
                }
                clear_breakpoints(breakpoints, state, d->colecovision_context);
            }
            if(fp == stdin) {
                char *line;
                line = readline("? ");
                if (line == NULL) {
                    printf("\n");
                    quit_requested = true;
                    run = true;
                } else {
                    if(strlen(line) > 0) {
                        add_history(line);
                    }
                    run = process_line(state, line);
                    free(line);
                }
            } else {
                char line[512];
                if(fgets(line, sizeof(line), fp) == NULL) {
                    break;
                }
                line[strlen(line) - 1] = '\0';
                run = process_line(state, line);
            }

        } while(!run);
    }

    previous_sigint = signal(SIGINT, mark_enter_debugger);
    state_may_have_changed = true;
}

#else /* ! PROVIDE_DEBUGGER */

struct Debugger
{
    Debugger(ColecoHW *colecohw, ColecovisionContext *colecovision_context, clk_t& clk) {}
};

#endif /* PROVIDE_DEBUGGER */

volatile bool run_fast = false;
volatile bool pause_cpu = false;

void usage(char *progname)
{
    printf("\n");
    printf("usage: %s [options] bios.bin cartridge.bin\n", progname);
    printf("\n");
    printf("options:\n");
#ifdef PROVIDE_DEBUGGER
    printf("\t--debugger init          Invoke debugger on startup\n");
#endif
    printf("\t                        \"init\" can be commands (separated by \";\"\n");
    printf("\t                        or a filename.  The initial commands can be\n");
    printf("\t                        the empty string.\n");
    printf("\n");
}

void do_vdp_test(const char *vdp_dump_name, const char *image_name)
{
    using namespace TMS9918A;
    std::array<uint8_t, 8> registers;
    std::array<uint8_t, 16384> memory;

    FILE *vdp_dump_in = fopen(vdp_dump_name, "r");
    char line[512];
    fgets(line, sizeof(line), vdp_dump_in);
    for(size_t i = 0; i < 8; i++) {
        int v;
        fscanf(vdp_dump_in, " %u", &v);
        registers[i] = v;
    }
    for(size_t i = 0; i < 16384; i++) {
        int v;
        fscanf(vdp_dump_in, " %u", &v);
        memory[i] = v;
    }
    fclose(vdp_dump_in);

    uint8_t framebuffer[SCREEN_X * SCREEN_Y * 3];
    auto pixel_setter = [&framebuffer](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        uint8_t *pixel = framebuffer + 3 * (x + y * SCREEN_X) + 0;
        SetColor(pixel, r, g, b);
    };
    CreateImageAndReturnFlags(registers.data(), memory.data(), pixel_setter);
    FILE *fp = fopen(image_name, "wb");
    write_rgb8_image_as_P6(framebuffer, SCREEN_X, SCREEN_Y, fp);
    fclose(fp);
}

void WriteVDPStateToFile(const char *base, int which, const uint8_t* registers, const uint8_t *memory, FILE *vdp_file)
{
    fprintf(vdp_file, "# %s_%02d.vdp, 8 register bytes, 16384 RAM bytes\n", base, which);
    for(size_t i = 0; i < 8; i++) {
        bool more_in_row = ((i + 1) % 8) != 0;
        fprintf(vdp_file, "%u%s", registers[i], more_in_row ? " " : "\n");
    }
    fputs("", vdp_file);
    for(size_t i = 0; i < 16384; i++) {
        bool more_in_row = ((i + 1) % 16) != 0;
        fprintf(vdp_file, "%u%s", memory[i], more_in_row ? " " : "\n");
    }
    fputs("", vdp_file);
}

void SaveVDPState(const TMS9918AEmulator *vdp, int which)
{
    char filename[512];
    sprintf(filename, "%s_%02d.vdp", getenv("VDP_OUT_BASE"), which);
    FILE *vdp_file = fopen(filename, "w");
    WriteVDPStateToFile(getenv("VDP_OUT_BASE"), which, vdp->registers.data(), vdp->memory.data(), vdp_file);
    fclose(vdp_file);
}

uint8_t GetPlatformControllerState(int index, bool JoystickNotKeypad)
{
    using namespace PlatformInterface;
    ControllerIndex controller = (index == 0) ? PlatformInterface::CONTROLLER_1 : PlatformInterface::CONTROLLER_2;
    if(JoystickNotKeypad) {
        return GetJoystickState(controller);
    } else {
        return GetKeypadState(controller);
    }
}

struct ControllerEvent
{
    clk_t clk;
    bool JoystickNotKeypad;
    int index;
    uint8_t bits_set;
    uint8_t bits_cleared;
};

void set_colecovision_context(ColecovisionContext *colecovision_context, RAMboard& RAM, ROMboard& BIOS, ROMboard& cartridge, ColecoHW* colecohw)
{
    colecovision_context->RAM = RAM.bytes.data();

    colecovision_context->BIOS.start = BIOS.base;
    colecovision_context->BIOS.length = BIOS.length;
    colecovision_context->BIOS.bytes = BIOS.bytes.data();

    colecovision_context->cartridge.start = cartridge.base;
    colecovision_context->cartridge.length = cartridge.length;
    colecovision_context->cartridge.bytes = cartridge.bytes.data();

    colecovision_context->cvhw = colecohw;
}


#if defined(ROSA)

extern "C" {
void HAL_Delay(uint32_t millis);
int coleco_main(int argc, char **argv);
void main_iterate(void);
uint32_t HAL_GetTick();
};

#define main coleco_main

#endif

static void sleep_for(int32_t millis)
{
    if(millis < 0) {
        return;
    }
#ifdef ROSA
    HAL_Delay(millis);
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(millis));
#endif
}

extern "C" {

uint8_t cv_in_byte(void* ctx_, uint16_t address16)
{
    auto *context = reinterpret_cast<ColecovisionContext*>(ctx_);
    auto* cvhw = reinterpret_cast<ColecoHW*>(context->cvhw);

    uint8_t b = 0;
    bool served = cvhw->io_read(address16 & 0xff, b);
    if(!served) {
        printf("IN %d (0x%02X) was not handled!\n", address16, address16);
    }
    return b;
}

void cv_out_byte(void* ctx_, uint16_t address16, uint8_t value)
{
    auto *context = reinterpret_cast<ColecovisionContext*>(ctx_);
    auto* cvhw = reinterpret_cast<ColecoHW*>(context->cvhw);

    bool accepted = cvhw->io_write(address16 & 0xff, value);
    if(!accepted) {
        printf("OUT %d (0x%02X), 0x%02X was not handled!\n", address16, address16, value);
    }
}

}; // extern "C" 

}; // namespace ColecovisionEmulator


int main(int argc, char **argv)
{
    using namespace PlatformInterface;
    using namespace ColecovisionEmulator;
    using namespace std::chrono_literals;
#ifdef PROVIDE_DEBUGGER
    bool do_debugger = false;
    char *debugger_argument = NULL;

    populate_command_handlers();
#endif
#ifdef ENABLE_AUTOMATION
    bool record_controllers = false;
    std::string record_controller_filename;
    bool playback_controllers = false;
    std::string playback_controller_filename;
#endif

    char *progname = argv[0];
    argc -= 1;
    argv += 1;

    while((argc > 0) && (argv[0][0] == '-')) {
	if(
            (strcmp(argv[0], "-help") == 0) ||
            (strcmp(argv[0], "--help") == 0) ||
            (strcmp(argv[0], "-h") == 0) ||
            (strcmp(argv[0], "-?") == 0))
         {
             usage(progname);
             exit(EXIT_SUCCESS);
	} else if(strcmp(argv[0], "--vdp-test") == 0) {
            if(argc < 3) {
                fprintf(stderr, "-vdp-test requires VDP register dump filename and output image filename\n");
                usage(progname);
                exit(EXIT_FAILURE);
            }
            do_vdp_test(argv[1], argv[2]);
            exit(0);
        }

#ifdef ENABLE_AUTOMATION
	else if(strcmp(argv[0], "--record-controllers") == 0) {

            if(argc < 2) {
                fprintf(stderr, "--record-controllers requires filename to which to record controller state\n");
                usage(progname);
                exit(EXIT_FAILURE);
            }
            if(playback_controllers) {
                fprintf(stderr, "only one of --record-controllers or --playback-controllers can be provided.\n");
                usage(progname);
                exit(EXIT_FAILURE);
            }

            record_controllers = true;
            record_controller_filename = argv[1];
            argv += 2;
            argc -= 2;

	} else if(strcmp(argv[0], "--playback-controllers") == 0) {

            if(argc < 2) {
                fprintf(stderr, "--playback-controllers requires filename from which to playback controller state\n");
                usage(progname);
                exit(EXIT_FAILURE);
            }
            if(record_controllers) {
                fprintf(stderr, "only one of --record-controllers or --playback-controllers can be provided.\n");
                usage(progname);
                exit(EXIT_FAILURE);
            }

            playback_controllers = true;
            playback_controller_filename = argv[1];
            argv += 2;
            argc -= 2;
        }
#endif

	else if(strcmp(argv[0], "--debugger") == 0) {
#ifdef PROVIDE_DEBUGGER
            if(argc < 2) {
                fprintf(stderr, "--debugger requires initial commands (can be empty, e.g. \"\"\n");
                usage(progname);
                exit(EXIT_FAILURE);
            }
            do_debugger = true;
            debugger_argument = argv[1];
	    argc -= 2;
	    argv += 2;
#else
            if(argc < 2) {
                fprintf(stderr, "The debugger is not enabled in this build\n");
                exit(EXIT_FAILURE);
            }
#endif
        }

	else {
	    fprintf(stderr, "unknown parameter \"%s\"\n", argv[0]);
            usage(progname);
	    exit(EXIT_FAILURE);
	}
    }

    if(argc < 2) {
        usage(progname);
        exit(EXIT_FAILURE);
    }

    uint32_t audioSampleRate;
    size_t preferredAudioBufferSampleCount;
    PlatformInterface::Start(audioSampleRate, preferredAudioBufferSampleCount);

    uint8_t rom_temp[32768];
    FILE *fp;

    char *bios_name = argv[0];
    char *cart_name = argv[1];

    fp = fopen(bios_name, "rb");
    if(fp == NULL) {
        fprintf(stderr, "failed to open %s for reading\n", bios_name);
        exit(EXIT_FAILURE);
    }
    size_t bios_length = fread(rom_temp, 1, sizeof(rom_temp), fp);
    if(bios_length != 0x2000) {
        fprintf(stderr, "ROM read from %s was unexpectedly %zd bytes\n", bios_name, bios_length);
        exit(EXIT_FAILURE);
    }
    fclose(fp);
    ROMboard bios_rom(0, bios_length, rom_temp);

    audio_flush_func audio_flush = [](uint8_t *buf, size_t sz){ PlatformInterface::EnqueueAudioSamples(buf, sz); };

    tms9918_scanout_func platform_scanout = [](const uint8_t *registers, const uint8_t *memory)->uint8_t {
        uint8_t status_result;
        PlatformInterface::Frame(registers, memory, status_result, 3.579f);
        return status_result;
    };

    fp = fopen(cart_name, "rb");
    if(fp == NULL) {
        fprintf(stderr, "failed to open %s for reading\n", cart_name);
        exit(EXIT_FAILURE);
    }
    size_t cart_length = fread(rom_temp, 1, sizeof(rom_temp), fp);
    if(cart_length < 0x2000) {
        fprintf(stderr, "ROM read from %s was unexpectedly short (%zd bytes)\n", cart_name, cart_length);
        exit(EXIT_FAILURE);
    }
    fclose(fp);

    ROMboard cart_rom(0x8000, cart_length, rom_temp);
    RAMboard RAM(RAM_START, RAM_ADDRESS_MASK + 1, RAM_LENGTH);

    clk_t clk = 0;

#ifdef ENABLE_AUTOMATION

    FILE *recording_output = nullptr;
    if(record_controllers) {
        recording_output = fopen(record_controller_filename.c_str(), "w");
        if(recording_output == NULL) {
            fprintf(stderr, "couldn't open %s to write controller data\n", record_controller_filename.c_str());
            exit(1);
        }
    }

    std::deque<ControllerEvent> playback_events;
    if(playback_controllers)
    {
        FILE *playback_input = fopen(playback_controller_filename.c_str(), "r");
        if(playback_input == NULL) {
            fprintf(stderr, "couldn't open %s to read controller data\n", playback_controller_filename.c_str());
            exit(1);
        }
        ControllerEvent event;
        char kind;
        int set, cleared;
        while(fscanf(playback_input, "%llu %c %d %d %d ", &event.clk, &kind, &event.index, &set, &cleared) == 5)
        {
            event.JoystickNotKeypad = (kind == 'j');
            event.bits_set = set;
            event.bits_cleared = cleared;
            playback_events.push_back(event);
        }
        fclose(playback_input);
        printf("Will play back %zd events\n", playback_events.size());

        // Run for a few more seconds at the end of playback
        {
            ControllerEvent event{
                .clk = ((playback_events.size() > 0) ? playback_events.back().clk : 0) + machine_clock_rate * 2,
                .JoystickNotKeypad = true,
                .index = 0,
                .bits_set = 0,
                .bits_cleared = 0,
            };
            playback_events.push_back(event);
        }
    }

    uint8_t joystick_state[2] = {127, 127};
    uint8_t keypad_state[2] = {127, 127};

    auto get_controller_state = [&clk, &joystick_state, &keypad_state, recording_output, record_controllers, playback_controllers, &playback_events](int index, bool JoystickNotKeypad) -> uint8_t {
        uint8_t previous = JoystickNotKeypad ? joystick_state[index] : keypad_state[index];
        uint8_t current = GetPlatformControllerState(index, JoystickNotKeypad);
        if(record_controllers && (current != previous)) {
            uint8_t difference = previous ^ current;
            uint8_t bits_set = difference & current;
            uint8_t bits_cleared = difference & previous;
            if(record_controllers) {
                assert(recording_output);
                fprintf(recording_output, "%llu %c %d %d %d\n", clk, JoystickNotKeypad ? 'j' : 'k', index, bits_set, bits_cleared);
            }
        }
        if(playback_controllers) {
            if(playback_events.size() == 0) {
                printf("playback ending\n");
                exit(0);
            }
            current = previous;
            const ControllerEvent& next = playback_events.front();
            if((clk >= next.clk) && (index == next.index) && (JoystickNotKeypad == next.JoystickNotKeypad)) {
                current = (previous | next.bits_set) & ~next.bits_cleared;
                printf("playback matched %llu at %llu, state was %d and became %d\n", next.clk, clk, previous, current);
                playback_events.pop_front();
            }
        }
        (JoystickNotKeypad ? joystick_state[index] : keypad_state[index]) = current;
        return current;
    };

#else

    auto get_controller_state = [](int index, bool JoystickNotKeypad) -> uint8_t {
        return GetPlatformControllerState(index, JoystickNotKeypad);
    };

#endif

    ColecoHW* colecohw = new ColecoHW(audioSampleRate, preferredAudioBufferSampleCount, get_controller_state);
    bool save_vdp = false;

    [[maybe_unused]] Debugger *debugger = NULL;
#ifdef PROVIDE_DEBUGGER
    if(do_debugger) {
        debugger = new Debugger(colecohw, &colecovision_context, clk);
    }
#endif

    Z80Reset(&z80state);

    ColecovisionContext *colecovision_context = new ColecovisionContext;
    set_colecovision_context(colecovision_context, RAM, bios_rom, cart_rom, colecohw);

#ifdef PROVIDE_DEBUGGER
    if(debugger) {
        enter_debugger = true;
        debugger->process_line(&z80state, debugger_argument);
    }
#endif

    bool nmi_was_issued = false;
    clk_t previous_field_start_clock = clk;
    std::chrono::time_point<std::chrono::system_clock> emulation_start_time = std::chrono::system_clock::now();
    uint32_t prevTick;
#if defined(ROSA)
    prevTick = HAL_GetTick();
#endif

    PlatformInterface::MainLoopBodyFunc main_loop_body = [colecovision_context, &clk, debugger, colecohw, &nmi_was_issued, &save_vdp, audio_flush, platform_scanout, &previous_field_start_clock, &emulation_start_time, &prevTick]() {
        (void)debugger; // If !PROVIDE_DEBUGGER then debugger is not referenced.
        (void)prevTick; // If !ROSA then prevTick is not referenced. // XXX move iterate call to platform main loop

#ifdef PROVIDE_DEBUGGER
        if(debugger && (enter_debugger || debugger->should_debug(&z80state))) {
            debugger->go(stdin, &z80state);
            enter_debugger = false;
        } else
#endif
        {
            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
            auto micros_since_start = std::chrono::duration_cast<std::chrono::microseconds>(now - emulation_start_time);
            clk_t clock_now = machine_clock_rate * micros_since_start.count() / 1000000;
            if(clock_now < clk) {
                /* if we get ahead somehow, sleep a little to fall back */
                sleep_for(2); // 1ms);
                return quit_requested;
            }
            clk_t target_clock = std::max(clk + 10000 /* machine_clock_rate / 1000 */ , clock_now);
            target_clock = clk + machine_clock_rate / 120;
            if(false) printf("was at %llu, need to be at %llu, need %llu (%.2f ms), will run %llu (%.2f ms)\n", clk, clock_now, clock_now - clk, (clock_now - clk) * 1000.0f / machine_clock_rate, target_clock - clk, (target_clock - clk) * 1000.0f / machine_clock_rate);

            // XXX THIS HAS TO REMAIN 1 UNTIL I CAN ISSUE NonMaskableInterrupt PER-INSTRUCTION
            static constexpr uint32_t iterated_clock_quantum = 1; // 1;

#if defined(ROSA)
            // RoDebugOverlayPrintf("%ld\n", (int)(target_clock - clk));
#endif /* ROSA */

            while(clk < target_clock) {
                std::string dummy;
                {
                    static int which = 0;
                    if(false && (which++ > 832600))
                    {
                        // print_state(&z80state);
                        disassemble(z80state.pc, [&dummy](uint16_t address, uint16_t& symbol_offset)->std::string&{symbol_offset = 0; return dummy;}, 1, colecovision_context);
                    }
                }
                uint8_t byte = cv_read_byte(colecovision_context, z80state.pc);
                bool is_HALT = byte == 0x76;
                // printf("PC is %04X\n", z80state.pc);
                clk_t clocks_this_step = Z80Emulate(&z80state, iterated_clock_quantum, colecovision_context);
                if(false && is_HALT) {
                    printf("VDP status register = %02X\n", colecohw->vdp.status_register);
                    printf("VDP interrupts %s\n", TMS9918A::InterruptsAreEnabled(colecohw->vdp.registers.data()) ? "enabled" : "disabled");
                    printf("VDP interrupt has %soccurred\n", TMS9918A::VSyncInterruptHasOccurred(colecohw->vdp.status_register) ? "" : "not ");
                    printf("im = %d\n", z80state.im);
                    printf("HALT took %llu clocks\n", clocks_this_step);
                }
#ifdef PROVIDE_DEBUGGER
                if(debugger) {
                    if(enter_debugger || debugger->should_debug(&z80state)) {
                        debugger->go(stdin, &z80state);
                        enter_debugger = false;
                    }
                }
#endif
                clk += clocks_this_step;

                uint64_t retrace_before = previous_field_start_clock / clocks_per_retrace;
                uint64_t retrace_after = clk / clocks_per_retrace;
                if(retrace_before != retrace_after) {
                    colecohw->vdp.perform_scanout(platform_scanout);
                    if(save_vdp) {
                        static int which = 0;
                        SaveVDPState(&colecohw->vdp, which++);
                        save_vdp = false;
                    }

                    colecohw->vdp.vsync();
                    colecohw->fill_flush_audio(clk, audio_flush);
                    previous_field_start_clock = clk;
                }

                if(colecohw->nmi_required()) {
                    if(!nmi_was_issued) {
                        clk += Z80NonMaskableInterrupt (&z80state, colecovision_context);
                        nmi_was_issued = true;
                    }
                } else {
                    nmi_was_issued = false;
                }
            }

#if defined(ROSA)
            uint32_t nowTick = HAL_GetTick();
#warning Setting this to + 16 made USB keyboard stop working.  
            if(nowTick >= prevTick) {
                main_iterate();
                prevTick = nowTick;
            }
#endif
        }

        while(PlatformInterface::EventIsWaiting()) {
            PlatformInterface::Event e = PlatformInterface::DequeueEvent();
            if(e.type == PlatformInterface::QUIT) {
                quit_requested = true;
            } else if(e.type == PlatformInterface::RESET) {
                Z80Reset(&z80state);
                colecohw->reset();
            } else if(e.type == PlatformInterface::DUMP_SOME_AUDIO) {
                dump_some_audio = 100;
            } else if(e.type == PlatformInterface::SAVE_VDP_STATE) {
                save_vdp = true;
            } else if(e.type == PlatformInterface::DEBUG_VDP_WRITES) {
                do_save_images_on_vdp_write = !do_save_images_on_vdp_write;
            } else {
                printf("warning: unhandled platform event type %d\n", e.type);
            }
        }
        return quit_requested;

    };

    PlatformInterface::MainLoopAndShutdown(main_loop_body);

    return 0;
}

