/* z80user.h
 * Add your code here to interface the emulated system with z80emu. See towards
 * the end of the file for an example for running zextest.
 *
 * Copyright (c) 2016, 2017 Lin Ke-Fong
 *
 * This code is free, do whatever you want with it.
 */

#ifndef __Z80USER_INCLUDED__
#define __Z80USER_INCLUDED__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Write the following macros for memory access and input/output on the Z80. 
 *
 * Z80_FETCH_BYTE() and Z80_FETCH_WORD() are used by the emulator to read the
 * code (opcode, constants, displacement, etc). The upper 16-bit of the address
 * parameters is undefined and must be reset to zero before actually reading 
 * memory (use & 0xffff). The value x read, must be an unsigned 8-bit or 16-bit
 * value in the endianness of the host processor. 
 *
 * Z80_READ_BYTE(), Z80_WRITE_BYTE(), Z80_READ_WORD(), and Z80_WRITE_WORD()
 * are used for general memory access. They obey the same rules as the code 
 * reading macros. The upper bits of the value x to write may be non-zero.
 * Z80_READ_WORD_INTERRUPT() and Z80_WRITE_WORD_INTERRUPT() are same as 
 * respectively Z80_READ_WORD() and Z80_WRITE_WORD(), except they are only used
 * for interrupt generation.
 * 
 * Z80_INPUT_BYTE() and Z80_OUTPUT_BYTE() are for input and output. The upper
 * bits of the port number to read or write are always zero. The input byte x 
 * must be an unsigned 8-bit value. The value x to write is an unsigned 8-bit 
 * with its upper bits zeroed.
 *
 * All macros have access to the following three variables:
 *
 *      state           Pointer to the current Z80_STATE. Because the 
 *			instruction is currently executing, its members may not
 *			be fully up to date, depending on when the macro is 
 *			called in the process. It is rather suggested to access 
 *			the state only when the emulator is stopped. 
 *
 *      elapsed_cycles  Number of cycles emulated. If needed, you may add wait 
 *			states to it for slow memory accesses. Because the 
 *			macros are called during the execution of the current 
 *			instruction, this number is only precise up to the 
 *			previous one.
 *
 *	context 	This is the (void *) context passed to the emulation 
 *			functions.
 *
 * Except for Z80_READ_WORD_INTERRUPT and Z80_WRITE_WORD_INTERRUPT, all macros 
 * also have access to: 
 *
 *      number_cycles   Number of cycles to emulate. After executing each
 *			instruction, the emulator checks if elapsed_cycles is
 *			greater or equal to number_cycles, and will stops if 
 *			so. Hence you may decrease or increase the value of 
 *			number_cycles to stop the emulation earlier or later.
 * 			In particular, if you set it to zero, the emulator will
 * 			stop after completion of the current instruction. 
 *
 *      registers       Current register decoding table, use it to determine if
 * 			the current instruction is prefixed. It points on:
 *                      
 *				state->dd_register_table for 0xdd prefixes; 
 *                      	state->fd_register_table for 0xfd prefixes;
 *				state->register_table otherwise.
 *
 *      pc              Current PC register (upper bits are undefined), points
 *                      on the opcode, the displacement or constant to read for
 *                      Z80_FETCH_BYTE() and Z80_FETCH_WORD(), or on the next
 *                      instruction otherwise.
 *
 * Except for Z80_FETCH_BYTE(), Z80_FETCH_WORD(), Z80_READ_WORD_INTERRUPT, and 
 * Z80_WRITE_WORD_INTERRUPT, all other macros can know which instruction is 
 * currently executing:
 *
 *      opcode          Opcode of the currently executing instruction.
 *
 *      instruction     Type of the currently executing instruction, see
 *                      instructions.h for a list.
 */

extern void cv_out_byte(void *context, uint16_t address, uint8_t data);
extern uint8_t cv_in_byte(void *context, uint16_t address);

#define RAM_START 0x6000
#define RAM_LENGTH 0x2000
#define RAM_ADDRESS_MASK 0x07FF

typedef struct Z80MemoryInfo
{
    uint16_t start;
    uint16_t length;
    uint8_t *bytes;
} Z80MemoryInfo;

typedef struct ColecovisionContext
{
    Z80MemoryInfo BIOS;
    Z80MemoryInfo cartridge;
    uint8_t* RAM;
    void* cvhw;
} ColecovisionContext;

static inline uint8_t cv_read_byte(void *ctx_, uint32_t address32)
{
    ColecovisionContext *ctx = (ColecovisionContext*)ctx_;
    uint16_t address = address32 & 0xFFFF;

    if(address >= ctx->cartridge.start && address < ctx->cartridge.start + ctx->cartridge.length) {
        return ctx->cartridge.bytes[address - ctx->cartridge.start];
    } else if(address >= ctx->BIOS.start && address < ctx->BIOS.start + ctx->BIOS.length) {
        return ctx->BIOS.bytes[address - ctx->BIOS.start];
    } else if(address >= RAM_START && address < RAM_START + RAM_LENGTH) {
        return ctx->RAM[(address - RAM_START) & RAM_ADDRESS_MASK];
    } else {
        return 0;
    }
}

#define Z80_READ_BYTE(address32, x) { (x) = cv_read_byte((context), (address32)); }

#define Z80_FETCH_BYTE(address32, x)		Z80_READ_BYTE((address32), (x))

static inline uint16_t cv_read_word(void *ctx_, uint32_t address32)
{
    ColecovisionContext *ctx = (ColecovisionContext*)ctx_;
    uint16_t address = (address32) & 0xFFFF;

    if(address >= ctx->cartridge.start && (address + 1) < ctx->cartridge.start + ctx->cartridge.length) {
        return ctx->cartridge.bytes[address - ctx->cartridge.start] |
                (ctx->cartridge.bytes[address + 1 - ctx->cartridge.start] << 8);
    } else if(address >= ctx->BIOS.start && (address + 1) < ctx->BIOS.start + ctx->BIOS.length) {
        return ctx->BIOS.bytes[address - ctx->BIOS.start] |
                (ctx->BIOS.bytes[address + 1 - ctx->BIOS.start] << 8);
    } else if(address >= RAM_START && (address + 1) < RAM_START + RAM_LENGTH) {
        return ctx->RAM[(address - RAM_START) & RAM_ADDRESS_MASK] |
                (ctx->RAM[(address + 1 - RAM_START) & RAM_ADDRESS_MASK] << 8);
    } else {
        return 0;
    }
}

#define Z80_READ_WORD(address32, x) { (x) = cv_read_word((context), (address32)); }

#define Z80_READ_WORD_INTERRUPT(address32, x)	Z80_READ_WORD((address32), (x))

#define Z80_FETCH_WORD(address32, x)		Z80_READ_WORD((address32), (x))

static inline void cv_write_byte(void *ctx_, uint32_t address32, uint8_t byte)
{
    ColecovisionContext *ctx = (ColecovisionContext*)ctx_;

    uint16_t address = (address32) & 0xFFFF;

    if(address >= RAM_START && address < RAM_START + RAM_LENGTH) {
        ctx->RAM[(address - RAM_START) & RAM_ADDRESS_MASK] = byte;
    }
}

#define Z80_WRITE_BYTE(address32, x) { cv_write_byte((context), (address32), (x)); }

static inline void cv_write_word(void *ctx_, uint32_t address32, uint16_t word)
{
    ColecovisionContext *ctx = (ColecovisionContext*)ctx_;
    uint16_t address = (address32) & 0xFFFF;

    if(address >= RAM_START && (address + 1) < RAM_START + RAM_LENGTH) {
        ctx->RAM[(address - RAM_START) & RAM_ADDRESS_MASK] = word & 0xff;
        ctx->RAM[(address + 1 - RAM_START) & RAM_ADDRESS_MASK] = (word >> 8) & 0xff;
    }
}

#define Z80_WRITE_WORD(address32, x) { cv_write_word((context), (address32), (x)); }

#define Z80_WRITE_WORD_INTERRUPT(address32, x)	Z80_WRITE_WORD((address32), (x))

#define Z80_INPUT_BYTE(port16, x) { (x) = cv_in_byte((context), (port16)); }

#define Z80_OUTPUT_BYTE(port16, x) { cv_out_byte((context), (port16), (x)); }

#ifdef __cplusplus
}
#endif

#endif
