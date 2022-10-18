#ifndef _ROCINANTE_H_
#define _ROCINANTE_H_

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define ROCINANTE 1

typedef enum Status {
    RO_SUCCESS = 0,

    RO_USER_DECLINED = 1,               // The user declined a UI prompt (not an error)
    RO_ITEMS_DISCARDED = 2,             // The user buffer was filled but more items were available

    RO_NO_VIDEO_SUBSYSTEM_SET = -1,        // The platform did not set the video subsystem
    RO_INVALID_VIDEO_MODE_NUMBER = -2,     // The index passed was not in the range of valid modes
    RO_INVALID_STRUCTURE_SIZE = -3,        // The "size" parameter did not match the size of the requested structure
    RO_INVALID_STRUCTURE_TYPE = -4,        // The "type" parameter did not match the size of the requested structure
    RO_VIDEO_MODE_DOES_NOT_MATCH = -5,     // The "type" parameter to VideoModeGetInfo did not match the requested mode
    RO_VIDEO_MODE_INFO_UNSUPPORTED = -6,   // The video subsystem does not support returning info on the requested mode
    RO_INVALID_WINDOW = -7,                // A window was not open or valid with the provided index
    RO_INVALID_PARAMETER_VALUE = -8,       // A passed parameter was outside the valid range
    RO_WINDOW_CREATION_FAILED = -9,        // Window could not be created because of memory allocation or possibly other reason
    RO_RESOURCE_EXHAUSTED = -10,           // There were no more objects of the requested type
    RO_SIZE_EXCEEDED = -11,                // Window could not be created because of memory allocation or possibly other reason
    RO_RESOURCE_NOT_FOUND = -12,           // Root resource e.g. "/" could not be found
} Status;

#define RO_FAILURE(status) ((status) < 0)

//----------------------------------------------------------------------------
// Debug Overlay

/*! Add a line to the debug overlay.  The actual formatting function is vsnprintf.
    \param fmt The printf-style format.
    \param ... Arguments used in the format.
*/
void RoDebugOverlayPrintf(const char *fmt, ...);

/*! Set a line in the debug overlay.
    \param line The line to set.
    \param str The buffer to copy.
    \param size The number of bytes to copy.
*/
void RoDebugOverlaySetLine(int line, const char *str, size_t size);

//----------------------------------------------------------------------------
// Audio

/*! Get stereo audio stream info.
    \param rate Populated with the sampling rate (samples per second).
    \param bufferLength Populated with the number of 2-byte (one byte each left and right) unsigned (0-255) samples in the buffer.
*/
void RoAudioGetSamplingInfo(float *rate, size_t *chunkSize);

/*! Write audio samples.  Block until the write won't overlap the current audio read cursor
    \param writeSize Size of buffer in bytes.
    \param buffer The buffer of stereo u8 samples to write into the audio stream buffer.
    \return The number of bytes that had to be played before writing
*/
size_t RoAudioEnqueueSamplesBlocking(size_t writeSize /* in bytes */, uint8_t* buffer);

/*! Clear the audio stream to silence
*/
void RoAudioClear();

//----------------------------------------------------------------------------
// Controllers; joysticks, keypads

typedef enum RoControllerIndex { CONTROLLER_1, CONTROLLER_2 } RoControllerIndex;

uint8_t RoGetJoystickState(RoControllerIndex which);
uint8_t RoGetKeypadState(RoControllerIndex which);

//----------------------------------------------------------------------------
// DAC

#define DAC_VALUE_LIMIT 0xFF

#define MAX_DAC_VOLTAGE 1.32f
#define MAX_DAC_VOLTAGE_F16 (132 * 65536 / 100)

#define INLINE inline

INLINE unsigned char RoDACVoltageToValue(float voltage)
{
    if(voltage < 0.0f) {
        return 0x0;
    }
    uint32_t value = (uint32_t)(voltage / MAX_DAC_VOLTAGE * 255);
    if(value >= DAC_VALUE_LIMIT) {
        return DAC_VALUE_LIMIT;
    }
    return value;
}

INLINE unsigned char RoDACVoltageToValueNoBounds(float voltage)
{
    return (uint32_t)(voltage / MAX_DAC_VOLTAGE * 255);
}

INLINE int RoDACVoltageToValueFixed16NoBounds(int voltage)
{
    return (uint32_t)(voltage * 65535 / MAX_DAC_VOLTAGE_F16) * 256;
}


//----------------------------------------------------------------------------
// NTSC timing and voltage levels

#define NTSC_COLORBURST_FREQUENCY       3579545

// Number of samples we target; if we're doing 4x colorburst at 228 cycles, that's 912 samples at 14.318180MHz

#define ROW_SAMPLES        912
#define NTSC_EQPULSE_LINES	3
#define NTSC_VSYNC_LINES	3
#define NTSC_VBLANK_LINES	11
#define NTSC_FRAME_LINES	525

/* these are in units of one scanline */
#define NTSC_EQ_PULSE_INTERVAL	.04
#define NTSC_VSYNC_BLANK_INTERVAL	.43
#define NTSC_HOR_SYNC_DUR	.075
#define NTSC_FRONTPORCH		.02
/* BACKPORCH including COLORBURST */
#define NTSC_BACKPORCH		.075

#define NTSC_COLORBURST_CYCLES  9

#define NTSC_FRAMES		(59.94 / 2)

#define NTSC_SYNC_TIP_VOLTAGE   0.0f
#define NTSC_SYNC_PORCH_VOLTAGE   .285f
#define NTSC_SYNC_BLACK_VOLTAGE   .339f
#define NTSC_SYNC_WHITE_VOLTAGE   1.0f  /* VCR had .912v */

#define NTSC_SYNC_BLACK_VOLTAGE_F16   22217
#define NTSC_SYNC_WHITE_VOLTAGE_F16   65536

typedef uint32_t ntsc_wave_t;

INLINE unsigned char RoNTSCYIQToDAC(float y, float i, float q, float tcycles)
{
// This is transcribed from the NTSC spec, double-checked.
    float w_t = tcycles * M_PI * 2;
    float sine = sinf(w_t + 33.0f / 180.0f * M_PI);
    float cosine = cosf(w_t + 33.0f / 180.0f * M_PI);
    float signal = y + q * sine + i * cosine;
// end of transcription

    return RoDACVoltageToValue(NTSC_SYNC_BLACK_VOLTAGE + signal * (NTSC_SYNC_WHITE_VOLTAGE - NTSC_SYNC_BLACK_VOLTAGE));
}

INLINE unsigned char RoNTSCYIQDegreesToDAC(float y, float i, float q, int degrees)
{
    float sine, cosine;
    if(degrees == 0) {
        sine = 0.544638f;
        cosine = 0.838670f;
    } else if(degrees == 90) {
        sine = 0.838670f;
        cosine = -0.544638f;
    } else if(degrees == 180) {
        sine = -0.544638f;
        cosine = -0.838670f;
    } else if(degrees == 270) {
        sine = -0.838670f;
        cosine = 0.544638f;
    } else {
        sine = 0;
        cosine = 0;
    }
    float signal = y + q * sine + i * cosine;

    return RoDACVoltageToValueNoBounds(NTSC_SYNC_BLACK_VOLTAGE + signal * (NTSC_SYNC_WHITE_VOLTAGE - NTSC_SYNC_BLACK_VOLTAGE));
}

INLINE ntsc_wave_t RoNTSCYIQToWave(float y, float i, float q)
{
    unsigned char b0 = RoNTSCYIQToDAC(y, i, q,  .0f);
    unsigned char b1 = RoNTSCYIQToDAC(y, i, q, .25f);
    unsigned char b2 = RoNTSCYIQToDAC(y, i, q, .50f);
    unsigned char b3 = RoNTSCYIQToDAC(y, i, q, .75f);

    return (b0 << 0) | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

// This is transcribed from the NTSC spec, double-checked.
INLINE void RoRGBToYIQ(float r, float g, float b, float *y, float *i, float *q)
{
    *y = .30f * r + .59f * g + .11f * b;
    *i = -.27f * (b - *y) + .74f * (r - *y);
    *q = .41f * (b - *y) + .48f * (r - *y);
}

// Alternatively, a 3x3 matrix transforming [r g b] to [y i q] is:
// (untested - computed from equation above)
// 0.300000 0.590000 0.110000
// 0.599000 -0.277300 -0.321700
// 0.213000 -0.525100 0.312100

// A 3x3 matrix transforming [y i q] back to [r g b] is:
// (untested - inverse of 3x3 matrix above)
// 1.000000 0.946882 0.623557
// 1.000000 -0.274788 -0.635691
// 1.000000 -1.108545 1.709007

// Using inverse 3x3 matrix above.  Tested numerically to be the inverse of RGBToYIQ
INLINE void RoYIQToRGB(float y, float i, float q, float *r, float *g, float *b)
{
    *r = 1.0f * y + .946882f * i + 0.623557f * q;
    *g = 1.000000f * y + -0.274788f * i + -0.635691f * q;
    *b = 1.000000f * y + -1.108545f * i + 1.709007f * q;
}

INLINE ntsc_wave_t RoNTSCRGBToWave(float r, float g, float b)
{
    float y, i, q;
    RoRGBToYIQ(r, g, b, &y, &i, &q);
    return RoNTSCYIQToWave(y, i, q);
}

typedef void (*RoNTSCModeFillRowBufferFunc)(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer);
typedef int (*RoNTSCModeNeedsColorburstFunc)();
void RoNTSCSetMode(int interlaced, RoNTSCModeFillRowBufferFunc fillBufferFunc, RoNTSCModeNeedsColorburstFunc needsColorBurstFunc);
void RoNTSCGetValueRange(unsigned char *black, unsigned char *white);

extern void RoNTSCWaitFrame(void);

void RoPanic(void);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _ROCINANTE_H_ */
