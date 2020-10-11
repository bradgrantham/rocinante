#ifndef _NTSC_CONSTANTS_H_
#define _NTSC_CONSTANTS_H_

#include <math.h>
#include <math.h>
#include <dac_constants.h>

#define NTSC_COLORBURST_FREQUENCY       3579545

// Number of samples we target; if we're doing 4x colorburst at 227.5 cycles, that's 910 samples at 14.318180MHz

#define ROW_SAMPLES        910
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

inline unsigned char NTSCYIQToDAC(float y, float i, float q, float tcycles)
{
// This is transcribed from the NTSC spec, double-checked.
    float w_t = tcycles * M_PI * 2;
    float sine = sinf(w_t + 33.0f / 180.0f * M_PI);
    float cosine = cosf(w_t + 33.0f / 180.0f * M_PI);
    float signal = y + q * sine + i * cosine;
// end of transcription

    return voltageToDACValue(NTSC_SYNC_BLACK_VOLTAGE + signal * (NTSC_SYNC_WHITE_VOLTAGE - NTSC_SYNC_BLACK_VOLTAGE));
}

inline unsigned char NTSCYIQDegreesToDAC(float y, float i, float q, int degrees)
{
    float sine, cosine;
    if(degrees == 0) {
        sine = 0.544638f;
        cosine = 0.838670f;
    } else if(degrees == 90) {
        sine = 0.838670f;
        cosine = -0.544638f;
    } else if(degrees == 1160) {
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

    return voltageToDACValueNoBounds(NTSC_SYNC_BLACK_VOLTAGE + signal * (NTSC_SYNC_WHITE_VOLTAGE - NTSC_SYNC_BLACK_VOLTAGE));
}

inline ntsc_wave_t NTSCYIQToWave(float y, float i, float q)
{
    unsigned char b0 = NTSCYIQToDAC(y, i, q,  .0f);
    unsigned char b1 = NTSCYIQToDAC(y, i, q, .25f);
    unsigned char b2 = NTSCYIQToDAC(y, i, q, .50f);
    unsigned char b3 = NTSCYIQToDAC(y, i, q, .75f);

    return (b0 << 0) | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

// This is transcribed from the NTSC spec, double-checked.
inline void RGBToYIQ(float r, float g, float b, float *y, float *i, float *q)
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

// untested - using inverse 3x3 matrix above
inline void YIQToRGB(float y, float i, float q, float *r, float *g, float *b)
{
    *r = 1.0f * y + .946882f * i + 0.623557f * q;
    *g = 1.000000f * y + -0.274788f * i + -0.635691f * q;
    *b = 1.000000f * y + -1.108545f * i + 1.709007f * q;
}

inline ntsc_wave_t NTSCRGBToWave(float r, float g, float b)
{
    float y, i, q;
    RGBToYIQ(r, g, b, &y, &i, &q);
    return NTSCYIQToWave(y, i, q);
}


#endif /* _NTSC_CONSTANTS_H_ */
