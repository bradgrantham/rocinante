#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

const int samples = 910;
const float sampleFreq = 16111886.06593406593406593406;
const float sampleLength = 1.0 / sampleFreq;

unsigned char blackDACValue = 110;
unsigned char whiteDACValue = 200;
unsigned char maxDACValue = 255;

unsigned char videoDACValue(unsigned char luminance) { return blackDACValue + luminance * (whiteDACValue - blackDACValue) / 255; }

#define NTSC_COLORBURST_FREQUENCY 3579545

const float freq = NTSC_COLORBURST_FREQUENCY;
// a is current sample time; return 0 to 1
float func(float a)
{
    return cosf(a * M_PI * 2 * freq) / 2.0 + .5;
}

void RGBToYIQ(float r, float g, float b, float *y, float *i, float *q)
{
    *y = .30f * r + .59f * g + .11f * b;
    *i = -.27f * (b - *y) + .74f * (r - *y);
    *q = .41f * (b - *y) + .48f * (r - *y);
}

/* [0.0, 1.0) */
float colorBurstPhaseFraction(float phaseFraction)
{
    float value = sinf(phaseFraction * M_PI * 2.0f);
    // fprintf(stderr, "phase %f -> sinf %f\n", phaseFraction, value);
    return value;
}

/* phaseFraction is [0.0, 1.0) */
unsigned char colorToDACValue(float y, float i, float q, float phaseFraction)
{
    float signal = y + q * colorBurstPhaseFraction(phaseFraction + 33.0f / 360.0f) + i * colorBurstPhaseFraction(phaseFraction + (33.0f + 90.0f) / 360.0f);
    // fprintf(stderr, "signal %f\n", signal);
    return videoDACValue((unsigned char)(signal * 255));
}

uint32_t colorTo4Samples(float y, float i, float q)
{
    unsigned char b0 = colorToDACValue(y, i, q,  .0f);
    unsigned char b1 = colorToDACValue(y, i, q, .25f);
    unsigned char b2 = colorToDACValue(y, i, q, .50f);
    unsigned char b3 = colorToDACValue(y, i, q, .75f);

    // fprintf(stderr, "bytes %u %u %u %u\n", b0, b1, b2, b3);

    return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}


int main(int argc, char **argv)
{
    unsigned char rgbRow[194][3]; 

    uint32_t palettized[224][194]; 

    unsigned char palette[256][3];
    uint32_t paletteUint32[256];
    int paletteNext = 0;

    for(int y = 0; y < 224; y++) {
        fread(rgbRow, sizeof(rgbRow), 1, stdin);
        for(int x = 0; x < 194; x++) {
            unsigned char r = rgbRow[x][0];
            unsigned char g = rgbRow[x][1];
            unsigned char b = rgbRow[x][2];
            int c;
            for(c = 0; c < paletteNext; c++) {
                if(palette[c][0] == r && palette[c][1] == g && palette[c][2] == b) {
                    break;
                }
            }
            if(c == paletteNext) {
                palette[c][0] = r;
                palette[c][1] = g;
                palette[c][2] = b;
                float y, i, q;
                RGBToYIQ(r / 255.0f, g / 255.0f, b / 255.0f, &y, &i, &q);
                paletteUint32[c] = colorTo4Samples(y, i, q);
            }
            palettized[y][x] = paletteUint32[c];
        }
    }
    printf("P5 %d %d 255\n", 194 * 4, 224);
    for(int y = 0; y < 224; y++) {
        fwrite(palettized[y], 194, 4, stdout);
    }
}
