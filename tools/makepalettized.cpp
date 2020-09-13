#include <math.h>
#include <stdio.h>
#include <cstdint>
#include <cassert>

#define DAC_VALUE_LIMIT 0xF0
#define NTSC_SYNC_BLACK_VOLTAGE   .339f
#define NTSC_SYNC_WHITE_VOLTAGE   1.0f  /* VCR had .912v */
#define MAX_PALETTE_ENTRIES 254

unsigned char blackDACValue;
unsigned char whiteDACValue;
unsigned char maxDACValue;

#define MAX_DAC_VOLTAGE  1.22f

unsigned char voltageToDACValue(float voltage)
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

// This is transcribed from the NTSC spec, double-checked.
void RGBToYIQ(float r, float g, float b, float *y, float *i, float *q)
{
    *y = .30f * r + .59f * g + .11f * b;
    *i = -.27f * (b - *y) + .74f * (r - *y);
    *q = .41f * (b - *y) + .48f * (r - *y);
}

unsigned char colorToDACValue(float y, float i, float q, float tcycles)
{
// This is transcribed from the NTSC spec, double-checked.
    float wt = tcycles * M_PI * 2;
    float sine = sinf(wt + 33.0f / 180.0f * M_PI);
    float cosine = cosf(wt + 33.0f / 180.0f * M_PI);
    float signal = y + q * sine + i * cosine;
// end of transcription

    if(signal < -1.0f) { signal = -1.0f; }
    if(signal > 1.0f) { signal = 1.0f; }

    return blackDACValue + signal * (maxDACValue - blackDACValue);
}

uint32_t colorTo4Samples(float y, float i, float q)
{
    unsigned char b0 = colorToDACValue(y, i, q,  .0f);
    unsigned char b1 = colorToDACValue(y, i, q, .25f);
    unsigned char b2 = colorToDACValue(y, i, q, .50f);
    unsigned char b3 = colorToDACValue(y, i, q, .75f);

    return (b0 << 0) | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

void colorHSVToRGB3f(float h, float s, float v, float *r, float *g, float *b)
{
    if(s < .00001) {
        *r = v; *g = v; *b = v;
    } else {
    	int i;
	float p, q, t, f;

	h = fmodf(h, M_PI * 2);	/* wrap just in case */

        i = floorf(h / (M_PI / 3));

	/*
	 * would have used "f = fmod(h, M_PI / 3);", but fmod seems to have
	 * a bug under Linux.
	 */

	f = h / (M_PI / 3) - floorf(h / (M_PI / 3));

	p = v * (1 - s);
	q = v * (1 - s * f);
	t = v * (1 - s * (1 - f));
	switch(i) {
	    case 0: *r = v; *g = t; *b = p; break;
	    case 1: *r = q; *g = v; *b = p; break;
	    case 2: *r = p; *g = v; *b = t; break;
	    case 3: *r = p; *g = q; *b = v; break;
	    case 4: *r = t; *g = p; *b = v; break;
	    case 5: *r = v; *g = p; *b = q; break;
	}
    }
}

int main()
{
    blackDACValue = voltageToDACValue(NTSC_SYNC_BLACK_VOLTAGE);
    whiteDACValue = voltageToDACValue(NTSC_SYNC_WHITE_VOLTAGE);
    maxDACValue = 255;

    unsigned char paletteRGB[254][3];
    uint32_t paletteQuads[254];
    unsigned char imageRGB[244][194][3];
    unsigned char imagePalettized[244][194];

    assert(fread(paletteRGB, sizeof(paletteRGB), 1, stdin) == 1);
    assert(fread(imageRGB, sizeof(imageRGB), 1, stdin) == 1);
    int paletteNext = 0;
    for(int y = 0; y < 224; y++) {
        for(int x = 0; x < 194; x++) {
            unsigned char r = imageRGB[y][x][0];
            unsigned char g = imageRGB[y][x][1];
            unsigned char b = imageRGB[y][x][2];
            int c;
            for(c = 0; c < paletteNext; c++) {
                if(paletteRGB[c][0] == r && paletteRGB[c][1] == g && paletteRGB[c][2] == b) {
                    break;
                }
            }
            if(c == paletteNext) {
                if(c == MAX_PALETTE_ENTRIES) {
                    printf("ran out of palette!\n");
                    return 1;
                } else {
                    paletteRGB[c][0] = r;
                    paletteRGB[c][1] = g;
                    paletteRGB[c][2] = b;
                    float y, i, q;
                    RGBToYIQ(r / 255.0f, g / 255.0f, b / 255.0f, &y, &i, &q);
                    paletteQuads[c] = colorTo4Samples(y, i, q);
                    paletteNext++;
                }
            }
            imagePalettized[y][x] = c;
        }
    }
    fwrite(paletteQuads, sizeof(paletteQuads), 1, stdout);
    fwrite(imagePalettized, sizeof(imagePalettized), 1, stdout);
}
