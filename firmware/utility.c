#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "utility.h"

void dump_buffer_hex(int indent, const unsigned char *data, int size)
{
    int address = 0;
    int i;

    while(size > 0) {
        int howmany = (size < 16) ? size : 16;

        printf("%*s0x%04X: ", indent, "", address);
        for(i = 0; i < howmany; i++)
            printf("%02X ", data[i]);
        printf("\n");

        printf("%*s        ", indent, "");
        for(i = 0; i < howmany; i++)
            printf(" %c ", isprint(data[i]) ? data[i] : '.');
        printf("\n");

        size -= howmany;
        data += howmany;
        address += howmany;
    }
}

#define BELL '\a'

void bell()
{
    putchar(BELL);
}

//--------------------------------------------------------------------------
// Color

// This is transcribed from the NTSC spec, double-checked.
void RGBToYIQ(float r, float g, float b, float *y, float *i, float *q)
{
    *y = .30f * r + .59f * g + .11f * b;
    *i = -.27f * (b - *y) + .74f * (r - *y);
    *q = .41f * (b - *y) + .48f * (r - *y);
}

void HSVToRGB3f(float h, float s, float v, float *r, float *g, float *b)
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

float Luminance(float r, float g, float b)
{
    return .30f * r + .59f * g + .11f * b;
}

float ColorDistance(float r0, float g0, float b0, float r1, float g1, float b1)
{
    return sqrtf((r1 - r0) * (r1 - r0) + (g1 - g0) * (g1 - g0) + (b1 - b0) * (b1 - b0));
}

//--------------------------------------------------------------------------
// String processing

// XXX changes bytes in p to NULs
int SplitString(char *p, char **words, int wordsCapacity)
{
    int count = 0;

    while(*p == ' ') { p++; } /* skip initial spaces */

    /* while haven't filled to capacity and p is not a space or NUL */
    while(count < wordsCapacity && *p) {
        words[count++] = p; /* store pointer to word */
        if(count != wordsCapacity) {
            while(*p && (*p != ' ')) { p++; } /* skip until NUL or space */
            if(!*p) { /* if p is NUL, done */
                return count;
            }
            *p++ = '\0';
            while(*p == ' ') { p++; } /* skip spaces */
        } 
    }
    return count;
}

