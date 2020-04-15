#ifndef __UTILITY_H__
#define __UTILITY_H__

#ifndef M_PI
#define M_PI 3.1415926
#endif // M_PI

void dump_buffer_hex(int indent, const unsigned char *data, int size);
void bell();

// Color

void RGBToYIQ(float r, float g, float b, float *y, float *i, float *q);
void HSVToRGB3f(float h, float s, float v, float *r, float *g, float *b);
float Luminance(float r, float g, float b);
float ColorDistance(float r0, float g0, float b0, float r1, float g1, float b1);

// String Processing

// XXX changes bytes in p to NULs
int SplitString(char *p, char **words, int wordsCapacity);

#endif /* __UTILITY_H__ */
