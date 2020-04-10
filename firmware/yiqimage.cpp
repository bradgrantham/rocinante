#include <math.h>
#include <stdio.h>

void colorHSVToRGB3f(float h, float s, float v, float *r, float *g, float *b)
{
    if(s < .00001) {
        *r = v; *g = v; *b = v;
    } else {
    	int i;
	float p, q, t, f;

	h = fmod(h, M_PI * 2);	/* wrap just in case */

        i = floor(h / (M_PI / 3));

	/*
	 * would have used "f = fmod(h, M_PI / 3);", but fmod seems to have
	 * a bug under Linux.
	 */

	f = h / (M_PI / 3) - floor(h / (M_PI / 3));

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
    unsigned char image[512][512][3];

    for(int y = 0; y < 512; y++) {
        for(int x = 0; x < 512; x++) {
            float u = (x - 256) / 256.0f;
            float v = (y - 256) / 256.0f;
            float hue = fmod(M_PI * 2 + atan2(-v, u) + (270.0f - 13.0f) / 180.0f * M_PI, M_PI * 2);
            float r, g, b;
            colorHSVToRGB3f(hue, 1, 1, &r, &g, &b);
            image[y][x][0] = r * 255.0;
            image[y][x][1] = g * 255.0;
            image[y][x][2] = b * 255.0;
        }
    }
    printf("P6 512 512 255\n");
    fwrite(image, sizeof(image), 1, stdout);
}
