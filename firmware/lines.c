#include <stdio.h>
#include <math.h>

int main(int argc, char **argv)
{
    printf("P6 384 128 255\n");
    for(int y = 0; y < 128; y++) {
        for(int x = 0; x < 384; x++) {
            int scale = 1 + y / 32;
            float v = x * M_PI / scale;
            unsigned char rgb[3];
            rgb[0] = 1 * cos(v) * 127 + 128 + 0 * cos(v + M_PI) * 127;
            rgb[1] = 1 * cos(v) * 127 + 128 + 0 * cos(v + M_PI) * 127;
            rgb[2] = 1 * cos(v) * 127 + 128 + 0 * cos(v + M_PI) * 127;
            fwrite(rgb, sizeof(rgb), 1, stdout);
        }
    }
}
