#include <stdint.h>
#include <stdio.h>
#include <math.h>

void RGBToYIQ(int r, int g, int b, int *y, int *i, int *q)
{
    *y = .30f * r + .59f * g + .11f * b;
    *i = -.27f * (b - *y) + .74f * (r - *y);
    *q = .41f * (b - *y) + .48f * (r - *y);
}

#define NTSC_SYNC_BLACK_VOLTAGE   .339f
#define NTSC_SYNC_WHITE_VOLTAGE   1.0f  /* VCR had .912v */

#define DAC_VALUE_LIMIT 0xFF

#define MAX_DAC_VOLTAGE 1.22f

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


float NTSCYIQToSignal(float y, float i, float q, float tcycles)
{
// This is transcribed from the NTSC spec, double-checked.
    float wt = tcycles * M_PI * 2;
    float sine = sinf(wt + 33.0f / 180.0f * M_PI);
    float cosine = cosf(wt + 33.0f / 180.0f * M_PI);
    return y + q * sine + i * cosine;
}

int main(int argc, char **argv)
{
    int mini = 1000;
    int maxi = -1000;
    int minq = 1000;
    int maxq = -1000;
    static int ivalues[400]; // bias by 200
    static int qvalues[400]; // bias by 200
    float signalMin = 1000.0f;
    float signalMax = -1000.0f;
    for(int b = 0; b < 256; b++) {
        for(int g = 0; g < 256; g++) {
            for(int r = 0; r < 256; r++) {
                int y, i, q;
                RGBToYIQ(r, g, b, &y, &i, &q);
                ivalues[i + 200] ++;
                qvalues[q + 200] ++;
                if(i < mini) {
                    mini = i;
                }
                if(i > maxi) {
                    maxi = i;
                }
                if(q < minq) {
                    minq = q;
                }
                if(q > maxq) {
                    maxq = q;
                }
                float signal;
                signal = NTSCYIQToSignal(y / 255.0, i / 255.0, q / 255.0, 0.0f);
                signalMax = (signal > signalMax) ? signal : signalMax;
                signalMin = (signal < signalMin) ? signal : signalMin;
            }
        }
    }
    printf("i = [%d, %d], q = [%d, %d]\n", mini, maxi, minq, maxq);
    printf("signalMin = %f, signalMax = %f\n", signalMin, signalMax);
    for(int n = 0; n < 400; n++) {
        if(ivalues[n] > 0) {
            printf("i = %d, count %d\n", n - 200, ivalues[n]);
        } 
    }
    for(int n = 0; n < 400; n++) {
        if(qvalues[n] > 0) {
            printf("q = %d, count %d\n", n - 200, qvalues[n]);
        } 
    }
}
