#include <algorithm>
#include <cstdio>
#include <cmath>

#define TAU (M_PI * 2)

// phaseIn in radians
// wave in whatever
// DC component is average of wave
// magnitude is largest of HF component
// phase is phase of HF component
void Decompose(float phaseIn, float wave[4], float *dc, float *magnitude, float *phase)
{
    *dc = (wave[0] + wave[1] + wave[2] + wave[3]) / 4;

    float waveHF[4];
    for(int i = 0; i < 4; i++) waveHF[i] = wave[i] - *dc;

    *magnitude = 0;
    for(int i = 0; i < 4; i++) *magnitude = std::max(*magnitude, fabsf(waveHF[i]));

    for(int i = 0; i < 4; i++) waveHF[i] /= *magnitude;

    float sine =
        waveHF[0] * sinf(phaseIn + TAU / 4 * 0) + 
        waveHF[1] * sinf(phaseIn + TAU / 4 * 1) + 
        waveHF[2] * sinf(phaseIn + TAU / 4 * 2) + 
        waveHF[3] * sinf(phaseIn + TAU / 4 * 3);
    float cosine =
        waveHF[0] * cosf(phaseIn + TAU / 4 * 0) + 
        waveHF[1] * cosf(phaseIn + TAU / 4 * 1) + 
        waveHF[2] * cosf(phaseIn + TAU / 4 * 2) + 
        waveHF[3] * cosf(phaseIn + TAU / 4 * 3);

    *phase = atan2(cosine, sine);
    if(*phase < -.0001) *phase += TAU;
}

int main(int argc, char **argv)
{
    for(int i = 0; i < 100; i++) {
        float f = TAU * i / 100;
        float wave[4];
        wave[0] = .5 + .5 * sin(f + TAU / 4 * 0);
        wave[1] = .5 + .5 * sin(f + TAU / 4 * 1);
        wave[2] = .5 + .5 * sin(f + TAU / 4 * 2);
        wave[3] = .5 + .5 * sin(f + TAU / 4 * 3);
        float dc, magnitude, phase;
        Decompose(0, wave, &dc, &magnitude, &phase);
        printf("%f : %f %f %f %f -> %f %f %f\n", f, wave[0], wave[1], wave[2], wave[3], dc, magnitude, phase);
    }
}
