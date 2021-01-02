#include <string>
#include <map>
#include <array>
#include <cmath>

typedef std::array<float,3> vec3f;
typedef std::array<int,4> vec4i;

constexpr float TAU = 6.283185307179586;

float Rad(float degrees)
{
    return degrees / 360.0f * TAU;
}

void NTSCWaveToYIQ(float tcycles, float wave[4], float *y, float *i, float *q)
{
    *y = (wave[0] + wave[1] + wave[2] + wave[3]) / 4;

    float waveHF[4];
    for(int j = 0; j < 4; j++) waveHF[j] = wave[j] - *y;

    // XXX why wasn't this correct?
    // float w_t = tcycles * TAU;
    float w_t = Rad(60.0f) - tcycles * TAU; /* 60 - 15 matches Apple ][ GR colors? */

    *i =
        waveHF[0] * sinf(w_t - TAU / 4.0f * 0.0f) / 2 +
        waveHF[1] * sinf(w_t - TAU / 4.0f * 1.0f) / 2 +
        waveHF[2] * sinf(w_t - TAU / 4.0f * 2.0f) / 2 +
        waveHF[3] * sinf(w_t - TAU / 4.0f * 3.0f) / 2;

    *q =
        waveHF[0] * cosf(w_t - TAU / 4.0f * 0.0f) / 2 +
        waveHF[1] * cosf(w_t - TAU / 4.0f * 1.0f) / 2 +
        waveHF[2] * cosf(w_t - TAU / 4.0f * 2.0f) / 2 +
        waveHF[3] * cosf(w_t - TAU / 4.0f * 3.0f) / 2;
}

// Using inverse 3x3 matrix above.  Tested numerically to be the inverse of RGBToYIQ
inline void YIQToRGB(float y, float i, float q, float *r, float *g, float *b)
{
    *r = 1.0f * y + .946882f * i + 0.623557f * q;
    *g = 1.000000f * y + -0.274788f * i + -0.635691f * q;
    *b = 1.000000f * y + -1.108545f * i + 1.709007f * q;
}


vec3f FindNTSCColor(const vec4i& pattern)
{
    float wave[4];
    for(int i = 0; i < 4; i++) {
        wave[i] = pattern[i] * 1.0;
    }
    float y, i, q;
    NTSCWaveToYIQ(0, wave, &y, &i, &q);
    float r, g, b;
    YIQToRGB(y, i, q, &r, &g, &b);
    r = std::clamp(r, 0.0f, 1.0f);
    g = std::clamp(g, 0.0f, 1.0f);
    b = std::clamp(b, 0.0f, 1.0f);
    return { r, g, b };
}

std::map<int, std::string> colorNames = 
{
    {0, "black"},
    {1, "red"},
    {2, "dark blue"},
    {3, "purple"},
    {4, "dark green"},
    {5, "gray 1"},
    {6, "medium blue"},
    {7, "light blue"},
    {8, "brown"},
    {9, "orange"},
    {10, "gray 2"},
    {11, "pink"},
    {12, "light green"},
    {13, "yello"},
    {14, "aqua"},
    {15, "white"},
};

int main(int argc, char **argv)
{
    printf("uint8_t artifact_colors[][3] = {\n");
    for(int i = 0; i < 16; i++) {
        vec4i pattern = {i & 1, (i >> 1) & 1, (i >> 2) & 1, (i >> 3) & 1};
        vec3f color = FindNTSCColor(pattern);
        printf("    {%3d, %3d, %3d}, // %2d \"%s\"%*s -> %d,%d,%d,%d -> {%f, %f, %f}\n",
            (int)(color[0] * 255), (int)(color[1] * 255), (int)(color[2] * 255),
            i, colorNames.at(i).c_str(),
            11 - (int)colorNames.at(i).size(), "",
            pattern[0], pattern[1], pattern[2], pattern[3],
            color[0], color[1], color[2]);

    }
    printf("};\n");

    FILE *fp = fopen("dhgr_colors.ppm", "wb");
    fprintf(fp, "P6 512 512 255\n");
    for(int rowIndex = 0; rowIndex < 512; rowIndex++) {
        for(int colIndex = 0; colIndex < 512; colIndex++) {
            int i = colIndex / 128 + rowIndex / 128 * 4;
            vec4i pattern = {i & 1, (i >> 1) & 1, (i >> 2) & 1, (i >> 3) & 1};
            vec3f color = FindNTSCColor(pattern);
            unsigned char rgb[3];
            for(int j = 0; j < 3; j++) {
               rgb[j] = color[j] * 255;
            }
            fwrite(rgb, 3, 1, fp);
        }
    }
}

