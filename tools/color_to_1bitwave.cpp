/*
load any PPM
    use pnmscale to scale to give as input
    write out PPM with 0 or 255 per pixel
        same size for DHGR mode
        or half-size for HGR mode
    beginning of line pattern is [0,0,0,0], index is 0
    For each pixel, two potential colors are 
        HGR mode, color from pattern with index%4 and (index + 1) % 4 as 00 or 11
            use average of two input colors?
        DHGR mode, color from pattern with index%4 as 0 or 1
hack version of newvideo hosted to convert a PPM assuming 0-255 to NTSC interpreted colors
*/

#include <array>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdlib>

unsigned int imageWidth;
unsigned int imageHeight;
float *imagePixels;

static void skipComments(FILE *fp, char ***comments, size_t *commentCount)
{
    int c;
    char line[512];

    while((c = fgetc(fp)) == '#') {
        fgets(line, sizeof(line) - 1, fp);
	line[strlen(line) - 1] = '\0';
	if(comments != NULL) {
	    *comments =
	        (char**)realloc(*comments, sizeof(char *) * (*commentCount + 1));
	    (*comments)[*commentCount] = strdup(line);
	}
	if(commentCount != NULL) {
	    (*commentCount) ++;
	}
    }
    ungetc(c, fp);
}


int pnmRead(FILE *file, unsigned int *w, unsigned int *h, float **pixels,
    char ***comments, size_t *commentCount)
{
    unsigned char	dummyByte;
    int			i;
    float		max;
    char		token;
    int			width, height;
    float		*rgbPixels;

    if(commentCount != NULL)
	*commentCount = 0;
    if(comments != NULL)
	*comments = NULL;

    fscanf(file, " ");

    skipComments(file, comments, commentCount);

    if(fscanf(file, "P%c ", &token) != 1) {
         fprintf(stderr, "pnmRead: Had trouble reading PNM tag\n");
	 return(0);
    }

    skipComments(file, comments, commentCount);

    if(fscanf(file, "%d ", &width) != 1) {
         fprintf(stderr, "pnmRead: Had trouble reading PNM width\n");
	 return(0);
    }

    skipComments(file, comments, commentCount);

    if(fscanf(file, "%d ", &height) != 1) {
         fprintf(stderr, "pnmRead: Had trouble reading PNM height\n");
	 return(0);
    }

    skipComments(file, comments, commentCount);

    if(token != '1' && token != '4')
        if(fscanf(file, "%f", &max) != 1) {
             fprintf(stderr, "pnmRead: Had trouble reading PNM max value\n");
	     return(0);
        }

    rgbPixels = (float*)malloc(width * height * 4 * sizeof(float));
    if(rgbPixels == NULL) {
         fprintf(stderr, "pnmRead: Couldn't allocate %lu bytes\n",
	     width * height * 4 * sizeof(float));
         fprintf(stderr, "pnmRead: (For a %d by %d image)\n", width,
	     height);
	 return(0);
    }

    if(token != '4')
	skipComments(file, comments, commentCount);

    if(token != '4')
    fread(&dummyByte, 1, 1, file);	/* chuck white space */

    if(token == '1') {
	for(i = 0; i < width * height; i++) {
	    int pixel;
	    fscanf(file, "%d", &pixel);
	    pixel = 1 - pixel;
	    rgbPixels[i * 4 + 0] = pixel;
	    rgbPixels[i * 4 + 1] = pixel;
	    rgbPixels[i * 4 + 2] = pixel;
	    rgbPixels[i * 4 + 3] = 1.0;
	}
    } else if(token == '2') {
	for(i = 0; i < width * height; i++) {
	    int pixel;
	    fscanf(file, "%d", &pixel);
	    rgbPixels[i * 4 + 0] = pixel / max;
	    rgbPixels[i * 4 + 1] = pixel / max;
	    rgbPixels[i * 4 + 2] = pixel / max;
	    rgbPixels[i * 4 + 3] = 1.0;
	}
    } else if(token == '3') {
	for(i = 0; i < width * height; i++) {
	    int r, g, b;
	    fscanf(file, "%d %d %d", &r, &g, &b);
	    rgbPixels[i * 4 + 0] = r / max;
	    rgbPixels[i * 4 + 1] = g / max;
	    rgbPixels[i * 4 + 2] = b / max;
	    rgbPixels[i * 4 + 3] = 1.0;
	}
    } else if(token == '4') {
        int bitnum = 0;

	for(i = 0; i < width * height; i++) {
	    unsigned char pixel;
	    unsigned char value = 0;

	    if(bitnum == 0) {
	        fread(&value, 1, 1, file);
            }

	    pixel = (1 - ((value >> (7 - bitnum)) & 1));
	    rgbPixels[i * 4 + 0] = pixel;
	    rgbPixels[i * 4 + 1] = pixel;
	    rgbPixels[i * 4 + 2] = pixel;
	    rgbPixels[i * 4 + 3] = 1.0;

	    if(++bitnum == 8 || ((i + 1) % width) == 0)
	        bitnum = 0;
	}
    } else if(token == '5') {
	for(i = 0; i < width * height; i++) {
	    unsigned char pixel;
	    fread(&pixel, 1, 1, file);
	    rgbPixels[i * 4 + 0] = pixel / max;
	    rgbPixels[i * 4 + 1] = pixel / max;
	    rgbPixels[i * 4 + 2] = pixel / max;
	    rgbPixels[i * 4 + 3] = 1.0;
	}
    } else if(token == '6') {
	for(i = 0; i < width * height; i++) {
	    unsigned char rgb[3];
	    fread(rgb, 3, 1, file);
	    rgbPixels[i * 4 + 0] = rgb[0] / max;
	    rgbPixels[i * 4 + 1] = rgb[1] / max;
	    rgbPixels[i * 4 + 2] = rgb[2] / max;
	    rgbPixels[i * 4 + 3] = 1.0;
	}
    }
    *w = width;
    *h = height;
    *pixels = rgbPixels;
    return(1);
}

typedef std::array<float,3> vec3f;
typedef std::array<int,4> vec4i;

int FindClosestColor(vec3f palette[], int paletteSize, const vec3f& color)
{
    float bestDiff = 1000;  // max should be 1.7321
    int c = -1;

    for(int i = 0; i < paletteSize; i++) {
        float pr = palette[i][0];
        float pg = palette[i][1];
        float pb = palette[i][2];
        float diff = (pr - color[0]) * (pr - color[0]) + (pg - color[1]) * (pg - color[1]) + (pb - color[2]) * (pb - color[2]);
        if(diff == 0) {
            return i;
        }
        if((i == 0) || (diff < bestDiff)) {
            bestDiff = diff;
            c = i;
        }
    }
    return c;
}

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

    // XXX why wasn't this and adding the phase below correct?
    // float w_t = tcycles * TAU;
    float w_t = Rad(60.0f) - tcycles * TAU;

    *i =
        waveHF[0] * sinf(w_t - TAU / 4.0f * 0.0f) +
        waveHF[1] * sinf(w_t - TAU / 4.0f * 1.0f) +
        waveHF[2] * sinf(w_t - TAU / 4.0f * 2.0f) +
        waveHF[3] * sinf(w_t - TAU / 4.0f * 3.0f);

    *q =
        waveHF[0] * cosf(w_t - TAU / 4.0f * 0.0f) +
        waveHF[1] * cosf(w_t - TAU / 4.0f * 1.0f) +
        waveHF[2] * cosf(w_t - TAU / 4.0f * 2.0f) +
        waveHF[3] * cosf(w_t - TAU / 4.0f * 3.0f);
}

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

int main(int argc, char **argv)
{
    if(argc < 3) {
        fprintf(stderr, "usage: %s inputppm outputppm\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    FILE *inputImage = fopen(argv[1], "rb");
    if(inputImage == NULL) {
        fprintf(stderr, "failed to open %s for reading\n", argv[1]);
        exit(EXIT_FAILURE);
    }
    int success = pnmRead(inputImage, &imageWidth, &imageHeight, &imagePixels, NULL, NULL);
    fclose(inputImage);
    if(!success) {
        fprintf(stderr, "couldn't read PPM from %s\n", argv[1]);
        exit(EXIT_FAILURE);
    }

    FILE *outputImage = fopen(argv[2], "wb");
    if(outputImage == NULL) {
        fprintf(stderr, "failed to open %s for writing\n", argv[2]);
        exit(EXIT_FAILURE);
    }

    unsigned char waveformRow[16384];

    vec3f black = {0.0f, 0.0f, 0.0f};

    std::array<vec3f,16384> errorThisRow;
    std::fill(errorThisRow.begin(), errorThisRow.end(), black);

    fprintf(outputImage, "P5 %d %d 255\n", imageWidth, imageHeight);
    
    for(int rowIndex = 0; rowIndex < imageHeight; rowIndex++) {

        vec4i pattern = {0, 0, 0, 0};

        std::array<vec3f,16384> errorNextRow;
        std::fill(errorNextRow.begin(), errorNextRow.end(), black);

        for(int colIndex = 0; colIndex < imageWidth; colIndex++) {

            float *pixel = imagePixels + (rowIndex * imageWidth + colIndex) * 4;

            // get the color with error diffused from previous pixels
            vec3f correctedRGB;
            for(int i = 0; i < 3; i++) {
                correctedRGB[i] = pixel[i] + errorThisRow[1 + colIndex][i];
            }

            // Find the closest color NTSC can offer given the last 3 samples
            vec3f palette[2];
            vec4i newPattern = pattern;

            newPattern[colIndex % 4] = 0;
            palette[0] = FindNTSCColor(newPattern);

            newPattern[colIndex % 4] = 1;
            palette[1] = FindNTSCColor(newPattern);

            int c;
            c = FindClosestColor(palette, 2, correctedRGB);

            assert((c == 0) || (c == 1));
            pattern[colIndex % 4] = c;

            // XXX Should templatize image buffer format to avoid branch
            waveformRow[colIndex] = (c == 0) ? 0 : 255;

            // Calculate our error between what we wanted and what we got
            // and distribute it a la Floyd-Steinberg
            vec3f error;
            for(int i = 0; i < 3; i++) {
                error[i] = correctedRGB[i] - palette[c][i];
            }
            if(0) {
                for(int i = 0; i < 3; i++) {
                    errorThisRow[1 + colIndex + 1][i] += error[i] * 7.0f / 16.0f;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRow[1 + colIndex - 1][i] += error[i] * 3.0f / 16.0f;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRow[1 + colIndex    ][i] += error[i] * 5.0f / 16.0f;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRow[1 + colIndex + 1][i] += error[i] * 1.0f / 16.0f;
                }
            } else if (0) {
                for(int i = 0; i < 3; i++) {
                    errorThisRow[1 + colIndex + 1][i] += error[i] * 16.0f / 16.0f;
                }
            } else if (1) {
                for(int i = 0; i < 3; i++) {
                    errorThisRow[1 + colIndex + 1][i] += error[i] * 7.0f / 16.0f;
                    errorThisRow[1 + colIndex + 2][i] += error[i] * 5.0f / 16.0f;
                    errorThisRow[1 + colIndex + 3][i] += error[i] * 3.0f / 16.0f;
                    errorThisRow[1 + colIndex + 4][i] += error[i] * 1.0f / 16.0f;
                }
            } else { 
                for(int i = 0; i < 3; i++) {
                    errorThisRow[1 + colIndex + 1][i] += error[i] * 7.0f / 16.0f;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRow[1 + colIndex - 1][i] += error[i] * 3.0f / 16.0f;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRow[1 + colIndex + 1][i] += error[i] * 5.0f / 16.0f;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRow[1 + colIndex + 2][i] += error[i] * 1.0f / 16.0f;
                }
            }
        }
        errorThisRow = errorNextRow; // Could move with a vector work here?

        fwrite(waveformRow, 1, imageWidth, outputImage);
    }
}
