#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "ff.h"

#define mapWidth 24
#define mapHeight 24

/* data cribbed from https://lodev.org/cgtutor/raycasting2.html */

const static int worldMap[mapWidth][mapHeight]=
{
  {8,8,8,8,8,8,8,8,8,8,8,4,4,6,4,4,6,4,6,4,4,4,6,4},
  {8,0,0,0,0,0,0,0,0,0,8,4,0,0,0,0,0,0,0,0,0,0,0,4},
  {8,0,3,3,0,0,0,0,0,8,8,4,0,0,0,0,0,0,0,0,0,0,0,6},
  {8,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6},
  {8,0,3,3,0,0,0,0,0,8,8,4,0,0,0,0,0,0,0,0,0,0,0,4},
  {8,0,0,0,0,0,0,0,0,0,8,4,0,0,0,0,0,6,6,6,0,6,4,6},
  {8,8,8,8,0,8,8,8,8,8,8,4,4,4,4,4,4,6,0,0,0,0,0,6},
  {7,7,7,7,0,7,7,7,7,0,8,0,8,0,8,0,8,4,0,4,0,6,0,6},
  {7,7,0,0,0,0,0,0,7,8,0,8,0,8,0,8,8,6,0,0,0,0,0,6},
  {7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,6,0,0,0,0,0,4},
  {7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,6,0,6,0,6,0,6},
  {7,7,0,0,0,0,0,0,7,8,0,8,0,8,0,8,8,6,4,6,0,6,6,6},
  {7,7,7,7,0,7,7,7,7,8,8,4,0,6,8,4,8,3,3,3,0,3,3,3},
  {2,2,2,2,0,2,2,2,2,4,6,4,0,0,6,0,6,3,0,0,0,0,0,3},
  {2,2,0,0,0,0,0,2,2,4,0,0,0,0,0,0,4,3,0,0,0,0,0,3},
  {2,0,0,0,0,0,0,0,2,4,0,0,0,0,0,0,4,3,0,0,0,0,0,3},
  {1,0,0,0,0,0,0,0,1,4,4,4,4,4,6,0,6,3,3,0,0,0,3,3},
  {2,0,0,0,0,0,0,0,2,2,2,1,2,2,2,6,6,0,0,5,0,5,0,5},
  {2,2,0,0,0,0,0,2,2,2,0,0,0,2,2,0,5,0,5,0,0,0,5,5},
  {2,0,0,0,0,0,0,0,2,0,0,0,0,0,2,5,0,5,0,5,0,5,0,5},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
  {2,0,0,0,0,0,0,0,2,0,0,0,0,0,2,5,0,5,0,5,0,5,0,5},
  {2,2,0,0,0,0,0,2,2,2,0,0,0,2,2,0,5,0,5,0,0,0,5,5},
  {2,2,2,2,1,2,2,2,2,2,2,1,2,2,2,5,5,5,5,5,5,5,5,5}
};

float epsilon = .000001f;

void Raycast(VideoWolfensteinInfo* info, VideoWolfensteinElement* buffer, float eyeX, float eyeZ, float rot, float fieldOfViewDegrees)
{
    float fovX = tanf(fieldOfViewDegrees  / 180.0f * M_PI / 2.0f);
    float fovY = fovX ; // XXX /aspect
    for(int column = 0; column < info->width; column++) {
        float dirX0 = ((column + .5) / info->width - .5) * fovX;
        float dirZ0 = -0.5f;

        float dirX = dirX0 * cosf(rot) + dirZ0 * sinf(rot);
        float dirZ = dirZ0 * cosf(rot) - dirX0 * sinf(rot);

        float T = 0;

        // These are the calculated next T to X and Z planes
        // If nextTX < nextTZ, then we'll step X before we'll step Z 
	// These should always be positive and increasing through
	// the step loop
        float nextTX, nextTZ;

        // these are the step in T to move one cell in X or Z
        float stepTX, stepTZ;

        // these are the step deltas (1 or -1) for moving forward in X or Z
        int stepX, stepZ;

        // starting cell
        int cellX = (int)floorf(eyeX);
        int cellZ = (int)floorf(eyeZ);

        if(dirX < 0.0f - epsilon) {
            // If ray is pointing negative X,
            // then next X is floorf(eyeX),
            // so T at that transition is distance divided by X component of dir
            nextTX = (floorf(eyeX) - eyeX) / dirX;
            stepTX = -1.0f / dirX;
            stepX = -1;
        } else if(dirX > 0.0f + epsilon) {
            // If ray is pointing positive X,
            // then next X is floorf(eyeX),
            // so T at that transition is distance to next X divided by X component of dir
            nextTX = (ceil(eyeX) - eyeX) / dirX;
            stepTX = 1.0f / dirX;
            stepX = 1;
        } else {
            nextTX = 100000000.0f;
            stepTX = 100000000.0f;
            stepX = 1;
        }

        if(dirZ < 0.0f - epsilon) {
            nextTZ = (floorf(eyeZ) - eyeZ) / dirZ;
            stepTZ = -1.0f / dirZ;
            stepZ = -1;
        } else if(dirZ > 0.0f + epsilon) {
            nextTZ = (ceil(eyeZ) - eyeZ) / dirZ;
            stepTZ = 1.0f / dirZ;
            stepZ = 1;
        } else {
            nextTZ = 100000000.0f;
            stepTZ = 100000000.0f;
            stepZ = 1;
        }

        buffer[column].height = 0;
        buffer[column].id = 0;

        int steps = 0;
        do {
            int hitX;

            if(nextTX < nextTZ) {
                // next hit in X is closer than next hit in Z
                hitX = 1; // hitX = 1 because we hit next in X
                T = nextTX;
                cellX += stepX;
                nextTX += stepTX;
            } else {
                // next hit in Z is closer than next hit in X
                hitX = 0; // hitX = 1 because we hit next in Z
                T = nextTZ;
                cellZ += stepZ;
                nextTZ += stepTZ;
            }

            if(worldMap[cellX][cellZ] != 0) {
                buffer[column].height = 1.0f / fovY / T;
                buffer[column].id = worldMap[cellX][cellZ] - 1;
                if(hitX) {
                    float s = eyeZ + dirZ * T;
                    s = s - floorf(s);
                    buffer[column].textureS = s;
                    buffer[column].bright = 0;
                } else {
                    float s = eyeX + dirX * T;
                    s = s - floorf(s);
                    buffer[column].textureS = s;
                    buffer[column].bright = 1;
                }
                break;
            }
            steps ++;
        } while(cellX >= 0 && cellX < mapWidth && cellZ >= 0 && cellZ < mapHeight);
    }
}

int empty(float x, float z)
{
    int cellX = (int)floorf(x);
    int cellZ = (int)floorf(z);
    return worldMap[cellX][cellZ] == 0;
}

static int AppWolf(int argc, char **argv)
{
    int chosen = -1;
    for(int i = 0; (chosen == -1) && (i < VideoGetModeCount()); i++) {
        enum VideoModeType type = VideoModeGetType(i);
        if(type == VIDEO_MODE_WOLFENSTEIN) {
            chosen = i;
        }
    }
    
    if(chosen == -1) {
        printf("Wolfenstein video mode not found\n");
        return COMMAND_FAILED;
    }

    VideoSetMode(chosen);
    VideoWolfensteinInfo info;
    VideoWolfensteinParameters params;
    VideoModeGetInfo(chosen, &info);
    VideoModeGetParameters(&params);

    VideoWolfensteinElement *elements;
    if((elements = malloc(sizeof(VideoWolfensteinElement) * info.width)) == NULL) {
        printf("ERROR: couldn't allocate raycasting elements\n");
        return COMMAND_FAILED;
    }
    
    float eyeX = 4.5;
    float eyeZ = 4.5;
    float rot = 0;

    setvbuf(stdin, NULL, _IONBF, 0);

    int quit = 0;
    do {
        int key = getchar();
        float newEyeX;
        float newEyeZ;
        switch(key) {
            case 'q': case 'Q': {
                quit = 1;
                break;
            }
            case 'a': {
                rot += M_PI / 50;
                break;
            }
            case 'd': {
                rot -= M_PI / 50;
                break;
            }
            case 'w': {
                newEyeZ = eyeZ - .1 * cosf(rot);
                newEyeX = eyeX - .1 * sinf(rot);
                if(empty(newEyeX, newEyeZ)) {
                    eyeX = newEyeX;
                    eyeZ = newEyeZ;
                }
                break;
            }
            case 's': {
                newEyeZ = eyeZ + .1 * cosf(rot);
                newEyeX = eyeX + .1 * sinf(rot);
                if(empty(newEyeX, newEyeZ)) {
                    eyeX = newEyeX;
                    eyeZ = newEyeZ;
                }
                break;
            }
        }
        Raycast(&info, elements, eyeX, eyeZ, rot, 80);
        VideoModeWaitFrame();
        params.setElements(elements);
    } while(!quit);

    free(elements);

    return COMMAND_CONTINUE;
}

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("wolf", 1, AppWolf, "",
        "Wolfenstein"
        );
}

