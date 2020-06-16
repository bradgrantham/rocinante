#include <memory>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cfloat>
#include <cmath>
#include <errno.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "commandline.h"
#include "pocketgl.h"

#ifndef HOSTED

extern void RasterizerStart() {}
extern void RasterizerClear(float r, float g, float b) {}
extern int RasterizerAddLine(const ScreenVertex& sv0, const ScreenVertex& sv1) { return 0; }
extern int RasterizerAddTriangle(const ScreenVertex& sv0, const ScreenVertex& sv1, const ScreenVertex& sv2) { return 0; }
extern void RasterizerEnd() {}

#endif

struct Vertex
{
    float v[3];
    float n[3];
};

void print_float(float f)
{
    printf("%d.%03d", (int)f, (int)(1000 * (f - (int)f)));
}


//----------------------------------------------------------------------------
// Globals for graphics state

#define CHECK_OPENGL(l) {int _glerr ; if((_glerr = pglGetError()) != GL_NO_ERROR) printf("GL Error: %04X at %d\n", _glerr, l); }

float light0_color[4] = {1, .5, 0, 1};
float light1_color[4] = {.5, 1, 1, 1};

float object_ambient[4] = {.1, .1, .1, 1};
float object_diffuse[4] = {.8, .8, .8, 1};
float object_specular[4] = {.5, .5, .5, 1};
float object_shininess = 50;

bool draw_wireframe = false;
bool flat_shade = false;
bool rotate_object = true;
bool rotate_lights = true;

float last_frame_time = 0;
float object_time = 0;
float lights_time = 0;

static void SetLights(float lights_time)
{
    float origin[4] = {0, 0, 0, 1};

    float light0_angle_x = 100 + lights_time * 45 / 0.9;
    float light0_angle_y = 50 + lights_time * 45 / 1.1;

    float light1_angle_x = 90 + lights_time * 45 / 0.95;
    float light1_angle_y = 200 + lights_time * 45 / 1.25;

    pglPushMatrix();
    CHECK_OPENGL(__LINE__);

    pglRotatef(light0_angle_x, 1, 0, 0);
    pglRotatef(light0_angle_y, 0, 1, 0);
    pglTranslatef(10, 10, 10);
    pglLightfv(GL_LIGHT0, GL_POSITION, origin);

    CHECK_OPENGL(__LINE__);
    pglPopMatrix();

    pglPushMatrix();

    pglRotatef(light1_angle_x, 1, 0, 0);
    pglRotatef(light1_angle_y, 0, 1, 0);
    pglTranslatef(10, 10, 10);
    pglLightfv(GL_LIGHT1, GL_POSITION, origin);

    CHECK_OPENGL(__LINE__);
    pglPopMatrix();
    CHECK_OPENGL(__LINE__);
}

static void DrawIndexedTriangles(float object_time, bool draw_wireframe, bool flat_shade, const Vertex *vertices, const int *indices, int triangleCount, float size, float center[3])
{
    if(flat_shade)
        pglShadeModel(GL_FLAT);
    else
        pglShadeModel(GL_SMOOTH);

    pglPushMatrix();

    pglMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, object_ambient);
    pglMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, object_diffuse);
    pglMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, object_specular);
    pglMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, object_shininess);

    float object_angle_x = object_time * 45;
    float object_angle_y = object_time * 45 / 1.3;

    pglRotatef(object_angle_x, 1, 0, 0);
    pglRotatef(object_angle_y, 0, 1, 0);
    pglScalef(0.5 / size, 0.5 / size, 0.5 / size);
    pglTranslatef(-center[0], -center[1], -center[2]);

    pglEnableClientState(GL_VERTEX_ARRAY);
    pglEnableClientState(GL_NORMAL_ARRAY);
    pglVertexPointer(3, GL_FLOAT, sizeof(vertices[0]), (void*)&vertices[0].v[0]);
    const float *normalPointer = &vertices[0].n[0];
    pglNormalPointer(GL_FLOAT, sizeof(vertices[0]), (void*)normalPointer);

    if(draw_wireframe) {
#if 0
        int vertex_offset = 0;
        for(int i = 0; i < primitive_count; i++)
        {
            switch(primitive_types[i]) {
                case GL_TRIANGLE_FAN:
                    pglDrawArrays(GL_LINE_LOOP, vertex_offset, primitive_vertex_counts[i]);
                    break;
                case GL_TRIANGLES:
                    for(int j = 0; i < primitive_vertex_counts[j] / 3; i++)
                        pglDrawArrays(GL_LINE_LOOP, vertex_offset + j * 3, 3);
                    break;
            }
            vertex_offset += primitive_vertex_counts[i];
        }
#endif
    } else {
        pglDrawElements(GL_TRIANGLES, triangleCount * 3, GL_UNSIGNED_INT, indices);
    }

    pglDisableClientState(GL_VERTEX_ARRAY);
    pglDisableClientState(GL_NORMAL_ARRAY);
    pglPopMatrix();
    CHECK_OPENGL(__LINE__);
}

void DrawFrame(float frame_time, bool draw_wireframe, bool flat_shade, const Vertex *vertices, const int *indices, int triangleCount, float size, float center[3])
{
    RasterizerStart();

    pglClear(GL_COLOR_BUFFER_BIT);
    CHECK_OPENGL(__LINE__);

    if(rotate_lights)
        lights_time += frame_time - last_frame_time;

    SetLights(lights_time);

    if(rotate_object)
        object_time += frame_time - last_frame_time;

    DrawIndexedTriangles(object_time, draw_wireframe, flat_shade, vertices, indices, triangleCount, size, center);

    last_frame_time = frame_time;

    RasterizerEnd();
}

void InitState()
{
    pglClearColor(1, 0, 0, 0);
    CHECK_OPENGL(__LINE__);

    pglMatrixMode(GL_MODELVIEW);
    pglLoadIdentity();
    pglTranslatef(0, 0, -1.5);

    pglMatrixMode(GL_PROJECTION);
    pglLoadIdentity();
    // pglFrustum(-.07, .07, -.07, .07, .2, 100);
    pglFrustum(-.03, .03, -.03, .03, .2, 100);

    pglMatrixMode(GL_MODELVIEW);

    pglEnable(GL_LIGHT0);
    pglLightfv(GL_LIGHT0, GL_DIFFUSE, light0_color);
    pglLightfv(GL_LIGHT0, GL_SPECULAR, light0_color);

    pglEnable(GL_LIGHT1);
    pglLightfv(GL_LIGHT1, GL_DIFFUSE, light1_color);
    pglLightfv(GL_LIGHT1, GL_SPECULAR, light1_color);

    pglEnable(GL_LIGHTING);
    CHECK_OPENGL(__LINE__);

    pglEnable(GL_CULL_FACE);

    pglEnable(GL_NORMALIZE);

    CHECK_OPENGL(__LINE__);
}

static int AppSpinModel(int argc, char **argv)
{
    const char *filename;

    filename = argv[1];

    int which = -1;
    for(int i = 0; i < VideoGetModeCount(); i++) {
        if(VideoModeGetType(i) == VIDEO_MODE_SEGMENTS) {
            which = i;
        }
    }

    if(which == -1) {
        printf("Couldn't find segments video mode\n");
        return COMMAND_FAILED;
    }

    VideoSetMode(which);

    VideoSegmentedInfo info;
    VideoSegmentedParameters params;
    VideoModeGetInfo(VideoGetCurrentMode(), &info);
    VideoModeGetParameters(&params);

    FILE *fp;
    fp = fopen (filename, "rb");
    if(fp == NULL) {
        printf("ERROR: couldn't open \"%s\" for reading, errno %d\n", filename, errno);
        return COMMAND_FAILED;
    }

    try {

        int vertexCount;
        if(fscanf(fp, "%d", &vertexCount) != 1) {
            printf("ERROR: couldn't read vertex count from \"%s\"\n", filename);
            return COMMAND_FAILED;
        }

        std::vector<Vertex> vertices(vertexCount);

        float boxmin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
        float boxmax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        for(int i = 0; i < vertexCount; i++) {
            Vertex& v = vertices[i];
            if(fscanf(fp, "%f %f %f %f %f %f", &v.v[0], &v.v[1], &v.v[2], &v.n[0], &v.n[1], &v.n[2]) != 6) {
                printf("ERROR: couldn't read vertex %d from \"%s\"\n", i, filename);
                return COMMAND_FAILED;
            }
            for(int i = 0; i < 3; i++) { boxmin[i] = std::min(boxmin[i], v.v[i]); }
            for(int i = 0; i < 3; i++) { boxmax[i] = std::max(boxmax[i], v.v[i]); }
        }
        float center[3];
        float axissize[3];
        for(int i = 0; i < 3; i++) {
            center[i] = (boxmax[i] + boxmin[i]) / 2.0f;
            axissize[i] = boxmax[i] - boxmin[i];
        }
        float size = sqrtf(axissize[0] * axissize[0] + 
            axissize[1] * axissize[1] + 
            axissize[2] * axissize[2]);

        int triangleCount;
        if(fscanf(fp, "%d", &triangleCount) != 1) {
            printf("ERROR: couldn't read triangle count from \"%s\"\n", filename);
            return COMMAND_FAILED;
        }

        std::vector<int> triangleIndices(triangleCount * 3);

        for(int i = 0; i < triangleCount; i++) {
            if(fscanf(fp, "%d %d %d",
                &triangleIndices[i * 3 + 0],
                &triangleIndices[i * 3 + 1],
                &triangleIndices[i * 3 + 2]) != 3)
            {
                printf("ERROR: couldn't read triangle %d from \"%s\"\n", i, filename);
                return COMMAND_FAILED;
            }
        }

        InitState();
        pglViewport(0, 0, info.width, info.height);

        bool done = false;
        float frame_time = 0.0f;
        do {
            int key = InputGetChar();
            switch(key) {
                case 'q': case 'Q': {
                    done = true;
                    break;
                }
                case 'a': {
                    // concat rotation -5 deg around Y
                    break;
                }
                case 'd': {
                    // concat rotation 5 deg around Y
                    break;
                }
                case 'w': {
                    // concat rotation -5 deg around X
                    break;
                }
                case 's': {
                    // concat rotation 5 deg around X
                    break;
                }
            }

            // draw triangles
            VideoModeWaitFrame();
            DrawFrame(frame_time, draw_wireframe, flat_shade, vertices.data(), triangleIndices.data(), triangleIndices.size() / 3, size, center);
            frame_time += 1 / 30.0f;

        } while(!done);

    } catch (std::bad_alloc& ba) {

        printf("Out of memory.\n");
    }

    fclose(fp);

    return COMMAND_CONTINUE;
}

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("spin", 2, AppSpinModel, "filename",
        "spin model in 3D"
        );
}


