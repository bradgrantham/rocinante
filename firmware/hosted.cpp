#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rocinante.h"
#include "videomode.h"

// c++ -I. hosted.cpp -x c graphics.c utility.c apps/showimage.c -o hosted

#if 0

#define GL_SILENCE_DEPRECATION

#if defined(__linux__)
#include <GL/glew.h>
#endif // defined(__linux__)

#define GLFW_INCLUDE_GLCOREARB
#include <GLFW/glfw3.h>

namespace UserInterface
{

int screenWidth = 512; 
int screenHeight = 512; 
int windowWidth;
int windowHeight;
static GLFWwindow* window;

static void resize(GLFWwindow *window, int x, int y)
{
    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);
}

static void errorCallback(int error, const char* description)
{
    fprintf(stderr, "GLFW: %s\n", description);
}

static void initialize()
{
    glfwSetErrorCallback(errorCallback);

    if(!glfwInit())
        exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); 

    // glfwWindowHint(GLFW_SAMPLES, 4);
    window = glfwCreateWindow(screenWidth, screenHeight, "Rocinante", NULL, NULL);
    if (!window) {
        glfwTerminate();
        fprintf(stdout, "Couldn't open main window\n");
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
    // printf("GL_RENDERER: %s\n", glGetString(GL_RENDERER));
    // printf("GL_VERSION: %s\n", glGetString(GL_VERSION));

    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    CheckOpenGL(__FILE__, __LINE__);

    glfwSetKeyCallback(window, key);
    // glfwSetMouseButtonCallback(window, button);
    // glfwSetCursorPosCallback(window, motion);
    // glfwSetScrollCallback(window, scroll);
    glfwSetFramebufferSizeCallback(window, resize);
    glfwSetWindowRefreshCallback(window, redraw);
    CheckOpenGL(__FILE__, __LINE__);
}

};

#endif

enum { MAX_COMMANDS = 50};
Command commands[MAX_COMMANDS];
int commandsCount = 0;

int RegisterApp(const char* name, int minWords, ProcessCommandFunc go, const char *form, const char *help)
{
    if(commandsCount >= MAX_COMMANDS) {
        printf("maximum command count reached!\n");
        return COMMAND_ADD_FAILED;
    }
    Command *newcmd = commands + commandsCount;
    commandsCount++;

    newcmd->name = name;
    newcmd->minWords = minWords;
    newcmd->go = go;
    // Need to close all open files,
    newcmd->form = form;
    newcmd->help = help;

    return 0;
}

void usage()
{
    int maxNeeded = 0;
    for(int which = 0; which < commandsCount; which++) {
        Command *cmd = commands + which;
        int needed = strlen(cmd->name) + 1 + strlen(cmd->form);
        maxNeeded = (needed > maxNeeded) ? needed : maxNeeded;
    }
    for(int which = 0; which < commandsCount; which++) {
        Command *cmd = commands + which;
        printf("%s %s", cmd->name, cmd->form);
        // XXX some day be smarter about word wrap etc
        printf("%*s - %s\n", maxNeeded - strlen(cmd->name) - 1 - strlen(cmd->form), "", cmd->help);
    }
}


int VideoGetCurrentMode()
{
    return 0;
}

void VideoModeGetInfo(int n, void *info)
{
    ((VideoPixmapInfo*)info)->width = 384;
    ((VideoPixmapInfo*)info)->height = 128;
    ((VideoPixmapInfo*)info)->pixelFormat = VideoPixmapInfo::PALETTE_8BIT;
    ((VideoPixmapInfo*)info)->paletteSize = 256;
    ((VideoPixmapInfo*)info)->color = 1;
    ((VideoPixmapInfo*)info)->overscan = 0;
}

unsigned char pixmap[128][384];
unsigned char palettes[2][256][3];
int rowPalettes[512];

int VideoModeSetPaletteEntry(int palette, int entry, float r, float g, float b)
{
    palettes[palette][entry][0] = r / 255.0f;
    palettes[palette][entry][1] = g / 255.0f;
    palettes[palette][entry][2] = b / 255.0f;
    return 1;
}

void VideoModeGetParameters(void *params)
{
    ((VideoPixmapParameters*)params)->base = (unsigned char*)pixmap;
    ((VideoPixmapParameters*)params)->rowSize = 384;
}

enum VideoModeType VideoModeGetType(int n)
{
    return VIDEO_MODE_PIXMAP;
}

int VideoModeSetRowPalette(int row, int palette)
{
    rowPalettes[row] = palette;
    return 1;
}

int main(int argc, char **argv)
{
    if(argc > 1) {
        int i;
        for(i = 0; i < commandsCount; i++) {
            if(strcmp(argv[1], commands[i].name) == 0) {
                break;
            }
        }
        if(i == commandsCount) { 
            usage();
            exit(EXIT_FAILURE);
        } else {
            commands[i].go(argc - 1, argv + 1);
            FILE *fp = fopen("color.ppm", "wb");
            fprintf(fp, "P6 384 128 255\n");
            for(int y = 0; y < 128; y++) {
                for(int x = 0; x < 384; x++) {
                    int value = pixmap[y][x];
                    fwrite(palettes[rowPalettes[y]][value], 3, 1, fp);
                }
            }
        }
    } else {
        usage();
        exit(EXIT_FAILURE);
    }
    exit(EXIT_FAILURE);
}
