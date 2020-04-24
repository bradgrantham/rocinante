#include <stdio.h>
#include "logprintf.h"

int gDebugLevel = DEBUG_WARNINGS;

void logprintf(int level, char *fmt, ...)
{
    va_list args;

    if(level > gDebugLevel)
        return;

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

