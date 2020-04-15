#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    int width = atoi(argv[1]);
    int height = atoi(argv[2]);
    fwrite(&width, sizeof(width), 1, stdout);
    fwrite(&height, sizeof(height), 1, stdout);
}
