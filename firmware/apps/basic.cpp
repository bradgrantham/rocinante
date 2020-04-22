#include <string>
#include <stdio.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "ff.h"

enum Error {
    SYNTAX_ERROR,
};

struct Result {
    Error error;
};

struct token {
    virtual Result evaluate() = 0;
};

/*
kinds of tokens
    string constant
    integer constant
        hex numbers &Habcd
    float constant
        <= 7 digits
        10E5
        50!
    double constant
        >= 8 digits
        10D5
        50#
    variables
        integer %
        float !
        double *
        N$
        ABC # implicitly a float... ?
        DEFINT
        DEFSTR
        DEFDBL
        DEFSNG
        ARRAY(N), ARRAY(M,N) # max dimensions is 255
    conversion?
    operation
        +, -, *, ^, / is float division
        \ is integer division
        MOD
        =, <>
        NOT, AND, OR, XOR
        IMP, EQV
        string concatenation +
        string equivalence = < > <>
    aggregate operation with ()
    AUTO # start entering lines with automatic numbers
    CALL # jump to an address - skip?
    CHAIN # call to another BASIC program - skip? and COMMON, MERGE, 
    CLEAR # clear variables, ignore the parameters
    CLOAD # cassette load - skip
    CLOSE # close a file
    CONT # continue execution where left off
    CSAVE # cassette save - skip
    DATA # list of constants for READ
    DEF FN # define function
    DEF USR # assembly function
*/

const std::string foo = "foobar";

static int AppBASIC(int argc, char **argv)
{
    printf("preinitialized foo is \"%s\"\n", foo.c_str());
    return COMMAND_SUCCESS;
}

extern "C" {

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("basic", 1, AppBASIC, "[prog.bas]",
        "RUN BASIC"
        );
}

};
