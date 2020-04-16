#include <string.h>

#include "videomode.h"

//----------------------------------------------------------------------------
// Functions regarding the textport area

static int textportColumns;
static int textportRows;

static unsigned char *textportBase;
static int textportRowSize;

static int *textportCursorXPtr;
static int *textportCursorYPtr;

static int cursor_x;
static int cursor_y;
static int cursor_hanging = 0;

static void textport_set_cursor(int x, int y)
{
    cursor_hanging = 0;
    if(y < 0) {
        x = 0; y = 0;
    } else if(y >= textportRows) {
        x = textportColumns - 1;
        y = textportRows - 1;
    } else if(x < 0) {
        x = 0;
    } else if(x >= textportColumns) {
        x = textportColumns - 1;
    }
    cursor_x = x;
    cursor_y = y;

    *textportCursorXPtr = x;
    *textportCursorYPtr = y;
}

static int saved_cursor_x;
static int saved_cursor_y;
static int saved_cursor_hanging;

static void save_cursor() 
{
    saved_cursor_x = cursor_x;
    saved_cursor_y = cursor_y;
    saved_cursor_hanging = cursor_hanging;
}

static void restore_cursor() 
{
    textport_set_cursor(saved_cursor_x, saved_cursor_y);
    cursor_hanging = saved_cursor_hanging;
}

void screen_scroll_one_row()
{
    for(int y = 0; y < textportRows - 1; y++) {
        memcpy(textportBase + textportRowSize * y, textportBase + textportRowSize * (y + 1), textportColumns);
    }
    memset(textportBase + textportRowSize * (textportRows - 1), ' ', textportColumns);
}

void process_hanging_cursor()
{
    cursor_hanging = 0;
    cursor_x = 0;
    cursor_y++;
    if(cursor_y >= textportRows) {
	screen_scroll_one_row();
	cursor_y--;
    }
    textport_set_cursor(cursor_x, cursor_y);
}

void screen_init()
{
    textport_set_cursor(0, 0);
}

void screen_clear()
{
    for(int y = 0; y < textportRows; y++) {
        memset(textportBase + textportRowSize * y, ' ', textportColumns);
    }
}

int text_flag_none = 0x00;
int text_flag_inverse = 0x01;


void textport_draw_character(unsigned char c, int flags)
{
    if(cursor_hanging)
	process_hanging_cursor();

    textportBase[textportRowSize * cursor_y + cursor_x] = c;
}

void screen_cursor_rewind()
{
    // XXX cursor_hanging
    cursor_hanging = 0;
    if(cursor_x == 0) {
        if(cursor_y == 0) {
            return;
        } else {
            cursor_x = textportColumns - 1;
            cursor_y--;
            textport_set_cursor(cursor_x, cursor_y);
        }
    } else {
        cursor_x--;
    }
}

void screen_cursor_forward()
{
    if(cursor_hanging)
	process_hanging_cursor();

    if(cursor_x == textportColumns - 1) {
        cursor_hanging = 1;
    } else {
	cursor_x ++;
    }
}

void textport_erase_end_of_screen(int i)
{
    for(int y = cursor_y + 1; y < textportRows; y++) {
        memset(textportBase + textportRowSize * y, ' ', textportColumns);
    }
}

/*
 * eraseline()
 *
 *   Erases part of the line.  The argument "which" can be 0 = to end
 *   of line, 1 = to beginning of line, and 2 = whole line.
 */

void textport_erase_end_of_line(int which)
{
    int start, end;

    switch (which) {
        case 0:
            start = cursor_x;
            end = textportColumns;
            break;
        case 1:
            start = 0;
            end = cursor_x + 1;
            break;
        case 2:
            start = 0;
            end = textportColumns; // Should be region
            break;
        default:
            return;
    }

    int prev_x = cursor_x;
    int prev_y = cursor_y;

    for (int x = start; x < end; x++) {
        textport_set_cursor(x, cursor_y);
        textport_draw_character(' ', text_flag_none);
    }

    textport_set_cursor(prev_x, prev_y);
}

void textport_insert_line(int i)
{
    // printf("XXX INSERT LINE %d\n", i);
}

void textport_delete_line(int i)
{
    // printf("XXX DELETE LINE %d\n", i);
}

void textport_insert_character(int i)
{
    // printf("XXX INSERT CHARACTER %d\n", i);
}

void textport_delete_character(int i)
{
    // printf("XXX DELETE CHARACTER %d\n", i);
}

void textport_line_up()
{
    // printf("XXX LINE UP\n");
}

void textport_line_down()
{
    if(cursor_hanging)
	process_hanging_cursor();
    cursor_y++;
    if(cursor_y >= textportRows) {
        screen_scroll_one_row();
        cursor_y--;
    }
    textport_set_cursor(cursor_x, cursor_y);
}

void textport_carriage_return()
{
    if(cursor_hanging)
	process_hanging_cursor();
    cursor_x = 0;
    textport_set_cursor(cursor_x, cursor_y);
}

int SimplePutchar(int c) 
{
    if(c == '\n') {

        cursor_x = 0;
        cursor_y++;

    } else if(c == 127 || c == '\b') {

        if(cursor_x > 0) {
            cursor_x--;
        }

    } else {

        textportBase[textportRowSize * cursor_y + cursor_x] = c;
        cursor_x++;
        if(cursor_x >= textportColumns) {
            cursor_x = 0;
            cursor_y++;
        }
    }

    if(cursor_y >= textportRows) {
        screen_scroll_one_row();
        cursor_y--;
    }

    *textportCursorXPtr = cursor_x;
    *textportCursorYPtr = cursor_y;

    return 0;
}

//----------------------------------------------------------------------------
// Functions regarding VT102 Terminal processing

//
// borrowed some structure and some code from dt/vt.c
// Thanks, Lawrence and Vallteri!
//
/* The VT100 states for v->state: */
enum VT100_state {
    ESnormal  = 0,		/* Nothing yet */
    ESesc     = 1,		/* Got ESC */
    ESsquare  = 2,		/* Got ESC [ */
    ESgetpars = 3,		/* About to get or getting the parameters */
    ESgotpars = 4,		/* Finished getting the parameters */
    ESfunckey = 5,		/* Function key */
    EShash    = 6,		/* DEC-specific stuff (screen align, etc.) */
    ESsetG0   = 7,		/* Specify the G0 character set */
    ESsetG1   = 8,		/* Specify the G1 character set */
    ESignore  = 9,		/* Ignore this sequence */
};

#define VT_MAXPARS 16

typedef struct terminal_state {
    enum VT100_state state;
    unsigned short tab_stop[10];
    int pars[VT_MAXPARS];
    int npars;
    int ques;
} terminal_state;

terminal_state terminal;

void init_terminal(terminal_state *state)
{
    state->state = ESnormal;

    state->tab_stop[0] = 0x0100;	/* First column not set	 */
    for(int i = 1; i < 9; i++)
        state->tab_stop[i] = 0x0101;

    state->npars = 0;
    state->ques = 0;
}

/*
 * send_string()
 *
 *   Sends the string as if it had been typed at the keyboard.
 */

static void send_string(const char * s)
{
    while (*s) {
        // send_character(vtnum, *s++);
    }
}
/*
 * send_number()
 *
 *   Sends the number as if it had been typed at the keyboard.
 */

static void 
send_number(int num)
{
    char    buf[10];
    int     i = sizeof(buf);

    buf[--i] = '\0';
    do {
        buf[--i] = num % 10 + '0';
        num /= 10;
    } while (num != 0);

    send_string(buf + i);
}

#define VT102ID		"\e[?6c"

/*
 * respond_ID()
 *
 *   Sends the ID of the terminal.
 */

static void 
respond_ID()
{
    send_string(VT102ID);
}
/*
 * status_report()
 *
 *   Sends a string to say that we're OK and alive
 */

static void 
status_report()
{
    // printf("XXX STATUS REPORT\n");
    send_string("\033[0n");
}
/*
 * cursor_report()
 *
 *   Sends the location of the cursor
 */

static void 
cursor_report()
{
    // printf("XXX CURSOR REPORT\n");
    send_string("\e[");
    send_number(cursor_y + 1);
    send_string(";");
    send_number(cursor_x + 1);
    send_string("R");
}

int VT102Putchar(unsigned char c)
{
    if((terminal.state == ESnormal) && (c >= ' ') && (c <= 255)) {
        textport_draw_character(c, text_flag_none);
        screen_cursor_forward();
        return 0;
    }
    
    switch(c) {
	case 0:
            return 0;		/* Ignore nuls (^@)	 */

	case 7:
            // beep();		/* Bell (^G) */
            return 0;

        case 8:
            screen_cursor_rewind();
            return 0;

	case 9: /* Tab (^I) */
            textport_carriage_return();
            // terminal.hanging_cursor = 0;
            do {
                textport_draw_character(' ', text_flag_none);
                screen_cursor_forward();
            } while ((terminal.tab_stop[cursor_x / 16] &
                    (1 << (cursor_x % 16))) == 0);
            return 0;

        case 10: // line feed (^J)
        case 11: // line feed (^K)
        case 12: // form feed (^L)
            textport_line_down();
            return 0;

        case 13: // carriage return (^M)
            textport_carriage_return();
            return 0;

	case 14:
            return 0;		/* Alternate font (^N)	 */

	case 15:
            return 0;		/* Normal font (^O)	 */

	case 24:		/* (^X)			 */
	case 26:
            terminal.state = ESnormal;	/* (^Z)			 */
            return 0;

	case 27:
            terminal.state = ESesc;	/* (^[)			 */
            return 0;

	case 127:
            return 0;		/* Rubout (Delete)	 */

	case 128 + 27:
            terminal.state = ESsquare;	/* Equivalent to ^[[	 */
            return 0;
    }

    switch (terminal.state) {
        case ESesc:
            terminal.state = ESnormal;
            switch (c) {
                case '[':
                    terminal.state = ESsquare;
                    break;

                case 'E':    /* New line         */
                    textport_line_down();
                    textport_carriage_return();
                    break;

                case 'M':    /* Inverse line feed     */
                    textport_line_up();
                    break;

                case 'D':    /* Line feed         */
                    textport_line_down();
                    break;

                case 'H':    /* Set tab stop         */
                    terminal.tab_stop[cursor_x / 16] |=
                        (1 << (cursor_x % 16));
                    break;

                case 'Z':    /* Send ID         */
                    respond_ID();
                    break;

                case '7':    /* Save cursor pos     */
                    save_cursor();
                    break;

                case '8':    /* Restore cursor pos     */
                    restore_cursor();
                    break;

                case '(':    /* Set G0         */
                    terminal.state = ESsetG0;
                    break;

                case ')':    /* Set G1         */
                    terminal.state = ESsetG1;
                    break;

                case '#':    /* Hash             */
                    terminal.state = EShash;
                    break;

                case 'c':    /* Reset terminal     */
                    textport_set_cursor(0, 0);
                    screen_clear();
                    break;

                case '>':    /* Numeric keypad     */
                    /* XXX */
                    break;

                case '=':    /* Appl. keypad         */
                    /* XXX */
                    break;

            }
            break;

        case ESsquare:
            for (int i = 0; i < VT_MAXPARS; i++) {
                terminal.pars[i] = 0;
            }
            terminal.npars = 0;
            terminal.state = ESgetpars;
            if (c == '[') {/* Function key         */
                terminal.state = ESfunckey;
                break;
            }
            terminal.ques = (c == '?');
            if (terminal.ques) {
                break;
            }
            /* FALL THROUGH */
        case ESgetpars:
            if (c == ';' && terminal.npars < VT_MAXPARS - 1) {
                terminal.npars++;
                break;
            } else
                if (c >= '0' && c <= '9') {
                    terminal.pars[terminal.npars] *= 10;
                    terminal.pars[terminal.npars] += c - '0';
                    break;
                } else {
                    terminal.npars++;
                    terminal.state = ESgotpars;
                }
            /* FALL THROUGH */
        case ESgotpars:
            terminal.state = ESnormal;
            switch (c) {
                case 'h':    /* Keyboard enable     */
                    /* XXX */
                    break;
                case 'l':    /* Keyboard disable     */
                    /* XXX */
                    break;
                case 'n':    /* Reports         */
                    if (!terminal.ques) {
                        if (terminal.pars[0] == 5) {
                            status_report();
                        } else
                            if (terminal.pars[0] == 6) {
                                cursor_report();
                            }
                    }
                    break;
            }
            if (terminal.ques) {
                terminal.ques = 0;
                break;
            }
            switch (c) {
                case 'G':    /* Go to column         */
                case '`':
                    textport_set_cursor(terminal.pars[0] - 1, cursor_y);
                    break;
                case 'A':    /* Up             */
                    if (terminal.pars[0] == 0) {
                        terminal.pars[0] = 1;
                    }
                    textport_set_cursor(cursor_x, cursor_y - terminal.pars[0]);
                    break;
                case 'B':    /* Down             */
                case 'e':    /* Down             */
                    if (terminal.pars[0] == 0) {
                        terminal.pars[0] = 1;
                    }
                    textport_set_cursor(cursor_x, cursor_y + terminal.pars[0]);
                    break;
                case 'C':    /* Right         */
                case 'a':    /* Right         */
                    if (terminal.pars[0] == 0) {
                        terminal.pars[0] = 1;
                    }
                    textport_set_cursor(cursor_x + terminal.pars[0], cursor_y);
                    break;
                case 'D':    /* Left             */
                    if (terminal.pars[0] == 0) {
                        terminal.pars[0] = 1;
                    }
                    textport_set_cursor(cursor_x - terminal.pars[0], cursor_y);
                    break;
                case 'E':    /* Down, first column     */
                    if (terminal.pars[0] == 0) {
                        terminal.pars[0] = 1;
                    }
                    textport_set_cursor(0, cursor_y + terminal.pars[0]);
                    break;
                case 'F':    /* Up, first column     */
                    if (terminal.pars[0] == 0) {
                        terminal.pars[0] = 1;
                    }
                    textport_set_cursor(0, cursor_y - terminal.pars[0]);
                    break;
                case 'd':    /* Go to line         */
                    textport_set_cursor(0, terminal.pars[0] - 1);
                    break;
                case 'h':    /* Enter insert mode     */
                    // printf("XXX INSERT MODE\n");
                    // if (terminal.pars[0] == 4)
                        // terminal.attr |= T_INSERT;
                    break;
                case 'l':    /* Exit insert mode     */
                    // printf("XXX EXIT INSERT MODE\n");
                    // if (terminal.pars[0] == 4)
                        // terminal.attr &= ~T_INSERT;
                    break;
                case 'H':    /* Go to location     */
                    textport_set_cursor(terminal.pars[1] - 1, terminal.pars[0] - 1);
                    /* XXX - Fix in all VT-100: */
                    // terminal.hanging_cursor = 0;
                    break;
                case 'J':    /* Clear part of screen     */
                    textport_erase_end_of_screen(terminal.pars[0]);
                    break;
                case 'K':    /* Clear part of line     */
                    textport_erase_end_of_line(terminal.pars[0]);
                    break;
                case 'L':    /* Insert n lines     */
                    textport_insert_line(terminal.pars[0]);
                    break;
                case 'M':    /* Delete n lines     */
                    textport_delete_line(terminal.pars[0]);
                    break;
                case 'P':    /* Delete n characters     */
                    textport_delete_character(terminal.pars[0]);
                    break;
                case 'c':    /* Respond with ID     */
                    if (terminal.pars[0] == 0) {
                        respond_ID();
                    }
                    break;
                case 'g':    /* Clear tab stop(s)     */
                    if (terminal.pars[0]) {
                        terminal.tab_stop[0] = 0;
                        terminal.tab_stop[1] = 0;
                        terminal.tab_stop[2] = 0;
                        terminal.tab_stop[3] = 0;
                        terminal.tab_stop[4] = 0;
                        terminal.tab_stop[5] = 0;
                        terminal.tab_stop[6] = 0;
                        terminal.tab_stop[7] = 0;
                        terminal.tab_stop[8] = 0;
                        terminal.tab_stop[9] = 0;
                    } else {
                        terminal.tab_stop[cursor_x / 16] &=
                            ~(1 << (cursor_x % 16));
                    }
                    break;
                case 'm':    /* Set color         */
                    //if (terminal.npars == 0) {
                        //terminal.npars = 1;
                    //}
                    //for (i = 0; i < terminal.npars; i++) {
                        //setVT100color(v, terminal.pars[i]);
                    //}
                    break;
                case 'r':    /* Set region         */
                    // printf("XXX SET REGION\n");
                    // if (terminal.pars[0] > 0) {
                        // terminal.pars[0]--;
                    // }
                    // if (terminal.pars[1] == 0) {
                        // terminal.pars[1] = terminal.numtrows;
                    // }
                    // if (terminal.pars[0] < terminal.pars[1] &&
                        // terminal.pars[1] <= terminal.numtrows) {
                        // terminal.toptrow = terminal.pars[0];
                        // terminal.bottrow = terminal.pars[1];
                        // terminal.x = 0;
                        // terminal.y = terminal.toptrow;
                    // }
                    break;
                case 's':    /* Save cursor pos     */
                    save_cursor();
                    break;
                case 'u':    /* Restore cursor pos     */
                    restore_cursor();
                    break;
                case '@':    /* Insert n characters     */
                    textport_insert_character(terminal.pars[0]);
                    break;
                case ']':    /* OS-defined         */
                    /* XXX */
                    break;
            }
            break;
        case ESfunckey:
            /* XXX What's the point of this? */
            terminal.state = ESnormal;
            break;
        case EShash:
            terminal.state = ESnormal;
            if (c == '8') {
                /* DEC screen alignment test */
            }
            break;
        case ESsetG0:
            if (c == '0') {
                /* Set graphics character set */
            } else
                if (c == 'B') {
                    /* Set normal character set */
                } else
                    if (c == 'U') {
                        /* Set null character set */
                    } else
                        if (c == 'K') {
                            /* Set user-defined character
                             * set */
                        }
            /* If currently G0, then make active set */
            terminal.state = ESnormal;
            break;
        case ESsetG1:
            if (c == '0') {
                /* Set graphics character set */
            } else
                if (c == 'B') {
                    /* Set normal character set */
                } else
                    if (c == 'U') {
                        /* Set null character set */
                    } else
                        if (c == 'K') {
                            /* Set user-defined character
                             * set */
                        }
            /* If currently G1, then make active set */
            terminal.state = ESnormal;
            break;
        default:
            terminal.state = ESnormal;
    }
    return 0;
}

int VT102Init()
{
    init_terminal(&terminal);

    screen_init();
    screen_clear();

    return 0;
}

//----------------------------------------------------------------------------
// Simple textport management

int vt102Mode = 0;

int TextportPutchar(char c)
{
    if(VideoGetCurrentMode() < 0) {
        return 1;
    }

    if(VideoModeGetType(VideoGetCurrentMode()) != VIDEO_MODE_TEXTPORT) {
        return 1;
    }

    if(vt102Mode) {
        return VT102Putchar(c);
    } else {
        return SimplePutchar(c);
    }
}

int TextportSetRaw(int raw)
{
    int oldvt102Mode = vt102Mode;

    vt102Mode = raw;

    if(!oldvt102Mode && vt102Mode) {
        VT102Init();
    } else if(oldvt102Mode && !vt102Mode) {
        textport_set_cursor(0, textportRows - 1);
    }
    return 0;
}

int TextportSetMode(int mode)
{
    if(VideoModeGetType(mode) != VIDEO_MODE_TEXTPORT) {
        return 1;
    }
    VideoSetMode(mode);

    VideoTextportInfo info;
    VideoTextportParameters params;
    VideoModeGetInfo(VideoGetCurrentMode(), &info);
    VideoModeGetParameters(&params);

    textportCursorXPtr = params.cursorX;
    textportCursorYPtr = params.cursorY;

    textportColumns = info.width;
    textportRows = info.height;
    textportBase = params.base;
    textportRowSize = params.rowSize;

    vt102Mode = 0;
    screen_clear();
    textport_set_cursor(0, textportRows - 1);

    return 0;
}
