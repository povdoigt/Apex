
#ifndef VT100_H
#define VT100_H

/**
 * @file    vt100.h
 * @brief   VT100/ANSI escape sequences for terminal control.
 *
 * These are raw byte sequences sent over UART/USB CDC.
 * The terminal on the host side (PuTTY, Tera Term, etc.) interprets them.
 * The STM32 does not need any library — just embed these strings in your
 * sprintf/CDC_Transmit_FS calls.
 *
 * -------------------------------------------------------------------------
 * USAGE EXAMPLES
 * -------------------------------------------------------------------------
 *
 * A) Compile-time literal parameters (string concatenation):
 *      sprintf(buf, "Line 1\n" "Line 2" VT100_CR VT100_CURSOR_UP(1));
 *
 * B) Runtime parameters (use the _FMT variants with sprintf):
 *      sprintf(buf, VT100_CURSOR_UP_FMT, n_lines);
 *
 * -------------------------------------------------------------------------
 * ⚠ NOTE: Requires a VT100/ANSI-compatible terminal on the host.
 *   Not supported by Arduino Serial Monitor, CoolTerm (basic mode), etc.
 *   Supported by: PuTTY, Tera Term, Minicom, VS Code Serial Monitor, etc.
 * -------------------------------------------------------------------------
 */

/* --------------------------------------------------------------------------
 * Core escape character
 * -------------------------------------------------------------------------- */
#define VT100_ESC           "\033"      /* 0x1B — ESC */
#define VT100_CSI           "\033["     /* Control Sequence Introducer */

/* --------------------------------------------------------------------------
 * Carriage / line control
 * -------------------------------------------------------------------------- */
#define VT100_CR            "\r"        /* Carriage return (begin of current line) */
#define VT100_LF            "\n"        /* Line feed (next line) */
#define VT100_CRLF          "\r\n"      /* New line */

/* --------------------------------------------------------------------------
 * Cursor movement — compile-time literal n
 * Usage: VT100_CURSOR_UP(2)  →  moves cursor up 2 lines
 * n MUST be an integer literal (1, 2, …), not a variable.
 * -------------------------------------------------------------------------- */
#define VT100_CURSOR_UP(n)          "\033[" #n "A"
#define VT100_CURSOR_DOWN(n)        "\033[" #n "B"
#define VT100_CURSOR_RIGHT(n)       "\033[" #n "C"
#define VT100_CURSOR_LEFT(n)        "\033[" #n "D"

#define VT100_CURSOR_COL(n)         "\033[" #n "G"  /* Move to column n */
#define VT100_CURSOR_POS(row, col)  "\033[" #row ";" #col "H"  /* Move to (row, col) */

/* --------------------------------------------------------------------------
 * Cursor movement — runtime format strings (use with sprintf)
 * Usage: sprintf(buf, VT100_CURSOR_UP_FMT, n);
 * -------------------------------------------------------------------------- */
#define VT100_CURSOR_UP_FMT         "\033[%dA"
#define VT100_CURSOR_DOWN_FMT       "\033[%dB"
#define VT100_CURSOR_RIGHT_FMT      "\033[%dC"
#define VT100_CURSOR_LEFT_FMT       "\033[%dD"
#define VT100_CURSOR_COL_FMT        "\033[%dG"
#define VT100_CURSOR_POS_FMT        "\033[%d;%dH"

/* --------------------------------------------------------------------------
 * Erase
 * -------------------------------------------------------------------------- */
#define VT100_ERASE_LINE            "\033[2K"   /* Erase entire current line */
#define VT100_ERASE_LINE_END        "\033[K"    /* Erase from cursor to end of line */
#define VT100_ERASE_LINE_BEGIN      "\033[1K"   /* Erase from beginning of line to cursor */
#define VT100_ERASE_SCREEN          "\033[2J"   /* Erase entire screen */
#define VT100_ERASE_SCREEN_END      "\033[J"    /* Erase from cursor to end of screen */

/* Overwrite current line from the start (CR + erase to end) */
#define VT100_OVERWRITE_LINE        "\r\033[K"

/* --------------------------------------------------------------------------
 * Cursor visibility
 * -------------------------------------------------------------------------- */
#define VT100_CURSOR_HIDE           "\033[?25l"
#define VT100_CURSOR_SHOW           "\033[?25h"

/* Save / restore cursor position */
#define VT100_CURSOR_SAVE           "\033[s"
#define VT100_CURSOR_RESTORE        "\033[u"

/* --------------------------------------------------------------------------
 * Screen
 * -------------------------------------------------------------------------- */
#define VT100_SCREEN_CLEAR          "\033[2J\033[H"   /* Clear screen + go to (1,1) */
#define VT100_SCREEN_HOME           "\033[H"          /* Move cursor to home (1,1) */

/* --------------------------------------------------------------------------
 * Text styles
 * -------------------------------------------------------------------------- */
#define VT100_RESET                 "\033[0m"   /* Reset all attributes */
#define VT100_BOLD                  "\033[1m"
#define VT100_DIM                   "\033[2m"
#define VT100_UNDERLINE             "\033[4m"
#define VT100_BLINK                 "\033[5m"
#define VT100_REVERSE               "\033[7m"   /* Swap foreground / background */
#define VT100_HIDDEN                "\033[8m"

/* --------------------------------------------------------------------------
 * Foreground colors
 * -------------------------------------------------------------------------- */
#define VT100_FG_BLACK              "\033[30m"
#define VT100_FG_RED                "\033[31m"
#define VT100_FG_GREEN              "\033[32m"
#define VT100_FG_YELLOW             "\033[33m"
#define VT100_FG_BLUE               "\033[34m"
#define VT100_FG_MAGENTA            "\033[35m"
#define VT100_FG_CYAN               "\033[36m"
#define VT100_FG_WHITE              "\033[37m"
#define VT100_FG_DEFAULT            "\033[39m"

/* Bright / high-intensity variants */
#define VT100_FG_BRIGHT_BLACK       "\033[90m"
#define VT100_FG_BRIGHT_RED         "\033[91m"
#define VT100_FG_BRIGHT_GREEN       "\033[92m"
#define VT100_FG_BRIGHT_YELLOW      "\033[93m"
#define VT100_FG_BRIGHT_BLUE        "\033[94m"
#define VT100_FG_BRIGHT_MAGENTA     "\033[95m"
#define VT100_FG_BRIGHT_CYAN        "\033[96m"
#define VT100_FG_BRIGHT_WHITE       "\033[97m"

/* --------------------------------------------------------------------------
 * Background colors
 * -------------------------------------------------------------------------- */
#define VT100_BG_BLACK              "\033[40m"
#define VT100_BG_RED                "\033[41m"
#define VT100_BG_GREEN              "\033[42m"
#define VT100_BG_YELLOW             "\033[43m"
#define VT100_BG_BLUE               "\033[44m"
#define VT100_BG_MAGENTA            "\033[45m"
#define VT100_BG_CYAN               "\033[46m"
#define VT100_BG_WHITE              "\033[47m"
#define VT100_BG_DEFAULT            "\033[49m"

/* Bright / high-intensity variants */
#define VT100_BG_BRIGHT_RED         "\033[101m"
#define VT100_BG_BRIGHT_GREEN       "\033[102m"
#define VT100_BG_BRIGHT_YELLOW      "\033[103m"
#define VT100_BG_BRIGHT_BLUE        "\033[104m"
#define VT100_BG_BRIGHT_CYAN        "\033[106m"
#define VT100_BG_BRIGHT_WHITE       "\033[107m"

#endif /* VT100_H */
