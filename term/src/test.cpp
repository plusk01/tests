#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <curses.h>
#include <time.h>

// Variables
int nlines;
int ncols;
int x;
int y;
int y0;
int x0;
int input;

// Constants
const int MAX_LINES = 10;
const int MAX_COLUMNS = 10;

// Main function
int main(void) {
    WINDOW * mainWindow;

    // Initialize ncurses
    if ( (mainWindow = initscr()) == NULL) {
        fprintf(stderr, "Could not initialize ncurses!\n");
        exit(EXIT_FAILURE);
    }

    // Call function to use color
    start_color();

    // Create my own color pairs
    init_pair(1, COLOR_CYAN, COLOR_BLACK);
    init_pair(2, COLOR_BLUE, COLOR_RED);

    // First clear off the screen
    clear();

    // Move the cursor
    y = 8;
    x = 30;
    move(y, x);

    // Refresh
    refresh();

    // Test output - working
    // printw("Testing...");

    waddch(mainWindow, 'T' | A_UNDERLINE | COLOR_PAIR(1));
    waddch(mainWindow, 'E' | A_UNDERLINE | COLOR_PAIR(2));
    waddch(mainWindow, 'S' | A_UNDERLINE | COLOR_PAIR(1));
    waddch(mainWindow, 'T' | A_UNDERLINE | COLOR_PAIR(2));
    waddch(mainWindow, 'I' | A_UNDERLINE | COLOR_PAIR(1));
    waddch(mainWindow, 'N' | A_UNDERLINE | COLOR_PAIR(2));
    waddch(mainWindow, 'G' | A_UNDERLINE | COLOR_PAIR(1));
    waddch(mainWindow, '.' | A_UNDERLINE | COLOR_PAIR(2));
    waddch(mainWindow, '.' | A_UNDERLINE | COLOR_PAIR(1));
    waddch(mainWindow, '.' | A_UNDERLINE | COLOR_PAIR(1));
    waddch(mainWindow, '.' | A_UNDERLINE | COLOR_PAIR(2));

    // Make background a different color
    wattron(mainWindow, COLOR_BLACK);

    // Hold until user inputs a character
    input =  getch();

    // Clean up
    delwin(mainWindow);
    endwin();
    refresh();

    return EXIT_SUCCESS;
}