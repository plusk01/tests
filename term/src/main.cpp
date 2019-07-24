#include <ncurses.h>
#include <cstring>

bool flag = false;

void printflag()
{
  int row, col;

  static char truemsg[] = "True";
  static char falsemsg[] = "False";

  getmaxyx(stdscr, row, col);

  attron(A_BOLD);

  if (flag) {
    mvprintw(row/2, (col-strlen(truemsg))/2, "%s", truemsg);
  } else 
  {
    mvprintw(row/2, (col-strlen(falsemsg))/2, "%s", falsemsg);
  }
}

// ============================================================================
// ============================================================================

int main(int argc, char *argv[])
{
  initscr();
  clear();

  start_color();

  // init_color(COLOR_RED, 800, 0, 0);
  // init_color(COLOR_YELLOW, 800, 800, 0);

  // init_pair(1, COLOR_WHITE, COLOR_RED);
  // init_pair(2, COLOR_BLACK, COLOR_YELLOW);

  init_color(COLOR_RED, 800, 0, 0);
  init_color(COLOR_YELLOW, 800, 800, 0);

  init_pair(1, COLOR_RED, COLOR_BLACK);
  init_pair(2, COLOR_YELLOW, COLOR_BLACK);

  bkgd(COLOR_PAIR(2));

  printw("Seems that you can use ncurses ...\n");

  curs_set(0); // invisible cursor
  noecho(); // don't echo what getch gets

  //
  // Main loop
  //

  timeout(1000);

  printflag();

  while (true) {
      char ch = getch();
      if (ch == ' ') {
        // printw("Spacebar pressed.\n");

        // toggle flag
        flag = !flag;

        if (flag) {
          bkgd(COLOR_PAIR(1));
        } else {
          bkgd(COLOR_PAIR(2));
        }

        clear();
        printflag();

      } else if (ch == ERR) {
        // printw("Timeout\n");
      }
  }

  endwin();

  return 0;
}