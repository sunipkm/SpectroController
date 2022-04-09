/**
 * @file main_ui.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com), modified from Mit Bailey's code.
 * @brief UI helper for Main UI and control for the modified SPEX 1000M Monochromator.
 * @version 1.0
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __MAIN_UI_H_
#define __MAIN_UI_H

#include <ncurses.h>
#include <menu.h>

#define WIN_READ_TIMEOUT 60 ///< wgetch() timeout, in ms

#define MIN_STATUS_WIN 9 ///< Status window min height
#define MIN_OPTS_WIN 16 ///< Options window min height

#define MIN_ROWS (MIN_STATUS_WIN + MIN_OPTS_WIN) ///< Total minimum window height
#define MIN_COLS 80 ///< Minimum window width

static WINDOW *win[2] = {0}; // Pointer to windows
static float win_w[2] = {1.f, 1.0f}; // Window width modifier
static float win_h[2] = {0.35, 0.65}; // Window height modifier
static int winmin_h[2] = {MIN_STATUS_WIN, MIN_OPTS_WIN}; // Window minimum height specifier

/**
 * @brief Helper function for WindowsInit, initializes a single window.
 *
 * @param col Window origin Y
 * @param row Window origin X
 * @param cols Window width
 * @param rows Window height
 * @return WINDOW*
 */
WINDOW *InitWin(int col, int row, int cols, int rows);

/**
 * @brief Helper function for WindowsDestroy, destroys a single window.
 *
 * @param local_win
 */
void DestroyWin(WINDOW *local_win);

/**
 * @brief Helper function to release menu resources.
 * 
 * @param menu Pointer to menu
 * @param n_choices Number of items in menu
 * @param items Pointer to items
 */
void DestroyMenu(MENU *menu, int n_choices, ITEM **items);

/**
 * @brief Initializes all windows.
 *
 * @param win Array of windows to create
 * @param win_w Array of window width specifiers
 * @param win_h Array of window height specifiers
 * @param rows Available height
 * @param cols Available width
 */
void WindowsInit(WINDOW *win[], float win_w[], float win_h[], int rows, int cols);

/**
 * @brief Destroys all windows.
 *
 * @param win
 * @param num_win
 */
void WindowsDestroy(WINDOW *win[], int num_win);

/**
 * @brief Performs NCURSES-specific initialization.
 *
 */
void ncurses_init();

/**
 * @brief Performs NCURSES-specific cleanup.
 *
 */
void ncurses_cleanup();

#endif