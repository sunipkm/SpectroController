/**
 * @file test.cpp
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief
 * @version See Git tags for version information.
 * @date 2022.03.16
 *
 * @copyright Copyright (c) 2022
 *
 */
#define MEB_DBGLVL MEB_DBG_ALL

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>

#include <iomotor.hpp>
#include "Adafruit/meb_print.h"

#include "scanmotor.hpp"
#include "iomotor.hpp"

#include "ui.hpp"
#include <string>

#define SMSHIELD_ADDR 0x63
#define IOMSHIELD_ADDR 0x60
#define MSHIELD_BUS 1

#define SMOT_REVS 200
#define IOMOT_REVS 50

// scanning motor limit switches
#define SMOT_LS1 36 // backward
#define SMOT_LS2 37 // forward
#define SMOT_PORT 2 // I2C 0x63, port 2

// output side motor limit switches
#define IOMOT_A_LS1 11 // forward
#define IOMOT_A_LS2 13 // backward
#define IOMOT_A_PORT 1 // I2C 0x60, port 1

// input side motor limit switches
#define IOMOT_B_LS1 29 // forward
#define IOMOT_B_LS2 31 // backward
#define IOMOT_B_PORT 2 // I2C 0x60, port 2

#define TRIGIN 18  // GPIO trigger in
#define TRIGOUT 16 // GPIO trigger out

static unsigned int scanmot_current_pos = 0;
const int scanmot_valid_magic = 0xbaaddaad;
const char *pos_fname = (char *)"posinfo.bin";
static unsigned int LoadCurrentPos();
static void InvalidateCurrentPos();
static void ValidateCurrentPos();
static void MotorSetup();
static void MotorCleanup();

void sighandler(int sig)
{
}

#define STEP_TO_CTR(x) (((double)x) / 250.0)
#define CTR_TO_STEP(x) (x * 250)
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

Adafruit::MotorShield *sm_shield = nullptr;
Adafruit::MotorShield *ioshield = nullptr;
ScanMotor *smotor = nullptr;
IOMotor *iomot_out = nullptr;
IOMotor *iomot_in = nullptr;

char *menu1_choices_desc[] = {
    (char *)"Select Input Port",
    (char *)"Select Output Port",
    (char *)"Go to Location",
    (char *)"Step Relative",
    (char *)"Exit Program",
    (char *)NULL};

char *menu_choices_idx[] = {
    (char *)"1:",
    (char *)"2:",
    (char *)"3:",
    (char *)"4:",
    (char *)"5:",
    (char *)NULL};

char *port_menu_desc[] = {
    (char *)"Port A",
    (char *)"Port B",
    (char *)"Back",
    (char *)NULL};

int main()
{
    signal(SIGINT, sighandler);
    atexit(MotorCleanup);
    MotorSetup();
    bprintlf(YELLOW_FG "Current pos: %u == %.2f", scanmot_current_pos, STEP_TO_CTR(scanmot_current_pos));
    ncurses_init();
    refresh();

    // Initialization and drawing of windows' static elements.
    int rows = 0, cols = 0;
    getmaxyx(stdscr, rows, cols);
    WindowsInit(win, win_w, win_h, rows, cols);

    // Menu1 setup.
    ITEM **menu1_items, **menu2_items;
    int c;
    MENU *menu1, *menu2;
    int menu1_n_choices, menu2_n_choices, i;

    // generate_menu(menu1_items, menu1, menu1_n_choices);

    { // Generate the menu.
        menu1_n_choices = ARRAY_SIZE(menu1_choices_desc);
        menu1_items = (ITEM **)calloc(menu1_n_choices, sizeof(ITEM *));
        for (i = 0; i < menu1_n_choices; ++i)
        {
            menu1_items[i] = new_item(menu_choices_idx[i], menu1_choices_desc[i]);
        }
        menu1 = new_menu((ITEM **)menu1_items);
        set_menu_win(menu1, win[1]);
        set_menu_sub(menu1, derwin(win[1], 6, 38, 2, 1));
        // set_menu_sub(my_menu, win[1]);
        set_menu_mark(menu1, " * ");
        post_menu(menu1);
        wrefresh(win[1]);

        menu2_n_choices = ARRAY_SIZE(port_menu_desc);
        menu2_items = (ITEM **)calloc(menu2_n_choices, sizeof(ITEM *));
        for (i = 0; i < menu2_n_choices; ++i)
        {
            menu2_items[i] = new_item(port_menu_desc[i], "");
        }
        menu2 = new_menu(menu2_items);
    }

    // Menu1 nav
    // Makes wgetch nonblocking so the menu isnt hogging all the cycles.
    wtimeout(stdscr, 10);
    unsigned int scanmot_old_pos = scanmot_current_pos;
    std::string iomot_in_port = iomot_in->getStateStr();
    std::string iomot_out_port = iomot_out->getStateStr();
    std::string scanmot_status = smotor->getStateStr();
    std::string iomot_in_port_old = iomot_in_port, iomot_out_port_old = iomot_out_port, scanmot_status_old = scanmot_status;
    bool moving = smotor->isMoving() || (iomot_in->getState() == IOMotor_State::MOVING) || (iomot_out->getState() == IOMotor_State::MOVING);
    bool old_moving = moving;
    while ((c = wgetch(stdscr)) != KEY_F(1))
    {
        // update win 0
        moving = smotor->isMoving() || (iomot_in->getState() == IOMotor_State::MOVING) || (iomot_out->getState() == IOMotor_State::MOVING);
        // do things with moving transition
        if (!old_moving && moving) // print message
        {
            if (smotor->isMoving()) // print motor stop message
            {
                mvwprintw(win[1], 2, 10, "Scanning motor is moving.");
                mvwprintw(win[1], 2, 11, "Press F1, S, Q or Space to stop.");
            }
            else // print iomotor message
            {
                mvwprintw(win[1], 2, 10, "IO Ports are changing.");
                mvwprintw(win[1], 2, 11, "Operations disabled.");
            }
            wrefresh(win[1]);
        }
        else if (old_moving && !moving) // clear message
        {
            mvwprintw(win[1], 2, 10, "                                        ");
            mvwprintw(win[1], 2, 11, "                                        ");
            wrefresh(win[1]);
        }
        // update
        old_moving = moving;
        scanmot_current_pos = smotor->getPos();
        iomot_in_port = iomot_in->getStateStr();
        iomot_out_port = iomot_out->getStateStr();
        scanmot_status = smotor->getStateStr();
        static bool redraw = true;
        if (scanmot_old_pos != scanmot_current_pos)
        {
            redraw = true;
            scanmot_old_pos = scanmot_current_pos;
        }
        if (iomot_in_port_old != iomot_in_port)
        {
            iomot_in_port_old = iomot_in_port;
            redraw = true;
        }
        if (iomot_out_port_old != iomot_out_port)
        {
            iomot_out_port_old = iomot_out_port;
            redraw = true;
        }
        if (iomot_in_port_old != iomot_in_port)
        {
            iomot_out_port_old = iomot_out_port;
            redraw = true;
        }
        if (redraw)
        {
            mvwprintw(win[0], 2, 2, "        ");
            mvwprintw(win[0], 2, 2, "%s", iomot_in_port.c_str());
            mvwprintw(win[0], 2, floor(win0spcg * floor(win_w[0] * cols)), "        ");
            mvwprintw(win[0], 2, floor(win0spcg * floor(win_w[0] * cols)), "%s", iomot_out_port.c_str());
            mvwprintw(win[0], 2, 2 * floor(win0spcg * floor(win_w[0] * cols)), "        ");
            mvwprintw(win[0], 2, 2 * floor(win0spcg * floor(win_w[0] * cols)), "%s", scanmot_status.c_str());
            mvwprintw(win[0], 2, 3 * floor(win0spcg * floor(win_w[0] * cols)), "              ");
            mvwprintw(win[0], 2, 3 * floor(win0spcg * floor(win_w[0] * cols)), "%u", scanmot_current_pos);
            mvwprintw(win[0], 3, 3 * floor(win0spcg * floor(win_w[0] * cols)), "              ");
            mvwprintw(win[0], 3, 3 * floor(win0spcg * floor(win_w[0] * cols)), "%.2f", STEP_TO_CTR(scanmot_current_pos));
            wrefresh(win[0]);
            // redraw = false;
        }
        // Menu handling.
        if (c == KEY_DOWN && !moving)
        {
            menu_driver(menu1, REQ_DOWN_ITEM);
            wrefresh(win[1]);
        }
        else if (c == KEY_UP && !moving)
        {
            menu_driver(menu1, REQ_UP_ITEM);
            wrefresh(win[1]);
        }
        else if (c == '\n' && !moving)
        {
            int sel = item_index(current_item(menu1));
            if (sel == 0 || sel == 1) // input/output port select
            {
                // Generate the menu.
                unpost_menu(menu1);
                wclear(win[1]);
                if (sel == 0)
                    mvwprintw(win[1], 0, 2, " Input Port ");
                else if (sel == 1)
                    mvwprintw(win[1], 0, 2, " Output Port ");
                else
                    mvwprintw(win[1], 0, 2, " Unknown %d ", sel);
                box(win[1], 0, 0);
                set_menu_win(menu2, win[1]);
                set_menu_sub(menu2, derwin(win[1], 6, 38, 2, 1));
                set_menu_mark(menu2, " * ");
                post_menu(menu2);
                wrefresh(win[1]);
                while ((c = wgetch(stdscr)) != KEY_F(1))
                {
                    if (c == KEY_DOWN && !moving)
                    {
                        menu_driver(menu2, REQ_DOWN_ITEM);
                        wrefresh(win[1]);
                    }
                    else if (c == KEY_UP && !moving)
                    {
                        menu_driver(menu2, REQ_UP_ITEM);
                        wrefresh(win[1]);
                    }
                    else if (c == '\n' && !moving)
                    {
                        int idx = item_index(current_item(menu2));
                        // do action
                        IOMotor_State st = IOMotor_State::ERROR;
                        if (idx == 0)
                            st = IOMotor_State::PORTA;
                        else if (idx == 1)
                            st = IOMotor_State::PORTB;
                        else if (idx == 2)
                            goto ret_menu1;
                        if (sel == 0)
                            iomot_in->setState(st);
                        else if (sel == 1)
                            iomot_out->setState(st);
                        // revert back to menu 1
ret_menu1:
                        unpost_menu(menu2);
                        wclear(win[1]);
                        mvwprintw(win[1], 0, 2, " Options ", sel);
                        box(win[1], 0, 0);
                        set_menu_win(menu1, win[1]);
                        set_menu_sub(menu1, derwin(win[1], 6, 38, 2, 1));
                        set_menu_mark(menu1, " * ");
                        post_menu(menu1);
                        wrefresh(win[1]);
                        break;
                    }
                }
            }
            else if (sel == 2 || sel == 3) // abs/rel position select
            {
                unpost_menu(menu1);
                wclear(win[1]);
                mvwprintw(win[1], 0, 2, " Location Input ");
                box(win[1], 0, 0);
                if (sel == 2)
                {
                    mvwprintw(win[1], 2, 2, "Enter absolute position (current: %u): ", scanmot_current_pos);
                }
                else if (sel == 3)
                {
                    mvwprintw(win[1], 2, 2, "Enter relative position (current: %u): ", scanmot_current_pos);
                }
                echo();
                wrefresh(win[1]);
                nodelay(win[1], false);
                int newloc = 0;
                wscanw(win[1], "%d", &newloc);
                noecho();
                wtimeout(win[1], 5);
                bool move = false;
                if (newloc != 0)
                {
                    if (sel == 3)
                    {
                        if (newloc > 1000)
                            newloc = 1000;
                        if (newloc < -1000)
                            newloc = -1000;
                        newloc = ((int)scanmot_current_pos) + newloc;
                        move = true;
                    }
                    else if (sel == 2 && newloc > 0)
                    {
                        move = true;
                    }
                    if (move)
                        smotor->goToPos(newloc);
                }
                wclear(win[1]);
                mvwprintw(win[1], 0, 2, " Options ");
                box(win[1], 0, 0);
                set_menu_win(menu1, win[1]);
                set_menu_sub(menu1, derwin(win[1], 6, 38, 2, 1));
                // set_menu_sub(my_menu, win[1]);
                set_menu_mark(menu1, " * ");
                post_menu(menu1);
                wrefresh(win[1]);
            }
            else if (sel == 4) // exit
            {
                break;
            }
        }
        else if ((c == '\n' || c == 's' || c == 'S' || c == ' ' || c == 'q' || c == 'Q' || c == KEY_F(4)) && smotor->isMoving())
        {
            smotor->eStop();
        }
        else if (c == KEY_RESIZE)
        {
            // Clean-up.
            // ncurses_cleanup();

            // // Re-initialization.
            // ncurses_init();
            // refresh();

            // getmaxyx(stdscr, rows, cols);
            // WindowsInit(win, win_w, win_h, rows, cols);

            // { // Generate the menu.
            //     menu1_n_choices = ARRAY_SIZE(menu1_choices);
            //     menu1_items = (ITEM **)calloc(menu1_n_choices, sizeof(ITEM *));
            //     for (i = 0; i < menu1_n_choices; ++i)
            //     {
            //         menu1_items[i] = new_item(menu1_choices[i], menu1_choices_desc[i]);
            //     }
            //     menu1 = new_menu((ITEM **)menu1_items);
            //     set_menu_win(menu1, win[1]);
            //     set_menu_sub(menu1, derwin(win[1], 6, 38, 2, 1));
            //     // set_menu_sub(my_menu, win[1]);
            //     set_menu_mark(menu1, " * ");
            //     post_menu(menu1);
            //     wrefresh(win[1]);
            // }
        }
        else
        {
        }
    }

    DestroyMenu(menu1, menu1_n_choices, menu1_items);
    DestroyMenu(menu2, menu2_n_choices, menu2_items);
    WindowsDestroy(win, ARRAY_SIZE(win));
    refresh();
    ncurses_cleanup();

    smotor->eStop(); // stop in case moving
    scanmot_current_pos = smotor->getPos();

    return 0;
}

static void MotorCleanup()
{
    delete smotor;
    delete iomot_out;
    delete iomot_in;
    delete sm_shield;
}

static void MotorSetup()
{
    // Instantiation.
    atexit(ValidateCurrentPos); // validate current position at exit
    scanmot_current_pos = LoadCurrentPos();
    // dbprintlf("Instantiating MotorShield and StepperMotor with address %d, bus %d, %d steps, and port %d.", MSHIELD_ADDR, MSHIELD_BUS, M_STEPS, M_PORT);
    sm_shield = new Adafruit::MotorShield(SMSHIELD_ADDR, MSHIELD_BUS);
    try
    {
        sm_shield->begin();
    }
    catch (std::exception &e)
    {
        dbprintlf("Exception: %s.", e.what());
    }
    Adafruit::StepperMotor *scanstepper = sm_shield->getStepper(SMOT_REVS, SMOT_PORT);

    ioshield = new Adafruit::MotorShield(IOMSHIELD_ADDR, MSHIELD_BUS);
    try
    {
        ioshield->begin();
    }
    catch (const std::exception &e)
    {
        dbprintlf("Exception: %s.", e.what());
    }
    Adafruit::StepperMotor *iostepper_out = ioshield->getStepper(IOMOT_REVS, IOMOT_A_PORT);
    Adafruit::StepperMotor *iostepper_in = ioshield->getStepper(IOMOT_REVS, IOMOT_B_PORT);

    smotor = new ScanMotor(scanstepper, SMOT_LS1, Adafruit::MotorDir::BACKWARD, SMOT_LS2, Adafruit::MotorDir::FORWARD, scanmot_current_pos, &InvalidateCurrentPos);
    iomot_out = new IOMotor(iostepper_out, IOMOT_A_LS1, IOMOT_A_LS2, true);
    iomot_in = new IOMotor(iostepper_in, IOMOT_B_LS1, IOMOT_B_LS2, false);

    bprintlf(GREEN_FG "Current pos: %u == %.2lf", scanmot_current_pos, STEP_TO_CTR(scanmot_current_pos));
    if (smotor->getState() == ScanMotor_State::LS1)
    {
        while (smotor->getState() != ScanMotor_State::GOOD)
        {
            scanmot_current_pos += smotor->posDelta(1, Adafruit::MotorDir::FORWARD, true);
        }
        scanmot_current_pos += smotor->posDelta(200, Adafruit::MotorDir::FORWARD);
    }
    else if (smotor->getState() == ScanMotor_State::LS2)
    {
        while (smotor->getState() != ScanMotor_State::GOOD)
        {
            scanmot_current_pos += smotor->posDelta(1, Adafruit::MotorDir::BACKWARD, true);
        }
        scanmot_current_pos += smotor->posDelta(200, Adafruit::MotorDir::BACKWARD);
    }
    else if (smotor->getState() == ScanMotor_State::ERROR)
    {
        dbprintlf("In error state, check wiring.");
        exit(0);
    }
    bprintlf(GREEN_FG "Setup complete, current pos: %u == %.2lf", scanmot_current_pos, STEP_TO_CTR(scanmot_current_pos));
}

static unsigned int LoadCurrentPos()
{
    // 1. Check if file exists, if not ask for step counter and clear all calibration
    int fd = open(pos_fname, O_RDWR | O_SYNC);
    unsigned int current_pos = 0;
    if (fd < 0)
    {
        bool valid = false;
        double readout;
        char buf[50];
        do
        {
            bprintlf(RED_FG "Current position not known, please enter current counter readout (must be in X.XX format): ");
            scanf("%49s", buf);
            char *loc = NULL;
            if ((loc = strchr(buf, '.')) == NULL)
            {
                bprintlf(RED_FG "Does not include '.'.");
                continue;
            }
            else if (strlen(loc) > 3)
            {
                bprintlf("Entered: %s, more digits after decimal point.", buf);
                continue;
            }
            else if (strlen(loc) < 3)
            {
                bprintlf("Entered: %s, not enough digits after decimal point.", buf);
                continue;
            }
            else
            {
                readout = atof(buf);
                if (readout <= 0)
                {
                    bprintlf("Value %.3f invalid.", readout);
                    continue;
                }
                current_pos = CTR_TO_STEP(readout);
                valid = true;
            }
        } while (!valid);
        int retry = 10;
        while (fd <= 0 && retry--)
            fd = open(pos_fname, O_CREAT);
        if (fd <= 0 && retry == 0)
        {
            dbprintlf("Could not create location save file.");
            exit(0);
        }
        int ret = 0;
        retry = 10;
        do
        {
            ret = write(fd, &current_pos, sizeof(current_pos));
            if (ret != sizeof(current_pos))
                lseek(fd, 0, SEEK_SET);
        } while (ret != sizeof(current_pos) && retry--);
        if (ret <= 0 && retry == 0)
        {
            dbprintlf("Could not store current position");
            close(fd);
            exit(0);
        }
        lseek(fd, sizeof(current_pos), SEEK_SET);
        retry = 10;
        do
        {
            ret = write(fd, &scanmot_valid_magic, sizeof(scanmot_valid_magic));
            if (ret != sizeof(scanmot_valid_magic))
                lseek(fd, sizeof(current_pos), SEEK_SET);
        } while (ret != sizeof(scanmot_valid_magic) && retry--);
        if (ret <= 0 && retry == 0)
        {
            dbprintlf("Could not store valid magic to save file.");
            close(fd);
            exit(0);
        }
        // wrote everything
        return current_pos;
    }
    // 2. File exists
    // step 1. Check structural validity
    off_t sz = lseek(fd, 0, SEEK_END);
    if (sz != sizeof(scanmot_valid_magic) + sizeof(scanmot_current_pos))
    {
        dbprintlf("Size of position file %s: %u, invalid. Please delete the file and restart the program.", pos_fname, (unsigned int)sz);
        close(fd);
        exit(0);
    }
    lseek(fd, sizeof(scanmot_current_pos), SEEK_SET);
    int magic = 0;
    read(fd, &magic, sizeof(scanmot_valid_magic));
    if (magic != scanmot_valid_magic)
    {
        dbprintlf("Magic 0x%x is invalid. Please delete the file %s and restart the program.", magic, pos_fname);
        close(fd);
        exit(0);
    }
    lseek(fd, 0, SEEK_SET);
    read(fd, &current_pos, sizeof(current_pos));
    if (current_pos == 0)
    {
        dbprintlf("Current position is 0, please delete the file %s and restart the program.", pos_fname);
        close(fd);
        exit(0);
    }
    return current_pos;
}

static void InvalidateCurrentPos()
{
    static bool firstRun = true;
    if (firstRun)
    {
        firstRun = false;
    }
    else
    {
        return;
    }
    const int badmagic = 0xdeadbeef;
    int fd = open(pos_fname, O_RDWR | O_SYNC);
    if (fd <= 0)
    {
        dbprintlf("%s file not found.", pos_fname);
        exit(0);
    }
    lseek(fd, sizeof(scanmot_current_pos), SEEK_SET);
    int retry = 10;
    int ret;
    do
    {
        ret = write(fd, &badmagic, sizeof(scanmot_valid_magic));
        if (ret != sizeof(scanmot_valid_magic))
            lseek(fd, sizeof(scanmot_current_pos), SEEK_SET);
    } while (ret != sizeof(scanmot_valid_magic) && retry--);
    if (ret <= 0 && retry == 0)
    {
        dbprintlf("Could not store invalid magic to save file.");
        close(fd);
        exit(0);
    }
    return;
}

static void ValidateCurrentPos()
{
    int fd = open(pos_fname, O_RDWR | O_SYNC);
    if (fd <= 0)
    {
        dbprintlf("%s file not found.", pos_fname);
        exit(0);
    }
    lseek(fd, 0, SEEK_SET);
    int retry = 10;
    int ret;
    do
    {
        ret = write(fd, &scanmot_current_pos, sizeof(scanmot_current_pos));
        if (ret != sizeof(scanmot_current_pos))
            lseek(fd, 0, SEEK_SET);
    } while (ret != sizeof(scanmot_current_pos) && retry--);
    if (ret <= 0 && retry == 0)
    {
        dbprintlf("Could not store current position to save file.");
        close(fd);
        return;
    }
    lseek(fd, sizeof(scanmot_current_pos), SEEK_SET);
    retry = 10;
    do
    {
        ret = write(fd, &scanmot_valid_magic, sizeof(scanmot_valid_magic));
        if (ret != sizeof(scanmot_valid_magic))
            lseek(fd, sizeof(scanmot_current_pos), SEEK_SET);
    } while (ret != sizeof(scanmot_valid_magic) && retry--);
    if (ret <= 0 && retry == 0)
    {
        dbprintlf("Could not store invalid magic to save file.");
        close(fd);
        return;
    }
    close(fd);
    bprintlf(GREEN_FG "Final position: %u == %.2f", scanmot_current_pos, scanmot_current_pos * 40.0 / 10000);
    return;
}

WINDOW *InitWin(int col, int row, int cols, int rows)
{
    WINDOW *local_win;

    local_win = newwin(rows, cols, row, col);
    box(local_win, 0, 0); // 0, 0 gives default characters for the vertical and horizontal lines.
    wrefresh(local_win);  // Show that box.

    return local_win;
}

void DestroyWin(WINDOW *local_win)
{
    /* box(local_win, ' ', ' '); : This won't produce the desired
     * result of erasing the window. It will leave it's four corners
     * and so an ugly remnant of window.
     */
    wborder(local_win, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
    /* The parameters taken are
     * 1. win: the window on which to operate
     * 2. ls: character to be used for the left side of the window
     * 3. rs: character to be used for the right side of the window
     * 4. ts: character to be used for the top side of the window
     * 5. bs: character to be used for the bottom side of the window
     * 6. tl: character to be used for the top left corner of the window
     * 7. tr: character to be used for the top right corner of the window
     * 8. bl: character to be used for the bottom left corner of the window
     * 9. br: character to be used for the bottom right corner of the window
     */
    wrefresh(local_win);
    delwin(local_win);
}

void DestroyMenu(MENU *menu, int n_choices, ITEM **items)
{
    // Unpost and free all the memory taken up.
    unpost_menu(menu);
    free_menu(menu);
    for (int i = 0; i < n_choices; ++i)
    {
        free_item(items[i]);
    }
}

void WindowsInit(WINDOW *win[], float win_w[], float win_h[], int rows, int cols)
{
    int win0w = floor(win_w[0] * cols);
    int win0h = floor(win_h[0] * rows);
    int win1w = floor(win_w[1] * cols);
    int win1h = floor(win_h[1] * rows);

    win0h = (win0h < winmin_h[0]) ? winmin_h[0] : win0h;
    win1h = (win1h < winmin_h[1]) ? winmin_h[1] : win1h;

    // int win1y = floor(1 + (win_h[0] * rows));
    // int win2y = floor(1 + (win_h[0] * rows));

    win[0] = InitWin(0, 0, win0w, win0h);
    {
        mvwprintw(win[0], 0, 2, " Status ");
        mvwprintw(win[0], 1, 2, "Input");
        mvwprintw(win[0], 1, floor(win0spcg * win0w), "Output");
        mvwprintw(win[0], 1, 2 * floor(win0spcg * win0w), "Scan");
        mvwprintw(win[0], 1, 3 * floor(win0spcg * win0w), "Step");
        mvwprintw(win[0], win0h - 1, win0w - 10, " %dx%d ", win0w, win0h);
        wrefresh(win[0]);
    }

    win[1] = InitWin(0, floor(win0h), win1w, win1h);
    {
        mvwprintw(win[1], 0, 2, " Options ");
        mvwprintw(win[1], win1h - 1, win1w - 10, " %dx%d ", win1w, win1h);
        wrefresh(win[1]);
    }
}

void WindowsDestroy(WINDOW *win[], int num_win)
{
    if (win == NULL || win == nullptr)
    {
        return;
    }

    for (int i = 0; i < num_win; i++)
    {
        DestroyWin(win[i]);
    }
}

void ncurses_init()
{
    initscr();
    cbreak();
    noecho(); // Doesn't echo input during getch().
    keypad(stdscr, TRUE);
}

void ncurses_cleanup()
{
    endwin();
    clear();
}
