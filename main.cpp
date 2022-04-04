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

#include <iomotor.hpp>
#include "Adafruit/meb_print.h"

#include "scanmotor.hpp"
#include "iomotor.hpp"

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
void ValidateCurrentPos();

int main()
{
    // Instantiation.
    atexit(ValidateCurrentPos); // validate current position at exit
    scanmot_current_pos = LoadCurrentPos();
    // dbprintlf("Instantiating MotorShield and StepperMotor with address %d, bus %d, %d steps, and port %d.", MSHIELD_ADDR, MSHIELD_BUS, M_STEPS, M_PORT);
    Adafruit::MotorShield sm_shield(SMSHIELD_ADDR, MSHIELD_BUS);
    try
    {
        sm_shield.begin();
    }
    catch (std::exception &e)
    {
        dbprintlf("Exception: %s.", e.what());
    }
    Adafruit::StepperMotor *scanstepper = sm_shield.getStepper(SMOT_REVS, SMOT_PORT);

    Adafruit::MotorShield io_shield(IOMSHIELD_ADDR, MSHIELD_BUS);
    try
    {
        io_shield.begin();
    }
    catch (const std::exception &e)
    {
        dbprintlf("Exception: %s.", e.what());
    }
    Adafruit::StepperMotor *iostepper_out = io_shield.getStepper(IOMOT_REVS, IOMOT_A_PORT);
    Adafruit::StepperMotor *iostepper_in = io_shield.getStepper(IOMOT_REVS, IOMOT_B_PORT);

    IOMotor iomot_out(iostepper_out, IOMOT_A_LS1, IOMOT_A_LS2, true);
    IOMotor iomot_in(iostepper_in, IOMOT_B_LS1, IOMOT_B_LS2, true);
    ScanMotor smotor(scanstepper, SMOT_LS1, Adafruit::MotorDir::BACKWARD, SMOT_LS2, Adafruit::MotorDir::FORWARD, scanmot_current_pos, &InvalidateCurrentPos);

    printf("Current pos: %u == %.2f\n", scanmot_current_pos, scanmot_current_pos * 40.0 / 10000);
    printf("Press any key to continue...");
    getchar();
    if (smotor.getState() == ScanMotor_State::LS1)
    {
        while (smotor.getState() != ScanMotor_State::OK)
        {
            scanmot_current_pos += smotor.posDelta(1, Adafruit::MotorDir::FORWARD, true);
        }
        scanmot_current_pos += smotor.posDelta(200, Adafruit::MotorDir::FORWARD);
    }
    else if (smotor.getState() == ScanMotor_State::LS2)
    {
        while (smotor.getState() != ScanMotor_State::OK)
        {
            scanmot_current_pos += smotor.posDelta(1, Adafruit::MotorDir::FORWARD, true);
        }
        scanmot_current_pos += smotor.posDelta(200, Adafruit::MotorDir::FORWARD);
    }
    else if (smotor.getState() == ScanMotor_State::ERROR)
    {
        dbprintlf("")
    }
    printf("Press any key to continue...");
    getchar();
    while (smotor.getState() == ScanMotor_State::OK)
    {
        scanmot_current_pos -= smotor.posDelta(200, Adafruit::MotorDir::BACKWARD);
    }
    printf("Current pos: %u == %.2f\n", scanmot_current_pos, scanmot_current_pos * 40.0 / 10000);

    return 0;
}

unsigned int LoadCurrentPos()
{
    // 1. Check if file exists, if not ask for step counter and clear all calibration
    int fd = open(pos_fname, O_RDWR | O_SYNC);
    unsigned int current_pos = 0;
    if (fd < 0)
    {
        bool valid = false;
        float readout;
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
                current_pos = readout * 250;
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
    dbprintlf("Called!");
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
    dbprintlf("Final position: %u == %.2f", scanmot_current_pos, scanmot_current_pos * 40.0 / 10000);
    return;
}