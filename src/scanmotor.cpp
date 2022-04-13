/**
 * @file scanmotor.cpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-04-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "scanmotor.hpp"
#include "gpiodev/gpiodev.h"
#include "Adafruit/meb_print.h"
#include <stdint.h>
#include <time.h>
#include <inttypes.h>

#include <thread>

static volatile sig_atomic_t scanmotor_internal_done = 0;

static void ScanMotor_sigHandler(int sig)
{
    scanmotor_internal_done = 1;
}

static inline uint64_t get_timestamp()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (ts.tv_sec * 1000000000LLU + ts.tv_nsec);
}

static inline char *get_datetime()
{
    static __thread char buf[128];
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    snprintf(buf, sizeof(buf), "[%04d-%02d-%02d, %02d:%02d:%02d]", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

ScanMotor::ScanMotor(Adafruit::StepperMotor *mot, int LimitSW1, Adafruit::MotorDir dir1, int LimitSW2, Adafruit::MotorDir dir2, int absPos, voidfptr_t _invalidFn, int trigin, int trigout)
{
    if (system("mkdir -p " LOG_FILE_DIR))
    {
        throw std::runtime_error("Could not create log directory " LOG_FILE_DIR);
    }
    FILE *fp = fopen(LOG_FILE_DIR "/" LOG_FILE_NAME, "a");
    if (fp == NULL)
    {
        throw std::runtime_error("Could not open log file " LOG_FILE_DIR "/" LOG_FILE_NAME ", check permissions or storage capacity.");
    }
    if (mot == NULL || mot == nullptr)
        throw std::runtime_error("Stepper motor pointer can not be null.");
    signal(SIGINT, ScanMotor_sigHandler);
    done = &scanmotor_internal_done;
    this->mot = mot;
    this->ls1 = LimitSW1;
    this->ls2 = LimitSW2;
    if (ls1 == ls2 || ls1 < 0 || ls2 < 0)
    {
        throw std::runtime_error("Limit switch invalid");
    }
    if (dir1 == dir2)
    {
        throw std::runtime_error("Limit switches can not be in the same direction.");
    }
    if (dir1 == Adafruit::MotorDir::BRAKE || dir1 == Adafruit::MotorDir::RELEASE)
    {
        throw std::runtime_error("Limit switch 1 direction is not forward or backward.");
    }
    if (dir2 == Adafruit::MotorDir::BRAKE || dir2 == Adafruit::MotorDir::RELEASE)
    {
        throw std::runtime_error("Limit switch 2 direction is not forward or backward.");
    }
    this->dir1 = dir1;
    this->dir2 = dir2;
    if (gpioSetMode(ls1, GPIO_IN) < 0)
    {
        throw std::runtime_error("Could not set pin " + std::to_string(ls1) + " as input pin.");
    }
    if (gpioSetPullUpDown(ls1, GPIO_PUD_UP) < 0)
    {
        throw std::runtime_error("Could not set pull up on pin " + std::to_string(ls1));
    }
    if (gpioSetMode(ls2, GPIO_IN) < 0)
    {
        throw std::runtime_error("Could not set pin " + std::to_string(ls2) + " as input pin.");
    }
    if (gpioSetPullUpDown(ls2, GPIO_PUD_UP) < 0)
    {
        throw std::runtime_error("Could not set pull up on pin " + std::to_string(ls2));
    }
    if (trigin > 0) // valid pin
    {
        if (gpioSetMode(trigin, GPIO_IRQ_RISE) < 0)
            throw std::runtime_error("Could not set pin " + std::to_string(trigin) + " as trigger input interrupt.");
        if (gpioSetPullUpDown(trigin, GPIO_PUD_DOWN) < 0)
            throw std::runtime_error("Could not set pull down on pin " + std::to_string(trigin));
    }
    if (trigout > 0) // valid pin
    {
        if (gpioSetMode(trigout, GPIO_OUT) < 0)
            throw std::runtime_error("Could not set pin " + std::to_string(trigout) + " as trigger output.");
        gpioWrite(trigout, GPIO_LOW);
    }
    gpioToState();
    if (state == ScanMotor_State::ERROR)
    {
        throw std::runtime_error("Both limit switches closed, indicates wiring error.");
    }
    scanning = false;
    invalidFn = _invalidFn;
    this->absPos = absPos;
    this->trigin = trigin;
    this->trigout = trigout;
    this->currentScan = 0;
    fprintf(fp, "\n%s: Init\n", get_datetime());
    fflush(fp);
    fclose(fp);
}

ScanMotor::~ScanMotor()
{
    cancelScan();
    eStop();
    FILE *fp = fopen(LOG_FILE_DIR "/" LOG_FILE_NAME, "a");
    if (fp != NULL)
    {
        fprintf(fp, "%s: Exit\n\n", get_datetime());
        fflush(fp);
        fclose(fp);
    }
}

int ScanMotor::goToPos(int target, bool override, bool blocking)
{
    std::thread thr(goToPosInternal, this, target, override);
    if (blocking)
        thr.join();
    else
        thr.detach();
    return absPos;
}

int ScanMotor::posDelta(int steps, Adafruit::MotorDir dir, bool override, Adafruit::MotorStyle style)
{
    int nsteps = steps;
    if (nsteps <= 0)
        return 0;
    gpioToState();
    FILE *fp = fopen(LOG_FILE_DIR "/" LOG_FILE_NAME, "a");
    if (fp == NULL)
    {
        dbprintlf("Could not open log file " LOG_FILE_DIR "/" LOG_FILE_NAME " for writing. Exiting.");
    }
    else
    {
        fprintf(fp, "%s: Moving from %d, intended %d, final loc ", get_datetime(), absPos, dir == dir1 ? -steps : steps);
        fflush(fp);
    }
    moving = true;
    while (moving && !(*done))
    {
        if (!override && state != ScanMotor_State::GOOD)
            break;
        nsteps--;
        if (dir == dir2)
            absPos++;
        else if (dir == dir1)
            absPos--;
        if (invalidFn != NULL)
            invalidFn();
        mot->onestep(dir, style);
        gpioToState();
        if (!nsteps)
            break;
    }
    moving = false;
    if (fp != NULL)
    {
        fprintf(fp, "%d\n", absPos);
        fclose(fp);
    }
    return (steps - nsteps);
}

std::string ScanMotor::initScan(int start, int stop, int step, int maxWait, int pulseWidthMs)
{
    if (scanning)
        std::string("");
    // sanity checks
    if (stop < start)
        std::string("");
    if (step <= 0)
        std::string("");
    if (((stop - start) / step) < 2)
        std::string("");
    if (maxWait <= 0)
        std::string("");
    if (pulseWidthMs <= 1)
        pulseWidthMs = 1;
    // save scan info
    char fname[256];
    snprintf(fname, sizeof(fname), LOG_FILE_DIR "/scan_%" PRIu64 ".log", get_timestamp());
    FILE *fp = fopen(fname, "a");
    if (fp == NULL)
    {
        dbprintlf("Could not open log file %s for writing.", fname);
    }
    else
    {
        int num = fprintf(fp, "%s: Initiating scan, (%d, %d, %d | %d, %d), saving to %s.\n", get_datetime(), start, stop, step, maxWait, pulseWidthMs, fp ? fname : "Error");
        for (int i = num; i > 0; i--)
            fprintf(fp, "-");
        fprintf(fp, "\n");
        fflush(fp);
    }
    FILE *lp = fopen(LOG_FILE_DIR "/" LOG_FILE_NAME, "a");
    if (lp != NULL)
    {
        fprintf(lp, "%s: Initiating scan, (%d, %d, %d | %d, %d), saving to %s.\n", get_datetime(), start, stop, step, maxWait, pulseWidthMs, fp ? fname : "Error");
        fclose(lp);
    }
    // initiate scan
    std::thread thr(initScanFn, this, start, stop, step, maxWait, pulseWidthMs, fp);
    thr.detach();
    return std::string(fname);
}

void ScanMotor::cancelScan()
{
    if (scanning)
        scanning = false;
    eStop();
}

std::string ScanMotor::getStateStr()
{
    gpioToState();
    std::string motorst;
    if (state == ScanMotor_State::ERROR)
        motorst = "Error";
    else if (state == ScanMotor_State::LS1)
        motorst = "LS1";
    else if (state == ScanMotor_State::LS2)
        motorst = "LS2";
    else if (state == ScanMotor_State::GOOD && moving)
        motorst = "Moving";
    else if (state == ScanMotor_State::GOOD && !moving)
        motorst = "Ready";
    else
        motorst = "Unknown " + std::to_string((int)state) + ".";
    return motorst;
}

void ScanMotor::gpioToState()
{
    int st_ls1 = gpioRead(ls1);
    int st_ls2 = gpioRead(ls2);
    if (st_ls1 == GPIO_HIGH && st_ls2 == GPIO_HIGH) // both switches closed, impossible, error!
    {
        state = ScanMotor_State::ERROR;
        return;
    }
    if (st_ls1 == GPIO_LOW && st_ls2 == GPIO_LOW) // both switches open, intermediate position
    {
        state = ScanMotor_State::GOOD;
        return;
    }
    if (st_ls1 == GPIO_HIGH && st_ls2 == GPIO_LOW) // LS1 closed, LS2 open
    {
        state = ScanMotor_State::LS1;
        return;
    }
    if (st_ls1 == GPIO_LOW && st_ls2 == GPIO_HIGH) // LS2 closed, LS1 open
    {
        state = ScanMotor_State::LS2;
        return;
    }
}

void ScanMotor::goToPosInternal(ScanMotor *self, int target, bool override)
{
    Adafruit::MotorDir dir = Adafruit::MotorDir::RELEASE;
    int _target = target;
    if (target <= 0)
        dbprintlf("Target position %d, invalid.", target);
    if (target > self->absPos)
        dir = self->dir2; // towards SW2
    else if (target < self->absPos)
    {
        dir = self->dir1; // towards SW1
        if (!override)
            target -= SCANMOT_BACKLASH_COMPENSATION; // 1 rev for backlash
    }
    else
        return;
    int steps = abs(target - self->absPos);
    self->posDelta(steps, dir, override);
    if (dir == self->dir1 && !override && self->absPos < _target) // in case of backlash and LS override is not applicable
    {
        self->posDelta(_target - self->absPos, self->dir2, override);
    }
}

void ScanMotor::initScanFn(ScanMotor *self, int start, int stop, int step, int maxWait, int pulseWidthMs, FILE *fp)
{
    self->scanning = false;
    // sanity checks
    if (stop < start)
        return;
    if (step <= 0)
        return;
    if (((stop - start) / step) < 2)
        return;
    if (maxWait <= 0)
        return;
    if (pulseWidthMs <= 1)
        pulseWidthMs = 1;
    // check complete
    self->scanning = true;
    if (fp != NULL)
        fprintf(fp, "[%" PRIu64 "] Moving to start: %d\n", get_timestamp(), start);
    self->goToPosInternal(self, start, false);
    if (self->absPos != start)
    {
        if (fp != NULL)
        {
            fprintf(fp, "[%" PRIu64 "] Could not reach start: %d, reached %d.\n", get_timestamp(), start, self->absPos);
            fclose(fp);
        }
        return;
    }
    // step 1: pulse at start
    if (fp != NULL)
    {
        fprintf(fp, "[%" PRIu64 "] Triggering at: %d.\n", get_timestamp(), self->absPos);
    }
    if (self->trigout > 0 && self->scanning)
    {
        gpioWrite(self->trigout, GPIO_HIGH);
        usleep(pulseWidthMs * 1000);
        gpioWrite(self->trigout, GPIO_LOW);
    }
    for (int i = start + step; i < stop && self->scanning;)
    {
        if (!self->scanning)
            break;
        // step 2: wait
        if (self->trigin > 0 && self->scanning)
        {
            gpioWaitIRQ(self->trigin, GPIO_IRQ_RISE, maxWait * 1000);
        }
        else if (self->scanning)
        {
            usleep(maxWait * 1000000LLU);
        }
        if (!self->scanning)
            break;
        // step 3: move
        if (self->scanning)
            self->goToPosInternal(self, i, false);
        self->currentScan = i;
        // step 4: pulse
        if (fp != NULL)
        {
            fprintf(fp, "[%" PRIu64 "] Triggering at: %d.\n", get_timestamp(), self->absPos);
        }
        if (self->trigout > 0 && self->scanning)
        {
            gpioWrite(self->trigout, GPIO_HIGH);
            usleep(pulseWidthMs * 1000);
            gpioWrite(self->trigout, GPIO_LOW);
        }
        if (!self->scanning)
            break;
        i += step;
    }
    self->scanning = false;
    if (fp != NULL)
    {
        fprintf(fp, "[%" PRIu64 "] Exiting at: %d.\n", get_timestamp(), self->absPos);
        fclose(fp);
    }
    FILE *lp = fopen(LOG_FILE_DIR "/" LOG_FILE_NAME, "a");
    if (lp != NULL)
    {
        fprintf(lp, "%s: Finished scan, (%d, %d, %d | %d, %d), current location %d.\n", get_datetime(), start, stop, step, maxWait, pulseWidthMs, self->absPos);
        fclose(lp);
    }
}
