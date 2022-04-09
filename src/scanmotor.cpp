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

#include <thread>

static volatile sig_atomic_t scanmotor_internal_done = 0;

static void ScanMotor_sigHandler(int sig)
{
    scanmotor_internal_done = 1;
}

ScanMotor::ScanMotor(Adafruit::StepperMotor *mot, int LimitSW1, Adafruit::MotorDir dir1, int LimitSW2, Adafruit::MotorDir dir2, int absPos, voidfptr_t _invalidFn, int trigin, int trigout)
{
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
        if (gpioSetPullUpDown(trigin, GPIO_PUD_UP) < 0)
            throw std::runtime_error("Could not set pull up on pin " + std::to_string(trigin));
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
    return (steps - nsteps);
}

void ScanMotor::initScan(int start, int stop, int step, int maxWait, int pulseWidthMs)
{
    if (scanning)
        return;
    std::thread thr(initScanFn, this, start, stop, step, maxWait, pulseWidthMs);
    thr.detach();
    return;
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

void ScanMotor::initScanFn(ScanMotor *self, int start, int stop, int step, int maxWait, int pulseWidthMs)
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
    self->goToPosInternal(self, start, false);
    if (self->absPos != start)
        return;
    for (int i = start + step; i < stop && self->scanning;)
    {
        // step 1: pulse
        if (self->trigout > 0 && self->scanning)
        {
            gpioWrite(self->trigout, GPIO_HIGH);
            usleep(pulseWidthMs * 1000);
            gpioWrite(self->trigout, GPIO_LOW);
        }
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
        i += step;
    }
    self->scanning = false;
}
