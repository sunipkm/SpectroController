/**
 * @file scanmotor.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-03-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SCANMOTOR_HPP__
#define __SCANMOTOR_HPP__

#include "gpiodev/gpiodev.h"
#include "Adafruit/meb_print.h"
#include "Adafruit/MotorShield.hpp"
#include <stdint.h>

#include <thread>

enum class ScanMotor_State : uint8_t
{
    GOOD = 0,
    LS1 = 1,
    LS2 = 2,
    ERROR = 3
};

typedef void (*voidfptr_t)();

static volatile sig_atomic_t scanmotor_internal_done = 0;

static void ScanMotor_sigHandler(int sig)
{
    scanmotor_internal_done = 1;
}

class ScanMotor
{
private:
    int ls1;                     // limit sw 1
    int ls2;                     // limit sw 2
    Adafruit::MotorDir dir1;     // limit sw 1 dir
    Adafruit::MotorDir dir2;     // limit sw 2 dir
    ScanMotor_State state;       // scan motor state
    int absPos;                  // absolute position
    volatile bool moving;        // move indicator
    Adafruit::StepperMotor *mot; // stepper motor
    voidfptr_t invalidFn;        // invalidate current position
    bool scanning;               // scanning now
    int trigin;                  // trig in pin
    int trigout;                 // trig out pin
    int currentScan;
    volatile sig_atomic_t *done;

public:
    ScanMotor(Adafruit::StepperMotor *mot, int LimitSW1, Adafruit::MotorDir dir1, int LimitSW2, Adafruit::MotorDir dir2, int absPos = 10000, voidfptr_t _invalidFn = NULL, int trigin = -1, int trigout = -1)
    {
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
        gpioToState();
        scanning = false;
        invalidFn = _invalidFn;
        this->absPos = absPos;
        this->trigin = trigin;
        this->trigout = trigout;
        this->currentScan = 0;
    }

    inline int getPos() const { return absPos; }

    inline int getCurrentScan() const { return currentScan; }

    int goToPos(int target, bool override = false, bool blocking = false)
    {
        std::thread thr(goToPosInternal, this, target, override);
        if (blocking)
            thr.join();
        else
            thr.detach();
        return absPos;
    }

    int posDelta(int steps, Adafruit::MotorDir dir, bool override = false, Adafruit::MotorStyle style = Adafruit::MotorStyle::DOUBLE)
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
            usleep(10000);
            gpioToState();
            if (!nsteps)
                break;
        }
        moving = false;
        return (steps - nsteps);
    }

    void initScan(int start, int stop, int step, int maxWait, int pulseWidthMs = 10)
    {
        if (scanning)
            return;
        std::thread thr(initScanFn, this, start, stop, step, maxWait, pulseWidthMs);
        thr.detach();
        return;
    }

    void cancelScan()
    {
        if (scanning)
            scanning = false;
        eStop();
    }

    bool isScanning() const { return scanning; }

    ScanMotor_State getState()
    {
        return state;
    }

    std::string getStateStr()
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

    void eStop()
    {
        moving = false;
    }

    bool isMoving() const
    {
        return moving;
    }

private:
    void gpioToState()
    {
        int st_ls1 = GPIO_LOW;
        int st_ls2 = GPIO_LOW;
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

    static void goToPosInternal(ScanMotor *self, int target, bool override)
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
                target -= 200; // 1 rev for backlash
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

    static void initScanFn(ScanMotor *self, int start, int stop, int step, int maxWait, int pulseWidthMs)
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
        for (int i = start + step; i < stop && self->scanning; )
        {
            // step 1: pulse
            if (self->scanning)
            {
                usleep(pulseWidthMs * 1000);
            }
            if (!self->scanning)
                break;
            // step 2: wait
            if (self->scanning)
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
};

#endif