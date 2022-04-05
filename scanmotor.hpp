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
    volatile sig_atomic_t *done;

public:
    ScanMotor(Adafruit::StepperMotor *mot, int LimitSW1, Adafruit::MotorDir dir1, int LimitSW2, Adafruit::MotorDir dir2, int absPos = 10000, voidfptr_t _invalidFn = NULL)
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
        gpioToState();
        if (state == ScanMotor_State::ERROR)
        {
            throw std::runtime_error("Both limit switches closed, indicates wiring error.");
        }
        invalidFn = _invalidFn;
        this->absPos = absPos;
    }

    inline int getPos() const { return absPos; }

    int goToPos(int target, bool blocking = false)
    {
        std::thread thr(goToPosInternal, this, target);
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
            mot->onestep(dir, style);
            gpioToState();
            if (!nsteps)
                break;
        }
        moving = false;
        return (steps - nsteps);
    }

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
            motorst = "Hit Limit SW 1";
        else if (state == ScanMotor_State::LS2)
            motorst = "Hit Limit SW 2";
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

    static void goToPosInternal(ScanMotor *self, int target)
    {
        Adafruit::MotorDir dir = Adafruit::MotorDir::RELEASE;
        if (target <= 0)
            dbprintlf("Target position %d, invalid.", target);
        if (target > self->absPos)
            dir = self->dir2; // towards SW2
        else if (target < self->absPos)
            dir = self->dir1; // towards SW1
        else
            return;
        int steps = abs(target - self->absPos);
        self->posDelta(steps, dir);
    }
};

#endif