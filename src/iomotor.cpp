/**
 * @file iomotor.cpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-03-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "iomotor.hpp"
#include "gpiodev/gpiodev.h"
#include "Adafruit/meb_print.h"
#include <stdint.h>
#include <unistd.h>

#include <thread>

IOMotor::IOMotor(Adafruit::StepperMotor *mot, int LimitSW1, int LimitSW2, bool LS1_is_PORTA)
{
    if (mot == NULL || mot == nullptr)
        throw std::runtime_error("Stepper motor pointer can not be null.");
    this->mot = mot;
    // this->mot->setSpeed(80);
    this->ls1 = LimitSW1;
    this->ls2 = LimitSW2;
    if (ls1 == ls2 || ls1 < 0 || ls2 < 0)
    {
        throw std::runtime_error("Limit switch invalid");
    }
    this->dir1 = Adafruit::MotorDir::FORWARD;
    this->dir2 = Adafruit::MotorDir::BACKWARD;
    ls1porta = LS1_is_PORTA;
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
    state = getState();
    if (state == IOMotor_State::ERROR)
    {
        throw std::runtime_error("Both limit switches are active, there is some connection error on motor.");
    }
    else if (state == IOMotor_State::MOVING)
    {
        if (setState(IOMotor_State::PORTA) != IOMotor_State::PORTA)
        {
            std::string motorst;
            if (state == IOMotor_State::ERROR)
                motorst = "Error";
            else if (state == IOMotor_State::PORTA)
                motorst = "PORT A";
            else if (state == IOMotor_State::PORTB)
                motorst = "PORT B";
            else if (state == IOMotor_State::MOVING)
                motorst = "Moving";
            else
                motorst = "Unknown " + std::to_string((int)state) + ".";
            throw std::runtime_error("Expected state PORT A, but current state " + motorst);
        }
    }
}

IOMotor::~IOMotor()
{
    if (getState() == IOMotor_State::MOVING)
        sleep(5);
}

IOMotor_State IOMotor::getState()
{
    gpioToState();
    return state;
}

std::string IOMotor::getStateStr()
{
    gpioToState();
    std::string motorst;
    if (state == IOMotor_State::ERROR)
        motorst = "Error";
    else if (state == IOMotor_State::PORTA)
        motorst = "PORT A";
    else if (state == IOMotor_State::PORTB)
        motorst = "PORT B";
    else if (state == IOMotor_State::MOVING)
        motorst = "Moving";
    else
        motorst = "Unknown " + std::to_string((int)state) + ".";
    return motorst;
}

IOMotor_State IOMotor::setState(IOMotor_State st, bool blocking, bool maxStepsLim)
{
    std::thread thr(setStateFcn, this, st, maxStepsLim, Adafruit::MotorStyle::DOUBLE);
    if (!blocking)
        thr.detach();
    else
        thr.join();
    gpioToState(); // final verification
    return state;
}

void IOMotor::gpioToState() // get state from GPIO inputs
{
    int st_ls1 = gpioRead(ls1);
    int st_ls2 = gpioRead(ls2);
    if (st_ls1 == GPIO_HIGH && st_ls2 == GPIO_HIGH) // both switches closed, impossible, error!
    {
        state = IOMotor_State::ERROR;
        return;
    }
    if (st_ls1 == GPIO_LOW && st_ls2 == GPIO_LOW) // both switches open, intermediate position
    {
        state = IOMotor_State::MOVING;
        return;
    }
    if (st_ls1 == GPIO_HIGH && st_ls2 == GPIO_LOW) // LS1 closed, LS2 open
    {
        state = ls1porta ? IOMotor_State::PORTA : IOMotor_State::PORTB;
        return;
    }
    if (st_ls1 == GPIO_LOW && st_ls2 == GPIO_HIGH) // LS2 closed, LS1 open
    {
        state = ls1porta ? IOMotor_State::PORTB : IOMotor_State::PORTA;
        return;
    }
}

void IOMotor::setStateFcn(IOMotor *self, IOMotor_State st, bool maxStepsLim, Adafruit::MotorStyle style)
{
    int maxSteps = 50 * 200; // max 200 revs
    Adafruit::MotorDir dir = Adafruit::MotorDir::RELEASE;
    if (st == IOMotor_State::PORTA) // requested port A
    {
        dir = self->ls1porta ? self->dir1 : self->dir2; // if limit sw 1 is port A, move towards it else move towards limit sw 2
    }
    else if (st == IOMotor_State::PORTB) // requested port B
    {
        dir = self->ls1porta ? self->dir2 : self->dir1; // if limit sw 1 is port A, move in the direction of limit sw 2 else move in the direction of limit sw 1
    }
    else // requested unsupported position
    {
        // Temporary change to allow testing.
        // throw std::runtime_error("Requested error or moving, not supported!");
        dbprintlf("Requested state change to ERROR or MOVING, which are invalid. Note: will not throw exception due to testing.");
    }
    self->gpioToState();                          // find out current state
    while ((self->state != st) && (maxSteps > 0)) // while we are not in target state and we have steps to move
    {
        self->mot->onestep(dir, style); // move one step
        // usleep(mot->getStepTime());
        self->gpioToState(); // check state
        if (maxStepsLim)     // if limit imposed, reduce max steps
            maxSteps--;
    }
    self->gpioToState(); // final verification
}