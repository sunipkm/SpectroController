/**
 * @file iomotor.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-03-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __IOMOTOR_HPP__
#define __IOMOTOR_HPP__

#include "gpiodev/gpiodev.h"
#include "Adafruit/MotorShield.hpp"
#include "Adafruit/meb_print.h"
#include <stdint.h>
#include <unistd.h>
enum class IOMotor_State: uint8_t
{
    MOVING = 0,
    PORTA = 1,
    PORTB = 2,
    ERROR = 3
};

class IOMotor
{
private:
    int motor_id;
    IOMotor_State state;     // current state
    int ls1;                 // limit sw 1 (forward)
    int ls2;                 // limit sw 2 (backward)
    Adafruit::MotorDir dir1; // limit sw 1 dir (forward)
    Adafruit::MotorDir dir2; // limit sw 2 dir (backward)
    bool ls1porta;           // if limit sw 1 is port A
    Adafruit::StepperMotor *mot;

public:
    IOMotor(Adafruit::StepperMotor *mot, int LimitSW1, int LimitSW2, bool LS1_is_PORTA)
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
                    motorst = "Unknown " + std::to_string((int) state) + ".";
                throw std::runtime_error("Expected state PORT A, but current state " + motorst);
            }
        }
    }
    IOMotor_State getState()
    {
        gpioToState();
        return state;
    }

    IOMotor_State setState(IOMotor_State st, bool maxStepsLim = true, Adafruit::MotorStyle style = Adafruit::MotorStyle::DOUBLE)
    {
        int maxSteps = 50 * 200; // max 200 revs
        Adafruit::MotorDir dir;
        if (st == IOMotor_State::PORTA) // requested port A
        {
            dir = ls1porta ? dir1 : dir2; // if limit sw 1 is port A, move towards it else move towards limit sw 2
        }
        else if (st == IOMotor_State::PORTB) // requested port B
        {
            dir = ls1porta ? dir2 : dir1; // if limit sw 1 is port A, move in the direction of limit sw 2 else move in the direction of limit sw 1
        }
        else // requested unsupported position
        {
            // Temporary change to allow testing.
            // throw std::runtime_error("Requested error or moving, not supported!");
            dbprintlf("Requested state change to ERROR or MOVING, which are invalid. Note: will not throw exception due to testing.");
            return IOMotor_State::ERROR;
        }
        gpioToState();                          // find out current state
        while ((state != st) && (maxSteps > 0)) // while we are not in target state and we have steps to move
        {
            mot->onestep(dir, style); // move one step
            // usleep(mot->getStepTime());
            gpioToState();   // check state
            if (maxStepsLim) // if limit imposed, reduce max steps
                maxSteps--;
        }
        gpioToState(); // final verification
        return state;
    }

private:
    void gpioToState() // get state from GPIO inputs
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
};

#endif