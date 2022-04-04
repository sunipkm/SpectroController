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

#include <iomotor.hpp>
#include "Adafruit/meb_print.h"

#define MSHIELD_ADDR 0x60
#define MSHIELD_BUS 1
#define M_STEPS 50
#define M_PORT 1
#define LSW1 11
#define LSW2 13

int main()
{
    // Instantiation.
    dbprintlf("Instantiating MotorShield and StepperMotor with address %d, bus %d, %d steps, and port %d.", MSHIELD_ADDR, MSHIELD_BUS, M_STEPS, M_PORT);
    Adafruit::MotorShield shield(MSHIELD_ADDR, MSHIELD_BUS);
    shield.begin();
    Adafruit::StepperMotor *smotor = shield.getStepper(M_STEPS, M_PORT);

    dbprintlf("Instantiating IOMotor with limit switches 1 and 2 on ports %d and %d, respectively.", LSW1, LSW2);
    IOMotor iomotor(smotor, LSW1, LSW2, true);

    // Initial state reading.
    dbprintlf("Current state: %d", iomotor.getState());

    // State changes. Note that these first two should error-out. The iomotor.hpp code should be changed to allow testing for failure without crashing the program with unhandled exceptions.
    dbprintlf(YELLOW_FG "Requesting iomotor change its state to MOVING; this should produce an error:");
    iomotor.setState(IOMotor_State::MOVING);

    dbprintlf("Current state: %d", iomotor.getState());

    dbprintlf(YELLOW_FG "Requesting iomotor change its state to ERROR; this should also produce an error:");
    iomotor.setState(IOMotor_State::ERROR);

    dbprintlf("Current state: %d", iomotor.getState());

    dbprintlf("Requesting iomotor change its state to PORTA:");
    iomotor.setState(IOMotor_State::PORTA);

    dbprintlf("Current state: %s", iomotor.getState());

    // dbprintlf("Requesting iomotor change its state to PORTB:");
    // iomotor.setState(IOMotor_State::PORTB);

    // dbprintlf("Current state: %d", iomotor.getState());

    // dbprintlf("Test complete.");

    return 0;
}