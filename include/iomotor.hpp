/**
 * @file iomotor.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-03-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __IOMOTOR_HPP__
#define __IOMOTOR_HPP__

#include "Adafruit/MotorShield.hpp"

/**
 * @brief Defines the possible states of an {@link IOMotor} object.
 * 
 */
enum class IOMotor_State : uint8_t
{
    MOVING = 0, /*!< Both limit switches are open, IOMotor in intermediate position. */
    PORTA = 1, /*!< Port A allows light to pass. */
    PORTB = 2, /*!< Port B allows light to pass. */
    ERROR = 3 /*!< Indicates wiring error on startup, or overstepping IOMotor without desired effect at runtime. */
};

/**
 * @brief IOMotor provides methods to switch the input/output selector 
 * mirrors between port A and port B. 
 * 
 */
class IOMotor
{
public:
    /**
     * @brief Creates a new IOMotor object. Throws runtime exceptions if
     * 1. mot is NULL or nullptr.
     * 2. LimitSW1 or LimitSW2 are invalid/equal.
     * 3. Could not set LimitSW1 or LimitSW2 as input pins with pull ups.
     * 4. Both limit switches register low, indicating both switces being closed.
     * 5. If the IOMotor is in state {@link IOMotor_State::MOVING} and could not be moved to {@link IOMotor_State::PORTA}.
     * 
     * @param mot {@link Adafruit::StepperMotor} that rotates the mirror.
     * @param LimitSW1 GPIO Pin corresponding to limit switch 1.
     * @param LimitSW2 GPIO Pin corresponding to limit switch 2.
     * @param LS1_is_PORTA Set to true to indicate limit switch 1 closing (GPIO_LOW) indicates port A has been selected, set to false otherwise.
     */
    _Catchable IOMotor(Adafruit::StepperMotor *mot, int LimitSW1, int LimitSW2, bool LS1_is_PORTA);

    /**
     * @brief Get the current state of the IOMotor object.
     * 
     * @return IOMotor_State Current state of the IOMotor object.
     */
    IOMotor_State getState();

    /**
     * @brief Get the string representation of the state of the IOMotor object.
     * 
     * @return std::string 
     */
    std::string getStateStr();

    /**
     * @brief Switch the IOMotor between Port A and Port B.
     * 
     * @param st IOMotor_State::PORTA or IOMotor_State::PORTB.
     * @param blocking Optional argument, default false. Set to true to make the call block until the state change is complete.
     * @param maxStepsLim Optional argument, default true. Set to false to allow indefinite motor stepping until desired point is reached. USE AT YOUR OWN RISK.
     * @return IOMotor_State Current state. Note: The state is not accurate in case of a non-blocking call.
     */
    IOMotor_State setState(IOMotor_State st, bool blocking = false, bool maxStepsLim = true);

private:
    /**
     * @brief Generates the current state of the IOMotor
     * reading the limit switches.
     *
     */
    void gpioToState();

    /**
     * @brief Function to execute the state change of the IOMotor.
     * 
     * @param self IOMotor object.
     * @param st Desired state.
     * @param maxStepsLim Enforce maximum steps limit.
     * @param style Set to {@link Adafruit::MotorStyle::DOUBLE}. Changing this parameter is NOT RECOMMENDED.
     */
    static void setStateFcn(IOMotor *self, IOMotor_State st, bool maxStepsLim, Adafruit::MotorStyle style);

private:
    IOMotor_State state;
    int ls1;
    int ls2;
    Adafruit::MotorDir dir1;
    Adafruit::MotorDir dir2;
    bool ls1porta;
    Adafruit::StepperMotor *mot;
};

#endif