/**
 * @file scanmotor.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-03-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SCANMOTOR_HPP__
#define __SCANMOTOR_HPP__

#include "Adafruit/MotorShield.hpp"
#include <stdio.h>

#include <string>

#ifdef _DOXYGEN_
/**
 * @brief Compensate for backlash by moving additional steps and then reversing back to the desired location.
 * This is active while moving in the 'reverse' direction.
 * 
 */
#define SCANMOT_BACKLASH_COMPENSATION 200
#endif

#ifndef LOG_FILE_DIR
/**
 * @brief Scanning motor movement log file directory.
 * 
 */
#define LOG_FILE_DIR "/var/log/monochromatord/"
#endif // LOG_FILE_DIR

#ifndef LOG_FILE_NAME
/**
 * @brief Scanning motor movement log file name.
 * 
 */
#define LOG_FILE_NAME "motion.log"
#endif

#ifndef SCANMOT_BACKLASH_COMPENSATION
#define SCANMOT_BACKLASH_COMPENSATION 200
#endif

/**
 * @brief Indicate state of the scanning motor.
 * 
 */
enum class ScanMotor_State : uint8_t
{
    GOOD = 0, /*!< Both limit switches open, scanning can be performed. */
    LS1 = 1, /*!< Limit SW 1 has been hit. */
    LS2 = 2, /*!< Limit SW 2 has been hit. */
    ERROR = 3 /*!< Both limit switches are closed, wiring error. */
};

/**
 * @brief Pointer to function of the form void f(void).
 * 
 */
typedef void (*voidfptr_t)();

/**
 * @brief Scanning motor object that allows for changing the grating position.
 * 
 */
class ScanMotor
{
public:
    /**
     * @brief Construct a new scanning motor object.
     * 
     * @param mot Pointer to an {@link Adafruit::StepperMotor} object to control the physical motor.
     * @param LimitSW1 GPIO pin corresponding to limit switch 1.
     * @param dir1 Direction of movement to reach limit switch 1.
     * @param LimitSW2 GPIO pin corresponding to limit switch 2.
     * @param dir2 Direction of movement to reach limit switch 2.
     * @param absPos Default: 10000. Current absolute position (integer > 0, should be chosen carefully so that the bottom limit switch is above zero.)
     * @param _invalidFn Default: NULL. Pointer to a function that is executed when the scanning motor is stepped. This function can be used to invalidate the last known absolute position of the motor to indicate loss of calibration after a crash while moving.
     * @param trigin Default: -1. GPIO pin to receive trigger input while scanning. Only used for scanning functionality if supplied.
     * @param trigout Default: -1. GPIO pin to receive trigger output while scanning. Only used for scanning functionality if supplied.
     */
    ScanMotor(Adafruit::StepperMotor *mot, int LimitSW1, Adafruit::MotorDir dir1, int LimitSW2, Adafruit::MotorDir dir2, int absPos = 100000, voidfptr_t _invalidFn = NULL, int trigin = -1, int trigout = -1);

    /**
     * @brief Stop current movements and destroy scanning motor.
     * 
     */
    ~ScanMotor();

    /**
     * @brief Get the current absolute position of the motor.
     * 
     * @return int 
     */
    inline int getPos() const { return absPos; }

    /**
     * @brief Get the current scan target position.
     * 
     * @return int 
     */
    inline int getCurrentScan() const { return currentScan; }

    /**
     * @brief Move the motor to a position. Takes backlash into account while moving in 'reverse' direction.
     * Set target higher than current position to move towards limit switch 2, and vice-versa.
     * 
     * @param target Target position, must be positive.
     * @param override Default: false. Set to true to move even when a limit switch is hit.
     * @param blocking Default: false. Set to true for the function to block.
     * @return int Absolute position at the point of return.
     */
    int goToPos(int target, bool override = false, bool blocking = false);

    /**
     * @brief Move the motor by a number of steps from the current position.
     * 
     * @param steps Number of steps to move from current position, must be positive.
     * @param dir Direction of movement.
     * @param style Default: {@link Adafruit::MotorStyle::DOUBLE}. DO NOT CHANGE unless you know what you are doing.
     * @return int 
     */
    int posDelta(int steps, Adafruit::MotorDir dir, bool override = false, Adafruit::MotorStyle style = Adafruit::MotorStyle::DOUBLE);

    /**
     * @brief Start a scanning procedure.
     * Note: If you stop the motor while it is moving during scanning, the scan has to be canceled first and initScan
     * has to be called again for the scan to resume. This is NOT a blocking call.
     * 
     * @param start Starting absolute position.
     * @param stop Stopping absolute position. (exclusive)
     * @param step Number of steps to move between each scan position.
     * @param maxWait Maximum time to wait at a scan position (in seconds).
     * @param pulseWidthMs Maximum time a TRIGOUT pulse is high for.
     * 
     * @return std::string Name of current scan log file.
     */
    std::string initScan(int start, int stop, int step, int maxWait, int pulseWidthMs = 10);

    /**
     * @brief Cancel an ongoing scan.
     * 
     */
    void cancelScan();

    /**
     * @brief Check if a scan is currently active.
     * 
     * @return bool
     */
    inline bool isScanning() const { return scanning; }

    /**
     * @brief Get the current state of the scanning motor.
     * 
     * @return ScanMotor_State 
     */
    inline ScanMotor_State getState() const { return state; }

    /**
     * @brief Get the current state of the scanning motor as a string.
     * 
     * @return std::string 
     */
    std::string getStateStr();

    /**
     * @brief Stop the scanning motor movement (initiated by goToPos or posDelta, which initScan also internally uses.)
     * 
     */
    inline void eStop() { moving = false; }

    /**
     * @brief Check if the scanning motor is moving.
     * 
     * @return bool
     */
    inline bool isMoving() const { return moving; }

private:
    /**
     * @brief Get the current state of the scanning motor by reading the GPIO pins.
     * 
     */
    void gpioToState();
    static void goToPosInternal(ScanMotor *self, int target, bool override);
    static void initScanFn(ScanMotor *self, int start, int stop, int step, int maxWait, int pulseWidthMs, FILE *fp);

private:
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
    volatile bool scanning;      // scanning now
    int trigin;                  // trig in pin
    int trigout;                 // trig out pin
    int currentScan;
    volatile sig_atomic_t *done;
};

#endif