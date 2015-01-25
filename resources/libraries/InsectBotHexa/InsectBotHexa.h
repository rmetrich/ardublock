#ifndef __INSECT_BOT_HEXA_H__
#define __INSECT_BOT_HEXA_H__

#if ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>

class InsectBotHexa
{
private:
    Servo f, m, r;                          // Front, middle and rear servos
    int fa, ma, ra;                         // Front, middle and rear servos current angle

    int s_delay;                            // Delay between <s_step> in milliseconds
    int s_center;                           // Servos default angle
    int s_step;                             // Angle step when moving servos

    int fpin, mpin, rpin;                   // Front, middle and rear servos digital pins

    int moving_legs_min;                    // Front and rear servos minimum angle: right leg front
    int moving_legs_max;                    // Front and rear servos maximum angle: left leg front
    int bending_legs_min;                   // Middle servo minimum angle: bend right
    int bending_legs_max;                   // Middle servo maximum angle: bend left

    int distanceIsValid;                    // True if no movement has been made
    int lastDistanceFromObstacle;           // Distance in centimeters

    int brightnessIsValid;                  // True if no movement has been made
    int brightnessThreshold;
    int lastBrightnessOnLeft;               // Raw value for left light sensor
    int lastBrightnessOnRight;              // Raw value for right light sensor

    // Analog sensors
    int distanceSensor;
    int lightSensorLeft;
    int lightSensorRight;

    bool isSetup;
    void setup(void);
    void lazySetup(void);
    void servo_move(Servo &s, int &current_angle, int target_angle);
    int cost_to_planned_position(int expected_fa, int expected_ma, int expected_ra);
    void computeDistanceFromObstacle(void);
    void computeBrightness(void);
  
public:
    InsectBotHexa(void);

    /* Rookie mode */
    bool isInDanger(void);
    /* Expert mode */
    int getDistanceFromObstacle(void);

    /* Rookie mode */
    bool isBrighterOnLeft(void);
    bool isBrighterOnRight(void);
    bool isBrightnessEqual(void);
    /* Expert mode */
    int getBrightnessOnLeft(void);
    int getBrightnessOnRight(void);

    void walkMode(void);
    void runMode(void);

    void goForward(void);
    void goBackward(void);
    void turnLeft(void);
    void turnRight(void);
};

#endif  /* __INSECT_BOT_HEXA_H__ */

/* vi: set autoindent expandtab softtabstop=4 shiftwidth=4 : */
