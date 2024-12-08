#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 Configuration
#define PCA9685_SDA 13
#define PCA9685_SCL 14
#define PCA9685_ADDRESS 0x5F
#define PWM_FREQUENCY 50

// Motor Pin Definitions
#define PIN_MOTOR_M1_IN1 15
#define PIN_MOTOR_M1_IN2 14
#define PIN_MOTOR_M2_IN1 9
#define PIN_MOTOR_M2_IN2 8
#define PIN_MOTOR_M3_IN1 12
#define PIN_MOTOR_M3_IN2 13
#define PIN_MOTOR_M4_IN1 10
#define PIN_MOTOR_M4_IN2 11

// Motor Direction Configuration
#define MOTOR_1_DIRECTION 1
#define MOTOR_2_DIRECTION 1
#define MOTOR_3_DIRECTION 1
#define MOTOR_4_DIRECTION 1

// Motor Speed Limits
#define MOTOR_SPEED_MIN -4095
#define MOTOR_SPEED_MAX 4095

// Default speed settings
#define DEFAULT_SPEED 2000
#define DEFAULT_TURN_SPEED 1500

// Function declarations
bool PCA9685_Setup(void);
bool isMotorSystemReady(void);
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed);

// Helper functions
void moveForward(int speed = DEFAULT_SPEED);
void moveBackward(int speed = DEFAULT_SPEED);
void turnLeft(int speed = DEFAULT_TURN_SPEED);
void turnRight(int speed = DEFAULT_TURN_SPEED);
void stopMotors(void);
void gradualStop(int delay_ms = 100);

#endif // MOTOR_CONTROL_H