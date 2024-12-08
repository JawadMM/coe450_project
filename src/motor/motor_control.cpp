#include "motor_control.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
bool motorSystemInitialized = false;

bool PCA9685_Setup(void) {
    Serial.println("Starting PCA9685 setup...");
    
    // Initialize I2C
    Wire.begin(PCA9685_SDA, PCA9685_SCL);
    delay(10);
    
    // Initialize PCA9685
    if (!pwm.begin()) {
        Serial.println("Failed to find PCA9685 chip");
        return false;
    }
    
    // Set PWM frequency
    pwm.setPWMFreq(PWM_FREQUENCY);
    delay(10);
    
    motorSystemInitialized = true;
    Serial.println("Motor system initialized successfully");
    return true;
}

bool isMotorSystemReady(void) {
    return motorSystemInitialized;
}

void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed) {
    if (!motorSystemInitialized) {
        Serial.println("Motor system not initialized!");
        return;
    }

    m1_speed = MOTOR_1_DIRECTION * constrain(m1_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    m2_speed = MOTOR_2_DIRECTION * constrain(m2_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    m3_speed = MOTOR_3_DIRECTION * constrain(m3_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    m4_speed = MOTOR_4_DIRECTION * constrain(m4_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

    // Motor 1
    if (m1_speed >= 0) {
        pwm.setPWM(PIN_MOTOR_M1_IN1, 0, m1_speed);
        pwm.setPWM(PIN_MOTOR_M1_IN2, 0, 0);
    } else {
        pwm.setPWM(PIN_MOTOR_M1_IN1, 0, 0);
        pwm.setPWM(PIN_MOTOR_M1_IN2, 0, -m1_speed);
    }

    // Motor 2
    if (m2_speed >= 0) {
        pwm.setPWM(PIN_MOTOR_M2_IN1, 0, m2_speed);
        pwm.setPWM(PIN_MOTOR_M2_IN2, 0, 0);
    } else {
        pwm.setPWM(PIN_MOTOR_M2_IN1, 0, 0);
        pwm.setPWM(PIN_MOTOR_M2_IN2, 0, -m2_speed);
    }

    // Motor 3
    if (m3_speed >= 0) {
        pwm.setPWM(PIN_MOTOR_M3_IN1, 0, m3_speed);
        pwm.setPWM(PIN_MOTOR_M3_IN2, 0, 0);
    } else {
        pwm.setPWM(PIN_MOTOR_M3_IN1, 0, 0);
        pwm.setPWM(PIN_MOTOR_M3_IN2, 0, -m3_speed);
    }

    // Motor 4
    if (m4_speed >= 0) {
        pwm.setPWM(PIN_MOTOR_M4_IN1, 0, m4_speed);
        pwm.setPWM(PIN_MOTOR_M4_IN2, 0, 0);
    } else {
        pwm.setPWM(PIN_MOTOR_M4_IN1, 0, 0);
        pwm.setPWM(PIN_MOTOR_M4_IN2, 0, -m4_speed);
    }
}

void moveForward(int speed) {
    if (!motorSystemInitialized) return;
    Motor_Move(speed, speed, speed, speed);
}

void moveBackward(int speed) {
    if (!motorSystemInitialized) return;
    Motor_Move(-speed, -speed, -speed, -speed);
}

void turnLeft(int speed) {
    if (!motorSystemInitialized) return;
    Motor_Move(-speed, -speed, speed, speed);
}

void turnRight(int speed) {
    if (!motorSystemInitialized) return;
    Motor_Move(speed, speed, -speed, -speed);
}

void stopMotors() {
    if (!motorSystemInitialized) return;
    Motor_Move(0, 0, 0, 0);
}

void gradualStop(int delay_ms) {
    if (!motorSystemInitialized) return;
    
    for (int speed = DEFAULT_SPEED; speed > 0; speed -= 200) {
        Motor_Move(speed, speed, speed, speed);
        delay(delay_ms);
    }
    stopMotors();
}