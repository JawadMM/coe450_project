#include "./camera/ei_camera.h"
#include "./camera/camera_config.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "./motor/motor_control.h" 
#include <Arduino.h>
#include <Wire.h>

void scanI2CDevices() {
    byte error, address;
    int nDevices = 0;
    Serial.println("\nScanning I2C bus...");
 
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            nDevices++;
        }
    }
    
    if (nDevices == 0) {
        Serial.println("No I2C devices found!");
    } else {
        Serial.printf("Found %d devices\n", nDevices);
    }
}

void testMotorSystem() {
    Serial.println("\n=== Starting Motor System Test ===\n");
    
    // Test 1: Initialization
    Serial.println("Test 1: Initializing PCA9685...");
    if (!PCA9685_Setup()) {
        Serial.println("❌ Initialization failed! Stopping tests.");
        return;
    }
    Serial.println("✓ Initialization successful\n");
    delay(1000);
    
    // Test 2: Basic Movement Tests
    Serial.println("Test 2: Testing basic movements");
    Serial.println("Moving Forward for 2 seconds...");
    moveForward(2000);
    delay(2000);
    stopMotors();
    
    Serial.println("Moving Backward for 2 seconds...");
    delay(1000);
    moveBackward(2000);
    delay(2000);
    stopMotors();
    
    Serial.println("Turning Left for 2 seconds...");
    delay(1000);
    turnLeft(1500);
    delay(2000);
    stopMotors();
    
    Serial.println("Turning Right for 2 seconds...");
    delay(1000);
    turnRight(1500);
    delay(2000);
    stopMotors();
    
    // Test 3: Speed Control Test
    Serial.println("\nTest 3: Testing different speeds");
    for (int speed = 500; speed <= 2000; speed += 500) {
        Serial.printf("Testing forward speed: %d\n", speed);
        moveForward(speed);
        delay(1000);
    }
    stopMotors();
    
    // Test 4: Gradual Stop Test
    Serial.println("\nTest 4: Testing gradual stop");
    moveForward(2000);
    delay(1000);
    gradualStop(100);
    
    Serial.println("\n=== Motor System Test Complete ===");
}


TwoWire I2Cbus = TwoWire(0);

void setup() {
    Serial.begin(115200);

    Wire.begin(PCA9685_SDA, PCA9685_SCL);
    delay(100);
    
    Serial.println("Starting I2C scan...");
    scanI2CDevices();
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
        return;
    }
    ei_printf("Camera initialized\r\n");

    ei_printf("\nStarting continuous inference in 2 seconds...\n");

    if (PCA9685_Setup()) {
        Serial.println("PCA9685 initialized successfully");
        delay(2000);  // Give time to place the car safely
        // testMotorSystem();
    } else {
        Serial.println("Failed to initialize PCA9685");
    }

    delay(2000);
}

void handleGesture(const ei_impulse_result_t& result) {
    bool gesture_detected = false;
    float highest_confidence = 0.0;
    const char* detected_gesture = nullptr;
    
    // Check bounding boxes for gestures
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
        auto bb = result.bounding_boxes[ix];
        // if (bb.value > 0.4 && bb.value > highest_confidence) {  // 40% confidence threshold
        if (bb.value > highest_confidence) {
            highest_confidence = bb.value;
            detected_gesture = bb.label;
            gesture_detected = true;
        }
    }
    
    // Control car based on detected gesture
    if (gesture_detected && detected_gesture != nullptr) {
        if (strcmp(detected_gesture, "forward") == 0) {
            moveForward(600); 
        }
        else if (strcmp(detected_gesture, "backward") == 0) {
            moveBackward(600);
        }
        else {
            stopMotors();
        }
    }
    else {
        stopMotors();  // No gesture detected or low confidence
    }
}

void loop() {
//   if (ei_sleep(5) != EI_IMPULSE_OK) {
//         return;
//     }

    // Allocate memory for camera frame
    snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * 
                                   EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 
                                   EI_CAMERA_FRAME_BYTE_SIZE);

    if (snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    // Set up signal for classifier
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Capture image
    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, 
                         (size_t)EI_CLASSIFIER_INPUT_HEIGHT, 
                         snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run classifier
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    // Handle motor control based on detected gesture
    handleGesture(result);

    free(snapshot_buf);
}