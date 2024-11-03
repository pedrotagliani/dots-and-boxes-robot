#include <Arduino.h>
// #include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_PWMServoDriver.h>
#include "AccelStepper.h"

// Must specify this before the include of "ServoEasing.hpp"
#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board
//#define PCA9685_ACTUAL_CLOCK_FREQUENCY 26000000L // Change it, if your PCA9685 has another than the default 25 MHz internal clock
//#define USE_SOFT_I2C_MASTER           // Saves 1756 bytes program memory and 218 bytes RAM compared with Arduino Wire
// #define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library
// #define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory
// #define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory
//#define DISABLE_MIN_AND_MAX_CONSTRAINTS    // Activating this disables constraints. Saves 4 bytes RAM per servo but strangely enough no program memory
//#define DISABLE_PAUSE_RESUME               // Activating this disables pause and resume functions. Saves 5 bytes RAM per servo
// #define ENABLE_MIN_AND_MAX_CONSTRAINTS
#define MAX_EASING_SERVOS 16
#define NUMBER_OF_SERVOS 3

/*
 * Specify which easings types should be available.
 * If no easing is defined, all easings are active.
 * This must be done before the #include "ServoEasing.hpp"
 */
//#define ENABLE_EASE_QUADRATIC
// #define ENABLE_EASE_CUBIC
//#define ENABLE_EASE_QUARTIC
#define ENABLE_EASE_SINE
//#define ENABLE_EASE_CIRCULAR
//#define ENABLE_EASE_BACK
//#define ENABLE_EASE_ELASTIC
//#define ENABLE_EASE_BOUNCE
//#define ENABLE_EASE_PRECISION
//#define ENABLE_EASE_USER

#include "ServoEasing.hpp"

// PCA9685 default address
#define PCA9685_DEFAULT_ADDRESS 0x40

// TCA9548A I2C Multiplexer Address (0x70 is the default)
#define TCA9548A_ADDRESS 0x70

// AS5600 address (identical for all encoders, hence the use of the I2C multiplexer)
#define AS5600_ADDRESS 0x36

// Set frequency for the servos
#define SERVO_FREQ 50

// Define the channels on the TCA9548A
#define PCA9685_CHANNEL 0
#define ENCODER_GRIPPER_CHANNEL 1
#define ENCODER_ELBOW_CHANNEL 2
#define ENCODER_SHOULDER_CHANNEL 3
#define ENCODER_BASE_CHANNEL 4

// Define the DIR pins from the encoders
#define DIR_ENCODER_GRIPPER 33
#define DIR_ENCODER_ELBOW 25
#define DIR_ENCODER_SHOULDER 26
#define DIR_ENCODER_BASE 27

// Define the servo pins connected on the PCA9685
#define SERVO_GRIPPER 12 // Articulation 4 (gripper)
#define SERVO_ELBOW 13 // Articulation 3 (elbow)
#define SERVO_SHOULDER 14 // Articulation 2 (shoulder)

// https://arminjo.github.io/ServoEasing/ServoEasing_8hpp_source.html#l00473
// https://naylampmechatronics.com/blog/41_tutorial-modulo-controlador-de-servos-pca9685-con-arduino.html
// PCA9685 -----> f = 50 Hz ------> p = 1/f = 20 ms
// 4096 units per 20 milliseconds => aMicroseconds = units * 4.8828
#define UNITS_TO_MICROSECONDS_CONSTANT 4.8828

// Define minimum and maximum pulse width for the servomotors specified in microseconds
#define SERVO_GRIPPER_MIN 172*UNITS_TO_MICROSECONDS_CONSTANT
#define SERVO_GRIPPER_MAX 548*UNITS_TO_MICROSECONDS_CONSTANT
#define SERVO_ELBOW_MIN 100*UNITS_TO_MICROSECONDS_CONSTANT
#define SERVO_ELBOW_MAX 468*UNITS_TO_MICROSECONDS_CONSTANT
#define SERVO_SHOULDER_MIN 114*UNITS_TO_MICROSECONDS_CONSTANT
#define SERVO_SHOULDER_MAX 474*UNITS_TO_MICROSECONDS_CONSTANT

// Functions declaration
void tca_select(uint8_t channel);
int read_raw_angle(uint8_t encoderChannel);
float convert_to_degrees(int rawAngle);
void checkMagnetPresence(uint8_t encoderChannel);

// Create the instances of the ServoEasing class
ServoEasing servoGripper(PCA9685_DEFAULT_ADDRESS);
ServoEasing servoElbow(PCA9685_DEFAULT_ADDRESS);
ServoEasing servoShoulder(PCA9685_DEFAULT_ADDRESS);

// Nema 17 stepper pins
#define STEP_PIN 32
#define DIR_PIN 19
#define MS1_PIN 16
#define MS2_PIN 17
#define MS3_PIN 18

// Create the instance of the AccelStepper class
AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define the pin where the limit switch is connected
// Our limit switch -----> Normally open
#define SWITCH_PIN 4








void setup() {
    Serial.begin(115200);
    Wire.begin();  // Initialize I2C communication

    // Set the DIR pin from the encoders as output
    pinMode(DIR_ENCODER_GRIPPER, OUTPUT);
    pinMode(DIR_ENCODER_ELBOW, OUTPUT);
    pinMode(DIR_ENCODER_SHOULDER, OUTPUT);
    pinMode(DIR_ENCODER_BASE, OUTPUT);

    // Set the direction polarity for each encoder
    // GND (or LOW) = values increase clockwise
    // VDD (or HIGH) = values increase counterclockwise
    digitalWrite(DIR_ENCODER_GRIPPER, LOW);
    digitalWrite(DIR_ENCODER_ELBOW, LOW);
    digitalWrite(DIR_ENCODER_SHOULDER, LOW);
    digitalWrite(DIR_ENCODER_BASE, HIGH);

    // checkMagnetPresence(ENCODER_SHOULDER_CHANNEL);

    tca_select(PCA9685_CHANNEL);  // Select channel 0 on TCA9548A
    // checkI2CConnection(PCA9685_DEFAULT_ADDRESS, &Serial);

    // attach (int aPin, int aInitialDegreeOrMicrosecond, int aMicrosecondsForServoLowDegree, int aMicrosecondsForServoHighDegree, int aServoLowDegree, int aServoHighDegree)
    // servoGripper.attach(SERVO_GRIPPER, 45, SERVO_GRIPPER_MIN, SERVO_GRIPPER_MAX, 0, 180);
    // servoElbow.attach(SERVO_ELBOW, 0, SERVO_ELBOW_MIN, SERVO_ELBOW_MAX, 0, 180);
    // servoShoulder.attach(SERVO_SHOULDER, 50, SERVO_SHOULDER_MIN, SERVO_SHOULDER_MAX, 0, 180);

    // attach (int aPin, int aMicrosecondsForServoLowDegree, int aMicrosecondsForServoHighDegree, int aServoLowDegree, int aServoHighDegree)
    servoGripper.attach(SERVO_GRIPPER, SERVO_GRIPPER_MIN, SERVO_GRIPPER_MAX, 0, 180);
    servoElbow.attach(SERVO_ELBOW, SERVO_ELBOW_MIN, SERVO_ELBOW_MAX, 0, 180);
    servoShoulder.attach(SERVO_SHOULDER, SERVO_SHOULDER_MIN, SERVO_SHOULDER_MAX, 0, 180);

    // Wait for servos to reach start position
    delay(2000);

    // Set the maximum speed of the stepper
    stepper.setMaxSpeed(1000.0);

    // Set microstepping pins as output
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(MS3_PIN, OUTPUT);

    // Set the 1/8 microstepping level for the servomotor
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);

    // Set the pin from the limit switch as input
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    // GPIO4 has a built-in pull-up resistor
    // The pin will read HIGH when the switch is open
    // When the switch is closed, the pin is pulled to LOW by connecting it to ground

    // Set the accelaration to find the starting position (0°)
    stepper.setAcceleration(1000);

    // Move the motor counterclockwise until the limit switch activates (closes)
    while (digitalRead(SWITCH_PIN) == HIGH) {  // While the switch is open (Note the pull-up resistor)
        stepper.moveTo(stepper.currentPosition() - 1); // Move one step counterclockwise
        stepper.run(); // Execute the movement
    }

    // Once the switch closes, stop the motor and mark the current position as 0°
    stepper.setCurrentPosition(0); // Set the current position as 0°

    Serial.println("HOME");
    
}











void loop() {

    tca_select(PCA9685_CHANNEL);
    // int rawAng1 = read_raw_angle(ENCODER_SHOULDER_CHANNEL);
    // float ang1 = convert_to_degrees(rawAng1);
    // Serial.println(ang1);
    // checkMagnetPresence(ENCODER_SHOULDER_CHANNEL);
    // delay(500);

    // servoGripper.setEasingType(EASE_SINE_IN_OUT);
    // servoGripper.easeTo(90,10);

    // servoElbow.setEasingType(EASE_SINE_IN_OUT);
    // servoElbow.easeTo(90,8);

    // servoShoulder.setEasingType(EASE_SINE_IN_OUT);
    // servoShoulder.easeTo(36,8);

    // stepper.setCurrentPosition(0);

    // while (stepper.currentPosition() != 800)  {
    //     stepper.setSpeed(50);
    //     stepper.runSpeed();
    //     Serial.println(stepper.currentPosition());
    // }

    // stepper.setCurrentPosition(0);

    // while(stepper.currentPosition() != -800){
    //     stepper.setSpeed(-50);
    //     stepper.runSpeed();
    // }



    // delay(2000);




}








// Function to select a TCA9548A channel
void tca_select(uint8_t channel) {
    // This function is key to switching between the different channels of the TCA9548A
    // It selects the active I2C channel by writing a bitmask to the TCA9548A

    if (channel > 7) { // TCA9548A only has channels 0-7
        return;
    }
    
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(1 << channel);  // Select the channel by writing a bit mask
    Wire.endTransmission();

    // Bit mask examples:
    // If channel = 0, 1 << 0 results in 00000001, which activates channel 0
    // If channel = 1, 1 << 1 results in 00000010, which activates channel 1
}

// Work with encoders using registries: https://youtu.be/yvrpIYc9Ll8?si=-Ygodj5sma54GQMu
// https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf
// Function to read the raw angle from an encoder
int read_raw_angle(uint8_t encoderChannel) { //  uint8_t ----> One-byte unsigned integer that can store values from 0 to 255
    uint8_t lowbyte; // Raw angle bits 7:0 (8 bits)
    uint8_t highbyte; // Raw angle bits 11:8 (4 bits)
    int rawAngle; // Final raw angle (12 bits)

    // Select the correct channel on the TCA9548A
    tca_select(encoderChannel);

    // Read low byte (bits 7:0)
    Wire.beginTransmission(AS5600_ADDRESS); // Connect to the sensor
    Wire.write(0x0D); // Figure 21 - register map: Specify register 0x0D (RAW ANGLE - lower 8 bits 7:0)
    Wire.endTransmission(); // End transmission
    Wire.requestFrom(AS5600_ADDRESS, 1); // Request 1 byte of data from the encoder

    while (Wire.available() == 0);  // Wait until data is available to read 
                                    // Wire.available() returns the number of bytes in the buffer; it will be 0 until the data arrives
    lowbyte = Wire.read(); // Read the data byte, which is the lower 8 bits of RAW ANGLE

    // Read high byte (bits 11:8)
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(0x0C); // Figure 21 - register map: Specify register 0x0C (RAW ANGLE - upper 4 bits 11:8)
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDRESS, 1);

    while (Wire.available() == 0);  // Wait until data is available
    highbyte = Wire.read(); // Read the data byte, which contains the upper 4 bits of RAW ANGLE

    // Combine high and low bytes to form the 12-bit raw angle
    // A 12-bit value has 2^12 = 4096 possible values, ranging from 0 to 4095
    rawAngle = (highbyte << 8) | lowbyte;
    return rawAngle;

    // For example, if highbyte was originally 0000 0111 (binary for 7), shifting it left by 8 bits gives 0000 0111 0000 0000
    // If highbyte (after shifting) is 0000 0111 0000 0000 and lowbyte is 0000 0000 1111 1111, then rawAngle becomes 0000 0111 1111 1111, representing the 12-bit binary value
    // In this case, rawAngle will hold the decimal value 2047 (0000 0111 1111 1111)
}

// Function to convert raw angle to degrees
float convert_to_degrees(int rawAngle) {
    // 360° is divided into 4096 equal parts (the encoder operates with a 12-bit resolution)
    float scalingFactor = 360.0/4096.0;

    // Multiplying the raw angle by the scaling factor gives the angle in degrees
    return rawAngle * scalingFactor;
}

void checkMagnetPresence(uint8_t encoderChannel)
{  
    uint8_t magnetStatus;

    tca_select(encoderChannel);

    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(0x0B); // Figure 21 - register map: Status ----> X X MD ML MH X X X
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDRESS, 1);

    while(Wire.available() == 0); 
    magnetStatus = Wire.read();

    // Serial.print("Magnet status: ");
    // Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)

    //Status register output: X X MD ML MH X X X  
    //MH: Too strong magnet
    //ML: Too weak magnet 
    //MD: OK magnet

    // Status bits
    const uint8_t AS5600_MAGNET_HIGH = 0x08; // 00001000 (MH)
    const uint8_t AS5600_MAGNET_LOW = 0x10; // 00010000 (ML)
    const uint8_t AS5600_MAGNET_DETECT = 0x20; // 00100000 (MD)

    // Apply a mask to keep only bits 3, 4 and 5
    magnetStatus = magnetStatus & 0b00111000;

    if(magnetStatus & AS5600_MAGNET_DETECT){
        Serial.println("Magnet was detected (MD).");
    }else if(magnetStatus & AS5600_MAGNET_HIGH){
        Serial.println("Magnet too strong (MH).");
    }else if(magnetStatus & AS5600_MAGNET_LOW){
        Serial.println("Magnet too weak (ML).");
    }

}