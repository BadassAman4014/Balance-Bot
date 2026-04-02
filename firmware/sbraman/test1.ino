//#include <MPU6050_light.h>
//#include "Wire.h"
//#include <SoftwareSerial.h>  // Include SoftwareSerial for Bluetooth communication
//#include <ServoTimer2.h>
//
//// Define SoftwareSerial pins for Bluetooth
//SoftwareSerial bluetooth(A1, A0);  // RX, TX for HC-05 Bluetooth module
//
//// Define MPU6050 object
//MPU6050 mpu(Wire);
//
//// Create Servo objects
//ServoTimer2 gripper; // Servo 1 connected to pin 10
//ServoTimer2 arm; // Servo 2 connected to pin 11
//
//int arm_position = 135; // Initialize to 135 degrees
//
//// Variables for tilt angle
//float tilt = 0.0, tilt_old = 0.0, tilt_dot = 0.0, integral_tilt = 0.0;
//
//// Final output for wheel velocity
//int control_output = 0;
//
//// PID gains 
//float kp = 47.0;
//float kd = 1.26;
//float ki = 475;
//
//// Timing variables
//unsigned long last_sample_time = 0;
//const unsigned long SAMPLE_INTERVAL = 20; // Sample every 20ms
//
//// Control variables
//const float ANGLE_STEP = 0.75; // Angle change for A/D keys
//float initial_tilt = 0.0;
//float desired_tilt = 0.0;
//
//// Define motor pins
//#define InL1 A3    // INA motor pin
//#define PWML 5     // PWM motor pin
//#define InL2 A2    // INB motor pin
//#define InR1 9     // INA motor pin
//#define PWMR 6     // PWM motor pin
//#define InR2 4     // INB motor pin
//
//// Maximum integral windup limit
//#define INTEGRAL_LIMIT 50.0
//
//// Motor initialization function
//void motor_init() {
//    pinMode(InL1, OUTPUT);
//    pinMode(InL2, OUTPUT);
//    pinMode(PWML, OUTPUT);
//    digitalWrite(InL1, LOW);
//    digitalWrite(InL2, LOW);
//    analogWrite(PWML, 0);
//
//    pinMode(InR1, OUTPUT);
//    pinMode(InR2, OUTPUT);
//    pinMode(PWMR, OUTPUT);
//    digitalWrite(InR1, LOW);
//    digitalWrite(InR2, LOW);
//    analogWrite(PWMR, 0);
//}
//
//// Timer1 ISR for IMU
//void timer1_init() {
//    cli();  // Clear global interrupts
//    TIMSK1 = 0x01;  // Timer1 overflow interrupt enable
//    TCCR1B = 0x00;  // Stop
//    TCNT1H = 0xA2;  // Counter higher 8 bit value
//    TCNT1L = 0x3F;  // Counter lower 8 bit value
//    TCCR1A = 0x00;
//    TCCR1C = 0x00;
//    TCCR1B = 0x02;  // Start Timer, prescaler 8
//    sei();  // Enable global interrupts
//}
//
//// Motor control function for left motor
//void motor_control_L(int pwm) {
//    if (pwm < 0) {
//        digitalWrite(InL1, HIGH);
//        digitalWrite(InL2, LOW);
//        pwm = -pwm;
//    } else {
//        digitalWrite(InL1, LOW);
//        digitalWrite(InL2, HIGH);
//    }
//    analogWrite(PWML, pwm);
//}
//
//// Motor control function for right motor
//void motor_control_R(int pwm) {
//    if (pwm < 0) {
//        digitalWrite(InR1, HIGH);
//        digitalWrite(InR2, LOW);
//        pwm = -pwm;
//    } else {
//        digitalWrite(InR1, LOW);
//        digitalWrite(InR2, HIGH);
//    }
//    analogWrite(PWMR, pwm);
//}
//
//void left_turn(int tspeed){
//  digitalWrite(InR1, LOW);
//  digitalWrite(InR2, HIGH);
//  digitalWrite(InR1, HIGH);
//  digitalWrite(InR2, LOW);
//  
//  analogWrite(PWML, tspeed);
//  analogWrite(PWMR, tspeed);
//}
//
//void right_turn(int tspeed){
//  digitalWrite(InR1, HIGH);
//  digitalWrite(InR2, LOW);
//  digitalWrite(InR1, LOW);
//  digitalWrite(InR2, HIGH);
//  
//  analogWrite(PWML, tspeed);
//  analogWrite(PWMR, tspeed);
//}
//
//// Feedback function
//void feedback() {
//    // Get tilt angle from MPU6050
//    tilt = mpu.getAngleY();
//    tilt_dot = (tilt - tilt_old) / 0.012;  // Calculate tilt rate
//    tilt_old = tilt;
//
//    // Calculate integral term and constrain to prevent windup
//    integral_tilt += tilt * 0.012;
//    integral_tilt = constrain(integral_tilt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
//}
//
//// Control equation function
//void control_eqn() {
//    // PID control
//    float control = (kp * (tilt - desired_tilt)) + (kd * tilt_dot) + (ki * integral_tilt);
//
//    // Constrain output for PWM (0-255)
//    control_output = constrain(control, -255, 255);
//
//    // Drive motors
//    motor_control_L(control_output);
//    motor_control_R(control_output);
//}
//
//// ISR for Timer1 overflow
//ISR(TIMER1_OVF_vect) {
//    sei();
//    TCNT1H = 0xA2;
//    TCNT1L = 0x3F;
//    mpu.update();  // Update IMU data
//    cli();
//    feedback();
//    control_eqn();
//}
//
//// Setup function
//void setup() {
//    Serial.begin(9600);
//
//    // Initialize buzzer on D12
//    pinMode(12, OUTPUT); // Initialize D12 as output for buzzer
//    digitalWrite(12, LOW); // Start with the buzzer off
//
//    // Attach the servos to their respective pins
//    gripper.attach(11);
//    arm.attach(10);
//  
//    // Initialize the servos to their default positions
//    gripper.write(0);   // Servo 1 starts at 0 degrees
//    arm.write(arm_position);   // Servo 2 starts at 0 degrees
//    
//    bluetooth.begin(9600); // Start Bluetooth communication at 9600 baud rate
//    Serial.println("Control System Starting...");
//    Serial.println("Bluetooth is ready!");
//
//    // MPU6050 setup
//    Wire.begin();
//    byte status = mpu.begin();
//    Serial.print(F("MPU6050 status: "));
//    Serial.println(status);
//    if (status != 0) {
//        Serial.println("MPU initialization failed! Check connections.");
//        while (1);  // Halt program if MPU initialization fails
//    }
//    Serial.println("MPU initialization done");
//    delay(1000);
//    mpu.calcOffsets(true, true);
//
//    // Get initial tilt angle
//    mpu.update();
//    initial_tilt = mpu.getAngleY();
//    desired_tilt = initial_tilt;
//    Serial.print("Initial tilt angle: ");
//    Serial.println(initial_tilt);
//
//    // Motor setup
//    motor_init();
//    Serial.println("Motors initialized");
//
//    // Timer setup
//    timer1_init();
//    Serial.println("Timer initialized");
//
//    Serial.println("System ready!");
//    Serial.println("Use 'A' to increase angle by 0.3°");
//    Serial.println("Use 'D' to decrease angle by 0.3°");
//    Serial.println("Format: time(ms),desired_angle,current_angle,control_output");
//}
//
//// Loop function
//void loop() {
//    // Check for data from Serial Monitor
//    if (bluetooth.available()) {
//        // Read the incoming character
//        char input = bluetooth.read();
//        Serial.println(input); // Optional for debugging
//
//        switch (input) {
//            case 'F':
//                desired_tilt += ANGLE_STEP;
//                Serial.print("New desired angle: ");
//                Serial.println(desired_tilt);
//                break;
//
//            case 'B':
//                desired_tilt -= ANGLE_STEP;
//                Serial.print("New desired angle: ");
//                Serial.println(desired_tilt);
//                break;
//
//            case 'L':
//                left_turn(255);
//                break;
//
//            case 'R':
//                right_turn(255);
//                break;
//                
//            case 'Z':
//                // Turn on the buzzer
//                digitalWrite(12, HIGH);
//                Serial.println("Buzzer ON");
//                break;
//
//            case 'z':
//                // Turn off the buzzer
//                digitalWrite(12, LOW);
//                Serial.println("Buzzer OFF");
//                break;
//
//            case 'O':
//                gripper.write(0);
//                Serial.println("Gripper Open");
//                break;
//                
//            case 'C':
//                gripper.write(180);
//                Serial.println("Gripper Close");
//                break;
//
//            case 'U':
//                arm_position += 10;
//                // Constrain the position to a maximum of 180 degrees
//                arm_position = constrain(arm_position, 130, 170);
//                arm.write(arm_position);
//                Serial.print("Gripper Opened to: ");
//                Serial.print(arm_position);
//                Serial.println(" degrees");
//                break;
//                
//            case 'D':
//                arm_position -= 10;
//                // Constrain the position to a maximum of 180 degrees
//                arm_position = constrain(arm_position, 130, 170);
//                arm.write(arm_position);
//                Serial.print("Gripper Opened to: ");
//                Serial.print(arm_position);
//                Serial.println(" degrees");
//                break;
//                
//            default:
//                Serial.println("Unknown command.");
//                break;
//        }
//    }
//}
