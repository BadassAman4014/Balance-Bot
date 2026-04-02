//#include <MPU6050_light.h>
//#include "Wire.h"
//#include <SoftwareSerial.h>  // Include SoftwareSerial for Bluetooth communication
//
//// Define SoftwareSerial pins for Bluetooth
//SoftwareSerial bluetooth(A1, A0);  // RX, TX for HC-05 Bluetooth module
//
//// Define MPU6050 object
//MPU6050 mpu(Wire);
//
//// Variables for tilt angle
//float tilt = 0.0, tilt_old = 0.0, tilt_dot = 0.0, integral_tilt = 0.0;
//
//// Final output for wheel velocity
//int control_output = 0;
//
//// PID gains
//float kp = 90.0;
//float kd = 0.66;
//float ki = 550;
//
//// Timing variables
//unsigned long last_sample_time = 0;
//const unsigned long SAMPLE_INTERVAL = 20; // Sample every 20ms
//
//// Control variables
//const float ANGLE_STEP = 1.5; // Angle change for A/D keys
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
//    
//    // Adjust for dead zone
//    if (pwm > 0 && pwm < 100) {
//        pwm = 100;
//    }
//    
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
//    
//    // Adjust for dead zone
//    if (pwm > 0 && pwm < 100) {
//        pwm = 100;
//    }
//    
//    analogWrite(PWMR, pwm);
//}
//
//
//// Feedback function
//void feedback() {
//    // Get tilt angle from MPU6050
//    tilt = mpu.getAngleY();
//    tilt_dot = (tilt - tilt_old) / 0.012;  // Calculate tilt rate
//    tilt_old = tilt;
//
//    // Calculate the error
//    float error = tilt - desired_tilt;
//
//    // Update integral term only if control output is not saturated
//    if (control_output > -255 && control_output < 255) {
//        integral_tilt += error * 0.012;  // Accumulate error over time
//        integral_tilt = constrain(integral_tilt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
//    } else {
//        integral_tilt = 0;
//    }
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
//    Serial.begin(115200);
//    bluetooth.begin(9600); // Start Bluetooth communication at 9600 baud rate
//    Serial.println("Control System Starting...");
//    bluetooth.println("Control System Starting...");
//    Serial.println("Bluetooth is ready!");
//    bluetooth.println("Bluetooth is ready!");
//
//    // MPU6050 setup
//    Wire.begin();
//    byte status = mpu.begin();
//    Serial.print(F("MPU6050 status: "));
//    Serial.println(status);
//    bluetooth.print(F("MPU6050 status: "));
//    bluetooth.println(status);
//    if (status != 0) {
//        Serial.println("MPU initialization failed! Check connections.");
//        bluetooth.println("MPU initialization failed! Check connections.");
//        while (1);  // Halt program if MPU initialization fails
//    }
//    Serial.println("MPU initialization done");
//    bluetooth.println("MPU initialization done");
//    delay(1000);
//    mpu.calcOffsets(true, true);
//
//    // Get initial tilt angle
//    mpu.update();
//    initial_tilt = mpu.getAngleY();
//    desired_tilt = initial_tilt;
//    Serial.print("Initial tilt angle: ");
//    Serial.println(initial_tilt);
//    bluetooth.print("Initial tilt angle: ");
//    bluetooth.println(initial_tilt);
//
//    // Motor setup
//    motor_init();
//    Serial.println("Motors initialized");
//    bluetooth.println("Motors initialized");
//
//    // Timer setup
//    timer1_init();
//    Serial.println("Timer initialized");
//    bluetooth.println("Timer initialized");
//    Serial.println("System ready!");
//    bluetooth.println("System ready!");
//}
//
//// Loop function for logging
//void loop() {
//    static unsigned long last_log_time = 0;
//    if (millis() - last_log_time >= 100) {  // Log every 100ms
//        last_log_time = millis();
//
//        // Log current tilt, error, integral, and control output
//        bluetooth.print("Tilt: "); bluetooth.print(tilt);
//        bluetooth.print(", Tilt Rate: "); bluetooth.print(tilt_dot);
//        bluetooth.print(", Integral Tilt: "); bluetooth.print(integral_tilt);
//        bluetooth.print(", Control Output: "); bluetooth.print(control_output);
//        bluetooth.println();
//    }
//}
