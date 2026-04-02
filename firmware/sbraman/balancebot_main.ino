//#include <MPU6050_light.h>
//#include "Wire.h"
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
//float ki = 450;
//
//// Define motor pins
//#define InL1 A3    // INA motor pin
//#define PWML 5     // PWM motor pin
//#define InL2 A2    // INB motor pin
//#define InR1 9     // INA motor pin
//#define PWMR 6     // PWM motor pin
//#define InR2 4     // INB motor pin
//
//// Desired tilt angle
//float desired_tilt = 0.0;
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
//// Setup function
//void setup() {
//    Serial.begin(115200);
//    Serial.println("Setup starting...");
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
//    tilt = mpu.getAngleY();
//    Serial.print("Initial tilt angle: ");
//    Serial.println(tilt);
//    desired_tilt = tilt;
//
//    // Motor setup
//    motor_init();
//    Serial.println("Motors initialized");
//
//    // Timer setup
//    timer1_init();
//    Serial.println("Timer initialized");
//
//    Serial.println("Setup completed!");
//}
//
//
//// Loop function
//void loop() {
//}
