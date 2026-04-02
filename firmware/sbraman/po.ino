//#include <MPU6050_light.h>
//#include "Wire.h"
//#include <SoftwareSerial.h>
//
//// Define SoftwareSerial pins for Bluetooth
//SoftwareSerial bluetooth(A1, A0);  // RX, TX for HC-05 Bluetooth module
//
//// Define MPU6050 object
//MPU6050 mpu(Wire);
//
//// Define encoder pins
//#define encodPinAR 7  // Encoder A pin (C2) 
//#define encodPinBR 2  // Encoder B pin (C1) 
//#define encodPinAL 8  // Encoder A pin (C2)
//#define encodPinBL 3  // Encoder B pin (C1)
//
//// Define motor pins
//#define InL1 A3    // INA motor pin
//#define PWML 5     // PWM motor pin
//#define InL2 A2    // INB motor pin
//#define InR1 9     // INA motor pin
//#define PWMR 6     // PWM motor pin
//#define InR2 4     // INB motor pin
//
//// Variables for tilt angle
//float tilt = 0.0, tilt_old = 0.0, tilt_dot = 0.0, integral_tilt = 0.0;
//
//// Variables for position control
//float position = 0.0;  // Current position in meters
//float desired_position = 0.0;  // Desired position in meters
//float position_old = 0.0;
//float position_dot = 0.0;
//float integral_position = 0.0;
//
//// Variable for wheel angle
//int wheel_pulse_count_left = 0;
//int wheel_pulse_count_right = 0;
//float wheel_angle_left = 0.0, wheel_angle_old_left = 0.0, wheel_dot_left = 0.0;
//float wheel_angle_right = 0.0, wheel_angle_old_right = 0.0, wheel_dot_right = 0.0;
//
//// Final output for wheel velocity
//int control_output = 0;
//
//// Angle PID gains
//float kp = 47.0;
//float kd = 1.26;
//float ki = 370;
//
//// Position control PID gains
//float kp_pos = 15;  // Start with conservative values
//float kd_pos = 0.2;
//float ki_pos = 50;
//
//// Encoder parameters
//const float WHEEL_RADIUS = 0.2;  // in meters
//const float PULSES_PER_REVOLUTION = 350.0;  // Adjust based on your encoder
//const float METERS_PER_PULSE = (2.0 * PI * WHEEL_RADIUS) / PULSES_PER_REVOLUTION;
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
//// Maximum integral windup limit
//#define INTEGRAL_LIMIT 5.0
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
//void left_turn(int tspeed) {
//    digitalWrite(InR1, LOW);
//    digitalWrite(InR2, HIGH);
//    digitalWrite(InR1, HIGH);
//    digitalWrite(InR2, LOW);
//    
//    analogWrite(PWML, tspeed);
//    analogWrite(PWMR, tspeed);
//}
//
//void right_turn(int tspeed) {
//    digitalWrite(InR1, HIGH);
//    digitalWrite(InR2, LOW);
//    digitalWrite(InR1, LOW);
//    digitalWrite(InR2, HIGH);
//    
//    analogWrite(PWML, tspeed);
//    analogWrite(PWMR, tspeed);
//}
//
//// Update position from encoder readings
//void update_position() {
//    // Calculate average position from both wheels
//    float avg_pulses = (wheel_pulse_count_left + wheel_pulse_count_right) / 2.0;
//    position = avg_pulses * METERS_PER_PULSE;
//    
//    // Calculate velocity
//    position_dot = (position - position_old) / 0.012;  // in m/s
//    position_old = position;
//}
//
//// Position control calculation
//float calculate_position_control() {
//    float position_error = position - desired_position;
//    float position_control = (kp_pos * position_error) + 
//                           (kd_pos * position_dot) + 
//                           (ki_pos * integral_position);
//    return position_control;
//}
//
//// Feedback function
//void feedback() {
//    // Get tilt angle from MPU6050
//    tilt = mpu.getAngleY();
//    tilt_dot = (tilt - tilt_old) / 0.012;  // Calculate tilt rate
//    tilt_old = tilt;
//
//    // Update position and velocity
//    update_position();
//
//    // Calculate integral terms
//    integral_tilt += tilt * 0.012;
//    integral_tilt = constrain(integral_tilt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
//    
//    integral_position += (position - desired_position) * 0.012;
//    integral_position = constrain(integral_position, -0.5, 0.5);  // Limit position integral
//}
//
//// Control equation function
//void control_eqn() {
//    // Calculate position control output
//    float position_control = calculate_position_control();
//    
//    // Combine position and angle control
//    // Position control modifies the desired angle
//    float angle_setpoint = -position_control;  // Negative because forward tilt moves forward
//    angle_setpoint = constrain(angle_setpoint, -5.0, 5.0);  // Limit maximum tilt
//    
//    // PID control for angle
//    float control = (kp * (tilt - angle_setpoint)) + 
//                   (kd * tilt_dot) + 
//                   (ki * integral_tilt);
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
//// ISR for left motor encoder
//void mot_rencoder_left() {
//    if (digitalRead(encodPinBL) > digitalRead(encodPinAL)) {
//        wheel_pulse_count_left = wheel_pulse_count_left + 1;
//    } 
//    else {
//        wheel_pulse_count_left = wheel_pulse_count_left - 1;
//    }
//}
//
//// ISR for right motor encoder
//void mot_rencoder_right() {
//    if (digitalRead(encodPinBR) > digitalRead(encodPinAR)) {
//        wheel_pulse_count_right = wheel_pulse_count_right - 1;
//    }
//    else {
//        wheel_pulse_count_right = wheel_pulse_count_right + 1;
//    }
//}
//
//// Setup function
//void setup() {
//    Serial.begin(115200);
//    bluetooth.begin(9600);
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
//        while (1);
//    }
//    Serial.println("MPU initialization done");
//    delay(1000);
//    mpu.calcOffsets(true, true);
//
//    // Encoder setup
//    pinMode(encodPinAL, INPUT_PULLUP);
//    pinMode(encodPinBL, INPUT_PULLUP);
//    pinMode(encodPinAR, INPUT_PULLUP);
//    pinMode(encodPinBR, INPUT_PULLUP);
//    
//    digitalWrite(encodPinAL, HIGH);
//    digitalWrite(encodPinBL, HIGH);
//    digitalWrite(encodPinAR, HIGH);
//    digitalWrite(encodPinBR, HIGH);
//    
//    attachInterrupt(digitalPinToInterrupt(encodPinBL), mot_rencoder_left, RISING);
//    attachInterrupt(digitalPinToInterrupt(encodPinBR), mot_rencoder_right, RISING);
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
//    Serial.println("Controls:");
//    Serial.println("F: Move forward 10cm");
//    Serial.println("B: Move backward 10cm");
//    Serial.println("L: Turn left");
//    Serial.println("R: Turn right");
//    Serial.println("S: Stop and hold position");
//}
//
//// Loop function
//void loop() {
//    if (bluetooth.available()) {
//        char input = bluetooth.read();
//        Serial.println(input);
//        
//        switch(input) {
//            case 'F':
//            case 'f':
//                desired_position += 0.1;  // Move forward 10cm
//                Serial.print("New desired position: ");
//                Serial.println(desired_position);
//                break;
//                
//            case 'B':
//            case 'b':
//                desired_position -= 0.1;  // Move backward 10cm
//                Serial.print("New desired position: ");
//                Serial.println(desired_position);
//                break;
//                
//            case 'L':
//            case 'l':
//                left_turn(255);
//                break;
//
//            case 'R':
//            case 'r':
//                right_turn(255);
//                break;
//                
//            case 'S':
//            case 's':
//                desired_position = position;  // Stop and hold current position
//                break;
//        }
//    }
//
//    // Optional: Debug output
//    if (millis() - last_sample_time >= SAMPLE_INTERVAL) {
//        last_sample_time = millis();
//        Serial.print("Position: ");
//        Serial.print(position);
//        Serial.print(" Desired: ");
//        Serial.print(desired_position);
//        Serial.print(" Tilt: ");
//        Serial.println(tilt);
//    }
//}
