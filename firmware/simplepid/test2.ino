//#include <MPU6050_light.h>
//#include "Wire.h"
//#include <ServoTimer2.h>
//#include <SoftwareSerial.h>  // For Bluetooth communication
//
//// Define encoder pins
//#define encodPinAR 7
//#define encodPinBR 2
//#define encodPinAL 8
//#define encodPinBL 3
//
//// Define motor pins
//#define InL1 A3
//#define PWML 5
//#define InL2 A2
//#define InR1 9
//#define PWMR 6
//#define InR2 4
//
//// Define servo pins
//#define gripperPin 11
//#define armPin 10
//
//// Define buzzer pin
//#define buzzerPin 12
//
//// MPU6050 object
//MPU6050 mpu(Wire);
//
//// Variables for tilt control
//float tilt = 0.0, tilt_old = 0.0, tilt_dot = 0.0, integral_tilt = 0.0;
//float desired_tilt = 0.0, initial_tilt = 0.0;
//
//// PID gains for tilt control
//float kp_tilt = 90.0;
//float kd_tilt = 1.26;
//float ki_tilt = 560;
//
//// Encoder variables
//volatile int wheel_pulse_count_left = 0;
//volatile int wheel_pulse_count_right = 0;
//float wheel_speed_left = 0.0, wheel_speed_right = 0.0;
//const float WHEEL_RADIUS = 0.03; // Radius in meters
//const int PULSES_PER_REVOLUTION = 360;
//const float SAMPLE_INTERVAL = 0.02; // 20ms
//
//// PID gains for speed control
//float kp_speed = 20.0;
//float kd_speed = 0.5;
//float ki_speed = 50.0;
//
//// Timing variables
//unsigned long last_sample_time = 0;
//
//// Servo variables
//ServoTimer2 servoGripper;
//ServoTimer2 servoArm;
//int currentAngleGripper = 60;  // Initial angle for the gripper servo
//int currentAngleArm = 0;       // Initial angle for the arm servo
//
//// Function prototypes
//void motor_init();
//void motor_control_L(int pwm);
//void motor_control_R(int pwm);
//void feedback();
//void control_eqn();
//void calculate_wheel_speed();
//int angleToPulse(int angle);
//void left_turn(int tspeed);
//void right_turn(int tspeed);
//
//// ISRs for encoders
//void mot_rencoder_left() {
//    if (digitalRead(encodPinBL) > digitalRead(encodPinAL)) {
//        wheel_pulse_count_left++;
//    } else {
//        wheel_pulse_count_left--;
//    }
//}
//
//void mot_rencoder_right() {
//    if (digitalRead(encodPinBR) > digitalRead(encodPinAR)) {
//        wheel_pulse_count_right--;
//    } else {
//        wheel_pulse_count_right++;
//    }
//}
//
//void setup() {
//    Serial.begin(9600);
//
//    // Initialize MPU6050
//    Wire.begin();
//    byte status = mpu.begin();
//    if (status != 0) {
//        Serial.println("MPU6050 initialization failed!");
//        while (1);
//    }
//    mpu.calcOffsets(true, true);
//    initial_tilt = mpu.getAngleY();
//    desired_tilt = initial_tilt;
//
//    // Initialize encoders
//    pinMode(encodPinAL, INPUT_PULLUP);
//    pinMode(encodPinBL, INPUT_PULLUP);
//    pinMode(encodPinAR, INPUT_PULLUP);
//    pinMode(encodPinBR, INPUT_PULLUP);
//    attachInterrupt(digitalPinToInterrupt(encodPinBL), mot_rencoder_left, RISING);
//    attachInterrupt(digitalPinToInterrupt(encodPinBR), mot_rencoder_right, RISING);
//
//    // Initialize motors
//    motor_init();
//
//    // Initialize servos
//    servoGripper.attach(gripperPin);
//    servoArm.attach(armPin);
//    servoGripper.write(angleToPulse(currentAngleGripper));
//    servoArm.write(angleToPulse(currentAngleArm));
//
//    // Initialize buzzer
//    pinMode(buzzerPin, OUTPUT);
//    digitalWrite(buzzerPin, LOW);
//
//    Serial.println("Setup complete.");
//}
//
//void loop() {
//    unsigned long current_time = millis();
//    if (current_time - last_sample_time >= SAMPLE_INTERVAL * 1000) {
//        last_sample_time = current_time;
//
//        mpu.update();
//        feedback();
//        calculate_wheel_speed();
//        control_eqn();
//
//        Serial.print("Tilt: ");
//        Serial.print(tilt);
//        Serial.print(", Speed L: ");
//        Serial.print(wheel_speed_left);
//        Serial.print(", Speed R: ");
//        Serial.print(wheel_speed_right);
//        Serial.println();
//    }
//}
//
//void motor_init() {
//    pinMode(InL1, OUTPUT);
//    pinMode(InL2, OUTPUT);
//    pinMode(PWML, OUTPUT);
//    pinMode(InR1, OUTPUT);
//    pinMode(InR2, OUTPUT);
//    pinMode(PWMR, OUTPUT);
//}
//
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
//void feedback() {
//    tilt = mpu.getAngleY();
//    tilt_dot = (tilt - tilt_old) / SAMPLE_INTERVAL;
//    tilt_old = tilt;
//
//    integral_tilt += tilt * SAMPLE_INTERVAL;
//    integral_tilt = constrain(integral_tilt, -10.0, 10.0);
//}
//
//void calculate_wheel_speed() {
//    static int last_pulse_left = 0, last_pulse_right = 0;
//    static unsigned long last_time = 0;
//
//    unsigned long current_time = millis();
//    float delta_time = (current_time - last_time) / 1000.0;
//
//    wheel_speed_left = (wheel_pulse_count_left - last_pulse_left) / (PULSES_PER_REVOLUTION * delta_time);
//    wheel_speed_right = (wheel_pulse_count_right - last_pulse_right) / (PULSES_PER_REVOLUTION * delta_time);
//
//    last_pulse_left = wheel_pulse_count_left;
//    last_pulse_right = wheel_pulse_count_right;
//    last_time = current_time;
//}
//
//void control_eqn() {
//    float tilt_control = (kp_tilt * (tilt - desired_tilt)) + (kd_tilt * tilt_dot) + (ki_tilt * integral_tilt);
//
//    float speed_error_left = tilt_control - wheel_speed_left;
//    float speed_error_right = tilt_control - wheel_speed_right;
//
//    float speed_control_left = (kp_speed * speed_error_left);
//    float speed_control_right = (kp_speed * speed_error_right);
//
//    int control_output_left = constrain(speed_control_left, -255, 255);
//    int control_output_right = constrain(speed_control_right, -255, 255);
//
//    motor_control_L(control_output_left);
//    motor_control_R(control_output_right);
//}
//
//int angleToPulse(int angle) {
//    return 750 + ((angle / 180.0) * 1500);
//}
