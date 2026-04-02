//#include <MPU6050_light.h>
//#include "Wire.h"
//#include <SoftwareSerial.h>  // Include SoftwareSerial for Bluetooth communication
//#include <ServoTimer2.h>
//
//// Define SoftwareSerial pins for Bluetooth
//SoftwareSerial bluetooth(A1, A0);  // RX (A1), TX (A0) for HC-05 Bluetooth module
//
//// Define MPU6050 object
//MPU6050 mpu (Wire);
//
//// Variables for tilt angle control
//float tilt = 0.0, tilt_old = 0.0, tilt_dot = 0.0, integral_tilt = 0.0;
//
//// Variables for wheel angle control
//int wheel_pulse_count = 0;
//float wheel_angle = 0.0, wheel_angle_old = 0.0, wheel_dot = 0.0, integral_wheel = 0.0;
//
//// Final output for wheel velocity
//int control_output = 0;
//
//// PID gains for tilt control
//float kp_tilt = 82.0;
//float kd_tilt = 0.758;
//float ki_tilt = 490;
//// PID gains for wheel control
//float kp_wheel = 5.2;
//float kd_wheel = 0.05;
//float ki_wheel = 8;
//
//// Motor control pins
//#define InL1 A3    // Left motor INA pin
//#define PWML 5     // Left motor PWM pin
//#define InL2 A2    // Left motor INB pin
//#define InR1 9     // Right motor INA pin
//#define PWMR 6     // Right motor PWM pin
//#define InR2 4     // Right motor INB pin
//
//// Encoder pins
//#define encodPinAR 2  // Encoder A pin
//#define encodPinBR 3  // Encoder B pin
//
//// Timing variables
//unsigned long last_sample_time = 0;
//const unsigned long SAMPLE_INTERVAL = 20; // Sampling interval (20ms)
//
//// Maximum integral windup limit
//#define INTEGRAL_LIMIT 15.0
//
//// Tilt control variables
//const float ANGLE_STEP = 1; // Increment/decrement step for desired angle
//float initial_tilt = 0.0;
//float desired_tilt = 0.0;
//
//// Servo pins and Bluetooth
//#define gripperPin  11
//#define armPin 10   // Pin for the second servo
//#define rxPin A1
//#define txPin A0
//
//ServoTimer2 servoGripper;    // Gripper servo
//ServoTimer2 servoArm;        // Arm servo
//
//int currentAngleGripper = 60;  // Initial angle for the gripper servo
//int currentAngleArm = 0;       // Initial angle for the arm servo
//
//// Function to initialize motor pins
//void motor_init() {
//    pinMode(InL1, OUTPUT);
//    pinMode(InL2, OUTPUT);
//    pinMode(PWML, OUTPUT);
//    pinMode(InR1, OUTPUT);
//    pinMode(InR2, OUTPUT);
//    pinMode(PWMR, OUTPUT);
//
//    // Stop motors initially
//    digitalWrite(InL1, LOW);
//    digitalWrite(InL2, LOW);
//    analogWrite(PWML, 0);
//    digitalWrite(InR1, LOW);
//    digitalWrite(InR2, LOW);
//    analogWrite(PWMR, 0);
//}
//
//// Timer1 setup for IMU updates
//void timer1_init() {
//    cli();  // Disable global interrupts
//    TIMSK1 = 0x01;  // Enable Timer1 overflow interrupt
//    TCCR1B = 0x00;  // Stop timer
//    TCNT1H = 0xA2;  // Load high byte
//    TCNT1L = 0x3F;  // Load low byte
//    TCCR1A = 0x00;
//    TCCR1C = 0x00;
//    TCCR1B = 0x02;  // Start timer with prescaler 8
//    sei();  // Enable global interrupts
//}
//
//// Function to control left motor
//void motor_control_L(int pwm) {
//    if (pwm < 0) {
//        digitalWrite(InL1, HIGH);
//        digitalWrite(InL2, LOW);
//        pwm = -pwm;  // Make PWM positive
//    } else {
//        digitalWrite(InL1, LOW);
//        digitalWrite(InL2, HIGH);
//    }
//    analogWrite(PWML, pwm);
//}
//
//// Function to control right motor
//void motor_control_R(int pwm) {
//    if (pwm < 0) {
//        digitalWrite(InR1, HIGH);
//        digitalWrite(InR2, LOW);
//        pwm = -pwm;  // Make PWM positive
//    } else {
//        digitalWrite(InR1, LOW);
//        digitalWrite(InR2, HIGH);
//    }
//    analogWrite(PWMR, pwm);
//}
//
//// Function for left turn
//void left_turn(int tspeed) {
//    digitalWrite(InR1, LOW);
//    digitalWrite(InR2, HIGH);
//    digitalWrite(InL1, HIGH);
//    digitalWrite(InL2, LOW);
//    analogWrite(PWML, tspeed);
//    analogWrite(PWMR, tspeed);
//}
//
//// Function for right turn
//void right_turn(int tspeed) {
//    digitalWrite(InR1, HIGH);
//    digitalWrite(InR2, LOW);
//    digitalWrite(InL1, LOW);
//    digitalWrite(InL2, HIGH);
//    analogWrite(PWML, tspeed);
//    analogWrite(PWMR, tspeed);
//}
//
//// Timer1 ISR for IMU updates
//ISR(TIMER1_OVF_vect) {
//    sei();
//    TCNT1H = 0xA2;
//    TCNT1L = 0x3F;
//    mpu.update();  // Update IMU data
//    cli();
//
//    feedback();     // Update feedback variables
//    control_eqn();  // Execute control equation
//}
//
//// Feedback function to calculate tilt and wheel parameters
//void feedback() {
//    tilt = mpu.getAngleY();  // Get tilt angle
//    tilt_dot = (tilt - tilt_old) / 0.012;  // Tilt rate (derivative)
//    tilt_old = tilt;
//
//    wheel_angle = (wheel_pulse_count * 360.0) / 700.0;  // Convert pulse count to degrees
//    wheel_dot = (wheel_angle - wheel_angle_old) / 0.012;  // Wheel speed (derivative)
//    wheel_angle_old = wheel_angle;
//
//    // Update integral terms with anti-windup
//    integral_tilt += tilt * 0.012;
//    integral_tilt = constrain(integral_tilt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
//
//    integral_wheel += wheel_angle * 0.012;
//    integral_wheel = constrain(integral_wheel, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
//}
//
//// Converts angle to pulse width for servo
//int angleToPulse(int angle) {
//    return 750 + ((angle / 180.0) * 1500);
//}
//
//// Control equation to combine tilt and wheel control
//void control_eqn() {
//    float tilt_control = (kp_tilt * tilt) + (kd_tilt * tilt_dot) + (ki_tilt * integral_tilt);
//    float wheel_control = (kp_wheel * wheel_angle) + (kd_wheel * wheel_dot) + (ki_wheel * integral_wheel);
//
//    control_output = tilt_control + wheel_control;  // Combine tilt and wheel control
//    control_output = constrain(control_output, -255, 255);  // Constrain output for motors
//
//    motor_control_L(control_output);  // Control left motor
//    motor_control_R(control_output);  // Control right motor
//}
//
//// ISR for encoder input
//void motor_encoder() {
//    if (digitalRead(encodPinBR) == HIGH) {
//        wheel_pulse_count--;  // Decrease pulse count
//    } else {
//        wheel_pulse_count++;  // Increase pulse count
//    }
//}
//
//// Setup function
//void setup() {
//    Serial.begin(115200);
//    Serial.begin(9600);
//
//    // Initialize buzzer on D12
//    pinMode(12, OUTPUT); // Initialize D12 as output for buzzer
//    digitalWrite(12, LOW); // Start with the buzzer off
//
//    // Attach the servos to their respective pins
//    servoGripper.attach(gripperPin);     // Attach the gripper servo pin
//    servoArm.attach(armPin);             // Attach the arm servo pin
//    servoGripper.write(angleToPulse(currentAngleGripper));  // Initial angle for the gripper servo
//    servoArm.write(angleToPulse(currentAngleArm));          // Initial angle for the arm servo
//    
//    bluetooth.begin(9600);  // Start Bluetooth communication at 9600 baud rate
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
//    attachInterrupt(digitalPinToInterrupt(2), motor_encoder, RISING);  // Attach encoder ISR
//    Serial.println("System ready!");
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
//                digitalWrite(12, HIGH);  // Turn on the buzzer
//                Serial.println("Buzzer ON");
//                break;
//
//            case 'z':
//                digitalWrite(12, LOW);  // Turn off the buzzer
//                Serial.println("Buzzer OFF");
//                break;
//
//            case 'U': // Decrease gripper angle
//                currentAngleGripper = constrain(currentAngleGripper - 10, 60, 130);
//                servoGripper.write(angleToPulse(currentAngleGripper));
//                Serial.print("Decreased gripper angle: ");
//                Serial.println(currentAngleGripper);
//                break;
//                
//            case 'D': // Increase gripper angle
//                currentAngleGripper = constrain(currentAngleGripper + 10, 60, 130);
//                servoGripper.write(angleToPulse(currentAngleGripper));
//                Serial.print("Increased gripper angle: ");
//                Serial.println(currentAngleGripper);
//                break;
//
//            case 'O': // Move arm servo to 0 degrees
//                currentAngleArm = 0;
//                servoArm.write(angleToPulse(currentAngleArm));
//                Serial.println("Arm servo moved to 0 degrees");
//                break;
//
//            case 'C': // Move arm servo to 180 degrees
//                currentAngleArm = 180;
//                servoArm.write(angleToPulse(currentAngleArm));
//                Serial.println("Arm servo moved to 180 degrees");
//                break;
//
//            case 'P': // Reset desired tilt
//                desired_tilt = initial_tilt;
//                break;
//                
//            default:
//                Serial.println("Unknown command.");
//                break;
//        }
//    }
//}
