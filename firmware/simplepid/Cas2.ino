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
//float kp_tilt = 87.0;
//float kd_tilt = 0.7458;
//float ki_tilt = 570;
//
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
//#define INTEGRAL_LIMIT 100.0
//
//// Tilt control variables
//const float ANGLE_STEP = 0.75; // Increment/decrement step for desired angle
//float initial_tilt = 0.0;
//float desired_tilt = 0.0;
//
//// Wheel control variables
//const float WHEEL_STEP = 1; // Increment/decrement step for desired angle
//float initial_angle = 0.0;
//float desired_wheel_angle = 0.0;
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
//int currentAngleGripper = 50;  // Initial angle for the gripper servo
//int currentAngleArm = 0;       // Initial angle for the arm servo
//
//bool tilt_command_received = false; // Flag to track if 'F' or 'B' command was received
//
//// Timeout variables for resetting tilt
//unsigned long last_command_time = 0;       // Tracks the last time 'F' or 'B' was received
//const unsigned long COMMAND_TIMEOUT = 15000; // Timeout period in milliseconds
//
//// Function to initialize motor pins
//void motor_init() {
//  pinMode(InL1, OUTPUT);
//  pinMode(InL2, OUTPUT);
//  pinMode(PWML, OUTPUT);
//  pinMode(InR1, OUTPUT);
//  pinMode(InR2, OUTPUT);
//  pinMode(PWMR, OUTPUT);
//
//  // Stop motors initially
//  digitalWrite(InL1, LOW);
//  digitalWrite(InL2, LOW);
//  analogWrite(PWML, 0);
//  digitalWrite(InR1, LOW);
//  digitalWrite(InR2, LOW);
//  analogWrite(PWMR, 0);
//}
//
//// Timer1 setup for IMU updates
//void timer1_init() {
//  cli();  // Disable global interrupts
//  TIMSK1 = 0x01;  // Enable Timer1 overflow interrupt
//  TCCR1B = 0x00;  // Stop timer
//  TCNT1H = 0xA2;  // Load high byte
//  TCNT1L = 0x3F;  // Load low byte
//  TCCR1A = 0x00;
//  TCCR1C = 0x00;
//  TCCR1B = 0x02;  // Start timer with prescaler 8
//  sei();  // Enable global interrupts
//}
//
//// Motor control function for left motor
//void motor_control_L(int pwm) {
//  if (pwm < 0) {
//    digitalWrite(InL1, HIGH);
//    digitalWrite(InL2, LOW);
//    pwm = -pwm;
//  } else {
//    digitalWrite(InL1, LOW);
//    digitalWrite(InL2, HIGH);
//  }
//  analogWrite(PWML, pwm);
//}
//
//// Motor control function for right motor
//void motor_control_R(int pwm) {
//  if (pwm < 0) {
//    digitalWrite(InR1, HIGH);
//    digitalWrite(InR2, LOW);
//    pwm = -pwm;
//  } else {
//    digitalWrite(InR1, LOW);
//    digitalWrite(InR2, HIGH);
//  }
//  analogWrite(PWMR, pwm);
//}
//
//void left_turn(int tspeed) {
//  digitalWrite(InR1, LOW);
//  digitalWrite(InR2, HIGH);
//  digitalWrite(InR1, HIGH);
//  digitalWrite(InR2, LOW);
//
//  analogWrite(PWML, tspeed);
//  analogWrite(PWMR, tspeed);
//}
//
//void right_turn(int tspeed) {
//  digitalWrite(InR1, HIGH);
//  digitalWrite(InR2, LOW);
//  digitalWrite(InR1, LOW);
//  digitalWrite(InR2, HIGH);
//
//  analogWrite(PWML, tspeed);
//  analogWrite(PWMR, tspeed);
//}
//
//// Timer1 ISR for IMU updates
//ISR(TIMER1_OVF_vect) {
//  sei();
//  TCNT1H = 0xA2;
//  TCNT1L = 0x3F;
//  mpu.update();  // Update IMU data
//  cli();
//
//  feedback();     // Update feedback variables
//  control_eqn();  // Execute control equation
//}
//
//// Feedback function to calculate tilt and wheel parameters
//void feedback() {
//  tilt = mpu.getAngleY();  // Get tilt angle
//  tilt_dot = (tilt - tilt_old) / 0.012;  // Tilt rate (derivative)
//  tilt_old = tilt;
//
//  wheel_angle = (wheel_pulse_count * 360.0) / 700.0;  // Convert pulse count to degrees
//  wheel_dot = (wheel_angle - wheel_angle_old) / 0.012;  // Wheel speed (derivative)
//  wheel_angle_old = wheel_angle;
//
//  // Update integral terms with anti-windup
//  integral_tilt += tilt * 0.012;
//  integral_tilt = constrain(integral_tilt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
//
//  integral_wheel += wheel_angle * 0.012;
//  integral_wheel = constrain(integral_wheel, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
//}
//
//// Converts angle to pulse width for servo
//int angleToPulse(int angle) {
//  return 750 + ((angle / 180.0) * 1500);
//}
//
//// Control equation to combine tilt and wheel control
//void control_eqn() {
//  float tilt_control = (kp_tilt * (tilt - desired_tilt)) + (kd_tilt * tilt_dot) + (ki_tilt * integral_tilt);
//  float wheel_control = (kp_wheel * wheel_angle - desired_wheel_angle) + (kd_wheel * wheel_dot) + (ki_wheel * integral_wheel);
//
//  control_output = tilt_control + wheel_control;  // Combine tilt and wheel control
//  control_output = constrain(control_output, -255, 255);  // Constrain output for motors
//
//  motor_control_L(control_output);  // Control left motor
//  motor_control_R(control_output);  // Control right motor
//}
//
//// ISR for encoder input
//void motor_encoder() {
//  if (digitalRead(encodPinBR) == HIGH) {
//    wheel_pulse_count--;  // Decrease pulse count
//    wheel_angle = wheel_pulse_count;
//  } else {
//    wheel_pulse_count++;  // Increase pulse count
//    wheel_angle = wheel_pulse_count;
//  }
//}
//
//// Setup function
//void setup() {
//  Serial.begin(115200);
//  Serial.begin(9600);
//
//  // Initialize buzzer on D12
//  pinMode(12, OUTPUT); // Initialize D12 as output for buzzer
//  digitalWrite(12, LOW); // Start with the buzzer off
//
//  // Attach the servos to their respective pins
//  servoGripper.attach(gripperPin);     // Attach the gripper servo pin
//  servoArm.attach(armPin);             // Attach the arm servo pin
//
//  servoGripper.write(angleToPulse(currentAngleGripper));  // Initial angle for the gripper servo
//  servoArm.write(angleToPulse(currentAngleArm));          // Initial angle for the arm servo
//  delay(1000);
//
//  // Attach the servos to their respective pins
//  servoGripper.detach();     // Attach the gripper servo pin
//  servoArm.detach();             // Attach the arm servo pin
//
//  bluetooth.begin(9600);  // Start Bluetooth communication at 9600 baud rate
//  Serial.println("Control System Starting...");
//  Serial.println("Bluetooth is ready!");
//
//  // MPU6050 setup
//  Wire.begin();
//  byte status = mpu.begin();
//  Serial.print(F("MPU6050 status: "));
//  Serial.println(status);
//  if (status != 0) {
//    Serial.println("MPU initialization failed! Check connections.");
//    while (1);  // Halt program if MPU initialization fails
//  }
//  Serial.println("MPU initialization done");
//  delay(1000);
//  mpu.calcOffsets(true, true);
//
//  // Get initial tilt angle
//  mpu.update();
//  initial_tilt = mpu.getAngleY();
//  desired_tilt = initial_tilt;
//  Serial.print("Initial tilt angle: ");
//  Serial.println(initial_tilt);
//
//  // Motor setup
//  motor_init();
//  Serial.println("Motors initialized");
//
//  // Timer setup
//  timer1_init();
//  Serial.println("Timer initialized");
//  attachInterrupt(digitalPinToInterrupt(2), motor_encoder, RISING);  // Attach encoder ISR
//  Serial.println("System ready!");
//}
//
//// Loop function
//void loop() {
//  tilt_command_received = false; // Reset flag at the start of each loop
//  unsigned long current_time = millis();
//  // Check for data from Serial Monitor
//  if (bluetooth.available()) {
//    // Read the incoming character
//    char input = bluetooth.read();
//    switch (input) {
//      case 'F':
//        last_command_time = millis();
//        desired_tilt += ANGLE_STEP;
//        tilt_command_received = true; // Set flag
//
//        break;
//
//      case 'B':
//        last_command_time = millis();
//        desired_tilt -= ANGLE_STEP;
//        tilt_command_received = true; // Set flag
//
//        break;
//
//      case 'L':
//        last_command_time = millis();
//        left_turn(200);
//        tilt_command_received = true; // Set flag
//        break;
//
//      case 'R':
//        last_command_time = millis();
//        right_turn(200);
//        tilt_command_received = true; // Set flag
//        break;
//
//      case 'Z':
//        digitalWrite(12, HIGH);  // Turn on the buzzer
//        Serial.println("Buzzer ON");
//        break;
//
//      case 'z':
//        digitalWrite(12, LOW);  // Turn off the buzzer
//        Serial.println("Buzzer OFF");
//        break;
//
//      case 'U': // Decrease gripper angle
//        servoGripper.attach(gripperPin);
//        currentAngleGripper = constrain(currentAngleGripper - 10, 50, 130);
//        servoGripper.write(angleToPulse(currentAngleGripper));
//        delay(50);
//        servoGripper.detach();
//        break;
//
//      case 'D': // Increase gripper angle
//        servoGripper.attach(gripperPin);
//        currentAngleGripper = constrain(currentAngleGripper + 10, 50, 130);
//
//        if (currentAngleGripper > 100) {
//        servoArm.attach(armPin);
//        currentAngleArm = 180;
//        servoArm.write(angleToPulse(currentAngleArm));
//        delay(50);
//        servoArm.detach();
//        }
//
//        servoGripper.write(angleToPulse(currentAngleGripper));
//        delay(50);
//        servoGripper.detach();
//        break;
//
//
//      case 'O': // Move arm servo to 0 degrees
//        servoArm.attach(armPin);
//        currentAngleArm = 0;
//        servoArm.write(angleToPulse(currentAngleArm));
//        delay(50);
//        servoArm.detach();
//        break;
//
//      case 'C': // Move arm servo to 180 degrees
//        servoArm.attach(armPin);
//        currentAngleArm = 180;
//        servoArm.write(angleToPulse(currentAngleArm));
//        delay(50);
//        servoArm.detach();
//        break;
//
//      default:
//        Serial.println("Unknown command.");
//        break;
//    }
//  }
//  // Check if the timeout has been exceeded
//  if (current_time - last_command_time > COMMAND_TIMEOUT) {
//    // Check the flag to determine whether to reset the tilt
//    if (!tilt_command_received) {
//      desired_tilt = initial_tilt; // Reset to initial tilt if no 'F' or 'B' command was received
//    }
//  }
//}
