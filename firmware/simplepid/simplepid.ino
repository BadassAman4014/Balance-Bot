#include <MPU6050_light.h>
#include "Wire.h"
#include <SoftwareSerial.h>  // Include SoftwareSerial for Bluetooth communication
#include <ServoTimer2.h>

// Define SoftwareSerial pins for Bluetooth
SoftwareSerial bluetooth(A1, A0);  // RX, TX for HC-05 Bluetooth module

// Define MPU6050 object
MPU6050 mpu(Wire);

// Variables for tilt angle
float tilt = 0.0, tilt_old = 0.0, tilt_dot = 0.0, integral_tilt = 0.0;

// Final output for wheel velocity
int control_output = 0;

// PID gains
float kp = 90.0;
float kd = 0.98;
float ki = 560;

// Timing variables
unsigned long last_sample_time = 0;
const unsigned long SAMPLE_INTERVAL = 20; // Sample every 20ms

// Timeout variables for resetting tilt
unsigned long last_command_time = 0;       // Tracks the last time 'F' or 'B' was received
const unsigned long COMMAND_TIMEOUT = 15000; // Timeout period in milliseconds

// Control variables
const float ANGLE_STEP = 0.80; // Angle change for A/D keys
float initial_tilt = 0.0;
float desired_tilt = 0.0;

// Define motor pins
#define InL1 A3    // INA motor pin
#define PWML 5     // PWM motor pin
#define InL2 A2    // INB motor pin
#define InR1 9     // INA motor pin
#define PWMR 6     // PWM motor pin
#define InR2 4     // INB motor pin

// Maximum integral windup limit
#define INTEGRAL_LIMIT 130.0

// define pins for the servos and Bluetooth
#define gripperPin  11
#define armPin 10   // pin for the second servo
#define rxPin    A1
#define txPin    A0

ServoTimer2 servoGripper;    // declare variable for the gripper servo
ServoTimer2 servoArm;   // declare variable for the arm servo

int currentAngleGripper = 60;  // initial angle for the gripper servo
int currentAngleArm = 0; // initial angle for the arm servo

bool tilt_command_received = false; // Flag to track if 'F' or 'B' command was received


// Motor initialization function
void motor_init() {
  pinMode(InL1, OUTPUT);
  pinMode(InL2, OUTPUT);
  pinMode(PWML, OUTPUT);
  digitalWrite(InL1, LOW);
  digitalWrite(InL2, LOW);
  analogWrite(PWML, 0);

  pinMode(InR1, OUTPUT);
  pinMode(InR2, OUTPUT);
  pinMode(PWMR, OUTPUT);
  digitalWrite(InR1, LOW);
  digitalWrite(InR2, LOW);
  analogWrite(PWMR, 0);
}

// Timer1 ISR for IMU
void timer1_init() {
  cli();  // Clear global interrupts
  TIMSK1 = 0x01;  // Timer1 overflow interrupt enable
  TCCR1B = 0x00;  // Stop
  TCNT1H = 0xA2;  // Counter higher 8 bit value
  TCNT1L = 0x3F;  // Counter lower 8 bit value
  TCCR1A = 0x00;
  TCCR1C = 0x00;
  TCCR1B = 0x02;  // Start Timer, prescaler 8
  sei();  // Enable global interrupts
}

// Motor control function for left motor
void motor_control_L(int pwm) {
  if (pwm < 0) {
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
  }
  analogWrite(PWML, pwm);
}

// Motor control function for right motor
void motor_control_R(int pwm) {
  if (pwm < 0) {
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
  }
  analogWrite(PWMR, pwm);
}

void left_turn(int tspeed) {
  digitalWrite(InR1, LOW);
  digitalWrite(InR2, HIGH);
  digitalWrite(InR1, HIGH);
  digitalWrite(InR2, LOW);

  analogWrite(PWML, tspeed);
  analogWrite(PWMR, tspeed);
}

void right_turn(int tspeed) {
  digitalWrite(InR1, HIGH);
  digitalWrite(InR2, LOW);
  digitalWrite(InR1, LOW);
  digitalWrite(InR2, HIGH);

  analogWrite(PWML, tspeed);
  analogWrite(PWMR, tspeed);
}

// Feedback function
void feedback() {
  // Get tilt angle from MPU6050
  tilt = mpu.getAngleY();
  tilt_dot = (tilt - tilt_old) / 0.012;  // Calculate tilt rate
  tilt_old = tilt;

  // Calculate integral term and constrain to prevent windup
  integral_tilt += tilt * 0.009;
  integral_tilt = constrain(integral_tilt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
}

// Control equation function
void control_eqn() {
  // PID control
  float control = (kp * (tilt - desired_tilt)) + (kd * tilt_dot) + (ki * integral_tilt);
  // Constrain output for PWM (0-255)
  control_output = constrain(control, -255, 255);
  // Drive motors
  motor_control_L(control_output);
  motor_control_R(control_output);

}

// this function calculates the pulse width for a given angle
int angleToPulse(int angle) {
  return 750 + ((angle / 180.0) * 1500);
}

// ISR for Timer1 overflow
ISR(TIMER1_OVF_vect) {
  sei();
  TCNT1H = 0xA2;
  TCNT1L = 0x3F;
  mpu.update();  // Update IMU data
  cli();
  feedback();
  control_eqn();
}

// Setup function
void setup() {
  Serial.begin(9600);

  // Initialize buzzer on D12
  pinMode(12, OUTPUT); // Initialize D12 as output for buzzer
  digitalWrite(12, LOW); // Start with the buzzer off

  // Attach the servos to their respective pins
  servoGripper.attach(gripperPin);     // Attach the gripper servo pin
  servoArm.attach(armPin);             // Attach the arm servo pin

  servoGripper.write(angleToPulse(currentAngleGripper));  // Initial angle for the gripper servo
  servoArm.write(angleToPulse(currentAngleArm));          // Initial angle for the arm servo
  delay(1000);

  // Attach the servos to their respective pins
  servoGripper.detach();     // Attach the gripper servo pin
  servoArm.detach();             // Attach the arm servo pin

  bluetooth.begin(9600); // Start Bluetooth communication at 9600 baud rate
  Serial.println("Control System Starting...");
  Serial.println("Bluetooth is ready!");

  // MPU6050 setup
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  if (status != 0) {
    Serial.println("MPU initialization failed! Check connections.");
    while (1);  // Halt program if MPU initialization fails
  }
  Serial.println("MPU initialization done");
  delay(1000);
  mpu.calcOffsets(true, true);

  // Get initial tilt angle
  mpu.update();
  initial_tilt = mpu.getAngleY();
  desired_tilt = initial_tilt;
  Serial.print("Initial tilt angle: ");
  Serial.println(initial_tilt);

  // Motor setup
  motor_init();
  Serial.println("Motors initialized");

  // Timer setup
  timer1_init();
  Serial.println("Timer initialized");
}



// Loop function
void loop() {
  tilt_command_received = false; // Reset flag at the start of each loop
  unsigned long current_time = millis();
  // Check for data from Serial Monitor
  if (bluetooth.available()) {
    // Read the incoming character
    char input = bluetooth.read();
    switch (input) {
      case 'F':
        last_command_time = millis();
        desired_tilt += ANGLE_STEP;
        tilt_command_received = true; // Set flag

        break;

      case 'B':
        last_command_time = millis();
        desired_tilt -= ANGLE_STEP;
        tilt_command_received = true; // Set flag

        break;

      case 'L':
        last_command_time = millis();
        left_turn(100);
        tilt_command_received = true; // Set flag
        break;

      case 'R':
        last_command_time = millis();
        right_turn(100);
        tilt_command_received = true; // Set flag
        break;

      case 'Z':
        digitalWrite(12, HIGH);  // Turn on the buzzer
        Serial.println("Buzzer ON");
        break;

      case 'z':
        digitalWrite(12, LOW);  // Turn off the buzzer
        Serial.println("Buzzer OFF");
        break;

      case 'U': // Decrease gripper angle
        servoGripper.attach(gripperPin);
        currentAngleGripper = constrain(currentAngleGripper - 10, 0, 130);
        servoGripper.write(angleToPulse(currentAngleGripper));
        delay(50);
        servoGripper.detach();
        break;

      case 'D': // Increase gripper angle
        servoGripper.attach(gripperPin);
        currentAngleGripper = constrain(currentAngleGripper + 10, 0, 130);

        if (currentAngleGripper > 100) {
          servoArm.attach(armPin);
          currentAngleArm = 180;
          servoArm.write(angleToPulse(currentAngleArm));
          delay(50);
          servoArm.detach();
        }
        servoGripper.write(angleToPulse(currentAngleGripper));
        delay(50);
        servoGripper.detach();
        break;


      case 'O': // Move arm servo to 0 degrees
        servoArm.attach(armPin);
        currentAngleArm = 0;
        servoArm.write(angleToPulse(currentAngleArm));
        delay(50);
        servoArm.detach();
        break;

      case 'C': // Move arm servo to 180 degrees
        servoArm.attach(armPin);
        currentAngleArm = 180;
        servoArm.write(angleToPulse(currentAngleArm));
        delay(50);
        servoArm.detach();
        break;

       case 'r': // Move arm servo to 0 degrees
        servoGripper.attach(gripperPin);
        currentAngleGripper = 60;
        servoGripper.write(angleToPulse(currentAngleGripper));
        delay(50);
        servoGripper.detach();
        break;

      default:
        Serial.println("Unknown command.");
        break;
    }
  }
  // Check if the timeout has been exceeded
  if (current_time - last_command_time > COMMAND_TIMEOUT) {
    // Check the flag to determine whether to reset the tilt
    if (!tilt_command_received) {
      desired_tilt = initial_tilt; // Reset to initial tilt if no 'F' or 'B' command was received
    }
  }
}
