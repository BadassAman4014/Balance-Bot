#include <MPU6050_light.h>
#include "Wire.h"
#include <SoftwareSerial.h>

#define RX_PIN A1 // Connect to TX of Bluetooth module
#define TX_PIN A0 // Connect to RX of Bluetooth module
SoftwareSerial bluetooth(RX_PIN, TX_PIN);

// Define MPU6050 object
MPU6050 mpu(Wire);

// Variables for tilt angle
float tilt = 0.0, tilt_old = 0.0, tilt_dot = 0.0, integral_tilt = 0.0;

// Final output for wheel velocity
int control_output = 0;

// PID gains
float kp = 47.0;
float kd = 1.26;
float ki = 750;

// Define motor pins
#define InL1 A3    // INA motor pin
#define PWML 5     // PWM motor pin
#define InL2 A2    // INB motor pin
#define InR1 9     // INA motor pin
#define PWMR 6     // PWM motor pin
#define InR2 4     // INB motor pin

// Desired tilt angle
float desired_tilt = 0.0;

// Maximum integral windup limit
#define INTEGRAL_LIMIT 10.0

String bluetoothBuffer = ""; // Buffer to store incoming data

// Variables to store PID values
float pid_p = 0, pid_i = 0, pid_d = 0;

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

// ISR for Timer1 overflow
ISR(TIMER1_OVF_vect) {
    sei();
    TCNT1H = 0xA2;
    TCNT1L = 0x3F;
    mpu.update();  // Update IMU data
    cli();
    feedback();
    control_eqn();  // Call control_eqn to apply updated PID values
}

// Feedback function
void feedback() {
    // Get tilt angle from MPU6050
    tilt = mpu.getAngleY();
    tilt_dot = (tilt - tilt_old) / 0.012;  // Calculate tilt rate
    tilt_old = tilt;

    // Calculate integral term and constrain to prevent windup
    integral_tilt += tilt * 0.012;
    integral_tilt = constrain(integral_tilt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
}

// Control equation function
void control_eqn() {
    // PID control (use updated kp, ki, kd)
    float control = (kp * (tilt - desired_tilt)) + (kd * tilt_dot) + (ki * integral_tilt);

    // Constrain output for PWM (0-255)
    control_output = constrain(control, -255, 255);

    // Drive motors
    motor_control_L(control_output);
    motor_control_R(control_output);
}

// Function to parse Bluetooth data and update PID gains
void parseBluetoothData(String data) {
  // Check and extract 'p', 'i', and 'd' values
  if (data.startsWith("p=")) {
    int raw_p = data.substring(2).toInt(); // Extract raw value
    pid_p = mapFloat(raw_p, 0, 255, 0, 150); // Scale to 0-150
    kp = pid_p; // Update kp with pid_p value
    Serial.print("Updated Kp: ");
    Serial.println(kp);
  } else if (data.startsWith("i=")) {
    int raw_i = data.substring(2).toInt(); // Extract raw value
    pid_i = mapFloat(raw_i, 0, 255, 0, 600); // Scale to 0-600
    ki = pid_i; // Update ki with pid_i value
    Serial.print("Updated Ki: ");
    Serial.println(ki);
  } else if (data.startsWith("d=")) {
    int raw_d = data.substring(2).toInt(); // Extract raw value
    pid_d = mapFloat(raw_d, 0, 255, 0, 50); // Scale to 0-50
    kd = pid_d; // Update kd with pid_d value
    Serial.print("Updated Kd: ");
    Serial.println(kd);
  }

  // Debugging printout of PID values
  Serial.println("PID values: P=" + String(pid_p, 2) + " I=" + String(pid_i, 2) + " D=" + String(pid_d, 2));
  Serial.println("Updated PID gains: Kp=" + String(kp, 2) + " Ki=" + String(ki, 2) + " Kd=" + String(kd, 2));
}

// Function to map a float value from one range to another
float mapFloat(int x, int in_min, int in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Setup function
void setup() {
    Serial.begin(115200);
    Serial.println("Setup starting...");

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
    tilt = mpu.getAngleY();
    Serial.print("Initial tilt angle: ");
    Serial.println(tilt);
    desired_tilt = tilt;

    // Motor setup
    motor_init();
    Serial.println("Motors initialized");

    // Timer setup
    timer1_init();
    Serial.println("Timer initialized");

    Serial.println("Setup completed!");
}

// Loop function
void loop() {
    // Check if data is available from Bluetooth
  if (bluetooth.available()) {
    char receivedChar = bluetooth.read(); // Read one character
    if (receivedChar == '\n' || receivedChar == '\r') {
      // End of a message
      if (bluetoothBuffer.length() > 0) {
        parseBluetoothData(bluetoothBuffer); // Parse the received data
        bluetoothBuffer = ""; // Clear buffer for next message
      }
    } else {
      bluetoothBuffer += receivedChar; // Append character to the buffer
    }
  }
}
