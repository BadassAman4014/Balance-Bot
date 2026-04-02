//#include <ServoTimer2.h>  // the servo library
//#include <SoftwareSerial.h> // for Bluetooth communication
//
//// define pins for the servos and Bluetooth
//#define gripperPin  11
//#define armPin 10   // pin for the second servo
//#define rxPin    A1
//#define txPin    A0
//
//ServoTimer2 servoGripper;    // declare variable for the gripper servo
//ServoTimer2 servoArm;   // declare variable for the arm servo
//SoftwareSerial bluetooth(rxPin, txPin); // RX, TX pins for Bluetooth
//
//int currentAngleGripper = 60;  // initial angle for the gripper servo
//int currentAngleArm = 0; // initial angle for the arm servo
//
//void setup() {
//  servoGripper.attach(gripperPin);     // attach the gripper servo pin
//  servoArm.attach(armPin);   // attach the arm servo pin
//  servoGripper.write(angleToPulse(currentAngleGripper));  // write the initial angle for the gripper servo
//  servoArm.write(angleToPulse(currentAngleArm)); // write the initial angle for the arm servo
//
//  bluetooth.begin(9600); // start Bluetooth communication
//  Serial.begin(9600); // optional for debugging
//}
//
//// this function calculates the pulse width for a given angle
//int angleToPulse(int angle) {
//  return 750 + ((angle / 180.0) * 1500);
//}
//
//void loop() {
//  if (bluetooth.available()) { // check if data is received via Bluetooth
//    char command = bluetooth.read(); // read the received command
//
//    switch (command) {
//      case 'U': // decrease the angle of the gripper
//        currentAngleGripper = constrain(currentAngleGripper - 10, 60, 130);
//        servoGripper.write(angleToPulse(currentAngleGripper));
//        Serial.print("Decreased gripper angle: ");
//        Serial.println(currentAngleGripper);
//        break;
//        
//      case 'D': // increase the angle of the gripper
//        currentAngleGripper = constrain(currentAngleGripper + 10, 60, 130);
//        servoGripper.write(angleToPulse(currentAngleGripper));
//        Serial.print("Increased gripper angle: ");
//        Serial.println(currentAngleGripper);
//        break;
//
//      case 'O': // move the arm servo to 0 degrees
//        currentAngleArm = 0;
//        servoArm.write(angleToPulse(currentAngleArm));
//        Serial.println("Arm servo moved to 0 degrees");
//        break;
//
//      case 'C': // move the arm servo to 180 degrees
//        currentAngleArm = 180;
//        servoArm.write(angleToPulse(currentAngleArm));
//        Serial.println("Arm servo moved to 180 degrees");
//        break;
//
//      default:
//        // Handle any undefined command if needed
//        break;
//    }
//  }
//
//  delay(10); // small delay to allow smooth operation
//}
