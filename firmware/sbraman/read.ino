//#define RX_PIN A1 // Connect to TX of Bluetooth module
//#define TX_PIN A0 // Connect to RX of Bluetooth module
//
//#include <SoftwareSerial.h>
//
//SoftwareSerial bluetooth(RX_PIN, TX_PIN);
//
//String bluetoothBuffer = ""; // Buffer to store incoming data
//
//// Variables to store PID values
//float pid_p = 0, pid_i = 0, pid_d = 0;
//
//void setup() {
//  Serial.begin(9600); // Serial Monitor baud rate
//  bluetooth.begin(9600); // Bluetooth module baud rate
//}
//
//void loop() {
//  // Check if data is available from Bluetooth
//  if (bluetooth.available()) {
//    char receivedChar = bluetooth.read(); // Read one character
//    if (receivedChar == '\n' || receivedChar == '\r') {
//      // End of a message
//      if (bluetoothBuffer.length() > 0) {
//        parseBluetoothData(bluetoothBuffer); // Parse the received data
//        bluetoothBuffer = ""; // Clear buffer for next message
//      }
//    } else {
//      bluetoothBuffer += receivedChar; // Append character to the buffer
//    }
//  }
//
//  // Check if data is available from the Serial Monitor
//  if (Serial.available()) {
//    char serialData = Serial.read();
//    bluetooth.print(serialData); // Send data to the Bluetooth module
//  }
//}
//
//// Function to parse Bluetooth data
//void parseBluetoothData(String data) {
//  // Check and extract 'p', 'i', and 'd' values
//  if (data.startsWith("p=")) {
//    int raw_p = data.substring(2).toInt(); // Extract raw value
//    pid_p = mapFloat(raw_p, 0, 255, 0, 150); // Scale to 0-150
//  } else if (data.startsWith("i=")) {
//    int raw_i = data.substring(2).toInt(); // Extract raw value
//    pid_i = mapFloat(raw_i, 0, 255, 0, 600); // Scale to 0-600
//  } else if (data.startsWith("d=")) {
//    int raw_d = data.substring(2).toInt(); // Extract raw value
//    pid_d = mapFloat(raw_d, 0, 255, 0, 50); // Scale to 0-50
//  }
//  Serial.println("PID values: P=" + String(pid_p, 2) + " I=" + String(pid_i, 2) + " D=" + String(pid_d, 2));
//}
//
//// Function to map a float value from one range to another
//float mapFloat(int x, int in_min, int in_max, float out_min, float out_max) {
//  return (float)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}
