//#include "Arduino.h"
//#include <SoftwareSerial.h>
//
//// Define pins for Bluetooth communication
//const byte rxPin = A1;
//const byte txPin = A0;
//SoftwareSerial BTSerial(rxPin, txPin); // RX TX
//
//const char START_TOKEN = '?';
//const char END_TOKEN = ';';
//const int CHAR_TIMEOUT = 50;
//
//bool waitingForStartToken = true;
//String messageBuffer = "";
//
//void setup() {
//  pinMode(rxPin, INPUT);
//  pinMode(txPin, OUTPUT);
//  BTSerial.begin(9600);
//  Serial.begin(9600);
//}
//
//void loop() {
//  if (BTSerial.available()) {
//    processBluetoothData();
//  }
//}
//
//void processBluetoothData() {
//  char nextData;
//
//  if (waitingForStartToken) {
//    nextData = readUntil(START_TOKEN);
//    if (nextData == START_TOKEN) {
//      waitingForStartToken = false;
//    }
//  }
//
//  if (!waitingForStartToken) {
//    while (BTSerial.available()) {
//      nextData = BTSerial.read();
//      if (nextData == END_TOKEN) {
//        handleCompleteMessage(messageBuffer);
//        messageBuffer = "";
//        waitingForStartToken = true;
//        return;
//      }
//      messageBuffer += nextData;
//
//      if (messageBuffer.length() > CHAR_TIMEOUT) {
//        messageBuffer = "";
//        waitingForStartToken = true;
//        return;
//      }
//    }
//  }
//}
//
//char readUntil(char token) {
//  while (BTSerial.available()) {
//    char data = BTSerial.read();
//    if (data == token) {
//      return data;
//    }
//  }
//  return '\0';
//}
//
//void handleCompleteMessage(const String &message) {
//  float pValue, iValue, dValue;
//
//  if (parsePIDMessage(message, pValue, iValue, dValue)) {
//    Serial.println("P = " + String(pValue, 2));
//    Serial.println("I = " + String(iValue, 2));
//    Serial.println("D = " + String(dValue, 2));
//  } else {
//    Serial.println("Invalid message format");
//  }
//}
//
//bool parsePIDMessage(const String &message, float &pValue, float &iValue, float &dValue) {
//  String P = extractValue(message, "p=");
//  String I = extractValue(message, "i=");
//  String D = extractValue(message, "d=");
//
//  if (P == "" || I == "" || D == "") {
//    return false;
//  }
//
//  pValue = P.toFloat() / 500.0;
//  iValue = I.toFloat() / 100.0;
//  dValue = D.toFloat() / 5000.0;
//
//  return true;
//}
//
//String extractValue(const String &text, const String &prefix) {
//  int startIndex = text.indexOf(prefix);
//  if (startIndex == -1) {
//    return "";
//  }
//  startIndex += prefix.length();
//  int endIndex = text.indexOf('&', startIndex);
//  if (endIndex == -1) {
//    endIndex = text.length();
//  }
//  return text.substring(startIndex, endIndex);
//}
