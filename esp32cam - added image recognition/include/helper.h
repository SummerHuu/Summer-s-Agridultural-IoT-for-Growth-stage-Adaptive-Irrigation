#ifndef HELPER_H
#define HELPER_H

// serial communication
// read UART
void readSerial(HardwareSerial &port, String label, String &value, String &valueTemp, volatile int &index);
// send UART
void sendSerial(HardwareSerial &port, char value);

// command identifier
bool identifyCommands(const char myArray[], const unsigned int length, char value);
void filterCommands(volatile int &index, String &value, const char pool[], const unsigned int poolLength);
String chooseCommands(String C1, String C2);

// WiFi
const char *getWiFiStatus(int status);

#endif