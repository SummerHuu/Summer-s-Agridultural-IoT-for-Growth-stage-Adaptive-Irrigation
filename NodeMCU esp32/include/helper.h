#ifndef HELPER_H
#define HELPER_H

// serial communication===============================================================================================
void readSerial(HardwareSerial &port, String label, String &value, String &valueTemp, int &index);

// WiFi=========================================================================================================
const char *getWiFiStatus(int status);

// water pump===============================================================================================
String chooseCommands(String C1, String C2);
bool identifyCommands(const char myArray[], const unsigned int length, char value);
void filterCommands(volatile int &index, String &value, const char pool[], const unsigned int poolLength);
bool pump_Delay(unsigned long current, unsigned long step, bool &status);
// void pump_count(float quota, float current, bool &status);

// water flow meter YF-S401===============================================================================================
void IRAM_ATTR countPulse();
void readFlow();

// DHT22 sensor===============================================================================================
void DHT22_readTH(float &t, float &h);

// soil moisture sensor===============================================================================================
void readMoisture(float &percent);

// soil quality assessment
void assessSoil(float parameter, int index);

// timer
boolean timer(unsigned long lrt, unsigned long interval);
boolean repeatedTimer(unsigned long &lrt, unsigned long interval);

#endif