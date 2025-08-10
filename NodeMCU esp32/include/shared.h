#ifndef SHARED_H
#define SHARED_H

// uart communication========================================================================================
extern String UART0_value;

// water pump===============================================================================================
// pin
extern const int pump_pin;
extern bool pump_status;
// command
extern const unsigned long command_interval;
extern unsigned long command_lastReadTime;
extern bool command_status;

// water flow meter YF-S401===============================================================================================
// pin configuration
extern const int flow_pin; // Pin connected to the flow meter
// pulse
extern const unsigned int flow_ratio;
extern volatile unsigned long flow_pulseCount; // Variable to hold the pulse count from the flow meter
extern volatile unsigned long pulse_single;
// calcultation
extern float flow_rate;
extern float flow_rateAverage;
extern float flow_singleLiters;
extern float flow_totalLiters;
extern int irrigationCounter;
extern unsigned long flow_singleTimer;
// calculation timer
extern const float flow_interval;
extern unsigned long flow_lastTime;
// print timer
extern const float flow_printInterval;
extern unsigned long flow_lastPrintTime;

// DHT22 sensor===============================================================================================
extern const int DHT_PIN;  // pin for the DHT22 sensor
extern const int DHT_TYPE; // type of the DHT sensor
extern DHT dht;            // create an instance of the DHT class
// timer
extern unsigned long HDT22_lastReadTime;

// soil mosture sensor===============================================================================================
// pin configuration
extern const int moisture_pin;
// calibration
extern const int moisture_dryValue;
extern const int moisture_wetValue;
// timer
extern unsigned long moisture_lastReadTime;

// soil quality assessment
extern const float soilMoistureClass[7][3];
extern char soilIndex;
extern const char soilIndexPool[];
extern String soilQuality;
extern const String soilQualityPool[];

#endif