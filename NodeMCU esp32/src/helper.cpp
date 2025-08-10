#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "helper.h"
#include "shared.h"

// serial communication===============================================================================================
// read UART
void readSerial(HardwareSerial &port, String label, String &value, String &valueTemp, int &index)
{
    // Read data from UART0 (laptop)
    while (port.available())
    {
        char c = port.read();

        // Check if the character is a newline character
        if (c == '\n') // If the character is a newline character, it indicates the end of a message
        {
            value = valueTemp;      // Assign the temporary value to the value string
            valueTemp = "";         // Reset the temporary value to empty after processing
            value.trim();           // Remove any leading or trailing whitespace from the value string
            index = 1;              // indicate that a new message has been received
            if (value.length() > 0) // Check if the value string is not empty
            {
                Serial.print("\n[" + label + "]: ");
                Serial.println(value);
            }
            else
            {
                Serial.println("No data received from " + label); // Print a message if no data was received
            }
            break;
        }
        else
        {
            valueTemp += c;
        }
    }
}

// WiFi=========================================================================================================
const char *getWiFiStatus(int status)
{
    switch (status)
    {
    case WL_IDLE_STATUS:
        return "Idle";
    case WL_NO_SSID_AVAIL:
        return "No SSID Available";
    case WL_SCAN_COMPLETED:
        return "Scan Completed";
    case WL_CONNECTED:
        return "Connected";
    case WL_CONNECT_FAILED:
        return "Connection Failed";
    case WL_CONNECTION_LOST:
        return "Connection Lost";
    case WL_DISCONNECTED:
        return "Disconnected";
    default:
        return "Unknown";
    }
}

// water pump===============================================================================================
// command identifier
bool identifyCommands(const char myArray[], const unsigned int length, char value)
{
    for (unsigned int i = 0; i < length; i++)
    {
        if (myArray[i] == value)
        {
            return true;
        }
    }
    return false;
}
// filter command
void filterCommands(volatile int &index, String &value, const char pool[], const unsigned int poolLength)
{
    if (index) // If a new message has been received on UART0
    {
        if (!identifyCommands(pool, poolLength, value.charAt(0)))
        {
            value = ""; // reset the UART_Value to empty after processing
        }
        index = 0; // reset the UART_ValueIndex to 0 after processing
    }
}
// choosing command
String chooseCommands(String C1, String C2)
{
    if (C1.length() > 0)
    {
        // Serial.println("C1: " + C1);
        return C1;
    }

    if (C2.length() > 0)
    {
        // Serial.println("C2: " + C2);
        return C2;
    }
    return "";
}
// delay
bool pump_Delay(unsigned long current, unsigned long step, bool &status)
{
    if (millis() >= current + step) // check if the time has passed
    {
        // ledcWrite(pwmChannel, 0);
        digitalWrite(pump_pin, 0); // stop the pum
        status = 0;                // reset the status to 0 to indicate that the pumps are stopped
        pump_status = 0;
        command_status = 0;
        command_lastReadTime = millis();

        Serial.println("[Irrigation] Average flow rate: " + String(flow_rateAverage, 2) + " L/min; Single: " + String(flow_singleLiters, 1) + "  mL; Irrigation counter: " + irrigationCounter + " ;Total: " + String(flow_totalLiters, 1) + " mL");
        Serial.println("\n[Irrigation] end");
        Serial.println("[Irrigation] command was disabled for " + String(command_interval) + " s\n");

        return true; // delay has passed
    }
    return false; // delay has not passed yet
}
// quota
/* void pump_count(float quota, float current, bool &status)
{
    if (current >= quota) // check if the time has passed
    {
        // ledcWrite(pwmChannel, 0);
        digitalWrite(pump_pin, 0); // stop the pum
        status = 0;                // reset the status to 0 to indicate that the pumps are stopped
        pump_status = 0;
        command_status = 0;
        command_lastReadTime = millis();

        Serial.println("[Irrigation] Average flow rate: " + String(flow_rateAverage, 2) + " L/min; Single: " + String(flow_singleLiters, 1) + "  mL; Irrigation counter: " + irrigationCounter + " ;Total: " + String(flow_totalLiters, 1) + " mL");
        Serial.println("\n[Irrigation] end");
        Serial.println("[Irrigation] command was disabled for " + String(command_interval) + " s\n");
    }
} */

// water flow meter YF-S401===============================================================================================
void IRAM_ATTR countPulse()
{
    // the code here must be very simple; no any print or calculation
    flow_pulseCount++;
}
void readFlow()
{
    // calculation timer
    float interval_ms = flow_interval * 1000; // convert input seconds to milliseconds
    unsigned long elapsedTime = millis() - flow_lastTime;

    // print timer
    float intervalP_ms = flow_printInterval * 1000;
    unsigned long elapsedTimeP = millis() - flow_lastPrintTime;

    if (elapsedTime >= (unsigned long)interval_ms) // every calculation interval
    {
        detachInterrupt(digitalPinToInterrupt(flow_pin)); // detach the interrupt to prevent counting during the calculation

        // average single flow rate
        pulse_single += flow_pulseCount;
        flow_rateAverage = float(pulse_single) / (millis() - flow_singleTimer) / flow_ratio * 60; // L/min

        // calculation
        float pulseSnapshot = float(flow_pulseCount) / elapsedTime * 1000; // pulses/s
        flow_pulseCount = 0;
        flow_rate = pulseSnapshot / flow_ratio * 60 / 1000; // L/min
        flow_singleLiters += flow_rate * elapsedTime / 60;  // mL
        flow_totalLiters += flow_rate * elapsedTime / 60;   // mL

        flow_lastTime = millis();

        // print
        if (elapsedTimeP >= (unsigned long)intervalP_ms) // every print interval
        {
            Serial.println("[Irrigation] Flow Rate: " + String(flow_rate, 2) + " L/min; Average flow rate: " + String(flow_rateAverage, 2) + " L/min; Single: " + String(flow_singleLiters, 1) + "  mL; Total: " + String(flow_totalLiters, 1) + " mL\n");
            // Serial.println("Single pulses: " + String(pulse_single));
            flow_lastPrintTime = millis();
        }

        attachInterrupt(digitalPinToInterrupt(flow_pin), countPulse, FALLING);
    }
}

// DHT22 sensor===============================================================================================
// periodically read temperature and humidity in every 2 seconds
void DHT22_readTH(float &t, float &h)
{
    h = dht.readHumidity();    // read humidity
    t = dht.readTemperature(); // read temperature in Celsius

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) // if the read values are not a number (NaN), it indicates a failure in reading the sensor
    {
        Serial.println("[Sensor] Failed to read from DHT sensor!");
        return;
    }

    // Print the results to the serial monitor
    // Serial.println("Temperature : " + String(t, 1) + "°C; Humidity : " + String(h, 1) + " %");

    // output to excel data streamer
    Serial.print("[Sensor] " + String(t) + "," + String(h) + ",");
}

// soil moisture sensor===============================================================================================
// Read and print the soil moisture sensor value
void readMoisture(float &percent)
{
    int Value = analogRead(moisture_pin);
    percent = ((float)(Value - moisture_dryValue) / (moisture_wetValue - moisture_dryValue)) * 100.0; // map the moisture value to a percentage
    percent = constrain(percent, 0.0, 100.0);
    percent = roundf(percent * 10.0) / 10.0; // limit to 1 decimal

    // Check if any reads failed and exit early (to try again).
    if (isnan(Value))
    {
        Serial.println("[Sensor] Failed to read moisture sensor!");
        return;
    }
    // Ensure the value is within the range of 0-100
    // Serial.println("Soil Moisture: " + String(percent, 1) + "%");

    // output to excel data streamer
    Serial.println(percent);
    // Serial.print("moisture value: ");
    // Serial.println(Value);
}

// soil moisture content assessment
void assessSoil(float parameter, int index)
{
    // index
    if (parameter > soilMoistureClass[index][2])
    { // wet
        soilIndex = soilIndexPool[2];
    }
    else if (parameter > soilMoistureClass[index][1])
    { // optimal
        soilIndex = soilIndexPool[1];
    }
    else
    { // dry
        soilIndex = soilIndexPool[0];
    }

    // moisture content
    switch (soilIndex)
    {
        // wet
    case '+':
        soilQuality = soilQualityPool[2];
        break;

        // optimal
    case '0':
        soilQuality = soilQualityPool[1];
        break;

        // dry
    case '-':
        soilQuality = soilQualityPool[0];
        break;
    }

    Serial.println("[Assessment] soil moisture content: " + soilQuality + "\n");
}

// timer
boolean timer(unsigned long lrt, unsigned long interval)
{
    if (millis() >= lrt + interval * 1000)
    {
        return true;
    }
    return false;
}
boolean repeatedTimer(unsigned long &lrt, unsigned long interval)
{
    if (millis() >= lrt + interval * 1000)
    {
        lrt = millis();
        return true;
    }
    return false;
}