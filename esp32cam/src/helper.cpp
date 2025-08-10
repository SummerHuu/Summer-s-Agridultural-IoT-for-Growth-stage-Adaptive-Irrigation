#include <Arduino.h>
#include "WiFi.h"
#include "helper.h"

// serial communication===========================================================================================
// read UART
void readSerial(HardwareSerial &port, String label, String &value, String &valueTemp, volatile int &index)
{
    // Read data from UART
    while (port.available())
    {
        char c = port.read();

        // Check if the character is a newline character
        if (c == '\n') // If the character is a newline character, it indicates the end of a message
        {
            value = valueTemp; // Assign the temporary value to the value string
            valueTemp = "";    // Reset the temporary value to empty after processing
            value.trim();      // Remove any leading or trailing whitespace from the value string
            index = 1;         // indicate that a new message has been received

            // print on UART0
            if (value.length() > 0) // Check if the value string is not empty
            {
                Serial.print("[" + label + "]: ");
                Serial.println(value);
            }
            else
            {
                Serial.println("No data received from laptop"); // Print a message if no data was received
            }
            break;
        }
        else
        {
            valueTemp += c; // Append the character to the value string
        }
    }
}
// send UART
void sendSerial(HardwareSerial &port, char value)
{
    port.println(value);
}

// WiFi================================================================================================================
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

// identify command
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
// choose command
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


