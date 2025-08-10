#include <Arduino.h>
#include <Servo.h>
#include "helper.h"
#include "shared.h"

// serial communication===============================================================================================
// read UART

void readSerial(HardwareSerial &port, String label, String &value, String &valueTemp, volatile int &index)
{
    // Read data from UART0 (laptop)
    while (port.available())
    {
        char c = port.read();

        // Check if the character is a newline character
        if (c == '\n') // If the character is a newline character, it indicates the end of a message
        {
            value = valueTemp; // Assign the temporary value to the value string
            valueTemp = "";    // Reset the temporary value to empty after processing
            value.trim();      // Remove any leading or trailing whitespace from the value string
            index = 1;         // Reset the index to 1 after reading a complete message
            break;
        }
        else
        {
            valueTemp += c; // Append the character to the value string
        }
    }
}

// DC motor===============================================================================================
// control functions
void Move_Forward(int car_speed)
{
    digitalWrite(motorA_DIR, HIGH);
    analogWrite(motorA_PWM, car_speed);
    digitalWrite(motorB_DIR, LOW);
    analogWrite(motorB_PWM, car_speed);
}
void Move_Backward(int car_speed)
{
    digitalWrite(motorA_DIR, LOW);
    analogWrite(motorA_PWM, car_speed);
    digitalWrite(motorB_DIR, HIGH);
    analogWrite(motorB_PWM, car_speed);
}
void Rotate_Left(int car_speed)
{
    digitalWrite(motorA_DIR, LOW);
    analogWrite(motorA_PWM, car_speed);
    digitalWrite(motorB_DIR, LOW);
    analogWrite(motorB_PWM, car_speed);
}
void Rotate_Right(int car_speed)
{
    digitalWrite(motorA_DIR, HIGH);
    analogWrite(motorA_PWM, car_speed);
    digitalWrite(motorB_DIR, HIGH);
    analogWrite(motorB_PWM, car_speed);
}
void Stop()
{
    digitalWrite(motorA_DIR, LOW);
    analogWrite(motorA_PWM, 0);
    digitalWrite(motorB_DIR, HIGH);
    analogWrite(motorB_PWM, 0);
}
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
// timer
void motor_Delay(int motorStep)
{

    if (millis() >= motor_CurrentTime + motorStep) // check if the time has passed
    {
        Stop();                // stop the motors
        motor_TimerStatus = 0; // reset the timer status to 0 to indicate that the motors are stopped
    }
}

// camera servo===============================================================================================
void Camera_Tilt_Up(int anglestep)
{
    cameraServo_CurrentAngle = constrain(cameraServo_CurrentAngle + anglestep, 0, 180); // constrain the angle to be between 0 and 180 degrees
    cameraServo.write(cameraServo_CurrentAngle);                                        // move the servo to the specified angle
}
void Camera_Tilt_Down(int anglestep)
{
    cameraServo_CurrentAngle = constrain(cameraServo_CurrentAngle - anglestep, 0, 180); // constrain the angle to be between 0 and 180 degreesdd
    cameraServo.write(cameraServo_CurrentAngle);                                        // move the servo to the specified angle
}

// ultrasonic sensor===============================================================================================
// check distance from the obstacle to head of chassis, in mm
float checkDistance()
{
    digitalWrite(ultrasonic_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonic_trigPin, LOW);
    float distance = pulseIn(ultrasonic_echoPin, HIGH) / 5.80 + 7; // convert time to distance in mm
    delay(10);
    return distance;
}
// obstacle avoidance function: active brake
void activeAvoidance(float safetyClearance)
{
    if (checkDistance() <= safetyClearance)
    {
        // Serial.println("Obstacle detected! Avoiding...");
        UART_Value = ""; // reset the UART_Value to empty after processing
        Stop();
        digitalWrite(buzzerPin, HIGH); // turn on the buzzer for continuous beeping
        delay(500);
        digitalWrite(buzzerPin, LOW); // turn off the buzzer
        Move_Backward(carSpeed);
        delay(500);
        Stop();
    }
}

// buzzer===============================================================================================
// intermittent beep
void playBeep(unsigned int onTime, unsigned int offTime)
{
    // Serial.println("playBeep function called"); // print to the serial monitor for debugging
    unsigned long currentMillis = millis();

    if (beepState && currentMillis - previousMillis >= onTime) // if the buzzer is on and the time since the last beep is greater than or equal to 200 milliseconds
    {
        // Turn off the buzzer
        // Serial.println("Stopping beep!"); // print stop beep to the serial monitor
        digitalWrite(buzzerPin, LOW);
        beepState = false;
        previousMillis = currentMillis;
    }
    else if (!beepState && currentMillis - previousMillis >= offTime) // if the buzzer is off and the time since the last beep is greater than or equal to 800 milliseconds
    {
        // Turn on the buzzer
        // Serial.println("Beeping!"); // print beep to the serial monitor
        digitalWrite(buzzerPin, HIGH);
        beepState = true;
        previousMillis = currentMillis;
    }
}
// startup melody
void playMelody()
{
    // Startup melody notes (frequency in Hz)
    int melody[] = {523, 659, 784, 1046};       // C5, E5, G5, C6 (ascending tone)
    int noteDurations[] = {200, 200, 200, 400}; // Duration of each note in milliseconds

    for (int i = 0; i < int(sizeof(melody) / sizeof(melody[0])); i++)
    {
        // Play the melody
        tone(buzzerPin, melody[i], noteDurations[i]);
        delay(noteDurations[i] * 1.3); // Slight pause
        noTone(buzzerPin);
    }
}
// WiFi sounds
void playWiFi()
{
    // "Wi" syllable - short, high tone
    tone(buzzerPin, 1000); // 1000 Hz
    delay(200);
    noTone(buzzerPin);
    delay(100);

    // "Fi" syllable - longer, lower tone
    tone(buzzerPin, 800); // 800 Hz
    delay(300);
    noTone(buzzerPin);
}
// BLE sounds
void playBLE()
{
    // "B"
    tone(buzzerPin, 1000);
    delay(150);
    noTone(buzzerPin);
    delay(80);

    // "L"
    tone(buzzerPin, 1200);
    delay(150);
    noTone(buzzerPin);
    delay(80);

    // "E"
    tone(buzzerPin, 1400);
    delay(200);
    noTone(buzzerPin);
}