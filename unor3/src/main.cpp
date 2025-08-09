#include <Arduino.h> // include the Arduino library for basic functions
#include <Servo.h>   // include the Servo library for controlling servos
#include "helper.h"  // include the custom header file for helper functions
#include "shared.h"  // include the custom header file for main definitions

// "volatile" is used to tell the compiler that this variable may be changed by an interrupt service routine (ISR),
// so it should not optimize it out or cache its value. This is important for variables that are accessed both in the main program and in an ISR.

// "const" is used to define a constant value that cannot be changed

// serial communication===============================================================================================
String UART_Value = "";      // string to hold the value received from UART
String UART_ValueTemp = "";  // string to hold the temperature value received from UART
volatile int UART_Index = 0; // index for the UART_Value string
// command pool
const char UART_CommandPool[] = {'F', 'B', 'L', 'R', 'S', 'U', 'D', 'N', 'M'};                      // array of valid commands
const unsigned int UART_CommandPoolLength = sizeof(UART_CommandPool) / sizeof(UART_CommandPool[0]); // calculate the length of the UART_CommandPool array

// DC motor===============================================================================================
// pin definitions
const int motorA_DIR = 2; // LOW for backword, HIGH for forward
const int motorA_PWM = 5; // left front, left rear
const int motorB_DIR = 4; // HIGH for fordward, LOW for backword
const int motorB_PWM = 6; // right front, right rear
// motor speed
const int carSpeed = 40; // speed for the motors (0-255)
// timer
const int motor_MoveStep = 3000;          // step size for the motors, used for delay in movement
volatile unsigned long motor_CurrentTime; // variable to hold the current time in milliseconds
volatile int motor_TimerStatus = 0;

// camera Servo===============================================================================================
Servo cameraServo;                           // create servo object to control a servo
const int cameraServoPin = 10;               // pin number for the servo
volatile int cameraServo_CurrentAngle = 180; // initialize the servo angle to 180 degrees
const int cameraServer_Step = 3;             // step size for the servo movement

// ultrasonic sensor, HC-SR04===============================================================================================
// Measuing angle cone < 15°,
// frequency: 40kHz,
// operating voltage: 5V, operating current: 15mA,
// measurement range: 2cm to 400cm, resolution: 3mm, accuracy: ±3mm, response time: < 100ms,
// 2 cm to ~100 cm (beyond this, accuracy decreases),
// resolution: 0.3cm, sensing period: 100ms
// pin definitions
const int ultrasonic_trigPin = 12; // pin for the ultrasonic sensor trigger
const int ultrasonic_echoPin = 13; // pin for the ultrasonic sensor echo
// safety clearance for obstacle avoidance
const float ultrasonic_safetyClearance = 50; // safety clearance in mm for the ultrasonic sensor

// buzzer===============================================================================================
// pin definitions
const int buzzerPin = 16; // pin for the buzzer
// beep
const unsigned int buzzer_BeepOn = 200;    // time in milliseconds for the buzzer to beep
const unsigned int buzzer_BeepOff = 800;   // time in milliseconds for the buzzer to pause
const float buzzer_BeepClearance = 200;    // distance in mm for the buzzer to start beeping
volatile unsigned long beepState = 0;      // variable to hold the state of the beep
volatile unsigned long previousMillis = 0; // variable to hold the previous time in milliseconds

void setup()
{
  // put your setup code here, to run once:

  // communication===============================================================================================
  // Serial communication
  Serial.begin(115200); // initialize serial communication at 9600 bits per second
  // Serial.println("uno r3 ready");

  // motors===============================================================================================
  pinMode(motorA_DIR, OUTPUT); // set pin 5 as output
  pinMode(motorB_DIR, OUTPUT); // set pin 6 as output

  // cmaera Servo===============================================================================================
  cameraServo.attach(cameraServoPin);          // attaches the servo on pin 10 to the servo object
  cameraServo.write(cameraServo_CurrentAngle); // set the servo angle to 180 degrees

  // ultrasonic sensor===============================================================================================
  pinMode(ultrasonic_trigPin, OUTPUT); // set pin 12 as output for ultrasonic sensor trigger
  pinMode(ultrasonic_echoPin, INPUT);  // set pin 13 as input for ultrasonic sensor echo

  // buzzer===============================================================================================
  pinMode(buzzerPin, OUTPUT); // set pin 16 as output for the buzzer
  // playBeep(2000);             // beep the buzzer for 2 seconds
  playMelody(); // play the melody on the buzzer
}

void loop()
{
  // put your main code here, to run repeatedly:

  // serial communication===============================================================================================
  // check and read UART
  readSerial(Serial, "UART", UART_Value, UART_ValueTemp, UART_Index); // read the serial port and store the value in UART_Value
  // identify commands' validity
  if (UART_Index)
  {
    if (identifyCommands(UART_CommandPool, UART_CommandPoolLength, String(UART_Value).charAt(0)))
    {
      // if the valid command is a motor command (not a servo command)
      if (String(UART_Value).charAt(0) != 'U' && String(UART_Value).charAt(0) != 'D')
      {
        // start timer for the motors
        motor_CurrentTime = millis(); // get the current time in milliseconds
        motor_TimerStatus = 1;        // set the timer status to 1 to indicate that the motors are moving}

        // complete received command
        UART_Index = 0; // reset the UART_Index to 0 after processing
      }
    }
    else // if the command is not valid
    {
      UART_Value = ""; // reset the UART_Value to empty after processing
      UART_Index = 0;
    }
  }

  // command processing===============================================================================================
  if (UART_Value.length() > 0) // String.length() returns the number of characters in a String object—excluding the null terminator ('\0')
  {
    switch (String(UART_Value).charAt(0)) // check the first character of the received string
    {
      // servo commands
    case 'U': // if the first character is 'u'
      Camera_Tilt_Up(cameraServer_Step);
      break;
    case 'D': // if the first character is 'd'
      Camera_Tilt_Down(cameraServer_Step);
      break;

      // motor commands
    case 'F':                 // if the first character is 'F'
      Move_Forward(carSpeed); // move forward at the specified speed
      break;

    case 'B':                  // if the first character is 'B'
      Move_Backward(carSpeed); // move backward at the specified speed
      break;

    case 'L':                // if the first character is 'L'
      Rotate_Left(carSpeed); // rotate left at the specified speed
      break;

    case 'R':                 // if the first character is 'R'
      Rotate_Right(carSpeed); // rotate right at the specified speed
      break;

    case 'S': // if the first character is 'S'
      Stop(); // stop the motors
      break;

      // beep commands
    case 'N': // if WiFi connected
      playWiFi();
      break;

    case 'M': // if BLE connected
      playBLE();
      break;
    }
    UART_Value = ""; // reset the UART_Value to empty after processing
  }

  // motor timer
  // check and reset the motor timer
  // if (motor_TimerStatus == 1) // motor_TimerStatus is set to distinguish motor commands from servo commands
  // {
  //   motor_Delay(motor_MoveStep);
  // }

  // ultrasonic sensor===============================================================================================
  // active obstacle avoidance
  // activeAvoidance(ultrasonic_safetyClearance); // call the ultrasonic avoidance function

  // buzzer===============================================================================================
  // beep for nearby obstacles
  /*   if (checkDistance() <= buzzer_BeepClearance) // check if the distance is less than or equal to the alarm clearance and the buzzer is not already active
    {
      playBeep(buzzer_BeepOn, buzzer_BeepOff); // call the playBeep function to beep the buzzer
    } */
}