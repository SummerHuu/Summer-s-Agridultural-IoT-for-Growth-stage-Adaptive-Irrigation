#ifndef SHARED_H
#define SHARED_H
// the vaiables are declared here, defined in the main.cpp file, and used in other files.

// "extern" tells the compiler “this variable exists, but it's defined elsewhere.”

// uart communication===============================================================================================
extern String UART_Value;

// DC motor===============================================================================================
// control
extern const int motorA_DIR;
extern const int motorA_PWM;
extern const int motorB_DIR;
extern const int motorB_PWM;
extern const int carSpeed;
// timer
extern volatile unsigned long motor_CurrentTime;
extern volatile int motor_TimerStatus;

// camera servo===============================================================================================
extern Servo cameraServo;
extern volatile int cameraServo_CurrentAngle;

// ultrasonic sensor===============================================================================================
extern const int ultrasonic_trigPin;
extern const int ultrasonic_echoPin;

// buzzer===============================================================================================
extern const int buzzerPin;
// beep
extern volatile unsigned long beepState;      // variable to hold the state of the beep
extern volatile unsigned long previousMillis; // variable to hold the previous time in milliseconds

#endif