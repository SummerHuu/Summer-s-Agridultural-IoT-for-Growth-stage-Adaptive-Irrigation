#ifndef HELPER_H
#define HELPER_H
// the functions are declared here, defined in the helper.cpp file, and used in other files.

// serial communication===============================================================================================
// read UART
void readSerial(HardwareSerial &port, String label, String &value, String &valueTemp, volatile int &index);

// DC motor===============================================================================================
// control functions
void Move_Forward(int car_speed);
void Move_Backward(int car_speed);
void Rotate_Left(int car_speed);
void Rotate_Right(int car_speed);
void Stop();
// command identifier
bool identifyCommands(const char myArray[], const unsigned int length, char value);
// timer
void motor_Delay(int motorstep);

// camera servo===============================================================================================
void Camera_Tilt_Up(int anglestep);
void Camera_Tilt_Down(int anglestep);

// ultrasonic sensor===============================================================================================
float checkDistance();
void activeAvoidance(float safetyClearance);

// buzzer===============================================================================================
void playBeep(unsigned int onTime, unsigned int offTime);
void playMelody();
void playWiFi();
void playBLE();

#endif