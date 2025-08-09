#include <Arduino.h>
#include "helper.h"
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <time.h>
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/viz/image_collection.h>
#include <Growth_stages_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// serial communication ========================================================================================================
// uart0 is connected to the laptop
String UART0_value = "";
String UART0_valueTemp = "";
volatile int UART0_index = 0; // Index to track the state of UART0 messages
// UART2
// pin configuration
const int UART2_RX_PIN = 15; // GPIO 16 for UART2 RX
const int UART2_TX_PIN = 13; // GPIO 17 for UART2 TX
// serial data
String UART2_value = "";
String UART2_valueTemp = "";
volatile int UART2_index = 0; // Index to track the state of UART2 messages
// command pool
const char UART_commandPool[] = {'F', 'B', 'L', 'R', 'S', 'U', 'D'};                                // array of valid commands
const unsigned int UART_commandPoolLength = sizeof(UART_commandPool) / sizeof(UART_commandPool[0]); // calculate the length of the UART_CommandPool array

// WiFi - SAT ================================================================================================================
// WiFi credentials
const char WiFi_SSID[] = "ZhouHaoRan_2.4G";
const char WiFi_Password[] = "Z20242024";
// timer for WiFi reconnect
unsigned long lastWiFiAttempt = 0;
const unsigned long retryInterval = 60; // in seconds
void connectToWiFi()
{
  Serial.println("\n[WiFi] Attempting to connect...");
  WiFi.disconnect();                    // Disconnect from any previous connection
  delay(100);                           // Small delay to ensure disconnection
  WiFi.mode(WIFI_STA);                  // Station mode (client)
  WiFi.begin(WiFi_SSID, WiFi_Password); // Connect to the specified WiFi network
}

// led and flash lamp ========================================================================================================
const int LED1 = 33;
const int flash_lamp = 4;
volatile int flash_status = 0;

// PWM
// PWM configuration
const int pwm_channel = 7;     // PWM channel 0
const int pwm_freqency = 5000; // PWM frequency (5kHz)
const int pwm_resolution = 8;  // PWM resolution (8 bits, 0-255)
const int pwm_duty = 128;      // setting bright of flash lamp 0-255

// growing stage assessment ==============================================================================================
char growingIndex = 'X';
const char growingIndexPool[] = {'0', '1', '2', '3', '4', '5', '6'};
String growingStage = "Unknown";
const String growingStagePool[] = {"Germination", "Seedling", "Vegetative", "Flowering", "Fruting", "Maturation", "Senescence"};

// timestamp
// Set timezone and NTP server
const char *ntpServer = "time.google.com";
const long gmtOffset_sec = 8 * 3600; // +8 for Malaysia/China time
const int daylightOffset_sec = 0;    // No DST
// timestamp for firebase
void getLocalTime(FirebaseJson &Fjson)
{
  // Get local time (already configured in setup with GMT+8)
  struct tm timeinfo;
  if (getLocalTime(&timeinfo))
  {
    char timeString[30];
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo); // Local format
    Fjson.set("timestamp", timeString);
    // Serial.println(timeString);
  }
  else
  {
    Fjson.set("timestamp", "N/A");
  }
}

// Firebase =================================================================================================================
// credentials
#define API_KEY "AIzaSyB8pqm5yKESLbNxbU4A11xSoLve2nCnH54"
#define DATABASE_URL "https://summer-s-agricultural-iot-default-rtdb.asia-southeast1.firebasedatabase.app/"
// Firebase auth accounts — use a dedicated email for ESP32-CAM
#define USER_EMAIL "esp32-cam@gmail.com"
#define USER_PASSWORD "123456"
// Firebase objects
FirebaseData fbdo;        // For normal get/set, handles writes/reads (e.g. uploading sensor data)
FirebaseData streamRobot; // listen to real-time changes of a path under Robot
FirebaseAuth auth;
FirebaseConfig config;
// timer
volatile unsigned long firebase_lastSendTime = 0;
const unsigned long interval = 60; // Interval in seconds
// push to firebase
// sends all data to /Sensor in one update, reducing bandwidth and Firebase operations.
bool sendFirebase(String node, String path, String stage)
{
  if (Firebase.ready() && auth.token.uid != "")
  {
    FirebaseJson json;

    json.set(path, stage);

    getLocalTime(json);

    if (Firebase.RTDB.updateNode(&fbdo, node, &json))
    {
      Serial.println("[Firebase] sent");
      return true;
    }
    else
    {
      Serial.printf("[Firebase] Failed to send: %s\n", fbdo.errorReason().c_str());
      return false;
    }
  }
  // This ensures all control paths return a value
  return false;
}
// Web command
String Web_value = "";
volatile int Web_index = 0; // Index to track the state of UART0 messages
// web command pool
const char commandPool[] = {'F', 'B', 'L', 'R', 'S', 'U', 'D', 'P', 'V', '0', '1', '2', '3', '4', '5', '6'}; // array of valid commands
const unsigned int commandPoolLength = sizeof(commandPool) / sizeof(commandPool[0]);                         // calculate the length of the array
// Callback function for pump command changes
void pumpStreamCallback(FirebaseStream data)
{
  if (data.dataPath() == "/command")
  {
    Web_value = data.stringData();
    Web_index = 1;
    Serial.printf("[Web] pump: %s\n", Web_value);
  }
}
// Callback function for robot command changes
void robotStreamCallback(FirebaseStream data)
{
  if (data.dataPath() == "/direction")
  {
    Web_value = data.stringData();
    Web_index = 1;
    Serial.printf("[Web] direction: %s\n", Web_value);
  }
  if (data.dataPath() == "/tilt")
  {
    Web_value = data.stringData();
    Web_index = 1;
    Serial.printf("[Web] tilt: %s\n", Web_value);
  }
}
void streamTimeoutCallback(bool timeout)
{
  if (timeout)
    Serial.println("[Firebase] Stream timeout, resuming...");
}

// Camera =========================================================================================================
using eloq::camera;
using eloq::viz::collectionServer;

void setup()
{
  // put your setup code here, to run once:

  // serial communication=========================================================================================================
  // UART0 configuration
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  Serial.println("\nesp32-cam is initializing");
  Serial.printf("[After Serial] Heap: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());
  // UART2 configuration
  Serial2.begin(115200, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN); // Initialize UART2 with 115200 baud rate, 8 data bits, no parity, 1 stop bit

  // WiFi=======================================================================================================================
  // WiFi client (SAT mode): connect to a WiFi network
  connectToWiFi(); // Initial attempt
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\n[WiFi] Connected to Wi-Fi");
  Serial.printf("[After WiFi] Heap: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());

  // led and flash lamp===========================================================================================================
  pinMode(LED1, OUTPUT);       // Set GPIO 33 as output for the built-in LED
  pinMode(flash_lamp, OUTPUT); // Set GPIO 4 as output for the flash lamp
  // pwm configuration for flash lamp
  ledcSetup(pwm_channel, pwm_freqency, pwm_resolution); // Set up LEDC channel 7 with a frequency of 5000 Hz and 8-bit
  ledcAttachPin(flash_lamp, pwm_channel);               // Attach GPIO 4 to LEDC channel 7 for flash lamp control
  // blink led and lamp
  ledcWrite(pwm_channel, pwm_duty); // Set the initial duty cycle of the flash lamp to 0 (off)
  digitalWrite(LED1, HIGH);
  delay(500);
  ledcWrite(pwm_channel, 0); // Set the initial duty cycle of the flash lamp to 0 (off)
  digitalWrite(LED1, LOW);

  // timestamp
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Firebase=========================================================================================================
  // configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  // Use Email/Password login instead of anonymous
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  // Sign up anonymously
  /* if (Firebase.signUp(&config, &auth, "", ""))
  {
    Serial.println("[Firebase] signUp OK");
  }
  else
  {
    Serial.println("[Firebase] filled signUp");
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  } */
  // start
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true); // automatically reconnect to firebase after disconnection
  Serial.print("[Firebase] ready?: ");
  Serial.println(Firebase.ready() ? "Yes" : "No");
  if (auth.token.uid != "")
  {
    Serial.printf("[Firebase] Logged in as: %s\n", auth.token.uid.c_str());
  }
  else
  {
    Serial.println("[Firebase] Login failed");
  }
  Serial.printf("[After Firebase] Heap: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());
  // listen on variable changes: update once variable listened on firebase changes
  // Robot
  if (!Firebase.RTDB.beginStream(&streamRobot, "/Robot"))
  {
    Serial.printf("[Firebase] Stream begin error: %s\n", streamRobot.errorReason().c_str());
  }
  Firebase.RTDB.setStreamCallback(&streamRobot, robotStreamCallback, streamTimeoutCallback);
  // initialize
  sendFirebase("/Pump", "command", "");
  sendFirebase("/Camera", "growth index", "");

  // Camera =========================================================================================================
  // camera settings
  // replace with your own model!
  camera.pinout.aithinker();
  camera.brownout.disable();
  // Edge Impulse models work on square images
  camera.resolution.face(); // face resolution is 240x240
  camera.quality.best();
  // init camera
  while (!camera.begin().isOk())
    Serial.println(camera.exception.toString());
  camera.sensor.hmirror(); // horizontal mirror
  Serial.println("[Camera] ready");
  Serial.printf("[After Camera] Heap: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());
  while (!collectionServer.begin().isOk())
    Serial.println(collectionServer.exception.toString());
  Serial.println(collectionServer.address());

  Serial.println("\nesp32-cam ready\n"); // Print a message to the serial monitor
}

// put your main code here, to run repeatedly:
void loop()
{
  // WiFi =================================================================================================================
  // Periodically check and reconnect if disconnected
  static bool wasConnected = false;
  if (WiFi.status() != WL_CONNECTED)
  {
    if (millis() - lastWiFiAttempt > retryInterval * 1000)
    {
      lastWiFiAttempt = millis();
      Serial.print("\n[WiFi] WiFi status: ");
      Serial.println(String(getWiFiStatus(WiFi.status())));

      // Only print RSSI if it makes sense
      if (WiFi.status() == WL_CONNECTED || WiFi.RSSI() != 0)
      {
        Serial.printf("RSSI: %s\n", String(WiFi.RSSI()));
      }

      connectToWiFi();
    }
    wasConnected = false; // Reset flag if disconnected
  }
  else
  {
    // print WiFi info if newly connected
    if (!wasConnected)
    {

      Serial.print("[WiFi] Connected to WiFi! SSID: ");
      Serial.print(WiFi.SSID());
      Serial.print(", IP Address: ");
      Serial.print(WiFi.localIP());
      Serial.println("\n");
      wasConnected = true;

      sendSerial(Serial2, 'N');

      while (!sendFirebase("/Camera", "ip", WiFi.localIP().toString())) // Set the IP address in Firebase
      {
        delay(1000); // Wait for a second to ensure the data is sent
      }

      // timestamp
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    }
  }

  // serial communication ===============================================================================================
  // check and read UART
  readSerial(Serial, "UART0", UART0_value, UART0_valueTemp, UART0_index); // Read data from UART0, the laptop

  // idtentify commands's validity  ============================================================================================================
  // UART0
  filterCommands(UART0_index, UART0_value, commandPool, commandPoolLength);
  // Web
  filterCommands(Web_index, Web_value, commandPool, commandPoolLength);

  // commands distribution and execution ===============================================================================================
  if (UART0_value.length() > 0 || Web_value.length() > 0)
  {
    String choosenCommand = chooseCommands(UART0_value, Web_value);
    // Serial.println("choosen command: " + choosenCommand.charAt(0));

    // send UART2
    if (identifyCommands(UART_commandPool, UART_commandPoolLength, choosenCommand.charAt(0)))
    {
      sendSerial(Serial2, choosenCommand.charAt(0));
    }

    switch (choosenCommand.charAt(0))
    {
    // flash lamp
    case 'P':
      if (flash_status == 0)
      {
        ledcWrite(pwm_channel, pwm_duty);
        flash_status = 1;
      }
      else
      {
        ledcWrite(pwm_channel, 0);
        flash_status = 0;
      }
      break;

    // image recognition
    case 'V':

      break;

    // growing stage assessment
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
      growingIndex = choosenCommand.charAt(0);
      // problem: esp32 can't respond if write to "/Pump/command" directly
      // solution: write to "/Camera/growingIndex" and copy it to "/Pump/command" by .js
      sendFirebase("/Camera", "growth index", String(growingIndex));
    }

    // clear after processing commands
    choosenCommand = "";
    UART0_value = "";
    Web_value = "";
  }

  // firebase ============================================================================================================
  // push data to firebase only when growing stage changes
  /*   static volatile char lastgrowingIndex = 'X';
    static bool initialized = false;
    if (growingIndex != lastgrowingIndex || !initialized)
    {
      // growing stage assessment
      growingStage = initialized ? growingStagePool[growingIndex - '0'] : "To be detected"; // ASCII codes '3' - '0' = 51 - 48 = 3
      sendFirebase("/Camera", "growing stage", growingStage);                               // push to firebase

      initialized = true;

      lastgrowingIndex = growingIndex;
    } */

  // Check memory usage every minute
  /*   static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 60000)
    {
      lastPrint = millis();
      Serial.printf("[Heap] Free: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());
    } */
}
