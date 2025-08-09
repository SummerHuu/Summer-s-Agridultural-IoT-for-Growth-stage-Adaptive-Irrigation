#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <time.h>
#include <ThingSpeak.h>
#include "helper.h"
#include "shared.h"

// uart communication========================================================================================================
// uart0 is connected to the laptop
String UART0_value = "";
String UART0_valueTemp = "";
int UART0_index = 0; // Index to track the state of UART0 messages

// WiFi communication=======================================================================================================
// WiFi credentials
const char WiFi_SSID[] = "ZhouHaoRan_2.4G";
const char WiFi_Password[] = "Z20242024";
// timer for WiFi reconnect
unsigned long lastWiFiAttempt = 0;
unsigned long retryInterval = 60;
void connectToWiFi()
{
  Serial.println("\n[WiFi] Attempting to connect...");
  WiFi.disconnect();                    // Disconnect from any previous connection
  delay(100);                           // Small delay to ensure disconnection
  WiFi.mode(WIFI_STA);                  // Station mode (client)
  WiFi.begin(WiFi_SSID, WiFi_Password); // Connect to the specified WiFi network
}

// water pump===============================================================================================================
// pin configuration
const int pump_pin = 33; // Pin connected to the water pump
bool pump_status = 0;
// command pool
const char commandPool[] = {'W', 'C', 'A', 'R', '0', '1', '2', '3', '4', '5', '6', '#'}; // array of valid commands
const unsigned int commandPoolLength = sizeof(commandPool) / sizeof(commandPool[0]);
const unsigned long command_interval = 1200; // in seconds
unsigned long command_lastReadTime = 0;
bool command_status = 1;
// timer
unsigned long pump_step = 0;        // time in milliseconds for the pump to move
unsigned long pump_currentTime = 0; // variable to hold the current time in milliseconds
bool pump_timerStatus = 0;
float pump_speed = 0.56; // tested water flow speed for water pump, L/min
// quota
bool pump_quotaStatus = 0;
float pump_quota = 0;                                       // mL
float pump_quotaCompensate = 15;                            // mL, compensate for consumption in pipe
float pump_quotaPool[] = {30, 30, 100, 100, 100, 100, 100}; // mL

// water flow meter YF-S401===============================================================================================
// Pulse Frequency: f = 98 * Q(L/min) ±2%.
// pulse duty cycle: 50% ±10%
// range: 0.3 ~ 6L/min ±10 %
// pin configuration
const int flow_pin = 32; // Pin connected to the flow meter
// pulse
const unsigned int flow_ratio = 45;         // the conversion ratio is obtained by statistics
volatile unsigned long flow_pulseCount = 0; // Variable to hold the pulse count from the flow meter
volatile unsigned long pulse_single = 0;
// calculation
float flow_rate = 0.0;         // L/min
float flow_rateAverage = 0.0;  // L/min
float flow_singleLiters = 0.0; // mL
float flow_totalLiters = 0.0;  // mL
int irrigationCounter = 0;
unsigned long flow_singleTimer = 0;
// calculation timer
const float flow_interval = 0.5; // Interval for reading the flow meter in seconds, >=0.001
unsigned long flow_lastTime = 0;
// print timer
const float flow_printInterval = 2; // Interval for reading the flow meter in seconds, >=0.001
unsigned long flow_lastPrintTime = 0;

// HDT22, Temperature and Humidity Sensor ===================================================================================
// operating voltage: 3.3V to 6V, operating current: 2.5mA,
// measurement range: 0-100% RH, -40 to +80°C
// accuracy: ±2% RH, ±0.5°C, response time: < 2 seconds
// resolution: 0.1% RH, 0.1°C,
// frequency: 0.5 HZ or 2 seconds
// pin configuration
const int DHT_PIN = 18;     // pin for the DHT22 sensor
const int DHT_TYPE = DHT22; // type of the DHT sensor
DHT dht(DHT_PIN, DHT_TYPE); // create an instance of the DHT class
// reading
float DHT22_temperature = 0.0; //
float DHT22_humidity = 0.0;
// timer
const unsigned long interval = 60; // Interval in seconds
unsigned long lastReadTime = 0;    // Last time the soil moisture sensor was read

// capacitive soil moisture sensor v1.2 ====================================================================================
// pin configuration
const int moisture_pin = 35; // Pin connected to the capacitive soil moisture sensor
// reading
float moisture_percent = 0.0;
int ADC_resolution = 10; // Set the ADC resolution to 10 bits (0-1023)
// field capacity calibration
const int moisture_dryValue = 650; // 0% field capacity
const int moisture_wetValue = 290; // 100% field capacity

// optimal soil moisture range
// const float optimalMoisture[] = {70, 60, 55, 65, 60, 50, 50};

// soil moisture content assessment ==============================================================================================
const float soilMoistureClass[7][3] = {{0, 60, 80}, {0, 50, 70}, {0, 50, 60}, {0, 60, 70}, {0, 60, 70}, {0, 50, 60}, {0, 50, 60}};
char soilIndex = 'X';
const char soilIndexPool[] = {'-', '0', '+'};
String soilQuality = "Unknown";
const String soilQualityPool[] = {"Dry", "Optimal", "Wet"};

// growing stage assessment ==============================================================================================
char growingIndex = '0';
const char growingIndexPool[] = {'0', '1', '2', '3', '4', '5', '6'};
String growingStage = "Unknown";
const String growingStagePool[] = {"Germination", "Seedling", "Vegetative", "Flowering", "Fruting", "Maturation", "Senescence"};
bool firstGrowingStage = false; // Flag to indicate if the first growing stage has been set

// decision =============================================================================================================
bool decisionIndex = false;
String decision = "Unknown";
bool automationIndex = true;

// timestamp =========================================================================================================
// Set timezone and NTP server
const char *ntpServer1 = "time.google.com";
const char *ntpServer2 = "pool.ntp.org";
const char *ntpServer3 = "sg.pool.ntp.org";
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
#define USER_EMAIL "esp32@gmail.com"
#define USER_PASSWORD "123456"
// Firebase objects
FirebaseData fbdo;       // For normal get/set, handles writes/reads (e.g. uploading sensor data)
FirebaseData streamPump; // listen to real-time changes of a path under Pump
FirebaseAuth auth;
FirebaseConfig config;
// push to firebase
// send Decision
bool sendDecision(String decision, bool automatic)
{
  if (Firebase.ready() && auth.token.uid != "")
  {
    FirebaseJson json;

    json.set("decision", decision);
    json.set("automation", automatic);

    getLocalTime(json);

    if (Firebase.RTDB.updateNode(&fbdo, "/Decision", &json))
    {
      Serial.println("[Firebase] Decision sent");
      return true;
    }
    else
    {
      Serial.printf("[Firebase] Failed to send Decision: %s\n", fbdo.errorReason().c_str());
      return false;
    }
  }
  else
  {
    Serial.println("[Firebase] not ready yet...");
    return false;
  }
}
// send a reminder once
bool sendReminder(String node, String path, String reminder)
{
  if (Firebase.ready() && auth.token.uid != "")
  {
    FirebaseJson cameraJson, sensorsJson;

    // Prepare data
    cameraJson.set(path, reminder);

    getLocalTime(cameraJson); // Add timestamp or similar if needed

    if (Firebase.RTDB.updateNode(&fbdo, node, &cameraJson))
    {
      // Serial.println("[Firebase] Sent reminder");
      return true;
    }
    else
    {
      Serial.printf("[Firebase] Failed to send reminder: %s\n", fbdo.errorReason().c_str());
      return false;
    }
  }
  else
  {
    Serial.println("[Firebase] Not ready yet...");
    return false;
  }
}
// sends Sensors: all data to /Sensor in one update, reducing bandwidth and Firebase operations.
bool sendSensorData(float temp, float hum, float moisture, String quality)
{
  if (Firebase.ready() && auth.token.uid != "")
  {
    FirebaseJson json;

    json.set("temperature", String(temp, 1));
    json.set("humidity", String(hum, 1));
    json.set("moisture", String(moisture, 1));
    json.set("soil quality", quality);

    getLocalTime(json);

    if (Firebase.RTDB.updateNode(&fbdo, "/Sensors", &json))
    {
      // Serial.println("[Firebase] Sensors sent");
      return true;
    }
    else
    {
      Serial.printf("[Firebase] Failed to send Sensors: %s\n", fbdo.errorReason().c_str());
      return false;
    }
  }
  else
  {
    Serial.println("[Firebase] not ready yet...");
    return false;
  }
}
// send Flowmeter
bool sendFlowData(float rate, float single, float total, unsigned int times)
{
  if (Firebase.ready() && auth.token.uid != "")
  {
    FirebaseJson json;

    json.set("flow rate", String(rate, 2));
    json.set("single millilitter", String(single, 1));
    json.set("total millilitter", String(total, 1));
    json.set("irrigation times", String(times));

    getLocalTime(json);

    if (Firebase.RTDB.updateNode(&fbdo, "/Flowmeter", &json))
    {
      // Serial.println("[Firebase] Flowmeter sent");
      return true;
    }
    else
    {
      Serial.printf("[Firebase] Failed to send Flowmeter: %s\n", fbdo.errorReason().c_str());
      return false;
    }
  }
  else
  {
    Serial.println("[Firebase] not ready yet...");
    return false;
  }
}
// listen on Firebase
String Web_value = "";
volatile int Web_index = 0; // Index to track the state of UART0 messages
// Callback function for pump command changes
void pumpStreamCallback(FirebaseStream data)
{
  if (data.dataType() == "string")
  {
    Web_value = data.stringData();
    Web_index = 1;
    Serial.printf("[Web] pump: %s\n", Web_value);
  }
}
void streamTimeoutCallback(bool timeout)
{
  if (timeout)
    Serial.println("[Firebase] Stream timeout, resuming...");
}

// Thingspeak ===========================================================================================================
// configuration
const char *host = "api.thingspeak.com";
const int httpPort = 80;
const unsigned long channelID = 2995057;
const String writeApiKey = "RUXD834FQ27MJZWL";
WiFiClient client;
// Set ThingSpeak fields
void sendThingspeak()
{
  // growing stage assessment

  ThingSpeak.setField(1, DHT22_temperature);
  ThingSpeak.setField(2, DHT22_humidity);
  ThingSpeak.setField(3, moisture_percent);
  ThingSpeak.setField(4, irrigationCounter);
  ThingSpeak.setField(5, flow_totalLiters);
  ThingSpeak.setField(6, soilQuality);
  ThingSpeak.setField(7, growingStage);
  ThingSpeak.setField(8, decision);

  // Send all data in one entry
  int x = ThingSpeak.writeFields(channelID, writeApiKey.c_str());

  if (x == 200)
  {
    // Serial.println("[Thingspeak] Data sent");
  }
  else
  {
    Serial.print("[Thingspeak] Failed to send data. HTTP error code: ");
    Serial.println(x);
  }
}

void setup()
{
  // UART communication=========================================================================================================
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  Serial.println("NodeMCU-esp32 is initializing\n");
  Serial.printf("[After Serial] Heap: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());

  // WiFi communication=====================================================================================================
  // WiFi client (SAT mode): connect to a WiFi network
  connectToWiFi(); // Initial attempt
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\n[WiFi] Connected to Wi-Fi");
  Serial.printf("[After WiFi] Heap: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());

  // water pump===============================================================================================
  // pin configuration
  pinMode(pump_pin, OUTPUT); // Set the pump pin as an output
  // PWM configuration
  // ledcSetup(pwmChannel, pwmFreqency, pwmResolution); // Set up the PWM channel with the specified frequency and resolution
  // ledcAttachPin(pump_pin, pwmChannel);               // Attach the pump pin to the PWM channel

  // water flow meter YF-S401===============================================================================================
  pinMode(flow_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flow_pin), countPulse, FALLING); // Attach an interrupt to the flow meter pin to count pulses on the falling edge

  // DHT22 sensor=============================================================================================
  dht.begin(); // initialize the DHT22 sensor

  // soil moisture sensor=====================================================================================
  analogReadResolution(ADC_resolution); // Set the ADC resolution to 10 bits (0-1023)

  // timestamp
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3); // Configure the time zone and NTP servers
  // Wait for NTP to sync
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo))
  {
    Serial.println("[NTP] Waiting for time sync...");
    delay(500);
  }
  Serial.printf("[NTP] Time synced: %s\n", asctime(&timeinfo));

  // Firebase =========================================================================================================
  // configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  // Use Email/Password login instead of anonymous
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  // Sign up anonymously
  /*   if (Firebase.signUp(&config, &auth, "", ""))
    {
      Serial.println("[Firebase] signUp OK");
    }
    else
    {
      Serial.println("[Firebase] filled signUp");
      Serial.printf("%s\n", config.signer.signupError.message.c_str());
    } */
  // Set BearSSL buffers
  fbdo.setBSSLBufferSize(2048 /*rx*/, 1024 /*tx*/); // optional: you can reduce if needed
  fbdo.setResponseSize(2048);                       // ensures enough response buffer
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
  // initialize
  sendReminder("/Pump", "command", "");
  sendReminder("/Camera", "growing stage", "To be detected");
  // listen on variable changes: update once variable listened on firebase changes
  // Pump
  if (!Firebase.RTDB.beginStream(&streamPump, "/Pump/command"))
  {
    Serial.printf("[Firebase] Stream begin error: %s\n", streamPump.errorReason().c_str());
  }
  else
  {
    Serial.println("[Firebase] Stream started for /Pump/command");
  }
  Firebase.RTDB.setStreamCallback(&streamPump, pumpStreamCallback, streamTimeoutCallback);

  // Thingspeak ===========================================================================================================
  ThingSpeak.begin(client);
  Serial.printf("[After Thingspeak] Heap: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());

  Serial.println("NodeMCU-esp32 ready\n"); // Print a message to the serial monitor
}

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
      Serial.print("[WiFi] WiFi status: ");
      Serial.println(String(getWiFiStatus(WiFi.status())));

      // Only print RSSI if it makes sense
      if (WiFi.status() == WL_CONNECTED || WiFi.RSSI() != 0)
      {
        Serial.printf("RSSI: %s", String(WiFi.RSSI()));
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

      // timetamp
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3); // Configure the time zone and NTP servers
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

  // command processing  ============================================================================================================
  // growing stage assessment ===============================================================================================
  if (UART0_value.length() > 0 || Web_value.length() > 0) // String.length() returns the number of characters in a String object—excluding the null terminator ('\0')
  {
    String choosenCommand = chooseCommands(UART0_value, Web_value);
    // Serial.println("choosenCommand: " + choosenCommand);

    switch (choosenCommand.charAt(0)) // check the first character of the received string
    {
      // pump command
    case 'W':             // counted in quota
      if (command_status) // command is enabled
      {
        if (pump_status == 0)
        {
          // ledcWrite(pwmChannel, pwm_duty); // start the pump by setting the duty cycle
          digitalWrite(pump_pin, 1); // start the pump
          // flowmeter
          flow_singleLiters = 0.0;
          pulse_single = 0;
          flow_singleTimer = millis();
          irrigationCounter++;

          // swith on-off
          pump_status = 1;

          // timer start
          pump_timerStatus = 1;
          Serial.println("\n[Command] watering counted in timer start\n");
          pump_step = (unsigned long)((pump_quota + pump_quotaCompensate) / (pump_speed * 1000 / 60000)); // convert quota to timer in milliseconds, compensate for consumption in pipe
          // Serial.printf("\n[Pump] Timer: %ld ms\n", pump_step);
          pump_currentTime = millis(); // update the current time in milliseconds

          // quota start
          /* pump_quotaStatus = 1;
          Serial.println("\n[Command] watering counted in quota start"); */
        }
        else
        {
          // quota stop
          digitalWrite(pump_pin, 0); // stop the pum
          // swith on-off
          pump_status = 0;

          // timer end
          pump_timerStatus = 0;

          // quota end
          // pump_quotaStatus = 0;

          // disable receiving command after watering
          command_status = 0;
          command_lastReadTime = millis();

          Serial.printf("\n[Command] Average flow rate: %.2f L/min; Single: %.1f mL; Irrigation counter: %d; Total: %.1f mL\n", flow_rateAverage, flow_singleLiters, irrigationCounter, flow_totalLiters);
          Serial.println("[Command] End");
          Serial.printf("[Command] Disabled for %ld s\n", command_interval);
          sendReminder("/Sensors", "soil quality", "To be detected");
        }
      }
      else
      {
        Serial.println("\n[Command] Disabled");
      }
      break;

    case 'C':
      flow_totalLiters = 0.0;
      irrigationCounter = 0;
      Serial.println("\n[Command] Clear total\n");
      // push flow data to firebase
      sendFlowData(flow_rate, flow_singleLiters, flow_totalLiters, irrigationCounter);
      sendThingspeak();
      break;

    // automatic or manual decision making
    case 'A':
      // swith auotomatic-manual
      automationIndex = automationIndex ? false : true;
      Serial.printf("\n[Command] Automatic decision: %s\n", automationIndex ? "ON" : "OFF");

      sendDecision(decision, automationIndex);
      sendThingspeak();
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
      growingStage = growingStagePool[growingIndex - '0']; // ASCII codes '3' - '0' = 51 - 48 = 3
      sendReminder("/Camera", "growing stage", growingStage);
      Serial.printf("\n[Command] Set growing stage: %s\n", growingStage);
      // update firebase
      if (soilIndex == '-')
      {
        decision = String("To water ") + pump_quotaPool[growingIndex - '0'];
        sendDecision(decision, automationIndex);
      }
      firstGrowingStage = true; // Set the flag to true after the first growing stage is set
      break;

    // manually set quota
    case '#':
      pump_quota = choosenCommand.substring(1).toFloat(); // skips the first character (#) and converts the remaining string to float
      Serial.printf("\n[Command] set quota: %.0f mL\n", pump_quota);
      break;

      // reset command_status
    case 'R':
      command_status = 1;
      // serial
      Serial.println("\n[Command] Ready\n");
      // firebase
      decision = "Enabled & Detecting";
      sendDecision(decision, automationIndex);
      break;

    default:
      Serial.println("\n[Command] Invalid");
      break;
    }

    // reset the value to empty after processing
    choosenCommand = "";
    UART0_value = "";
    Web_value = "";
  }

  // command
  // disable command for even dispersion of watering =============================================================================================
  // push flow data to firebase =============================================================================================
  // irrigation decision making: automatic or manual =============================================================================================
  static bool alreadyTriggered = false;
  if (!command_status) // command is disabled
  {
    if (timer(command_lastReadTime, command_interval))
    {
      UART0_value = "R"; // reset command_status
    }

    // push flow data to firebase once after irrigation
    if (!alreadyTriggered)
    {
      alreadyTriggered = sendFlowData(flow_rate, flow_singleLiters, flow_totalLiters, irrigationCounter) ? true : false;
      decision = "Executed & Disabled";
      sendDecision(decision, automationIndex);
      sendThingspeak();
    }
  }
  else // command is enabled
  {
    alreadyTriggered = false;

    // irrigation decision making
    static char lastSoilIndex = ' ';
    switch (soilIndex)
    {
    case '-':
      lastSoilIndex = soilIndex;
      // automatic decision
      decisionIndex = automationIndex ? true : false;
      // exectued once
      if (decisionIndex && firstGrowingStage)
      {
        // set quota according to growing stage
        pump_quota = pump_quotaPool[growingIndex - '0']; // ASCII codes '3' - '0' = 51 - 48 = 3
        Serial.printf("\nset quota and watering automatically: %.0f mL\n", pump_quota);

        // activate watering
        UART0_value = "W";

        decision = "Watering";
        sendDecision(decision, automationIndex); // push Decision
        sendThingspeak();                        // push growing stage

        soilIndex = 'X'; // reset soilIndex to avoid repeating command 'W'
        decisionIndex = false;
      }
      break;

    case '0': // soil "Optimal"
    case '+': // soil "Wet"
      decision = "No action";
      sendDecision(decision, automationIndex);
      soilIndex = 'X'; // reset soilIndex
      break;
    }
  }

  // pump ===============================================================================================================
  // water flow meter YF-S401 ===========================================================================================
  if (pump_timerStatus || pump_quotaStatus) // pump_timerStatus is set to distinguish pump commands from servo commands
  {

    // water flow meter YF-S401
    readFlow();

    // timer
    if (pump_timerStatus)
    {
      if (pump_Delay(pump_currentTime, pump_step, pump_timerStatus))
      {
        sendReminder("/Sensors", "soil quality", "To be detected");
      };
    }

    // quota
    /*     if (pump_quotaStatus)
        {
          pump_count(pump_quota, flow_singleLiters, pump_quotaStatus);
        } */
  }

  // read sensor ==============================================================================================================
  // soil moisture content assessment ==============================================================================================
  // firebase ============================================================================================================
  // Thingspeak ============================================================================================================
  if (!pump_timerStatus && !pump_quotaStatus)
  {
    static float lastQuota = 0;
    if (pump_quota != lastQuota)
    {
      lastQuota = pump_quota;                                // update lastQuota only if it has changed
      sendReminder("/Pump", "quota", String(pump_quota, 0)); // refresh quota in firebase
    }

    static bool initialized = false;
    if (repeatedTimer(lastReadTime, interval) || !initialized)
    {

      // DHT22
      DHT22_readTH(DHT22_temperature, DHT22_humidity);

      // soil moisture sensor
      readMoisture(moisture_percent);

      // soil moisture content assessment
      assessSoil(moisture_percent, growingIndex - '0'); // Pass the index of the soil type based on the growing stage

      // Firebase: periodically push sensor data
      sendSensorData(DHT22_temperature, DHT22_humidity, moisture_percent, soilQuality);
      if (soilIndex == '-')
      {
        decision = firstGrowingStage ? String("To water ") + pump_quotaPool[growingIndex - '0'] : "To be detected";
        sendDecision(decision, automationIndex);
      }

      // initialize data to firebase only once
      if (!initialized)
      {
        sendFlowData(flow_rate, flow_singleLiters, flow_totalLiters, irrigationCounter);
        sendReminder("/Pump", "quota", String(pump_quota, 0));
        sendReminder("/Decision", "decision", "To be detected");
        decision = "To be detected";
        sendDecision(decision, automationIndex);
      }

      initialized = true;

      // Thingspeak
      sendThingspeak(); // push sensor data
    }
  }

  // Check memory usage
  /*   static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 60000)
    {
      lastPrint = millis();
      Serial.printf("[Heap] Free: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());
    } */
}
