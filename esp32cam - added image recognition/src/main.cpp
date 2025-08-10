#include <Arduino.h>
#include "helper.h"
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <time.h>

#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "esp_http_server.h"
// Edge Impulse includes (exported Arduino library)
#include "Growth_stages_detection_inferencing.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"

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
// =========== Camera pins (AI-Thinker) ===========
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// =========== MJPEG stream helpers ===========
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char *_STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

// =========== Inference timing (ms) ===========
const uint32_t INFERENCE_INTERVAL_MS = 800; // run inference every 800 ms

// =========== Globals for Edge Impulse input ===========
static uint8_t *ei_rgb888_buf = nullptr;
static size_t ei_input_pixels = 0;

// Streaming status flag
volatile bool streaming_active = false;

// Debug flag
static bool debug_nn = false;

// Forward declaration
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

// =========== EI get_data implementation ===========
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
  if (!ei_rgb888_buf)
    return -1;
  size_t pixel_ix = offset * 3;
  for (size_t i = 0; i < length; i++)
  {
    uint8_t r = ei_rgb888_buf[pixel_ix + 0];
    uint8_t g = ei_rgb888_buf[pixel_ix + 1];
    uint8_t b = ei_rgb888_buf[pixel_ix + 2];
    uint32_t packed = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
    out_ptr[i] = (float)packed;
    pixel_ix += 3;
  }
  return 0;
}

// =========== Inference task ===========
void ei_inference_task(void *pvParameters)
{
  (void)pvParameters;
  const size_t model_w = EI_CLASSIFIER_INPUT_WIDTH;
  const size_t model_h = EI_CLASSIFIER_INPUT_HEIGHT;
  const size_t model_pixels = model_w * model_h;
  ei_input_pixels = model_pixels;

  ei_rgb888_buf = (uint8_t *)malloc(model_pixels * 3);
  if (!ei_rgb888_buf)
  {
    Serial.println("ERR: Failed to alloc ei_rgb888_buf");
    vTaskDelete(NULL);
  }

  uint8_t *tmp_rgb = nullptr;
  size_t tmp_rgb_size = 0;

  while (true)
  {
    if (!streaming_active)
    {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    uint32_t t0 = millis();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("ERR: esp_camera_fb_get() failed");
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // Check memory usage after capture
    Serial.printf("[Mem] Heap: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());

    size_t needed_rgb_size = fb->width * fb->height * 3;
    if (!tmp_rgb || tmp_rgb_size < needed_rgb_size)
    {
      if (tmp_rgb)
        free(tmp_rgb);
      tmp_rgb = (uint8_t *)malloc(needed_rgb_size);
      tmp_rgb_size = needed_rgb_size;
    }
    bool conv_ok = fmt2rgb888(fb->buf, fb->len, fb->format, tmp_rgb);
    esp_camera_fb_return(fb);

    if (!conv_ok)
    {
      Serial.println("ERR: fmt2rgb888 failed");
      free(tmp_rgb);
      tmp_rgb = nullptr;
      tmp_rgb_size = 0;
      vTaskDelay(pdMS_TO_TICKS(300));
      continue;
    }

    ei::image::processing::crop_and_interpolate_rgb888(
        tmp_rgb, fb->width, fb->height,
        ei_rgb888_buf, model_w, model_h);

    ei::signal_t signal;
    signal.total_length = model_w * model_h;
    signal.get_data = &ei_camera_get_data;

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR ei_err = run_classifier(&signal, &result, debug_nn);
    if (ei_err == EI_IMPULSE_OK)
    {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
      if (result.bounding_boxes_count > 0 && result.bounding_boxes != nullptr)
      {
        Serial.println("=== Object detection results ===");
        for (uint32_t i = 0; i < result.bounding_boxes_count; i++)
        {
          auto bb = result.bounding_boxes[i];
          if (bb.value == 0)
            continue;
          Serial.printf("%s (%.2f) [x:%u y:%u w:%u h:%u]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
          // Update growing stage based on detected bounding box=================================================================
          if (bb.value > 0.95) // Confidence threshold
          {
            UART0_value = String(bb.label); // Update UART0_value with the detected label
          }
        }
      }
      else
      {
        Serial.println("No bounding boxes detected.");
      }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
      Serial.printf("Anomaly: %.3f\n", result.anomaly);
#endif
    }
    else
    {
      Serial.printf("ERR: run_classifier failed (%d)\n", ei_err);
    }

    uint32_t elapsed = millis() - t0;
    if (elapsed < INFERENCE_INTERVAL_MS)
      vTaskDelay(pdMS_TO_TICKS(INFERENCE_INTERVAL_MS - elapsed));
    else
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =========== Stream handler ===========
static esp_err_t stream_handler(httpd_req_t *req)
{
  streaming_active = true;
  camera_fb_t *fb = nullptr;
  esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    streaming_active = false;
    return res;
  }

  char part_buf[128];

  while (true)
  {
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("ERR: Camera capture failed");
      streaming_active = false;
      return ESP_FAIL;
    }

    uint8_t *jpg_buf = nullptr;
    size_t jpg_len = 0;
    if (fb->format == PIXFORMAT_JPEG)
    {
      jpg_buf = fb->buf;
      jpg_len = fb->len;
    }
    else
    {
      if (!frame2jpg(fb, 80, &jpg_buf, &jpg_len))
      {
        Serial.println("ERR: frame2jpg failed");
        esp_camera_fb_return(fb);
        streaming_active = false;
        return ESP_FAIL;
      }
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, jpg_len);
    res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);

    if (fb->format != PIXFORMAT_JPEG && jpg_buf)
      free(jpg_buf);
    esp_camera_fb_return(fb);

    if (res != ESP_OK)
      break;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  streaming_active = false;
  return res;
}

void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL};

  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
  else
  {
    Serial.println("ERR: httpd_start failed");
  }
}

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
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  if (esp_camera_init(&config) != ESP_OK)
  {
    Serial.println("ERR: Camera init failed");
    while (true)
      delay(1000);
  }

  startCameraServer();
  Serial.println("HTTP stream ready at /stream");

  xTaskCreatePinnedToCore(ei_inference_task, "ei_infer", 12 * 1024, NULL, 1, NULL, 1);

  Serial.println("[Camera] ready");
  Serial.printf("[After Camera] Heap: %lu | PSRAM: %lu\n", ESP.getFreeHeap(), ESP.getFreePsram());

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

    // image recognition switch
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
