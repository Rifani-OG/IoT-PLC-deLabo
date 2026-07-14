#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "arduinoFFT.h"
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

const char* ssid = "WIFI_NAME";
const char* password = "WIFI_PASSWORD";

const char* mqtt_server = "40de46779e2d4b859adacde420d9ce62.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "deLabo";
const char* mqtt_pass = "deLabo123";
const char* mqtt_topic = "ciren/motor1/vibration";

#define SAMPLES 128
#define SAMPLING_FREQ 1000.0
#define POWER_DETECT_PIN 35
#define SLEEP_DURATION 10
#define MOVING_AVG_WINDOW 4

WiFiClientSecure espClient; 
PubSubClient mqttClient(espClient);

double vRealX[SAMPLES], vImagX[SAMPLES];
double vRealY[SAMPLES], vImagY[SAMPLES];
double vRealZ[SAMPLES], vImagZ[SAMPLES];

ArduinoFFT<double> FFT_X(vRealX, vImagX, SAMPLES, SAMPLING_FREQ);
ArduinoFFT<double> FFT_Y(vRealY, vImagY, SAMPLES, SAMPLING_FREQ);
ArduinoFFT<double> FFT_Z(vRealZ, vImagZ, SAMPLES, SAMPLING_FREQ);

QueueHandle_t mqttQueue;
SemaphoreHandle_t mqttMutex;

typedef struct {
  char payload[256];
} MqttMessage;

// power management 
bool isPowerPlugged() {
  return (analogRead(POWER_DETECT_PIN) > 500);
}

// deep sleep mode
void goToDeepSleep() {
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000ULL);
  esp_deep_sleep_start();
}

// filter function (sum & count now passed per-axis, no longer shared/static)
float movingAvg(float newVal, float *buffer, float *sum, int *index, int *count, int size) {
  *sum -= buffer[*index];
  buffer[*index] = newVal;
  *sum += newVal;
  *index = (*index + 1) % size;
  if (*count < size) (*count)++;
  return *sum / *count;
}

// mqtt reconnect
void mqttReconnect() {
  if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {
    while (!mqttClient.connected()) {
      String clientId = "ESP32_Rakata_" + String(random(0xffff), HEX);
      if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
        Serial.println(" Terhubung!");
        break;
      } else {
        Serial.print(mqttClient.state());
        xSemaphoreGive(mqttMutex); 
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        xSemaphoreTake(mqttMutex, portMAX_DELAY);
      }
    }
    xSemaphoreGive(mqttMutex);
  }
}

// sensors and FFT (priority 1)
void TaskSensor(void *pvParameters) {
  unsigned int sampling_period_us = round(1000000.0 / SAMPLING_FREQ);
  unsigned long startMillis = millis();

  float bufX[MOVING_AVG_WINDOW] = {0};
  float bufY[MOVING_AVG_WINDOW] = {0};
  float bufZ[MOVING_AVG_WINDOW] = {0};
  int idxX = 0, idxY = 0, idxZ = 0;
  float sumX = 0, sumY = 0, sumZ = 0;
  int countX = 0, countY = 0, countZ = 0;

  while (1) {
    // dummy data
    float elapsed = (millis() - startMillis) / 1000.0;
    float freqY = 50.0 + (elapsed / 45.0) * 70.0;
    if (freqY > 120.0) freqY = 120.0;
    float ampY = 1.0 + (elapsed / 45.0) * 2.5;
    if (ampY > 3.5) ampY = 3.5;

    double sumSqX = 0, sumSqY = 0, sumSqZ = 0;

    // Sampling
    for (int i = 0; i < SAMPLES; i++) {
      float t = (float)i / SAMPLING_FREQ;
      float rawX = 1.2 * sin(2 * PI * 50.0 * t) + 0.4 * sin(2 * PI * 150.0 * t) + (random(-50,50)/1000.0);
      float rawY = ampY * sin(2 * PI * freqY * t) + (random(-50,50)/1000.0);
      float rawZ = 0.6 * sin(2 * PI * 25.0 * t) + (random(-50,50)/1000.0);

      // EMI Filter / Digital Low-Pass (per-axis, isolated accumulators)
      float filtX = movingAvg(rawX, bufX, &sumX, &idxX, &countX, MOVING_AVG_WINDOW);
      float filtY = movingAvg(rawY, bufY, &sumY, &idxY, &countY, MOVING_AVG_WINDOW);
      float filtZ = movingAvg(rawZ, bufZ, &sumZ, &idxZ, &countZ, MOVING_AVG_WINDOW);

      sumSqX += (double)filtX * filtX;
      sumSqY += (double)filtY * filtY;
      sumSqZ += (double)filtZ * filtZ;

      vRealX[i] = filtX;
      vRealY[i] = filtY;
      vRealZ[i] = filtZ;
      vImagX[i] = vImagY[i] = vImagZ[i] = 0;

      delayMicroseconds(sampling_period_us);
    }

    double rmsX = sqrt(sumSqX / SAMPLES);
    double rmsY = sqrt(sumSqY / SAMPLES);
    double rmsZ = sqrt(sumSqZ / SAMPLES);

    // FFT
    FFT_X.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT_X.compute(FFTDirection::Forward);
    FFT_X.complexToMagnitude();
    
    FFT_Y.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT_Y.compute(FFTDirection::Forward);
    FFT_Y.complexToMagnitude();
    
    FFT_Z.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT_Z.compute(FFTDirection::Forward);
    FFT_Z.complexToMagnitude();

    double peakX = FFT_X.majorPeak();
    double peakY = FFT_Y.majorPeak();
    double peakZ = FFT_Z.majorPeak();

    // payload to mqtt
    MqttMessage msg;
    snprintf(msg.payload, sizeof(msg.payload),
      "{\"peak_hz_X\":%.2f,\"peak_hz_Y\":%.2f,\"peak_hz_Z\":%.2f,"
      "\"rms_X\":%.3f,\"rms_Y\":%.3f,\"rms_Z\":%.3f}",
      peakX, peakY, peakZ, rmsX, rmsY, rmsZ);
    xQueueSend(mqttQueue, &msg, portMAX_DELAY);

    // power mode
    if (!isPowerPlugged()) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      goToDeepSleep();
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// mqtt connection
void TaskMQTT(void *pvParameters) {
  MqttMessage msg;
  while (1) {
    if (xQueueReceive(mqttQueue, &msg, portMAX_DELAY) == pdTRUE) {
      if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {
        if (mqttClient.connected()) {
          mqttClient.publish(mqtt_topic, msg.payload);
          Serial.println(msg.payload); //debug massage
        }
        xSemaphoreGive(mqttMutex);
      }
    }
  }
}

// connection monitoring
void TaskMonitor(void *pvParameters) {
  while (1) {
    if (!mqttClient.connected()) {
      mqttReconnect();
    }
    if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {
      mqttClient.loop();
      xSemaphoreGive(mqttMutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// setup
void setup() {
  Serial.begin(115200);
  pinMode(POWER_DETECT_PIN, INPUT);
  mqttMutex = xSemaphoreCreateMutex();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ");
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print("."); 
  }
  Serial.println("\nConnected");
  espClient.setInsecure(); 
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttReconnect();

  mqttQueue = xQueueCreate(5, sizeof(MqttMessage));

  // Pinned Task RTOS
  xTaskCreatePinnedToCore(TaskSensor, "Sensor", 4096, NULL, 1, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(TaskMQTT, "MQTT", 4096, NULL, 2, NULL, 1); // Core 1
  xTaskCreatePinnedToCore(TaskMonitor, "Monitor", 2048, NULL, 1, NULL, 1); // Core 1

  vTaskDelete(NULL);
}

void loop() {
}
