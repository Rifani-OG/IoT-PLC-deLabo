#include <cmath>
#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h> 
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "BluetoothSerial.h"

// ==========================================
// --- DEFINISI PIN & KONSTANTA ---
// ==========================================
#define BUZZER_PIN 14
#define LED_PIN 4
#define THROTTLE_PIN 34

#define SD_CS 5  
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define GPS_RX 16
#define GPS_TX 17

#define RS485_RX 32
#define RS485_TX 33

#define LORA_RX 25 
#define LORA_TX 26 

const float SHUNT_FACTOR = 1.95; 

// --- OBJEK GLOBAL ---
BluetoothSerial SerialBT;
bool bt_connected = false;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
SoftwareSerial loraSerial(LORA_RX, LORA_TX);
HardwareSerial pzemSerial(1);

// --- LOGGING SD CARD ---
int attempt_count = 0;
bool sd_initialized = false;
File log_file;
const char* log_filename = "/coba_telem/log_telem.txt";
int flush_counter = 0;
const int FLUSH_EVERY_N_LINES = 10;

// ==========================================
// --- STRUCT DATA & MUTEX (FREE RTOS) ---
// ==========================================
struct PowerData {
  float current_mA;
  float bus_voltage_V;
  float power_W;
  float energy_Wh; 
};
PowerData latest_power;
SemaphoreHandle_t power_mutex;

struct VehicleData {
  double lat;             // Diperbaiki: Harus Double untuk Presisi GPS
  double lng;             // Diperbaiki: Harus Double untuk Presisi GPS
  float speedKmph;
  double totalDistanceKm; // Diperbaiki: Harus Double untuk Presisi Haversine
  int satellites;
  int throttlePercent;
};
VehicleData latest_vehicle;
SemaphoreHandle_t vehicle_mutex;

// ==========================================
// --- FUNGSI UTILITAS ---
// ==========================================
uint16_t ModbusCRC16(byte *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

int readAttemptCount() {
  if (!SD.exists(log_filename)) return 1;
  File file = SD.open(log_filename, FILE_READ);
  if (!file) return 1;
  String lastLine = "";
  while (file.available()) { lastLine = file.readStringUntil('\n'); }
  file.close();
  if (lastLine.length() == 0) return 1;
  int lastComma = lastLine.lastIndexOf(',');
  if (lastComma == -1) return 1;
  return lastLine.substring(lastComma + 1).toInt() + 1;
}

bool initLogging() {
  delay(100);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 4000000)) return false;
  
  if (!SD.exists("/coba_telem")) SD.mkdir("/coba_telem");
  
  attempt_count = readAttemptCount();
  log_file = SD.open(log_filename, FILE_APPEND);
  if (!log_file) return false;
  
  if (log_file.size() == 0) {
    log_file.println("Lat,Lng,Speed,Sat,Throttle,Current_mA,Voltage,Power,Energy,Distance,Millis,Attempt");
    log_file.flush();
  }
  sd_initialized = true;
  return true;
}

void writeToLog(const char* data) {
  if (!sd_initialized) {
    static unsigned long last_sd_retry = 0;
    if (millis() - last_sd_retry > 5000) {
      last_sd_retry = millis();
      if (initLogging()) {
        Serial.println("[SD] SD Card terdeteksi menyusul. Logging Aktif!");
      } else {
        Serial.println("[SD] SD Card belum ada...");
      }
    }
    return; // Keluar dari fungsi jika masih belum ada SD Card
  }

  // Auto-Reopen SD Card jika sebelumnya sempat putus/terlepas
  if (!log_file) {
    log_file = SD.open(log_filename, FILE_APPEND);
    if (!log_file) return; 
  }

  // --- PERBAIKAN SARAN 1: Keamanan Pointer File ---
  if (log_file.print(data) == 0) {
    log_file.close(); 
    // Timpa dengan objek File kosong agar logika (!log_file) di atas 
    // 100% tereksekusi pada siklus berikutnya (lebih aman dari nullptr di ESP32)
    log_file = File(); 
    return;
  }
  
  log_file.print(",");
  log_file.println(attempt_count);
  
  flush_counter++;
  if (flush_counter % 10 == 0) {
      Serial.println(data); 
  }
  
  if (flush_counter >= FLUSH_EVERY_N_LINES) {
    log_file.flush();
    flush_counter = 0;
  }
}

// ==========================================
// --- TASK 1: PZEM-017 (POWER) - CORE 1 ---
// ==========================================
void pzem_read(void *pvParameters){
  byte requestPZEM[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x06, 0x70, 0x08};
  
  double total_energy_Ws = 0.0; 
  unsigned long last_pzem_time = millis();

  while(true){
    while(pzemSerial.available()) { pzemSerial.read(); }
    
    pzemSerial.write(requestPZEM, sizeof(requestPZEM));
    pzemSerial.flush(); 

    byte buffer[20];
    int len = 0;
    unsigned long timeout = millis() + 300; 

    while (millis() < timeout) {
      if (pzemSerial.available()) {
        byte b = pzemSerial.read();
        
        if (len == 0 && b != 0x01) continue;
        if (len == 1 && b != 0x04) { len = 0; continue; }
        if (len == 2 && b != 0x0C) { len = 0; continue; }
        
        buffer[len++] = b;
        if (len == 17) break; 
      } else {
        vTaskDelay(1 / portTICK_PERIOD_MS); 
      }
    }

    if (len == 17) {
      uint16_t calculatedCRC = ModbusCRC16(buffer, 15);
      uint16_t receivedCRC = (buffer[16] << 8) | buffer[15]; 
      
      if (calculatedCRC == receivedCRC) {
        float v_V = ((buffer[3] << 8) | buffer[4]) / 100.0;
        float i_A = (((buffer[5] << 8) | buffer[6]) / 100.0) / SHUNT_FACTOR; 
        float i_mA = i_A * 1000.0;
        
        // Diperbaiki: Explicit Casting ke uint32_t untuk mencegah bug bit-shift overflow 
        uint32_t p_raw = ((uint32_t)buffer[9] << 24) | ((uint32_t)buffer[10] << 16) | ((uint32_t)buffer[7] << 8) | (uint32_t)buffer[8];
        float p_W = (p_raw / 10.0) / SHUNT_FACTOR;

        // Diperbaiki: Integrasi Energi dengan Presisi Murni Double
        unsigned long now = millis();
        double dt_sec = (double)(now - last_pzem_time) / 1000.0;
        last_pzem_time = now;
        
        total_energy_Ws += ((double)p_W * dt_sec);
        float e_Wh = (float)(total_energy_Ws / 3600.0); // Casting kembali ke float khusus untuk struct

        if (xSemaphoreTake(power_mutex, portMAX_DELAY)) {
            latest_power.current_mA = i_mA;
            latest_power.bus_voltage_V = v_V;
            latest_power.power_W = p_W;
            latest_power.energy_Wh = e_Wh;
            xSemaphoreGive(power_mutex);
        }
      } 
    }
    vTaskDelay(200 / portTICK_PERIOD_MS); 
  }
}

// ==========================================
// --- TASK 2: VEHICLE STATE - CORE 0 ---
// ==========================================
void vehicle_state_task(void *pvParameters){
  // Deklarasi presisten di luar loop agar tidak di-reset saat sinyal GPS terputus
  double temp_distance = 0; 
  double last_lat = 0;
  double last_lng = 0;
  
  double t_lat = 0;
  double t_lng = 0;
  float t_speed = 0;
  int t_sat = 0;

  while(true){
    while (gpsSerial.available() > 0) { gps.encode(gpsSerial.read()); }
    
    t_sat = gps.satellites.value();

    if (gps.location.isValid()) { 
      t_lat = gps.location.lat(); 
      t_lng = gps.location.lng(); 

      // Kalkulasi Jarak dengan Threshold Noise Filter (1.5m)
      if (last_lat != 0 && last_lng != 0) {
        double dist_m = TinyGPSPlus::distanceBetween(last_lat, last_lng, t_lat, t_lng);
        if (dist_m > 1.5) { 
          temp_distance += (dist_m / 1000.0); 
          last_lat = t_lat;
          last_lng = t_lng;
        }
      } else {
        last_lat = t_lat;
        last_lng = t_lng;
      }
    } 

    if (gps.speed.isValid() && gps.location.age() < 2000) { 
      t_speed = gps.speed.kmph(); 
    } else {
      t_speed = 0; 
    }

    int raw_adc = analogRead(THROTTLE_PIN);
    int t_throttle = (raw_adc * 100) / 4095; 
    if (t_throttle < 1) t_throttle = 0;
    if (t_throttle > 100) t_throttle = 100;

    if (xSemaphoreTake(vehicle_mutex, portMAX_DELAY)) {
      latest_vehicle.lat = t_lat;
      latest_vehicle.lng = t_lng;
      latest_vehicle.speedKmph = t_speed;
      latest_vehicle.totalDistanceKm = temp_distance;
      latest_vehicle.satellites = t_sat;
      latest_vehicle.throttlePercent = t_throttle;
      xSemaphoreGive(vehicle_mutex);
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS); 
  }
}

// ==========================================
// --- TASK 3: BLUETOOTH - CORE 0 ---
// ==========================================
void bluetooth_task(void *pvParameters) {
  char bt_payload[250];
  while (1) {
    if (SerialBT.hasClient() && !bt_connected) { 
      bt_connected = true;
      digitalWrite(LED_PIN, HIGH);
      tone(BUZZER_PIN, 800); vTaskDelay(100 / portTICK_PERIOD_MS); noTone(BUZZER_PIN); 
      Serial.printf("[BT] HP Driver Tersambung!\n");
    } 
    else if (!SerialBT.hasClient() && bt_connected) { 
      bt_connected = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println("[BT] HP Driver Terputus.");
    }

    if (bt_connected) {
      PowerData p; VehicleData v;
      
      if (xSemaphoreTake(power_mutex, portMAX_DELAY)) { p = latest_power; xSemaphoreGive(power_mutex); }
      if (xSemaphoreTake(vehicle_mutex, portMAX_DELAY)) { v = latest_vehicle; xSemaphoreGive(vehicle_mutex); }
      
      snprintf(bt_payload, sizeof(bt_payload),
               "%.6f,%.6f,%.2f,%d,%d,%.3f,%.2f,%.2f,%.4f,%.3f,%lu\n",
               v.lat, v.lng, v.speedKmph, v.satellites, v.throttlePercent,
               p.current_mA, p.bus_voltage_V, p.power_W, p.energy_Wh,
               v.totalDistanceKm, millis());
              
      SerialBT.print(bt_payload);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// --- TASK 4: LORA & SD CARD - CORE 0 ---
// ==========================================
void lora_sd_task(void *pvParameters) {
  char log_buffer[350];
  while (1) {
    PowerData p; VehicleData v;
    
    if (xSemaphoreTake(power_mutex, portMAX_DELAY)) { p = latest_power; xSemaphoreGive(power_mutex); }
    if (xSemaphoreTake(vehicle_mutex, portMAX_DELAY)) { v = latest_vehicle; xSemaphoreGive(vehicle_mutex); }

    snprintf(log_buffer, sizeof(log_buffer), 
             "%.6f,%.6f,%.2f,%d,%d,%.3f,%.2f,%.2f,%.4f,%.3f,%lu",
             v.lat, v.lng, v.speedKmph, v.satellites, v.throttlePercent,
             p.current_mA, p.bus_voltage_V, p.power_W, p.energy_Wh,
             v.totalDistanceKm, millis());
    
    writeToLog(log_buffer); 
    
    if (loraSerial.availableForWrite() >= strlen(log_buffer)) {
        loraSerial.println(log_buffer);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// --- SETUP & LOOP ---
// ==========================================
void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  
  power_mutex = xSemaphoreCreateMutex();
  vehicle_mutex = xSemaphoreCreateMutex();
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); 
  
  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
  loraSerial.begin(9600); 
  pzemSerial.begin(9600, SERIAL_8N2, RS485_RX, RS485_TX); 
  
  tone(BUZZER_PIN, 523); delay(150); noTone(BUZZER_PIN); 
  
  if (initLogging()) Serial.println("Logging SD Card AKTIF");
  
  SerialBT.begin("Rakata_Dashboard"); 
  
  // Diperbaiki: Kembalikan alokasi memory stack VEHICLE ke 4096 untuk komputasi trigonometri ganda (Haversine)
  xTaskCreatePinnedToCore(bluetooth_task, "BT", 8192, NULL, 6, NULL, 0);
  xTaskCreatePinnedToCore(vehicle_state_task, "VEHICLE", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(lora_sd_task, "TX", 8192, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(pzem_read, "PZEM", 4096, NULL, 5, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}