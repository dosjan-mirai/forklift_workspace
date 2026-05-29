#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include<freertos/FreeRTOS.h>
#include<freertos/task.h>
#include "esp_adc_cal.h"
#define Button_pin 26
#define R_PIN 32
#define G_PIN 33
#define B_PIN 25

#define Battery_PIN 34
#define NUM_SAMPLES 100
#define PWM_FREQ 5000
#define PWM_RES 8
#define R_CH 0
#define G_CH 1
#define B_CH 2

int64_t last_send_time = 0;
int64_t current_time = 0;
float batteryLevel=0;

int64_t last_battery_check_time = 0;
typedef struct {
  uint8_t button;
  uint16_t battery;
} Packet;

Packet txPacket;
esp_adc_cal_characteristics_t adc_chars;

uint8_t receiverMAC[] = {0xF4,0x2D,0xC9,0x71,0xAE,0x4C};
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
float readBattery() {

 uint32_t sum = 0;

    for(int i = 0; i < NUM_SAMPLES; i++)
    {
        sum += analogRead(Battery_PIN);
    }

    uint32_t raw = sum / NUM_SAMPLES;

    uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

    float battery = (voltage / 1000.0) * 2.0;

    return battery;
}

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(R_CH, r);
  ledcWrite(G_CH, g);
  ledcWrite(B_CH, b);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  pinMode(Button_pin, INPUT);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
 analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_11,
        ADC_WIDTH_BIT_12,
        1100,
        &adc_chars
    );
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  ledcSetup(R_CH, PWM_FREQ, PWM_RES);
  ledcSetup(G_CH, PWM_FREQ, PWM_RES);
  ledcSetup(B_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_PIN, R_CH);
  ledcAttachPin(G_PIN, G_CH);
  ledcAttachPin(B_PIN, B_CH);

}

void loop() {
  
  current_time= xTaskGetTickCount() * portTICK_PERIOD_MS;
  if(current_time -last_battery_check_time >= 3000) { // Check battery every 3 seconds
    last_battery_check_time = current_time;
    batteryLevel = readBattery();
    Serial.print("Battery Voltage: ");
    Serial.println(batteryLevel);
  }
if(batteryLevel>=4.0)
  setColor(0,0,255);
else if(batteryLevel>=3.9)
  setColor(0,0,255);
else if(batteryLevel>=3.8)
  setColor(0,0,128);
else if(batteryLevel>=3.7)
  setColor(0,0,64);
else if(batteryLevel>=3.6)
  setColor(255,0,0);
else if(batteryLevel>=3.5)
  setColor(0,0,255);
else
  setColor(0,0,128);

  if (current_time - last_send_time >= 10) {
    last_send_time = current_time;
    txPacket.button = digitalRead(Button_pin);
    txPacket.battery = batteryLevel * 10; // Convert to percentage
    esp_now_send(receiverMAC, (uint8_t *)&txPacket, sizeof(txPacket));
  }
  
}
