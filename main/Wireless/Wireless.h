#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "nvs_flash.h" 
#include "esp_log.h"

#include <stdio.h>
#include <string.h>  // For memcpy
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "freertos/queue.h"

#define DEFAULT_SCAN_LIST_SIZE 20

extern uint16_t BLE_NUM;
extern uint16_t WIFI_NUM;
extern bool Scan_finish;
// extern QueueHandle_t wifi_queue;

void Wireless_Init(void);
void WIFI_Init(void *arg);
uint16_t WIFI_Scan(wifi_ap_record_t *result_buffer, uint16_t size);
void BLE_Init(void *arg);
uint16_t BLE_Scan(void);

void ConnectToWiFi(const char* ssid, const char* password);
bool GetStoredPassword(const char* ssid, char* password_buf, size_t buf_size);
void StorePassword(const char* ssid, const char* password);
void DeleteStoredPassword(const char* ssid);