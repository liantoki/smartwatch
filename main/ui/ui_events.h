// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#ifndef _UI_EVENTS_H
#define _UI_EVENTS_H

#ifdef __cplusplus
extern "C" {
#endif

void update_call(lv_event_t * e);
void delet_obj(lv_event_t * e);
void wifi_update(lv_event_t * e);
void wifi_connected(lv_event_t * e);
void get_ssid(lv_event_t * e);
void wificfg_update(lv_event_t * e);
void musiclist_create(lv_event_t * e);
void musicplay_create(lv_event_t * e);
void backlight_change(lv_event_t * e);
void datetime_setting(lv_event_t * e);
void label_update(lv_event_t * e);
void datetime_update(lv_event_t * e);
void calendar_handler(lv_event_t * e);
void alarm_update(lv_event_t * e);
void alarm_start(lv_event_t * e);
void alarm_add(lv_event_t * e);
void alarm_clear(lv_event_t * e);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
