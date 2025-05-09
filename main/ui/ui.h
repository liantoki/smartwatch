// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"
#include "ui_theme_manager.h"
#include "ui_themes.h"


// SCREEN: ui_home
void ui_home_screen_init(void);
void ui_event_home(lv_event_t * e);
extern lv_obj_t * ui_home;
extern lv_obj_t * ui_stepArc;
extern lv_obj_t * ui_stepImage;
extern lv_obj_t * ui_steplabel;
extern lv_obj_t * ui_batArc;
extern lv_obj_t * ui_Image4;
extern lv_obj_t * ui_batlabel;
extern lv_obj_t * ui_Container3;
extern lv_obj_t * ui_Labeltime;
extern lv_obj_t * ui_Label5;
// CUSTOM VARIABLES

// SCREEN: ui_menu
void ui_menu_screen_init(void);
void ui_event_menu(lv_event_t * e);
extern lv_obj_t * ui_menu;
extern lv_obj_t * ui_Container7;
void ui_event_wifiButton(lv_event_t * e);
extern lv_obj_t * ui_wifiButton;
void ui_event_musicButton(lv_event_t * e);
extern lv_obj_t * ui_musicButton;
void ui_event_settingButton(lv_event_t * e);
extern lv_obj_t * ui_settingButton;
void ui_event_calendarButton(lv_event_t * e);
extern lv_obj_t * ui_calendarButton;
// CUSTOM VARIABLES

// SCREEN: ui_wifi
void ui_wifi_screen_init(void);
void ui_event_wifi(lv_event_t * e);
extern lv_obj_t * ui_wifi;
extern lv_obj_t * ui_Container1;
void ui_event_Button3(lv_event_t * e);
extern lv_obj_t * ui_Button3;
void ui_event_ImgButton2(lv_event_t * e);
extern lv_obj_t * ui_ImgButton2;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_Image1;
extern lv_obj_t * ui_Label10;
void ui_event_homeButton(lv_event_t * e);
extern lv_obj_t * ui_homeButton;
// CUSTOM VARIABLES

// SCREEN: ui_wificfg
void ui_wificfg_screen_init(void);
void ui_event_wificfg(lv_event_t * e);
extern lv_obj_t * ui_wificfg;
extern lv_obj_t * ui_Label7;
extern lv_obj_t * ui_Label8;
// CUSTOM VARIABLES

// SCREEN: ui_musiclist
void ui_musiclist_screen_init(void);
void ui_event_musiclist(lv_event_t * e);
extern lv_obj_t * ui_musiclist;
extern lv_obj_t * ui_Container2;
void ui_event_Button1(lv_event_t * e);
extern lv_obj_t * ui_Button1;
extern lv_obj_t * ui_Label2;
// CUSTOM VARIABLES

// SCREEN: ui_musicplay
void ui_musicplay_screen_init(void);
void ui_event_musicplay(lv_event_t * e);
extern lv_obj_t * ui_musicplay;
// CUSTOM VARIABLES

// SCREEN: ui_setting
void ui_setting_screen_init(void);
void ui_event_setting(lv_event_t * e);
extern lv_obj_t * ui_setting;
extern lv_obj_t * ui_Container4;
void ui_event_timesetbutton(lv_event_t * e);
extern lv_obj_t * ui_timesetbutton;
extern lv_obj_t * ui_Label3;
void ui_event_alarmsetbutton(lv_event_t * e);
extern lv_obj_t * ui_alarmsetbutton;
extern lv_obj_t * ui_Label22;
void ui_event_backlightsetbutton(lv_event_t * e);
extern lv_obj_t * ui_backlightsetbutton;
extern lv_obj_t * ui_Label4;
// CUSTOM VARIABLES

// SCREEN: ui_timesetting
void ui_timesetting_screen_init(void);
void ui_event_timesetting(lv_event_t * e);
extern lv_obj_t * ui_timesetting;
extern lv_obj_t * ui_Container6;
extern lv_obj_t * ui_Label6;
extern lv_obj_t * ui_Container10;
extern lv_obj_t * ui_hourlabel;
extern lv_obj_t * ui_Label16;
extern lv_obj_t * ui_minuteslabel;
extern lv_obj_t * ui_Label18;
extern lv_obj_t * ui_secondslabel;
extern lv_obj_t * ui_Container5;
void ui_event_hour(lv_event_t * e);
extern lv_obj_t * ui_hour;
extern lv_obj_t * ui_Label11;
void ui_event_minutes(lv_event_t * e);
extern lv_obj_t * ui_minutes;
extern lv_obj_t * ui_Label12;
void ui_event_seconds(lv_event_t * e);
extern lv_obj_t * ui_seconds;
extern lv_obj_t * ui_Container9;
void ui_event_Button2(lv_event_t * e);
extern lv_obj_t * ui_Button2;
extern lv_obj_t * ui_Label14;
// CUSTOM VARIABLES

// SCREEN: ui_calendar
void ui_calendar_screen_init(void);
void ui_event_calendar(lv_event_t * e);
extern lv_obj_t * ui_calendar;
extern lv_obj_t * ui_Container8;
extern lv_obj_t * ui_Label13;
void ui_event_Calendar2(lv_event_t * e);
extern lv_obj_t * ui_Calendar2;
// CUSTOM VARIABLES

// SCREEN: ui_datesetting
void ui_datesetting_screen_init(void);
void ui_event_datesetting(lv_event_t * e);
extern lv_obj_t * ui_datesetting;
extern lv_obj_t * ui_Container11;
extern lv_obj_t * ui_Label9;
extern lv_obj_t * ui_Container12;
extern lv_obj_t * ui_yearlabel;
extern lv_obj_t * ui_Label15;
extern lv_obj_t * ui_monthlabel;
extern lv_obj_t * ui_Label17;
extern lv_obj_t * ui_daylabel;
extern lv_obj_t * ui_Container13;
void ui_event_year(lv_event_t * e);
extern lv_obj_t * ui_year;
extern lv_obj_t * ui_Label19;
void ui_event_month(lv_event_t * e);
extern lv_obj_t * ui_month;
extern lv_obj_t * ui_Label20;
void ui_event_day(lv_event_t * e);
extern lv_obj_t * ui_day;
extern lv_obj_t * ui_Container14;
void ui_event_Button4(lv_event_t * e);
extern lv_obj_t * ui_Button4;
extern lv_obj_t * ui_Label21;
// CUSTOM VARIABLES

// SCREEN: ui_alarmsetting
void ui_alarmsetting_screen_init(void);
void ui_event_alarmsetting(lv_event_t * e);
extern lv_obj_t * ui_alarmsetting;
extern lv_obj_t * ui_Container15;
extern lv_obj_t * ui_Label23;
extern lv_obj_t * ui_Container16;
extern lv_obj_t * ui_Alarmhourlabel;
extern lv_obj_t * ui_Label24;
extern lv_obj_t * ui_Alarmminuteslabel;
extern lv_obj_t * ui_Label25;
extern lv_obj_t * ui_Alarmsecondslabel;
extern lv_obj_t * ui_Container17;
void ui_event_Alarmhour(lv_event_t * e);
extern lv_obj_t * ui_Alarmhour;
extern lv_obj_t * ui_Label26;
void ui_event_Alarmminutes(lv_event_t * e);
extern lv_obj_t * ui_Alarmminutes;
extern lv_obj_t * ui_Label27;
void ui_event_Alarmseconds(lv_event_t * e);
extern lv_obj_t * ui_Alarmseconds;
extern lv_obj_t * ui_Container18;
void ui_event_Button5(lv_event_t * e);
extern lv_obj_t * ui_Button5;
extern lv_obj_t * ui_Label28;
// CUSTOM VARIABLES

// SCREEN: ui_alarmScreen
void ui_alarmScreen_screen_init(void);
void ui_event_alarmScreen(lv_event_t * e);
extern lv_obj_t * ui_alarmScreen;
extern lv_obj_t * ui_Container19;
extern lv_obj_t * ui_Label29;
extern lv_obj_t * ui_alarmContainer;
void ui_event_alarmButton(lv_event_t * e);
extern lv_obj_t * ui_alarmButton;
extern lv_obj_t * ui_alarmLabel;
void ui_event_alarmSwitch(lv_event_t * e);
extern lv_obj_t * ui_alarmSwitch;
extern lv_obj_t * ui_Container20;
void ui_event_Button7(lv_event_t * e);
extern lv_obj_t * ui_Button7;
extern lv_obj_t * ui_Label31;
void ui_event_Button6(lv_event_t * e);
extern lv_obj_t * ui_Button6;
extern lv_obj_t * ui_Label30;
// CUSTOM VARIABLES

// EVENTS

extern lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS
LV_IMG_DECLARE(ui_img_ui_bg_png);    // assets/ui_bg.png
LV_IMG_DECLARE(ui_img_footsteps_png);    // assets/footsteps.png
LV_IMG_DECLARE(ui_img_battery_png);    // assets/battery.png
LV_IMG_DECLARE(ui_img_ui_bg2_png);    // assets/ui_bg2.png
LV_IMG_DECLARE(ui_img_1837598647);    // assets/Wi-Fi.png
LV_IMG_DECLARE(ui_img_1640525322);    // assets/pressed_Wi-Fi.png
LV_IMG_DECLARE(ui_img_music_png);    // assets/music.png
LV_IMG_DECLARE(ui_img_presser_music_png);    // assets/presser_music.png
LV_IMG_DECLARE(ui_img_setting_png);    // assets/setting.png
LV_IMG_DECLARE(ui_img_pressed_setting_png);    // assets/pressed_setting.png
LV_IMG_DECLARE(ui_img_calendar_png);    // assets/calendar.png
LV_IMG_DECLARE(ui_img_pressed_calendar_png);    // assets/pressed_calendar.png
LV_IMG_DECLARE(ui_img_ui_bg3_png);    // assets/ui_bg3.png
LV_IMG_DECLARE(ui_img_rightarrow_png);    // assets/rightarrow.png
LV_IMG_DECLARE(ui_img_pressed_rightarrow_png);    // assets/pressed_rightarrow.png
LV_IMG_DECLARE(ui_img_home_png);    // assets/home.png
LV_IMG_DECLARE(ui_img_pressed_home_png);    // assets/pressed_home.png

// FONTS
LV_FONT_DECLARE(ui_font_chinese12);
LV_FONT_DECLARE(ui_font_chinese16);

// UI INIT
void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
