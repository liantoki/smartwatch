// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_menu_screen_init(void)
{
    ui_menu = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_menu, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_menu, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_menu, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_menu, &ui_img_ui_bg2_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container7 = lv_obj_create(ui_menu);
    lv_obj_remove_style_all(ui_Container7);
    lv_obj_set_width(ui_Container7, 360);
    lv_obj_set_height(ui_Container7, 360);
    lv_obj_set_align(ui_Container7, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container7, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Container7, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Container7, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_wifiButton = lv_imgbtn_create(ui_Container7);
    lv_imgbtn_set_src(ui_wifiButton, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_1837598647, NULL);
    lv_imgbtn_set_src(ui_wifiButton, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_1640525322, NULL);
    lv_imgbtn_set_src(ui_wifiButton, LV_IMGBTN_STATE_CHECKED_RELEASED, NULL, &ui_img_1640525322, NULL);
    lv_obj_set_width(ui_wifiButton, 48);
    lv_obj_set_height(ui_wifiButton, 48);
    lv_obj_set_x(ui_wifiButton, -64);
    lv_obj_set_y(ui_wifiButton, -64);
    lv_obj_set_align(ui_wifiButton, LV_ALIGN_CENTER);
    lv_obj_set_style_border_width(ui_wifiButton, 3, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_color(ui_wifiButton, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_opa(ui_wifiButton, 126, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_x(ui_wifiButton, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_y(ui_wifiButton, 0, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_musicButton = lv_imgbtn_create(ui_Container7);
    lv_imgbtn_set_src(ui_musicButton, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_music_png, NULL);
    lv_imgbtn_set_src(ui_musicButton, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_presser_music_png, NULL);
    lv_imgbtn_set_src(ui_musicButton, LV_IMGBTN_STATE_CHECKED_RELEASED, NULL, &ui_img_presser_music_png, NULL);
    lv_obj_set_width(ui_musicButton, 48);
    lv_obj_set_height(ui_musicButton, 48);
    lv_obj_set_x(ui_musicButton, 10);
    lv_obj_set_y(ui_musicButton, -64);
    lv_obj_set_align(ui_musicButton, LV_ALIGN_CENTER);
    lv_obj_set_style_radius(ui_musicButton, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_musicButton, lv_color_hex(0x020000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_musicButton, 126, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_musicButton, 3, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_color(ui_musicButton, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_opa(ui_musicButton, 126, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_x(ui_musicButton, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_y(ui_musicButton, 0, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_settingButton = lv_imgbtn_create(ui_Container7);
    lv_imgbtn_set_src(ui_settingButton, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_setting_png, NULL);
    lv_imgbtn_set_src(ui_settingButton, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_pressed_setting_png, NULL);
    lv_imgbtn_set_src(ui_settingButton, LV_IMGBTN_STATE_CHECKED_RELEASED, NULL, &ui_img_pressed_setting_png, NULL);
    lv_obj_set_height(ui_settingButton, 48);
    lv_obj_set_width(ui_settingButton, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_settingButton, 80);
    lv_obj_set_y(ui_settingButton, -62);
    lv_obj_set_align(ui_settingButton, LV_ALIGN_CENTER);

    ui_calendarButton = lv_imgbtn_create(ui_Container7);
    lv_imgbtn_set_src(ui_calendarButton, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_calendar_png, NULL);
    lv_imgbtn_set_src(ui_calendarButton, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_pressed_calendar_png, NULL);
    lv_imgbtn_set_src(ui_calendarButton, LV_IMGBTN_STATE_CHECKED_RELEASED, NULL, &ui_img_pressed_calendar_png, NULL);
    lv_obj_set_height(ui_calendarButton, 48);
    lv_obj_set_width(ui_calendarButton, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_calendarButton, 80);
    lv_obj_set_y(ui_calendarButton, -62);
    lv_obj_set_align(ui_calendarButton, LV_ALIGN_CENTER);

    lv_obj_add_event_cb(ui_wifiButton, ui_event_wifiButton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_musicButton, ui_event_musicButton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_settingButton, ui_event_settingButton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_calendarButton, ui_event_calendarButton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_menu, ui_event_menu, LV_EVENT_ALL, NULL);

}
