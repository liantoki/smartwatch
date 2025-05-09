// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_setting_screen_init(void)
{
    ui_setting = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_setting, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_img_src(ui_setting, &ui_img_ui_bg3_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container4 = lv_obj_create(ui_setting);
    lv_obj_remove_style_all(ui_Container4);
    lv_obj_set_width(ui_Container4, 360);
    lv_obj_set_height(ui_Container4, 360);
    lv_obj_set_align(ui_Container4, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container4, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(ui_Container4, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container4, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Container4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Container4, 50, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_timesetbutton = lv_btn_create(ui_Container4);
    lv_obj_set_width(ui_timesetbutton, 360);
    lv_obj_set_height(ui_timesetbutton, 50);
    lv_obj_set_align(ui_timesetbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_timesetbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_timesetbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_timesetbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_timesetbutton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_timesetbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label3 = lv_label_create(ui_timesetbutton);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "时间日期设置");
    lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0x070707), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label3, &ui_font_chinese16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_alarmsetbutton = lv_btn_create(ui_Container4);
    lv_obj_set_width(ui_alarmsetbutton, 360);
    lv_obj_set_height(ui_alarmsetbutton, 50);
    lv_obj_set_x(ui_alarmsetbutton, 1);
    lv_obj_set_y(ui_alarmsetbutton, 50);
    lv_obj_set_align(ui_alarmsetbutton, LV_ALIGN_TOP_MID);
    lv_obj_add_flag(ui_alarmsetbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_alarmsetbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_alarmsetbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_alarmsetbutton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_alarmsetbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label22 = lv_label_create(ui_alarmsetbutton);
    lv_obj_set_width(ui_Label22, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label22, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label22, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label22, "闹钟");
    lv_obj_set_style_text_color(ui_Label22, lv_color_hex(0x070707), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label22, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label22, &ui_font_chinese16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_backlightsetbutton = lv_btn_create(ui_Container4);
    lv_obj_set_width(ui_backlightsetbutton, 360);
    lv_obj_set_height(ui_backlightsetbutton, 50);
    lv_obj_set_x(ui_backlightsetbutton, 1);
    lv_obj_set_y(ui_backlightsetbutton, 50);
    lv_obj_set_align(ui_backlightsetbutton, LV_ALIGN_TOP_MID);
    lv_obj_add_flag(ui_backlightsetbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_backlightsetbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_backlightsetbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_backlightsetbutton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_backlightsetbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label4 = lv_label_create(ui_backlightsetbutton);
    lv_obj_set_width(ui_Label4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label4, "亮度设置");
    lv_obj_set_style_text_color(ui_Label4, lv_color_hex(0x070707), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label4, &ui_font_chinese16, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_timesetbutton, ui_event_timesetbutton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_alarmsetbutton, ui_event_alarmsetbutton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_backlightsetbutton, ui_event_backlightsetbutton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_setting, ui_event_setting, LV_EVENT_ALL, NULL);

}
