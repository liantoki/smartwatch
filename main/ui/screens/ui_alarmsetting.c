// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_alarmsetting_screen_init(void)
{
    ui_alarmsetting = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_alarmsetting, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_img_src(ui_alarmsetting, &ui_img_ui_bg2_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container15 = lv_obj_create(ui_alarmsetting);
    lv_obj_remove_style_all(ui_Container15);
    lv_obj_set_width(ui_Container15, 360);
    lv_obj_set_height(ui_Container15, 360);
    lv_obj_set_align(ui_Container15, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container15, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_Container15, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container15, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label23 = lv_label_create(ui_Container15);
    lv_obj_set_width(ui_Label23, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label23, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label23, 0);
    lv_obj_set_y(ui_Label23, 10);
    lv_obj_set_align(ui_Label23, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_Label23, "闹钟设置");
    lv_obj_set_style_text_font(ui_Label23, &ui_font_chinese16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container16 = lv_obj_create(ui_Container15);
    lv_obj_remove_style_all(ui_Container16);
    lv_obj_set_width(ui_Container16, 360);
    lv_obj_set_height(ui_Container16, 57);
    lv_obj_set_x(ui_Container16, -2);
    lv_obj_set_y(ui_Container16, -96);
    lv_obj_set_align(ui_Container16, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container16, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Container16, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container16, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Alarmhourlabel = lv_label_create(ui_Container16);
    lv_obj_set_width(ui_Alarmhourlabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Alarmhourlabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Alarmhourlabel, -1);
    lv_obj_set_y(ui_Alarmhourlabel, 4);
    lv_obj_set_align(ui_Alarmhourlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Alarmhourlabel, "15");
    lv_obj_set_style_text_font(ui_Alarmhourlabel, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label24 = lv_label_create(ui_Container16);
    lv_obj_set_width(ui_Label24, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label24, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label24, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label24, ":");
    lv_obj_set_style_text_font(ui_Label24, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Alarmminuteslabel = lv_label_create(ui_Container16);
    lv_obj_set_width(ui_Alarmminuteslabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Alarmminuteslabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Alarmminuteslabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Alarmminuteslabel, "15");
    lv_obj_set_style_text_font(ui_Alarmminuteslabel, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Alarmminuteslabel, 2, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_side(ui_Alarmminuteslabel, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN | LV_STATE_FOCUSED);

    ui_Label25 = lv_label_create(ui_Container16);
    lv_obj_set_width(ui_Label25, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label25, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label25, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label25, ":");
    lv_obj_set_style_text_font(ui_Label25, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Alarmsecondslabel = lv_label_create(ui_Container16);
    lv_obj_set_width(ui_Alarmsecondslabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Alarmsecondslabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Alarmsecondslabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Alarmsecondslabel, "15");
    lv_obj_set_style_text_font(ui_Alarmsecondslabel, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container17 = lv_obj_create(ui_Container15);
    lv_obj_remove_style_all(ui_Container17);
    lv_obj_set_width(ui_Container17, 360);
    lv_obj_set_height(ui_Container17, 106);
    lv_obj_set_x(ui_Container17, 7);
    lv_obj_set_y(ui_Container17, 88);
    lv_obj_set_align(ui_Container17, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container17, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Container17, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container17, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Alarmhour = lv_roller_create(ui_Container17);
    lv_roller_set_options(ui_Alarmhour,
                          "0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23", LV_ROLLER_MODE_NORMAL);
    lv_roller_set_selected(ui_Alarmhour, 15, LV_ANIM_OFF);
    lv_obj_set_width(ui_Alarmhour, 75);
    lv_obj_set_height(ui_Alarmhour, 100);
    lv_obj_set_x(ui_Alarmhour, -91);
    lv_obj_set_y(ui_Alarmhour, 35);
    lv_obj_set_align(ui_Alarmhour, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_Alarmhour, lv_color_hex(0x4FABF7), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Alarmhour, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Alarmhour, lv_color_hex(0x005295), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Alarmhour, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Alarmhour, 1, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_Label26 = lv_label_create(ui_Container17);
    lv_obj_set_width(ui_Label26, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label26, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label26, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label26, "\n:");
    lv_obj_set_style_text_font(ui_Label26, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Alarmminutes = lv_roller_create(ui_Container17);
    lv_roller_set_options(ui_Alarmminutes,
                          "0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59",
                          LV_ROLLER_MODE_NORMAL);
    lv_roller_set_selected(ui_Alarmminutes, 15, LV_ANIM_OFF);
    lv_obj_set_width(ui_Alarmminutes, 75);
    lv_obj_set_height(ui_Alarmminutes, 100);
    lv_obj_set_x(ui_Alarmminutes, -91);
    lv_obj_set_y(ui_Alarmminutes, 35);
    lv_obj_set_align(ui_Alarmminutes, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_Alarmminutes, lv_color_hex(0x4FABF7), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Alarmminutes, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Alarmminutes, lv_color_hex(0x005295), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Alarmminutes, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Alarmminutes, 1, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_Label27 = lv_label_create(ui_Container17);
    lv_obj_set_width(ui_Label27, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label27, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label27, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label27, "\n:");
    lv_obj_set_style_text_font(ui_Label27, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Alarmseconds = lv_roller_create(ui_Container17);
    lv_roller_set_options(ui_Alarmseconds,
                          "0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59",
                          LV_ROLLER_MODE_NORMAL);
    lv_roller_set_selected(ui_Alarmseconds, 15, LV_ANIM_OFF);
    lv_obj_set_width(ui_Alarmseconds, 75);
    lv_obj_set_height(ui_Alarmseconds, 100);
    lv_obj_set_x(ui_Alarmseconds, -91);
    lv_obj_set_y(ui_Alarmseconds, 35);
    lv_obj_set_align(ui_Alarmseconds, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_Alarmseconds, lv_color_hex(0x4FABF7), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Alarmseconds, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Alarmseconds, lv_color_hex(0x005295), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Alarmseconds, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Alarmseconds, 1, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_Container18 = lv_obj_create(ui_Container15);
    lv_obj_remove_style_all(ui_Container18);
    lv_obj_set_width(ui_Container18, 360);
    lv_obj_set_height(ui_Container18, 84);
    lv_obj_set_x(ui_Container18, 2);
    lv_obj_set_y(ui_Container18, 146);
    lv_obj_set_align(ui_Container18, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container18, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Container18, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container18, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Button5 = lv_btn_create(ui_Container18);
    lv_obj_set_width(ui_Button5, 100);
    lv_obj_set_height(ui_Button5, 50);
    lv_obj_set_align(ui_Button5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button5, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button5, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label28 = lv_label_create(ui_Button5);
    lv_obj_set_width(ui_Label28, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label28, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label28, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label28, "确认");
    lv_obj_set_style_text_font(ui_Label28, &ui_font_chinese16, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Alarmhour, ui_event_Alarmhour, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Alarmminutes, ui_event_Alarmminutes, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Alarmseconds, ui_event_Alarmseconds, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button5, ui_event_Button5, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_alarmsetting, ui_event_alarmsetting, LV_EVENT_ALL, NULL);

}
