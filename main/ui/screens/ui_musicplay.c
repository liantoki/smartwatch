// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_musicplay_screen_init(void)
{
    ui_musicplay = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_musicplay, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    lv_obj_add_event_cb(ui_musicplay, ui_event_musicplay, LV_EVENT_ALL, NULL);

}
