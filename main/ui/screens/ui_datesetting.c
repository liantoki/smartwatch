// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_datesetting_screen_init(void)
{
    ui_datesetting = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_datesetting, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_img_src(ui_datesetting, &ui_img_ui_bg_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container11 = lv_obj_create(ui_datesetting);
    lv_obj_remove_style_all(ui_Container11);
    lv_obj_set_width(ui_Container11, 360);
    lv_obj_set_height(ui_Container11, 360);
    lv_obj_set_align(ui_Container11, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container11, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_Container11, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container11, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label9 = lv_label_create(ui_Container11);
    lv_obj_set_width(ui_Label9, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label9, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label9, 0);
    lv_obj_set_y(ui_Label9, 10);
    lv_obj_set_align(ui_Label9, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_Label9, "日期设置");
    lv_obj_set_style_text_font(ui_Label9, &ui_font_chinese16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container12 = lv_obj_create(ui_Container11);
    lv_obj_remove_style_all(ui_Container12);
    lv_obj_set_width(ui_Container12, 360);
    lv_obj_set_height(ui_Container12, 57);
    lv_obj_set_x(ui_Container12, -2);
    lv_obj_set_y(ui_Container12, -96);
    lv_obj_set_align(ui_Container12, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container12, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Container12, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container12, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_yearlabel = lv_label_create(ui_Container12);
    lv_obj_set_width(ui_yearlabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_yearlabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_yearlabel, -1);
    lv_obj_set_y(ui_yearlabel, 4);
    lv_obj_set_align(ui_yearlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_yearlabel, "2025");
    lv_obj_set_style_text_font(ui_yearlabel, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label15 = lv_label_create(ui_Container12);
    lv_obj_set_width(ui_Label15, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label15, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label15, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label15, ".");
    lv_obj_set_style_text_font(ui_Label15, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_monthlabel = lv_label_create(ui_Container12);
    lv_obj_set_width(ui_monthlabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_monthlabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_monthlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_monthlabel, "6");
    lv_obj_set_style_text_font(ui_monthlabel, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_monthlabel, 2, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_side(ui_monthlabel, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN | LV_STATE_FOCUSED);

    ui_Label17 = lv_label_create(ui_Container12);
    lv_obj_set_width(ui_Label17, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label17, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label17, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label17, ".");
    lv_obj_set_style_text_font(ui_Label17, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_daylabel = lv_label_create(ui_Container12);
    lv_obj_set_width(ui_daylabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_daylabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_daylabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_daylabel, "16");
    lv_obj_set_style_text_font(ui_daylabel, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container13 = lv_obj_create(ui_Container11);
    lv_obj_remove_style_all(ui_Container13);
    lv_obj_set_width(ui_Container13, 360);
    lv_obj_set_height(ui_Container13, 106);
    lv_obj_set_x(ui_Container13, 7);
    lv_obj_set_y(ui_Container13, 88);
    lv_obj_set_align(ui_Container13, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container13, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Container13, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container13, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_year = lv_roller_create(ui_Container13);
    lv_roller_set_options(ui_year,
                          "1970\n1971\n1972\n1973\n1974\n1975\n1976\n1977\n1978\n1979\n1980\n1981\n1982\n1983\n1984\n1985\n1986\n1987\n1988\n1989\n1990\n1991\n1992\n1993\n1994\n1995\n1996\n1997\n1998\n1999\n2000\n2001\n2002\n2003\n2004\n2005\n2006\n2007\n2008\n2009\n2010\n2011\n2012\n2013\n2014\n2015\n2016\n2017\n2018\n2019\n2020\n2021\n2022\n2023\n2024\n2025\n2026\n2027\n2028\n2029\n2030\n2031\n2032\n2033\n2034\n2035\n2036\n2037\n2038\n2039\n2040\n2041\n2042\n2043\n2044\n2045\n2046\n2047\n2048\n2049\n2050\n2051\n2052\n2053\n2054\n2055\n2056\n2057\n2058\n2059\n2060\n2061\n2062\n2063\n2064\n2065\n2066\n2067\n2068\n2069\n2070\n2071\n2072\n2073\n2074\n2075\n2076\n2077\n2078\n2079\n2080\n2081\n2082\n2083\n2084\n2085\n2086\n2087\n2088\n2089\n2090\n2091\n2092\n2093\n2094\n2095\n2096\n2097\n2098\n2099\n2100\n2101\n2102\n2103\n2104\n2105\n2106\n2107\n2108\n2109\n2110\n2111\n2112\n2113\n2114\n2115\n2116\n2117\n2118\n2119\n2120\n2121\n2122\n2123\n2124\n2125\n2126\n2127\n2128\n2129\n2130\n2131\n2132\n2133\n2134\n2135\n2136\n2137\n2138\n2139\n2140\n2141\n2142\n2143\n2144\n2145\n2146\n2147\n2148\n2149\n2150\n2151\n2152\n2153\n2154\n2155\n2156\n2157\n2158\n2159\n2160\n2161\n2162\n2163\n2164\n2165\n2166\n2167\n2168\n2169\n2170\n2171\n2172\n2173\n2174\n2175\n2176\n2177\n2178\n2179\n2180\n2181\n2182\n2183\n2184\n2185\n2186\n2187\n2188\n2189\n2190\n2191\n2192\n2193\n2194\n2195\n2196\n2197\n2198\n2199\n2200",
                          LV_ROLLER_MODE_NORMAL);
    lv_roller_set_selected(ui_year, 55, LV_ANIM_OFF);
    lv_obj_set_width(ui_year, 75);
    lv_obj_set_height(ui_year, 100);
    lv_obj_set_x(ui_year, -87);
    lv_obj_set_y(ui_year, 98);
    lv_obj_set_align(ui_year, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_year, lv_color_hex(0x4FABF7), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_year, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_year, lv_color_hex(0x005295), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_year, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_year, 1, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_Label19 = lv_label_create(ui_Container13);
    lv_obj_set_width(ui_Label19, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label19, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label19, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label19, "\n:");
    lv_obj_set_style_text_font(ui_Label19, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_month = lv_roller_create(ui_Container13);
    lv_roller_set_options(ui_month, "1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12", LV_ROLLER_MODE_NORMAL);
    lv_roller_set_selected(ui_month, 5, LV_ANIM_OFF);
    lv_obj_set_width(ui_month, 75);
    lv_obj_set_height(ui_month, 100);
    lv_obj_set_x(ui_month, -91);
    lv_obj_set_y(ui_month, 35);
    lv_obj_set_align(ui_month, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_month, lv_color_hex(0x4FABF7), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_month, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_month, lv_color_hex(0x005295), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_month, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_month, 1, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_Label20 = lv_label_create(ui_Container13);
    lv_obj_set_width(ui_Label20, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label20, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label20, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label20, "\n:");
    lv_obj_set_style_text_font(ui_Label20, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_day = lv_roller_create(ui_Container13);
    lv_roller_set_options(ui_day,
                          "1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31",
                          LV_ROLLER_MODE_NORMAL);
    lv_roller_set_selected(ui_day, 15, LV_ANIM_OFF);
    lv_obj_set_width(ui_day, 75);
    lv_obj_set_height(ui_day, 100);
    lv_obj_set_x(ui_day, -91);
    lv_obj_set_y(ui_day, 35);
    lv_obj_set_align(ui_day, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_day, lv_color_hex(0x4FABF7), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_day, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_day, lv_color_hex(0x005295), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_day, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_day, 1, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_Container14 = lv_obj_create(ui_Container11);
    lv_obj_remove_style_all(ui_Container14);
    lv_obj_set_width(ui_Container14, 360);
    lv_obj_set_height(ui_Container14, 84);
    lv_obj_set_x(ui_Container14, 2);
    lv_obj_set_y(ui_Container14, 146);
    lv_obj_set_align(ui_Container14, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container14, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Container14, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container14, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Button4 = lv_btn_create(ui_Container14);
    lv_obj_set_width(ui_Button4, 100);
    lv_obj_set_height(ui_Button4, 50);
    lv_obj_set_align(ui_Button4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button4, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label21 = lv_label_create(ui_Button4);
    lv_obj_set_width(ui_Label21, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label21, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label21, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label21, "确认");
    lv_obj_set_style_text_font(ui_Label21, &ui_font_chinese16, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_year, ui_event_year, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_month, ui_event_month, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_day, ui_event_day, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button4, ui_event_Button4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_datesetting, ui_event_datesetting, LV_EVENT_ALL, NULL);

}
