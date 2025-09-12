#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;
uint32_t active_theme_index = 0;

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 320, 240);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    {
        lv_obj_t *parent_obj = obj;
        {
            // RPM_GAUGE
            lv_obj_t *obj = lv_arc_create(parent_obj);
            objects.rpm_gauge = obj;
            lv_obj_set_pos(obj, 79, 39);
            lv_obj_set_size(obj, 162, 162);
            lv_arc_set_range(obj, 0, 200);
            lv_arc_set_value(obj, 180);
            lv_obj_set_style_arc_width(obj, 8, LV_PART_INDICATOR | LV_STATE_DEFAULT);
            lv_obj_set_style_arc_rounded(obj, false, LV_PART_INDICATOR | LV_STATE_DEFAULT);
            lv_obj_set_style_arc_color(obj, lv_color_hex(0xff47bad3), LV_PART_INDICATOR | LV_STATE_DEFAULT);
            lv_obj_set_style_arc_opa(obj, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
            lv_obj_set_style_arc_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // CURRENT_GAUGE
            lv_obj_t *obj = lv_arc_create(parent_obj);
            objects.current_gauge = obj;
            lv_obj_set_pos(obj, 87, 47);
            lv_obj_set_size(obj, 146, 146);
            lv_arc_set_range(obj, 0, 8000);
            lv_arc_set_value(obj, 7600);
            lv_obj_set_style_arc_width(obj, 5, LV_PART_INDICATOR | LV_STATE_DEFAULT);
            lv_obj_set_style_arc_rounded(obj, false, LV_PART_INDICATOR | LV_STATE_DEFAULT);
            lv_obj_set_style_arc_color(obj, lv_color_hex(0xffde5858), LV_PART_INDICATOR | LV_STATE_DEFAULT);
            lv_obj_set_style_arc_opa(obj, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
            lv_obj_set_style_arc_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // RPM_SCALE
            lv_obj_t *obj = lv_scale_create(parent_obj);
            objects.rpm_scale = obj;
            lv_obj_set_pos(obj, 85, 45);
            lv_obj_set_size(obj, 150, 150);
            lv_scale_set_mode(obj, LV_SCALE_MODE_ROUND_OUTER);
            lv_scale_set_range(obj, 0, 200);
            lv_scale_set_total_tick_count(obj, 17);
            lv_scale_set_major_tick_every(obj, 2);
            lv_scale_set_label_show(obj, true);
        }
        {
            // CURRENT_SCALE
            lv_obj_t *obj = lv_scale_create(parent_obj);
            objects.current_scale = obj;
            lv_obj_set_pos(obj, 85, 45);
            lv_obj_set_size(obj, 150, 150);
            lv_scale_set_mode(obj, LV_SCALE_MODE_ROUND_INNER);
            lv_scale_set_range(obj, 0, 8);
            lv_scale_set_total_tick_count(obj, 9);
            lv_scale_set_major_tick_every(obj, 1);
            lv_scale_set_label_show(obj, true);
            lv_obj_set_style_line_rounded(obj, false, LV_PART_INDICATOR | LV_STATE_DEFAULT);
            lv_obj_set_style_line_opa(obj, 150, LV_PART_INDICATOR | LV_STATE_DEFAULT);
        }
        {
            // mode_display
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.mode_display = obj;
            lv_obj_set_pos(obj, 0, 210);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_long_mode(obj, LV_LABEL_LONG_DOT);
            lv_obj_set_style_align(obj, LV_ALIGN_TOP_MID, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text(obj, "Mode");
        }
        {
            // temperature_display
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.temperature_display = obj;
            lv_obj_set_pos(obj, -7, -108);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_align(obj, LV_ALIGN_RIGHT_MID, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text(obj, "Temp");
        }
        {
            // fault_display
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.fault_display = obj;
            lv_obj_set_pos(obj, 138, 0);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_align(obj, LV_ALIGN_LEFT_MID, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xffff0000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text(obj, "FAULT");
        }
        {
            // position_display
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.position_display = obj;
            lv_obj_set_pos(obj, -123, 4);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_long_mode(obj, LV_LABEL_LONG_DOT);
            lv_obj_set_style_align(obj, LV_ALIGN_TOP_MID, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text(obj, "Position");
        }
    }
    
    tick_screen_main();
}

void tick_screen_main() {
}



typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

void create_screens() {
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_main();
}
