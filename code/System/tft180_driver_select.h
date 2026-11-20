#ifndef TFT180_DRIVER_SELECT_H
#define TFT180_DRIVER_SELECT_H

#include "System/usr_system.h"
#include "zf_device_tft180.h"

#if USE_NEW_TFT180
#include "zf_device_tft180_new.h"

#define tft180_width_max             tft180_new_width_max
#define tft180_height_max            tft180_new_height_max

#define tft180_clear                 tft180_new_clear
#define tft180_full                  tft180_new_full
#define tft180_set_dir               tft180_new_set_dir
#define tft180_set_font              tft180_new_set_font
#define tft180_set_color             tft180_new_set_color
#define tft180_draw_point            tft180_new_draw_point
#define tft180_draw_line             tft180_new_draw_line

#define tft180_show_char             tft180_new_show_char
#define tft180_show_string           tft180_new_show_string
#define tft180_show_int              tft180_new_show_int
#define tft180_show_uint             tft180_new_show_uint
#define tft180_show_float            tft180_new_show_float

#define tft180_show_binary_image     tft180_new_show_binary_image
#define tft180_show_gray_image       tft180_new_show_gray_image
#define tft180_show_rgb565_image     tft180_new_show_rgb565_image

#define tft180_show_wave             tft180_new_show_wave
#define tft180_show_chinese          tft180_new_show_chinese
#define tft180_init                  tft180_new_init

#undef tft180_displayimage7725
#undef tft180_displayimage03x
#undef tft180_displayimage8660
#define tft180_displayimage7725      tft180_new_displayimage7725
#define tft180_displayimage03x       tft180_new_displayimage03x
#define tft180_displayimage8660      tft180_new_displayimage8660

#endif

#endif
