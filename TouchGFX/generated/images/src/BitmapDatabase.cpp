// 4.18.0 0x6644bfa1
// Generated by imageconverter. Please, do not edit!

#include <BitmapDatabase.hpp>
#include <touchgfx/Bitmap.hpp>

extern const unsigned char image_bg[]; // BITMAP_BG_ID = 0, Size: 480x272 pixels
extern const unsigned char image_blue_progressindicators_bg_large_progress_indicator_bg_square_0_degrees[]; // BITMAP_BLUE_PROGRESSINDICATORS_BG_LARGE_PROGRESS_INDICATOR_BG_SQUARE_0_DEGREES_ID = 1, Size: 404x34 pixels
extern const unsigned char image_blue_progressindicators_fill_tiling_progress_indicator_fill_striped_wide_horizontal[]; // BITMAP_BLUE_PROGRESSINDICATORS_FILL_TILING_PROGRESS_INDICATOR_FILL_STRIPED_WIDE_HORIZONTAL_ID = 2, Size: 40x40 pixels
extern const unsigned char image_ca_logo_480x72[]; // BITMAP_CA_LOGO_480X72_ID = 3, Size: 480x79 pixels
extern const unsigned char image_counter_box[]; // BITMAP_COUNTER_BOX_ID = 4, Size: 152x154 pixels
extern const unsigned char image_down_btn[]; // BITMAP_DOWN_BTN_ID = 5, Size: 130x56 pixels
extern const unsigned char image_down_btn_disabled[]; // BITMAP_DOWN_BTN_DISABLED_ID = 6, Size: 130x56 pixels
extern const unsigned char image_down_btn_pressed[]; // BITMAP_DOWN_BTN_PRESSED_ID = 7, Size: 130x56 pixels
extern const unsigned char image_up_btn[]; // BITMAP_UP_BTN_ID = 8, Size: 130x56 pixels
extern const unsigned char image_up_btn_disabled[]; // BITMAP_UP_BTN_DISABLED_ID = 9, Size: 130x56 pixels
extern const unsigned char image_up_btn_pressed[]; // BITMAP_UP_BTN_PRESSED_ID = 10, Size: 130x56 pixels

const touchgfx::Bitmap::BitmapData bitmap_database[] = {
    { image_bg, 0, 480, 272, 0, 0, 480, ((uint8_t)touchgfx::Bitmap::RGB565) >> 3, 272, ((uint8_t)touchgfx::Bitmap::RGB565) & 0x7 },
    { image_blue_progressindicators_bg_large_progress_indicator_bg_square_0_degrees, 0, 404, 34, 0, 0, 404, ((uint8_t)touchgfx::Bitmap::RGB565) >> 3, 34, ((uint8_t)touchgfx::Bitmap::RGB565) & 0x7 },
    { image_blue_progressindicators_fill_tiling_progress_indicator_fill_striped_wide_horizontal, 0, 40, 40, 30, 20, 10, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 20, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_ca_logo_480x72, 0, 480, 79, 0, 0, 480, ((uint8_t)touchgfx::Bitmap::RGB565) >> 3, 79, ((uint8_t)touchgfx::Bitmap::RGB565) & 0x7 },
    { image_counter_box, 0, 152, 154, 0, 0, 152, ((uint8_t)touchgfx::Bitmap::RGB565) >> 3, 154, ((uint8_t)touchgfx::Bitmap::RGB565) & 0x7 },
    { image_down_btn, 0, 130, 56, 3, 0, 124, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 55, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_down_btn_disabled, 0, 130, 56, 3, 0, 124, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 55, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_down_btn_pressed, 0, 130, 56, 3, 0, 124, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 55, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_up_btn, 0, 130, 56, 3, 0, 124, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 55, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_up_btn_disabled, 0, 130, 56, 3, 0, 124, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 55, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_up_btn_pressed, 0, 130, 56, 3, 0, 124, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 55, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 }
};

namespace BitmapDatabase
{
const touchgfx::Bitmap::BitmapData* getInstance()
{
    return bitmap_database;
}

uint16_t getInstanceSize()
{
    return (uint16_t)(sizeof(bitmap_database) / sizeof(touchgfx::Bitmap::BitmapData));
}
} // namespace BitmapDatabase
