/* DO NOT EDIT THIS FILE */
/* This file is autogenerated by the text-database code generator */

#ifndef TOUCHGFX_APPLICATIONFONTPROVIDER_HPP
#define TOUCHGFX_APPLICATIONFONTPROVIDER_HPP

#include <touchgfx/FontManager.hpp>

namespace touchgfx
{
class FlashDataReader;
}

struct Typography
{
    static const touchgfx::FontId DEFAULT = 0;
    static const touchgfx::FontId DESC = 1;
    static const touchgfx::FontId VALUE = 2;
    static const touchgfx::FontId SETTINGS = 3;
    static const touchgfx::FontId TYPOGRAPHY_00 = 4;
};

struct TypographyFontIndex
{
    static const touchgfx::FontId DEFAULT = 0;       // Asap_Regular_40_4bpp
    static const touchgfx::FontId DESC = 1;          // micross_25_4bpp
    static const touchgfx::FontId VALUE = 2;         // micross_65_4bpp
    static const touchgfx::FontId SETTINGS = 3;      // corbell_30_4bpp
    static const touchgfx::FontId TYPOGRAPHY_00 = 4; // consola_70_4bpp
    static const uint16_t NUMBER_OF_FONTS = 5;
};

class ApplicationFontProvider : public touchgfx::FontProvider
{
public:
    virtual touchgfx::Font* getFont(touchgfx::FontId typography);

    static void setFlashReader(touchgfx::FlashDataReader* /* flashReader */)
    {
    }
    static touchgfx::FlashDataReader* getFlashReader()
    {
        return 0;
    }
};

#endif // TOUCHGFX_APPLICATIONFONTPROVIDER_HPP
