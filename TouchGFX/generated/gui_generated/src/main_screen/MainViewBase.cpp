/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/main_screen/MainViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

MainViewBase::MainViewBase() :
    buttonCallback(this, &MainViewBase::buttonCallbackHandler)
{

    __background.setPosition(0, 0, 1024, 600);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));

    backgroundBox.setPosition(0, 0, 1024, 600);
    backgroundBox.setVisible(false);
    backgroundBox.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));

    counterBackgroundImage.setXY(487, 28);
    counterBackgroundImage.setBitmap(touchgfx::Bitmap(BITMAP_COUNTER_BOX_ID));

    countTxt.setPosition(487, 50, 152, 89);
    countTxt.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    countTxt.setLinespacing(0);
    Unicode::snprintf(countTxtBuffer, COUNTTXT_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_J4RQ).getText());
    countTxt.setWildcard(countTxtBuffer);
    countTxt.setTypedText(touchgfx::TypedText(T_TEXTID1));

    buttonDown.setXY(868, 506);
    buttonDown.setBitmaps(touchgfx::Bitmap(BITMAP_DOWN_BTN_ID), touchgfx::Bitmap(BITMAP_DOWN_BTN_PRESSED_ID));
    buttonDown.setAction(buttonCallback);

    buttonUp.setXY(133, 506);
    buttonUp.setBitmaps(touchgfx::Bitmap(BITMAP_UP_BTN_ID), touchgfx::Bitmap(BITMAP_UP_BTN_PRESSED_ID));
    buttonUp.setAction(buttonCallback);

    imageProgress2.setXY(361, 209);
    imageProgress2.setProgressIndicatorPosition(2, 2, 400, 30);
    imageProgress2.setRange(0, 100);
    imageProgress2.setDirection(touchgfx::AbstractDirectionProgress::RIGHT);
    imageProgress2.setBackground(touchgfx::Bitmap(BITMAP_BLUE_PROGRESSINDICATORS_BG_LARGE_PROGRESS_INDICATOR_BG_SQUARE_0_DEGREES_ID));
    imageProgress2.setBitmap(BITMAP_BLUE_PROGRESSINDICATORS_FILL_TILING_PROGRESS_INDICATOR_FILL_STRIPED_WIDE_HORIZONTAL_ID);
    imageProgress2.setValue(60);
    imageProgress2.setAnchorAtZero(false);

    button1.setXY(104, 152);
    button1.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_PRESSED_ID));

    add(__background);
    add(backgroundBox);
    add(counterBackgroundImage);
    add(countTxt);
    add(buttonDown);
    add(buttonUp);
    add(imageProgress2);
    add(button1);
}

void MainViewBase::setupScreen()
{

}

void MainViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &buttonDown)
    {
        //DecreaseValue
        //When buttonDown clicked call virtual function
        //Call decreaseValue
        decreaseValue();
    }
    else if (&src == &buttonUp)
    {
        //IncreaseValue
        //When buttonUp clicked call virtual function
        //Call increaseValue
        increaseValue();
    }
}
