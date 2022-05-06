#ifndef DISPLAYSCREENVIEW_HPP
#define DISPLAYSCREENVIEW_HPP

#include <gui_generated/displayscreen_screen/DisplayScreenViewBase.hpp>
#include <gui/displayscreen_screen/DisplayScreenPresenter.hpp>

class DisplayScreenView : public DisplayScreenViewBase
{
public:
    DisplayScreenView();
    virtual ~DisplayScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // DISPLAYSCREENVIEW_HPP
