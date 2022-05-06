#ifndef OFFSCREENVIEW_HPP
#define OFFSCREENVIEW_HPP

#include <gui_generated/offscreen_screen/OffScreenViewBase.hpp>
#include <gui/offscreen_screen/OffScreenPresenter.hpp>

class OffScreenView : public OffScreenViewBase
{
public:
    OffScreenView();
    virtual ~OffScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void btnScreenOnClick();

protected:
};

#endif // OFFSCREENVIEW_HPP
