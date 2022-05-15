#ifndef MAIN_VIEW_HPP
#define MAIN_VIEW_HPP

#include <gui_generated/main_screen/MainViewBase.hpp>
#include <gui/main_screen/MainPresenter.hpp>

class MainView : public MainViewBase
{
public:
    MainView();
    void SetOnAllOutput();
    virtual ~MainView() {}
    virtual void setupScreen();
    virtual void ToggleHDMI();
    virtual  void ToggleRCA();
    virtual  void ToggleBNC();
    virtual  void ToggleXLR();

    virtual void handleTickEvent();

    void RefreshAudioAndClockInfo(); 

    void RefreshBNCOutput();
    void RefreshRCAOutput();
    void RefreshHDMIOutput();
    void RefreshXLROutput();

    void Refresh24Thermal();
    void Refresh245Thermal();
    void Refresh22Thermal();
    void RefreshIntExt();

    void SetDSDPCM(int p_AudiFormat);
    void SetBitDepth(int p_AudiFormat);
    void SetFreq(int p_AudiFormat);

    void SetTemp(int p_Temp);
    void PaintDot(colortype p_Dot1, colortype p_Dot2, colortype p_Dot3);

    colortype GetOutputColor(bool p_State);
    colortype GetThermalColor(bool p_State);
    bool ToBinary(int number, int p_Position);
    void CopyBit(int input, int* output, int CopyFrom, int CopyTo);
    void SetBit(uint8_t* input, bool bit, int SetTo);

    virtual void OpenScreenoff();
    virtual void ShowDipslay();

#ifdef SIMULATOR
    uint8_t GuiItfGetKarunaStatus();
    void GuiItfKarunaControl(uint8_t p_Output);
#endif
       

protected:

private:
    uint8_t count;
};

#endif // MAIN_VIEW_HPP
