#include <gui/settingsscreen_screen/SettingsScreenView.hpp>

uint8_t mUptimeCounter;
//Gui Refresh
int mTickGUICount;

#ifndef SIMULATOR
extern "C"
{ 
	uint8_t GetUptimeCount();
}

#else

//SIMULATED
uint8_t SettingsScreenView::GetUptimeCount()
{
	mUptimeCounter++;
	return mUptimeCounter;
}

#endif



SettingsScreenView::SettingsScreenView()
{

}

void SettingsScreenView::setupScreen()
{
    SettingsScreenViewBase::setupScreen();
}

void SettingsScreenView::tearDownScreen()
{
    SettingsScreenViewBase::tearDownScreen();
}

void SettingsScreenView::handleTickEvent()
{
	mTickGUICount++;
	//Wait for 0.5sec
	if (mTickGUICount % 30 == 0)
	{
		uint8_t uptime = GetUptimeCount();

		Unicode::snprintf(lblKarunaUptimeBuffer , 4, "%d", uptime); 
		lblKarunaUptime.invalidate();
	}
}
