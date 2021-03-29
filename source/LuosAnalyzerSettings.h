#ifndef LUOS_ANALYZER_SETTINGS
#define LUOS_ANALYZER_SETTINGS

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

class LuosAnalyzerSettings : public AnalyzerSettings
{
public:
	LuosAnalyzerSettings();
	virtual ~LuosAnalyzerSettings();

	virtual bool SetSettingsFromInterfaces();
	void UpdateInterfacesFromSettings();
	virtual void LoadSettings( const char* settings );
	virtual const char* SaveSettings();

	Channel mRxChannel;
	Channel mTxChannel;

	U32 mBitRate;

protected:
	std::auto_ptr< AnalyzerSettingInterfaceChannel >	mRxChannelInterface;
	std::auto_ptr< AnalyzerSettingInterfaceChannel >	mTxChannelInterface;
	std::auto_ptr< AnalyzerSettingInterfaceInteger >	mBitRateInterface;
};

#endif //LUOS_ANALYZER_SETTINGS
