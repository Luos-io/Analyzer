#include "LuosAnalyzerSettings.h"
#include <AnalyzerHelpers.h>


LuosAnalyzerSettings::LuosAnalyzerSettings()
:	mTxChannel( UNDEFINED_CHANNEL ),
	mRxChannel( UNDEFINED_CHANNEL ),
	mBitRate( 9600 )
{

	mTxChannelInterface.reset( new AnalyzerSettingInterfaceChannel() );
	mTxChannelInterface->SetTitleAndTooltip( "Tx/One_wire", "Serial Data Line" );
	mTxChannelInterface->SetChannel( mTxChannel );

	mRxChannelInterface.reset( new AnalyzerSettingInterfaceChannel() );
	mRxChannelInterface->SetTitleAndTooltip( "Rx", "Serial Data Line" );
	mRxChannelInterface->SetChannel( mRxChannel );
	mRxChannelInterface->SetSelectionOfNoneIsAllowed( true );

	mBitRateInterface.reset( new AnalyzerSettingInterfaceInteger() );
	mBitRateInterface->SetTitleAndTooltip( "Bit Rate (Bits/S)",  "Specify the bit rate in bits per second." );
	mBitRateInterface->SetMax( 6000000 );
	mBitRateInterface->SetMin( 1 );
	mBitRateInterface->SetInteger( mBitRate );

	AddInterface( mTxChannelInterface.get() );
	AddInterface( mRxChannelInterface.get() );
	AddInterface( mBitRateInterface.get() );

	AddExportOption( 0, "Export as text/csv file" );
	AddExportExtension( 0, "text", "txt" );
	AddExportExtension( 0, "csv", "csv" );

	ClearChannels();
	AddChannel( mTxChannel, "Tx/One_wire", false );
	AddChannel( mRxChannel, "Rx", false );
}

LuosAnalyzerSettings::~LuosAnalyzerSettings()
{
}

bool LuosAnalyzerSettings::SetSettingsFromInterfaces()
{
	mTxChannel = mTxChannelInterface->GetChannel();
	mRxChannel = mRxChannelInterface->GetChannel();

	//////////////add a comparison if the same input channels
	mBitRate = mBitRateInterface->GetInteger();

	ClearChannels();
	AddChannel( mTxChannel, "Tx/One_wire", true );
	AddChannel( mRxChannel, "Rx", true );

	return true;
}

void LuosAnalyzerSettings::UpdateInterfacesFromSettings()
{
	mTxChannelInterface->SetChannel( mTxChannel );
	mRxChannelInterface->SetChannel( mRxChannel );
	mBitRateInterface->SetInteger( mBitRate );
}

void LuosAnalyzerSettings::LoadSettings( const char* settings )
{
	SimpleArchive text_archive;
	text_archive.SetString( settings );

	text_archive >> mTxChannel;
	text_archive >> mRxChannel;
	text_archive >> mBitRate;

	ClearChannels();
	AddChannel( mTxChannel, "Tx/One_wire", true );
	AddChannel( mRxChannel, "Rx", true );


	UpdateInterfacesFromSettings();
}

const char* LuosAnalyzerSettings::SaveSettings()
{
	SimpleArchive text_archive;

	text_archive << mTxChannel;
	text_archive << mRxChannel;
	text_archive << mBitRate;

	return SetReturnString( text_archive.GetString() );
}
