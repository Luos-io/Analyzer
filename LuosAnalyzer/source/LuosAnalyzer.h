#ifndef LUOS_ANALYZER_H
#define LUOS_ANALYZER_H

#include <Analyzer.h>
#include "LuosAnalyzerResults.h"
#include "LuosSimulationDataGenerator.h"
#include <stdint.h>

class LuosAnalyzerSettings;
class ANALYZER_EXPORT LuosAnalyzer : public Analyzer2
{
public:
	LuosAnalyzer();
	virtual ~LuosAnalyzer();

	virtual void SetupResults();
	virtual void WorkerThread();

	virtual U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );
	virtual U32 GetMinimumSampleRateHz();

	virtual const char* GetAnalyzerName() const;
	virtual bool NeedsRerun();

protected: //vars
	std::auto_ptr< LuosAnalyzerSettings > mSettings;
	std::auto_ptr< LuosAnalyzerResults > mResults;
	AnalyzerChannelData* mTx;
	AnalyzerChannelData* mRx;

	LuosSimulationDataGenerator mSimulationDataGenerator;
	bool mSimulationInitilized;

	//Serial analysis vars:
	U32 mSampleRateHz;
	U32 mStartOfStopBitOffset;
	U32 mEndOfStopBitOffset;
};

//void One_Wire_Config();

void ComputeCRC (uint8_t data);

typedef enum { PROTOCOL,
							 TARGET,
						 	 TARGET_MODE,
						   SOURCE,
						   CMD,
						   SIZE,
							 DATA,
						 	 CRC,
						   ACK,
						 	 WAIT };


extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif //LUOS_ANALYZER_H
