#ifndef LUOS_SIMULATION_DATA_GENERATOR
#define LUOS_SIMULATION_DATA_GENERATOR

#include <SimulationChannelDescriptor.h>
#include <string>
class LuosAnalyzerSettings;

class LuosSimulationDataGenerator
{
public:
	LuosSimulationDataGenerator();
	~LuosSimulationDataGenerator();

	void Initialize( U32 simulation_sample_rate, LuosAnalyzerSettings* settings );
	U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel );

protected:
	LuosAnalyzerSettings* mSettings;
	U32 mSimulationSampleRateHz;

protected:
	void CreateSerialByte();
	std::string mSerialText;
	U32 mStringIndex;

	SimulationChannelDescriptor mSerialSimulationData;

};
#endif //LUOS_SIMULATION_DATA_GENERATOR
