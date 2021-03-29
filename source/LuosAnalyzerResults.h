#ifndef LUOS_ANALYZER_RESULTS
#define LUOS_ANALYZER_RESULTS

#include <AnalyzerResults.h>
#include <sstream>
#include <string>


class LuosAnalyzer;
class LuosAnalyzerSettings;

class LuosAnalyzerResults : public AnalyzerResults
{
public:
	LuosAnalyzerResults( LuosAnalyzer* analyzer, LuosAnalyzerSettings* settings );
	virtual ~LuosAnalyzerResults();

	virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
	virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

	virtual void GenerateFrameTabularText(U64 frame_index, DisplayBase display_base );
	virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
	virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

protected: //functions

protected:  //vars
	LuosAnalyzerSettings* mSettings;
	LuosAnalyzer* mAnalyzer;
};




#endif //LUOS_ANALYZER_RESULTS
