#include "LuosAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "LuosAnalyzer.h"
#include "LuosAnalyzerSettings.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <string>


std::stringstream ss;

LuosAnalyzerResults::LuosAnalyzerResults( LuosAnalyzer* analyzer, LuosAnalyzerSettings* settings )
:	AnalyzerResults(),
	mSettings( settings ),
	mAnalyzer( analyzer )
{
}

LuosAnalyzerResults::~LuosAnalyzerResults()
{
}

void LuosAnalyzerResults::GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base )
{
	ClearResultStrings();

	Frame frame = GetFrame( frame_index );

	DataTranslation(frame.mData1, frame.mData2, display_base);

	AddResultString( ss.str().c_str() );
}

void LuosAnalyzerResults::GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id )
{
	std::ofstream file_stream( file, std::ios::out );

	U64 trigger_sample = mAnalyzer->GetTriggerSample();
	U32 sample_rate = mAnalyzer->GetSampleRate();

	file_stream << "Time [s],Value" << std::endl;

	U64 num_frames = GetNumFrames();
	for( U32 i=0; i < num_frames; i++ )
	{
		Frame frame = GetFrame( i );

		char time_str[128];
		AnalyzerHelpers::GetTimeString( frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128 );
		if (i==0) {
			char number_str[128];
			AnalyzerHelpers::GetNumberString( frame.mData1, ASCII, 8, number_str, 128 );

			file_stream << time_str << "," << number_str << std::endl;

		}
		else {
			char number_str[128];
			AnalyzerHelpers::GetNumberString( frame.mData1, ASCII, 8, number_str, 128 );

			file_stream << time_str << "," << number_str << std::endl;

		}

		if( UpdateExportProgressAndCheckForCancel( i, num_frames ) == true )
		{
			file_stream.close();
			return;
		}
	}

	file_stream.close();
}

void LuosAnalyzerResults::GenerateFrameTabularText( U64 frame_index, DisplayBase display_base )
{
#ifdef SUPPORTS_PROTOCOL_SEARCH
	Frame frame = GetFrame( frame_index );

	ClearTabularText();

	DataTranslation(frame.mData1, frame.mData2, display_base);

	AddTabularText( ss.str().c_str() );

#endif
}


void DataTranslation( U64 frame_data1, U64 frame_data2,  DisplayBase display_base) {

	ss.str(std::string());
	char number_str[128];
	char number_str2[128];
	U32 bit_num=8;

	AnalyzerHelpers::GetNumberString( frame_data1, ASCII, 8, number_str, 128 );

	switch (frame_data1) {
		case 'PROT': {
			bit_num = 4;
			strcpy(number_str, "PROTOCOL");
			break;
		}
		case 'TRGT': {
			bit_num = 12;
			strcpy(number_str, "TARGET");
			break;
		}
		case 'MODE': {
			bit_num = 4;
			strcpy(number_str, "TARGET MODE");
			break;
		}
		case 'SRC': {
			bit_num = 12;
			strcpy(number_str, "SOURCE");
			break;
		}
		case 'CMD': {
			bit_num = 8;
			strcpy(number_str, "CMD");
			break;
		}
		case 'SIZE': {
			bit_num = 16;
			strcpy(number_str, "SIZE");
			break;
		}
		case 'CRC': {
			bit_num = 16;
			strcpy(number_str, "CRC");
			break;
		}
		case 'NOT': {
			bit_num = 16;
			strcpy(number_str, "CRC");
			break;
		}
		default:{
			bit_num = 8;
			strcpy(number_str, "DATA");
			break;
		}
	}

	if (frame_data1 == 'SIZE') {	AnalyzerHelpers::GetNumberString( frame_data2, Decimal, bit_num, number_str2, 128 );	}
	else {	AnalyzerHelpers::GetNumberString( frame_data2, display_base, bit_num, number_str2, 128 ); }


	if (frame_data1 == 'ACK') {
		switch(frame_data2) {
			case 0x0F: {
				ss << "ACK = " << number_str2;
				break;
			}
			case 0x1F: {
				ss << "NAK = " << number_str2;
				break;
			}
			case 0x2F: {
				ss << "ACK TIMEOUT = " << number_str2;
				break;
			}
			case 0x3F: {
				ss << "ACK FRAMING ERROR = " << number_str2;
				break;
			}
		}
	}
	else if (frame_data1 == 'MODE') {
		switch (frame_data2) {
			case 0: {
				ss << number_str << " = " << number_str2 << " ID";
				break;
			}
			case 1: {
				ss << number_str << " = " << number_str2 << " IDACK";
				break;
			}
			case 2: {
				ss << number_str << " = " << number_str2 << " TYPE";
				break;
			}
			case 3: {
				ss << number_str << " = " << number_str2 << " BROADCAST";
				break;
			}
			case 4: {
				ss << number_str << " = " << number_str2 << " MULTICAST";
				break;
			}
			case 5: {
				ss << number_str << " = " << number_str2 << " NODEID";
				break;
			}
			case 6: {
				ss << number_str << " = " << number_str2 << "  NODEIDACK";
				break;
			}
		}
	}
	else if (frame_data1 == 'CMD') {
		switch (frame_data2) {
			case 0: {
				ss << number_str << " = " << number_str2 << " WRITE_NODE_ID";
				break;
			}
			case 1: {
				ss << number_str << " = " << number_str2 << " RESET_DETECTION";
				break;
			}
			case 2: {
				ss << number_str << " = " << number_str2 << " SET_BAUDRATE";
				break;
			}
			case 3: {
				ss << number_str << " = " << number_str2 << " ASSERT";
				break;
			}
			case 4: {
				ss << number_str << " = " << number_str2 << " RTB_CMD";
				break;
			}
			case 5: {
				ss << number_str << " = " << number_str2 << " WRITE_ALIAS";
				break;
			}
			case 6: {
				ss << number_str << " = " << number_str2 << " UPDATE_PUB";
				break;
			}
			case 7: {
				ss << number_str << " = " << number_str2 << " NODE_UUID";
				break;
			}
			case 8: {
				ss << number_str << " = " << number_str2 << " REVISION";
				break;
			}
			case 9: {
				ss << number_str << " = " << number_str2 << " LUOS_REVISION";
				break;
			}
			case 10: {
				ss << number_str << " = " << number_str2 << " LUOS_STATISTICS";
				break;
			}
			case 11: {
				ss << number_str << " = " << number_str2 << " ASK_PUB_CMD";
				break;
			}
			case 12: {
				ss << number_str << " = " << number_str2 << " COLOR";
				break;
			}
			case 13: {
				ss << number_str << " = " << number_str2 << " COMPLIANT";
				break;
			}
			case 14: {
				ss << number_str << " = " << number_str2 << " IO_STATE";
				break;
			}
			case 15: {
				ss << number_str << " = " << number_str2 << " RATIO";
				break;
			}
			case 16: {
				ss << number_str << " = " << number_str2 << " PEDOMETER";
				break;
			}
			case 17: {
				ss << number_str << " = " << number_str2 << " ILLUMINANCE";
				break;
			}
			case 18: {
				ss << number_str << " = " << number_str2 << " VOLTAGE";
				break;
			}
			case 19: {
				ss << number_str << " = " << number_str2 << " CURRENT";
				break;
			}
			case 20: {
				ss << number_str << " = " << number_str2 << " POWER";
				break;
			}
			case 21: {
				ss << number_str << " = " << number_str2 << " TEMPERATURE";
				break;
			}
			case 22: {
				ss << number_str << " = " << number_str2 << " TIME";
				break;
			}
			case 23: {
				ss << number_str << " = " << number_str2 << " FORCE";
				break;
			}
			case 24: {
				ss << number_str << " = " << number_str2 << " MOMENT";
				break;
			}
			case 25: {
				ss << number_str << " = " << number_str2 << " CONTROL";
				break;
			}
			case 26: {
				ss << number_str << " = " << number_str2 << " REGISTER";
				break;
			}
			case 27: {
				ss << number_str << " = " << number_str2 << " REINIT";
				break;
			}
			case 28: {
				ss << number_str << " = " << number_str2 << " PID";
				break;
			}
			case 29: {
				ss << number_str << " = " << number_str2 << " RESOLUTION";
				break;
			}
			case 30: {
				ss << number_str << " = " << number_str2 << " REDUCTION";
				break;
			}
			case 31: {
				ss << number_str << " = " << number_str2 << " DIMENSION";
				break;
			}
			case 32: {
				ss << number_str << " = " << number_str2 << " OFFSET";
				break;
			}
			case 33: {
				ss << number_str << " = " << number_str2 << " SETID";
				break;
			}
			case 34: {
				ss << number_str << " = " << number_str2 << " ANGULAR_POSITION";
				break;
			}
			case 35: {
				ss << number_str << " = " << number_str2 << " ANGULAR_SPEED";
				break;
			}
			case 36: {
				ss << number_str << " = " << number_str2 << " LINEAR_POSITION";
				break;
			}
			case 37: {
				ss << number_str << " = " << number_str2 << " LINEAR_SPEED";
				break;
			}
			case 38: {
				ss << number_str << " = " << number_str2 << " ACCEL_3D";
				break;
			}
			case 39: {
				ss << number_str << " = " << number_str2 << " GYRO_3D";
				break;
			}
			case 40: {
				ss << number_str << " = " << number_str2 << " QUATERNION";
				break;
			}
			case 41: {
				ss << number_str << " = " << number_str2 << " COMPASS_3D";
				break;
			}
			case 42: {
				ss << number_str << " = " << number_str2 << " EULER_3D";
				break;
			}
			case 43: {
				ss << number_str << " = " << number_str2 << " ROT_MAT";
				break;
			}
			case 44: {
				ss << number_str << " = " << number_str2 << " LINEAR_ACCEL";
				break;
			}
			case 45: {
				ss << number_str << " = " << number_str2 << " GRAVITY_VECTOR";
				break;
			}
			case 46: {
				ss << number_str << " = " << number_str2 << " HEADING";
				break;
			}
			case 47: {
				ss << number_str << " = " << number_str2 << " ANGULAR_POSITION_LIMIT";
				break;
			}
			case 48: {
				ss << number_str << " = " << number_str2 << " LINEAR_POSITION_LIMIT";
				break;
			}
			case 49: {
				ss << number_str << " = " << number_str2 << " RATIO_LIMIT";
				break;
			}
			case 50: {
				ss << number_str << " = " << number_str2 << " CURRENT_LIMIT";
				break;
			}
			case 51: {
				ss << number_str << " = " << number_str2 << " ANGULAR_SPEED_LIMIT";
				break;
			}
			case 52: {
				ss << number_str << " = " << number_str2 << " LINEAR_SPEED_LIMIT";
				break;
			}
			case 53: {
				ss << number_str << " = " << number_str2 << " TORQUE_LIMIT";
				break;
			}
			case 54: {
				ss << number_str << " = " << number_str2 << " DXL_WHEELMODE";
				break;
			}
			case 55: {
				ss << number_str << " = " << number_str2 << " HANDY_SET_POSITION";
				break;
			}
			case 56: {
				ss << number_str << " = " << number_str2 << " PARAMETERS";
				break;
			}
			case 57: {
				ss << number_str << " = " << number_str2 << " LUOS_PROTOCOL_NB";
				break;
			}
		}
	}
	else if (frame_data1 == 'NOT') {
		ss << number_str << " = " << number_str2 << " - NOT GOOD";
	}
	else if (frame_data1 == 'CRC') {
		ss <<number_str << " = " << number_str2 << " - GOOD";
	}
	else if (strcmp(number_str, "DATA")==0) {
		ss << number_str << "[" << frame_data1 << "] = " << number_str2;
	}
	else {
		ss << number_str << " = " << number_str2;
	}
}

void LuosAnalyzerResults::GeneratePacketTabularText( U64 packet_id, DisplayBase display_base )
{
	//not supported

}

void LuosAnalyzerResults::GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base )
{
	//not supported
}
