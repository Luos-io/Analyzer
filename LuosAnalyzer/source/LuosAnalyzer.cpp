#include "LuosAnalyzer.h"
#include "LuosAnalyzerSettings.h"
#include <AnalyzerChannelData.h>
#include <stdlib.h>
#include <stdint.h>

LuosAnalyzer::LuosAnalyzer()
	: Analyzer2(),
	mSettings(new LuosAnalyzerSettings()),
	mSimulationInitilized(false)
{
	SetAnalyzerSettings(mSettings.get());
}


U16 crc_val;
U8 ONE_WIRE;

LuosAnalyzer::~LuosAnalyzer()
{
	KillThread();
}

void LuosAnalyzer::SetupResults()
{
	mResults.reset(new LuosAnalyzerResults(this, mSettings.get()));
	SetAnalyzerResults(mResults.get());
	mResults->AddChannelBubblesWillAppearOn(mSettings->mTxChannel);

	//if user does not define Rx, we have one_wire config
	if (mSettings->mRxChannel == UNDEFINED_CHANNEL)
		ONE_WIRE = 1;
	else
	{
		ONE_WIRE = 0;
		mResults->AddChannelBubblesWillAppearOn(mSettings->mRxChannel);
	}

}

void LuosAnalyzer::WorkerThread()
{


	mSampleRateHz = GetSampleRate();
	mTx = GetAnalyzerChannelData(mSettings->mTxChannel);

	if (mTx->GetBitState() == BIT_LOW)
		mTx->AdvanceToNextEdge();

	


	U32 samples_per_bit = mSampleRateHz / (mSettings->mBitRate);
	U32 samples_to_first_center_of_first_data_bit = U32(1.5 * double(mSampleRateHz) / double(mSettings->mBitRate)); //advance 1.5 bit
	U16 bit_counter = 0;    //counter from 0 to 8 - detects the end of a data byte
	U8 data_byte = 0;       //data_byte entered to crc function
	U16 size = 0, data_idx = 0, target = 0, source = 0;
	bool ack = 0, Rx_msg = 0, collision_detection = 0, first_byte = 0, noop = 0;    //bool signals
	U64 tracking = 0;           //keeps the start and the end of ack timeout period
	U32 state = 0;							//initialization of state machine
	//Initial Position of the first bit -- falling edge -- beginning of the start bit
	mTx->AdvanceToNextEdge();
	U32 timeout = 20 * (1000000 / mSettings->mBitRate); //Timeout = 2*10*(1sec/baudrate)

	//used for aknowledgement time tracking


	//Process for one_wire config
	if (ONE_WIRE)
	{
		for ( ; ; )
		{
			CheckIfThreadShouldExit();      //kill thread in case of infinite loop
			U64 label = 0, data = 0;													//frames' info
			U64 value = 0, value_byte = 0;								//data & crc calculation helpers
			U8 dd = 0;																			//
			U64 starting_sample = mTx->GetSampleNumber();	//points to the beginning of a frame
			bool transmission_error = 0, noop = 0;						//reset and collision notifiers

			//state machine for the transmission-reception of a message
			switch (state) {
			case PROTOCOL:
			{
                //reset values - beginning of a new msg
                data_byte = 0;
                data_idx = 0;
                crc_val = 0xFFFF;     //initial value of crc
                
				mTx->Advance(samples_to_first_center_of_first_data_bit);    //samples to the center of the first bit
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);    //error
					mTx->AdvanceToNextEdge();   //skip no data period
					noop = 1;               //no frame will be added
					break;
				}
				starting_sample += samples_per_bit; 	//skip the start bit
				for (U32 i = 0; i < 4; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1; //this variable will break this state
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel); //error
						mTx->AdvanceToNextEdge();   //skip the no data period
						break;
					}
					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
					bit_counter++;

					//store the 4 bits protocol value (lsb transformation)
					if (mTx->GetBitState() == BIT_HIGH) {
						value = 1;
						for (U32 j = 0; j < i; j++) {
							value *= 2;
						}
						data += value;
						data_byte += (uint8_t)value;
					}
					label = 'PROT';
					mTx->Advance(samples_per_bit);
				}
				//reset, because we found a big period with no new data
				if (transmission_error) {
					state = WAIT;
					transmission_error = 0;
					noop = 1;
					break;
				}
				first_byte = 1;     //this variable shows that the next four bits will be taken into consideration for the crc calculation of the first byte
				state = TARGET;     //next state
				break;
			}
			case TARGET:
			{
				starting_sample -= samples_per_bit / 2;     //new frame's position

				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);    //Transmission Error - Collision
					mTx->AdvanceToNextEdge();   //skip the no data period
					noop = 1;                   //no new frame will be added
					break;
				}
				for (U32 i = 0; i < 12; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);    //Transmission Error - Collision
						mTx->AdvanceToNextEdge();       //skip the no data period
						break;
					}
					//if 8 bits are sampled, skip the stop and start bit
					if (bit_counter == 8) {
						dd = dd << 4;
						data_byte += dd;            //data_byte contains now the 4 bits of protocol and 4 bits of target for the crc comp.
						ComputeCRC(data_byte);
						mTx->AdvanceToNextEdge();   //skip stop and start bit
						mTx->Advance(samples_to_first_center_of_first_data_bit);    //sample to the center of the next data bit
						data_byte = 0;      //reset data byte values
						dd = 0;
						bit_counter = 0;
						first_byte = 0;
					}
					//let's put a dot exactly where we sample this bit:
                    mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
					bit_counter++;

					//target value calculation + lsb first inversion
					if (mTx->GetBitState() == BIT_HIGH) {
						value = 1;
						value_byte = 1;
						for (U32 j = 0; j < i; j++) {
							value *= 2;
						}
						if (!first_byte) {
							for (U32 j = 0; j < i - 4; j++) {
								value_byte *= 2;
							}
							dd += (uint8_t)value_byte;
						}
						else {
							dd += (uint8_t)value;
						}
						data += value;
					}
					label = 'TRGT';
					mTx->Advance(samples_per_bit);          //next bit
				}
                //reset, because we found a long period with no new data
				if (transmission_error) {
					state = WAIT;
					transmission_error = 0;
					noop = 1;       //no frame will be added
					break;
				}
				data_byte = dd;
				ComputeCRC(data_byte);  //crc comp. of second data byte
				data_byte = 0;
				state = TARGET_MODE;    //next state
				break;
			}
			case TARGET_MODE:
			{
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);    //transmission error - collision
					mTx->AdvanceToNextEdge();   //skip the no data period
					noop = 1;                   //no new frame
					break;
				}
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;			//skip start & stop bit
				mTx->AdvanceToNextEdge();
				mTx->Advance(samples_to_first_center_of_first_data_bit);                        //sample to the center of the next data bit

				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
					mTx->AdvanceToNextEdge();
					noop = 1;
					break;
				}
				bit_counter = 0;
				for (U32 i = 0; i < 4; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mTx->AdvanceToNextEdge();
						break;
					}

					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
					bit_counter++;
					//target mode data calculation - lsb first inversion
					if (mTx->GetBitState() == BIT_HIGH) {
						value = 1;
						for (U32 j = 0; j < i; j++) {

							value *= 2;
						}
						data += value;
						data_byte += (uint8_t)value;
					}
					label = 'MODE';
					//sample next bit
					mTx->Advance(samples_per_bit);

				}

				// if mode = IDACK | NODEIDACK -> ack notifier is ON
				if (data == 1 || data == 6) {
					ack = 1;
				}
				else {
					ack = 0;
				}

				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error = 0;
					noop = 1;
					break;
				}
				state = SOURCE;
				first_byte = 1;
				break;
			}
			case SOURCE:
			{
				starting_sample -= samples_per_bit / 2;

				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
					mTx->AdvanceToNextEdge();
					noop = 1;
					break;
				}
				for (U32 i = 0; i < 12; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mTx->AdvanceToNextEdge();
						break;
					}
					//when 8 bits sampled -> skip stop and start bit
					if (bit_counter == 8) {
						dd = dd << 4;
						data_byte += dd;
						ComputeCRC(data_byte);
						mTx->AdvanceToNextEdge();
						mTx->Advance(samples_to_first_center_of_first_data_bit);
						bit_counter = 0;
						data_byte = 0;
						dd = 0;
						first_byte = 0;
					}
					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
					bit_counter++;

					//source value computation - lsb first inversion
					if (mTx->GetBitState() == BIT_HIGH) {
						value = 1;
						value_byte = 1;
						for (U32 j = 0; j < i; j++) {
							value *= 2;
						}
						if (!first_byte)
						{
							for (U32 j = 0; j < i - 4; j++)
							{
								value_byte *= 2;
							}
							dd += (uint8_t)value_byte;
						}
						else {
							dd += (uint8_t)value;
						}
						data += value;
					}
					label = 'SRC';
					mTx->Advance(samples_per_bit);

				}
                //if there's no edge for the duration of timeout -> reset
				if (transmission_error) {
					state = WAIT;
					transmission_error = 0;
					bit_counter = 0;
					noop = 1;
					break;
				}
				data_byte = dd;
				ComputeCRC(data_byte);
				data_byte = 0;
				source = (uint16_t)data;
				state = CMD;
				break;
			}
			case CMD:
			{
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;		//skip start & stop bit
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
					mTx->AdvanceToNextEdge();
					noop = 1;
					break;
				}
				mTx->AdvanceToNextEdge();
				mTx->Advance(samples_to_first_center_of_first_data_bit);

				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
					mTx->AdvanceToNextEdge();
					noop = 1;
					break;
				}
				bit_counter = 0;
				for (U32 i = 0; i < 8; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mTx->AdvanceToNextEdge();
						break;
					}
					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
					bit_counter++;
					//cmd computation - lsb first inversion
					if (mTx->GetBitState() == BIT_HIGH) {
						value = 1;
						for (U32 j = 0; j < i; j++) {
							value *= 2;
						}
						data += value;
						data_byte += (uint8_t)value;
					}
					label = 'CMD';
					mTx->Advance(samples_per_bit);

				}
				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error = 0;
					noop = 1;
					break;
				}
				state = SIZE;
				first_byte = 1;
				ComputeCRC(data_byte);
				data_byte = 0;
				break;
			}
			case SIZE:
			{
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
					mTx->AdvanceToNextEdge();
					noop = 1;
					break;
				}
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;
				mTx->AdvanceToNextEdge();
				mTx->Advance(samples_to_first_center_of_first_data_bit);
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
					mTx->AdvanceToNextEdge();
					noop = 1;
					break;
				}
				bit_counter = 0;
				for (U32 i = 0; i < 16; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mTx->AdvanceToNextEdge();
						break;
					}
					//when 8 bits are sampled ->skip stop and start bit
					if (bit_counter == 8) {
						ComputeCRC(data_byte);
						mTx->AdvanceToNextEdge();
						mTx->Advance(samples_to_first_center_of_first_data_bit);
						bit_counter = 0;
						data_byte = 0;
						first_byte = 0;
					}
					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
					bit_counter++;
					if (mTx->GetBitState() == BIT_HIGH) {
						value = 1;
						value_byte = 1;
						for (U32 j = 0; j < i; j++) {

							value *= 2;
						}
						if (!first_byte) {
							for (U32 j = 0; j < i - 8; j++) {

								value_byte *= 2;
							}
							data_byte += (uint8_t)value_byte;
						}
						else {
							data_byte += (uint8_t)value;
						}
						data += value;
					}
					label = 'SIZE';
					mTx->Advance(samples_per_bit);
				}
				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error = 0;
					noop = 1;
					break;
				}
				size = data;
				ComputeCRC(data_byte);
				if (size == 0) state = CRC;				//if no data -> go to crc state
				else state = DATA;
				data_byte = 0;
				break;
			}
			case DATA:
			{
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
					mTx->AdvanceToNextEdge();
					noop = 1;
					break;
				}
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;
				mTx->AdvanceToNextEdge();
				mTx->Advance(samples_to_first_center_of_first_data_bit);
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					noop = 1;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
					mTx->AdvanceToNextEdge();
					data_idx = 0;
					break;
				}

				for (U32 i = 0; i < 8; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mTx->AdvanceToNextEdge();
						break;
					}
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);

					//data computation - lsb first inversion
					if (mTx->GetBitState() == BIT_HIGH)
					{
						value = 1;
						for (U32 j = 0; j < i; j++)
							value *= 2;

						data += value;
						data_byte += (uint8_t)value;
					}
					label = data_idx;
					mTx->Advance(samples_per_bit);
				}
				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error = 0;
					noop = 1;
					data_idx = 0;
					break;
				}
				ComputeCRC(data_byte);
				data_idx++;
				data_byte = 0;
				//if data finished -> go to next state, else continue with data comp
				if (data_idx == size || data_idx == 128) {
					state = CRC;
					data_idx = 0;
				}
				break;
			}
			case CRC:
			{
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
					mTx->AdvanceToNextEdge();
					noop = 1;
					break;
				}
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;
				mTx->AdvanceToNextEdge();
				mTx->Advance(samples_to_first_center_of_first_data_bit);
				bit_counter = 0;
				for (U32 i = 0; i < 16; i++)
				{
					if (bit_counter == 8)  //if 8 bits sampled -> skip stop and start bit
					{
						tracking = mTx->GetSampleNumber();
						mTx->AdvanceToNextEdge();
						mTx->Advance(samples_to_first_center_of_first_data_bit);
						bit_counter = 0;
					}
					//crc computation - lsb first inversion
					if (mTx->GetBitState() == BIT_HIGH)
					{
						value = 1;
						for (U32 j = 0; j < i; j++) {

							value *= 2;
						}
						data += value;
					}

					if ((i == 15) && (ack == 1))
					{
						//Last bit of CRC - Timeout timer is on!
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
						tracking = mTx->GetSampleNumber() + samples_per_bit / 2;
						mResults->AddMarker(tracking, AnalyzerResults::Start, mSettings->mTxChannel);
                        
                        tracking += timeout * samples_per_bit;
                        if (mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
                            state = ACK;
                        }
                        else {
                            mResults->AddMarker(tracking, AnalyzerResults::ErrorX, mSettings->mTxChannel);
                            state=WAIT;
                        }
					}
					else
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);

					bit_counter++;
					mTx->Advance(samples_per_bit);
				}

				//crc sent compared to crc computed - if not equal, data corrupted
				if (data == crc_val)
					label = 'CRC';
				else {
					label = 'NOT';
					transmission_error = 1;
				}
				if (transmission_error) {
					state = WAIT;
					data_idx = 0;
					break;
				}
                if (ack == 0)
                    state = WAIT;
				bit_counter = 0;
				break;
			}
			case ACK:
			{
				//we enter this case after the wait state and only if we find and ack before timeout
                mTx->AdvanceToNextEdge();
				mTx->Advance(samples_to_first_center_of_first_data_bit);
                starting_sample = mTx->GetSampleNumber() - (samples_per_bit/2);
				for (U32 i = 0; i < 8; i++)
                {
					mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
					//ack computation - lsb first inversion
					if (mTx->GetBitState() == BIT_HIGH) {
						value = 1;
						for (U32 j = 0; j < i; j++)
							value *= 2;

						data += value;
					}
					label = 'ACK';
					timer++;
					mTx->Advance(samples_per_bit);
				}
				state = WAIT;
				ack = 0;
                mResults->AddMarker(tracking, AnalyzerResults::Stop, mSettings->mTxChannel);
				break;
			}
			default:
			{
				state = WAIT;
				break;
			}
			}
			if (noop) ack = 0;

			//Set Frame Data.
			Frame frame;
			frame.mData1 = label;
			frame.mData2 = data;
			frame.mFlags = 0;
			frame.mStartingSampleInclusive = starting_sample;
			frame.mEndingSampleInclusive = mTx->GetSampleNumber() - samples_per_bit / 2;

			//send a frame only if there is not a transmission_error
			if (!noop) {
				mResults->AddFrame(frame);
				mResults->CommitResults();
				ReportProgress(frame.mEndingSampleInclusive);
			}
			//wait state is used when we wait for an ack or for a new message
			if (state == WAIT)
			{
				while (1)
				{
					CheckIfThreadShouldExit();      //kill thread in case of infinite loop
					bit_counter = 0;
					//Case where CRC was not good - in wait state until it finds a new msg - timeout period without a new msg
					if (transmission_error)
					{
						if (mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							mTx->AdvanceToNextEdge();
							continue;
						}
						mTx->AdvanceToNextEdge();
						state = PROTOCOL;
						ack = 0;
						break;
					}
					//when timeout ends after the end of the ack transmission
					if (!noop)
						mTx->AdvanceToNextEdge();

					starting_sample += samples_per_bit;
					if (mTx->GetBitState() == BIT_LOW)
					{
						state = PROTOCOL;
						break;
					}
				}
			}
		}
	}
	else {																								//Rx and Tx Channels

		//Initial Position in Rx Channel
		mRx = GetAnalyzerChannelData(mSettings->mRxChannel);
		if (mRx->GetBitState() == BIT_LOW)
			mRx->AdvanceToNextEdge();
		mRx->AdvanceToNextEdge();
		mRx->Advance(samples_to_first_center_of_first_data_bit);

		if (mRx->GetSampleNumber() < mTx->GetSampleNumber()) {      //if Rx earlier than Tx -> Rx_msg
			if (mTx->GetSampleNumber() - mRx->GetSampleNumber() < timeout * samples_per_bit)    //if Tx close to Rx ->collision detection
				collision_detection = 1;
			Rx_msg = 1;
		}
		else Rx_msg = 0;

		for ( ; ; )
		{
			CheckIfThreadShouldExit();      //kill thread in case of infinite loop
			U64 label = 0, data = 0, received_data = 0;							//frames' info
			U64 value = 0, value_byte = 0;										//data & crc calculation helpers
			U8 dd = 0, dd2 = 0;																		//
			bool transmission_error = 0;					//reset and collision notifiers
			U64 starting_sample = 0;														//points to the beginning of a frame

			//if Rx is 1 then msg in Rx, else msg in Tx

			//find the beginning of a frame, depending on the channel we have the msg
			if (Rx_msg)
				starting_sample = mRx->GetSampleNumber();
			else
				starting_sample = mTx->GetSampleNumber();

			switch (state) {
			case PROTOCOL:
			{
                //reset of the values - new msg initialization
				data_byte = 0;
				data_idx = 0;
				noop = 0;
				ack = 0;
				bit_counter = 0;
				crc_val = 0xFFFF;											//crc initial value
				if (!Rx_msg) {	//when data exist in channel Tx, move the Tx pointer
					mTx->Advance(samples_to_first_center_of_first_data_bit);
					starting_sample += samples_per_bit;
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {   //long period with no data?->reset
						transmission_error = 1;
						noop = 1;       //no new frame will be added
						state = WAIT;
						break;
					}
				}
				else {
					starting_sample -= samples_per_bit / 2; //skip the start bit
					if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) { //long period with no data?->reset
						transmission_error = 1;
						noop = 1;   //no new frame will be added
						state = WAIT;
						break;
					}
				}
				for (U32 i = 0; i < 4; i++)
				{
					if (Rx_msg)				//msg in Rx
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);

						if (collision_detection)			//add error X
							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						//protocol value computation -> lsb first inversion
						if (mRx->GetBitState() == BIT_HIGH) {
							value = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							data += value;
							data_byte += (uint8_t)value;
						}
						label = 'PROT';
						mRx->Advance(samples_per_bit);  //go to next data bit
					}
					else {
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						//let's put a dot exactly where we sample this bit:
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
						//square - bit received by Rx
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Square, mSettings->mRxChannel);
						//protocol value computation - lsb first inversion
						if (mTx->GetBitState() == BIT_HIGH) {
							value = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							data += value;
							data_byte += (uint8_t)value;
						}
						//protocol value in Rx channel - lsb first inversion    -- to check collision
						if (mRx->GetBitState() == BIT_HIGH) {
							value = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							received_data += value;
						}
						label = 'PROT';
						mTx->Advance(samples_per_bit);
						mRx->Advance(samples_per_bit);
					}
					bit_counter++;
				}
				//reset if long period with no new data
				if (transmission_error) {
					state = WAIT;
					noop = 1;
					bit_counter = 0;
					break;
				}
				//if data in Tx and Rx not equal -> collision detection
				if (!Rx_msg && received_data != data) {
					collision_detection = 1;
					Rx_msg = 1;
					data_byte = (uint8_t)received_data;
				}
				first_byte = 1;
				state = TARGET;
				break;
			}
			case TARGET:
			{
				starting_sample -= samples_per_bit / 2;

				for (U32 i = 0; i < 12; i++)
				{
					//if 8 bits are sampled, skip the stop and start bit
					if (bit_counter == 8) {
						dd = dd << 4;
						data_byte += dd;
						ComputeCRC(data_byte);
						if (!Rx_msg)				//when data exist in channel Tx, move the Tx pointer
						{
							if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
								transmission_error = 1;
								break;
							}
							mTx->AdvanceToNextEdge();
							mTx->Advance(samples_to_first_center_of_first_data_bit);
						}
						if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						mRx->AdvanceToNextEdge();
						mRx->Advance(samples_to_first_center_of_first_data_bit);
						data_byte = 0;
						dd = 0;
						bit_counter = 0;
						first_byte = 0;
					}

					if (Rx_msg) {
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							//mRx->AdvanceToNextEdge();
							break;
						}
						if (collision_detection)		//if collision ad an errorX to Tx
							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);
					}
					else
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							//mTx->AdvanceToNextEdge();
							break;
						}
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Square, mSettings->mRxChannel);
					}
					bit_counter++;
					if (Rx_msg)
					{
						//calculate target id - lsb first inversion
						if (mRx->GetBitState() == BIT_HIGH) {
							value = 1;
							value_byte = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							if (!first_byte) {
								for (U32 j = 0; j < i - 4; j++) {
									value_byte *= 2;
								}
								dd += (uint8_t)value_byte;
							}
							else {
								dd += (uint8_t)value;
							}
							data += value;
						}
					}
					else
					{
						//calculate target id in Tx - lsb first inversion
						if (mTx->GetBitState() == BIT_HIGH) {
							value = 1;
							value_byte = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							if (!first_byte) {
								for (U32 j = 0; j < i - 4; j++) {
									value_byte *= 2;
								}
								dd += (uint8_t)value_byte;
							}
							else {
								dd += (uint8_t)value;
							}
							data += value;
						}
						//calculate target id in Rx - lsb first inversion
						if (mRx->GetBitState() == BIT_HIGH) {
							value = 1;
							value_byte = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							if (!first_byte) {
								for (U32 j = 0; j < i - 4; j++) {
									value_byte *= 2;
								}
								dd2 += (uint8_t)value_byte;
							}
							else {
								dd2 += (uint8_t)value;
							}
							received_data += value;
						}
					}
					//if Rx and Tx data are not equal -> collision detection
					//after a collision detection, the data of Rx are monitored
					if (!Rx_msg && received_data != data) {
						collision_detection = 1;
						Rx_msg = 1;
						data = received_data;
						dd = dd2;
					}
					label = 'TRGT';
					if (!Rx_msg)
						mTx->Advance(samples_per_bit);
					mRx->Advance(samples_per_bit);
				}
				//reset
				if (transmission_error) {
					state = WAIT;
					//transmission_error = 0;
					noop = 1;
					break;
				}
				data_byte = dd;
				ComputeCRC(data_byte);
				target = (uint16_t)data;
				data_byte = 0;
				state = TARGET_MODE;
				break;
			}
			case TARGET_MODE:
			{
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;
				if (!Rx_msg)							//when data exist in channel Tx, move the Tx pointer
				{

					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						state = WAIT;
						noop = 1;
						transmission_error = 1;
						break;
					}
					mTx->AdvanceToNextEdge();
					mTx->Advance(samples_to_first_center_of_first_data_bit);
				}

				if (Rx_msg) {
					if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						state = WAIT;
						noop = 1;
						break;
					}
				}
				mRx->AdvanceToNextEdge();
				mRx->Advance(samples_to_first_center_of_first_data_bit);
				bit_counter = 0;
				for (U32 i = 0; i < 4; i++)
				{
					bit_counter++;
					if (Rx_msg)		//msg in Rx channel
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						if (collision_detection)
						{
							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						}
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);
						//Target mode computation - lsb first inversion
						if (mRx->GetBitState() == BIT_HIGH) {
							value = 1;
							for (U32 j = 0; j < i; j++) {

								value *= 2;
							}
							data += value;
							data_byte += (uint8_t)value;
						}
						label = 'MODE';
						mRx->Advance(samples_per_bit);
					}
					else
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Square, mSettings->mRxChannel);
						//Target mode computation - lsb first inversion
						if (mTx->GetBitState() == BIT_HIGH) {
							value = 1;
							for (U32 j = 0; j < i; j++) {

								value *= 2;
							}
							data += value;
							data_byte += (uint8_t)value;
						}
						//Target mode received in Rx channel
						if (mRx->GetBitState() == BIT_HIGH) {
							value = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							received_data += value;
						}
						label = 'MODE';
						mTx->Advance(samples_per_bit);
						mRx->Advance(samples_per_bit);
					}
				}
				//reset
				if (transmission_error) {
					state = WAIT;
					noop = 1;
					bit_counter = 0;
					break;
				}
				if (!Rx_msg && received_data != data) {
					collision_detection = 1;
					Rx_msg = 1;
					data_byte = (uint8_t)received_data;
					data = received_data;
				}
				//when target = IDACK | NODEIDACK, ack notifier is ON
				if (data == 1 || data == 6) {
					ack = 1;
				}
				state = SOURCE;
				first_byte = 1;
				break;
			}
			case SOURCE:
			{
				starting_sample -= samples_per_bit / 2;
				for (U32 i = 0; i < 12; i++)
				{
					//if 8 bits are sampled, skip the stop and start bit
					if (bit_counter == 8) {
						dd = dd << 4;
						data_byte += dd;
						ComputeCRC(data_byte);
						if (!Rx_msg)	//when data in Tx, move Tx pointer
						{
							if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
								transmission_error = 1;
								break;
							}
							mTx->AdvanceToNextEdge();
							mTx->Advance(samples_to_first_center_of_first_data_bit);
						}
						if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						mRx->AdvanceToNextEdge();
						mRx->Advance(samples_to_first_center_of_first_data_bit);
						bit_counter = 0;
						data_byte = 0;
						dd = 0;
						first_byte = 0;
					}
					bit_counter++;
					if (Rx_msg)		//msg in Rx
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						if (collision_detection)	//error X
							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);
						//Source id computation - lsb first inversion
						if (mRx->GetBitState() == BIT_HIGH) {
							value = 1;
							value_byte = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							if (!first_byte)
							{
								for (U32 j = 0; j < i - 4; j++)
								{
									value_byte *= 2;
								}
								dd += value_byte;
							}
							else {
								dd += value;
							}
							data += value;
						}
						label = 'SRC';
						mRx->Advance(samples_per_bit);
					}
					else
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Square, mSettings->mRxChannel);
						//Source id sent from Tx - lsb first inversion
						if (mTx->GetBitState() == BIT_HIGH) {
							value = 1;
							value_byte = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							if (!first_byte)
							{
								for (U32 j = 0; j < i - 4; j++)
								{
									value_byte *= 2;
								}
								dd += value_byte;
							}
							else {
								dd += value;
							}
							data += value;
						}
						if (mTx->GetBitState() == BIT_HIGH) {
							value = 1;
							value_byte = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							if (!first_byte)
							{
								for (U32 j = 0; j < i - 4; j++)
								{
									value_byte *= 2;
								}
								dd2 += value_byte;
							}
							else {
								dd2 += value;
							}
							received_data += value;
						}

						if (!Rx_msg && received_data != data) {
							collision_detection = 1;
							Rx_msg = 1;
							data = received_data;
							dd = dd2;
						}
						label = 'SRC';
						mTx->Advance(samples_per_bit);
						mRx->Advance(samples_per_bit);
					}
				}
				//reset
				if (transmission_error) {
					state = WAIT;
					noop = 1;
					break;
				}
				data_byte = dd;
				ComputeCRC(data_byte);
				data_byte = 0;
				source = (uint16_t)data;
				state = CMD;
				break;
			}
			case CMD:
			{
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;
				if (!Rx_msg)		//when data in Tx , move Tx pointer
				{

					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						state = WAIT;
						noop = 1;
						transmission_error = 1;
						break;
					}
					mTx->AdvanceToNextEdge();
					mTx->Advance(samples_to_first_center_of_first_data_bit);
				}


				if (Rx_msg) {
					if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						state = WAIT;
						noop = 1;
						break;
					}
				}
				mRx->AdvanceToNextEdge();
				mRx->Advance(samples_to_first_center_of_first_data_bit);
				bit_counter = 0;
				for (U32 i = 0; i < 8; i++)
				{
					//let's put a dot exactly where we sample this bit:
					bit_counter++;
					if (Rx_msg)
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						if (collision_detection) //error X
							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);
						//cmd computation - lsb first inversion
						if (mRx->GetBitState() == BIT_HIGH) {
							value = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							data += value;
							data_byte += (uint8_t)value;
						}
						label = 'CMD';
						mRx->Advance(samples_per_bit);
					}
					else
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);

						if (mTx->GetBitState() == BIT_HIGH) {
							value = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							data += value;
							data_byte += (uint8_t)value;
						}
						label = 'CMD';
						mTx->Advance(samples_per_bit);
						mRx->Advance(samples_per_bit);
					}
				}
				if (mRx->GetSampleNumber() > mTx->GetSampleNumber())
					mTx->AdvanceToAbsPosition(mRx->GetSampleNumber());
				if (transmission_error) {
					state = WAIT;
					noop = 1;
					break;
				}
				state = SIZE;
				first_byte = 1;
				ComputeCRC(data_byte);
				data_byte = 0;
				break;
			}
			case SIZE:
			{
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;
				if (Rx_msg)
				{
					if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						state = WAIT;
						noop = 1;
						break;
					}
					mRx->AdvanceToNextEdge();
					mRx->Advance(samples_to_first_center_of_first_data_bit);
				}
				else
				{
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						state = WAIT;
						noop = 1;
						break;
					}
					mTx->AdvanceToNextEdge();
					mTx->Advance(samples_to_first_center_of_first_data_bit);
				}
				bit_counter = 0;
				for (U32 i = 0; i < 16; i++)
				{
					//if 8 bits are sampled, skip the stop and start bit
					if (bit_counter == 8) {
						ComputeCRC(data_byte);
						if (Rx_msg)				//if msg in Rx, move Rx pointer
						{
							mRx->AdvanceToNextEdge();
							mRx->Advance(samples_to_first_center_of_first_data_bit);
						}
						else {						//if msg in Tx, move Tx pointer
							mTx->AdvanceToNextEdge();
							mTx->Advance(samples_to_first_center_of_first_data_bit);
						}
						bit_counter = 0;
						data_byte = 0;
						first_byte = 0;
					}

					if (Rx_msg) {				//msg in Rx
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						if (collision_detection)
							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);
						bit_counter++;
						//Size - lsb first inversion
						if (mRx->GetBitState() == BIT_HIGH) {
							value = 1;
							value_byte = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							if (!first_byte) {
								for (U32 j = 0; j < i - 8; j++) {

									value_byte *= 2;
								}
								data_byte += (uint8_t)value_byte;
							}
							else {
								data_byte += (uint8_t)value;
							}
							data += value;
						}
						label = 'SIZE';
						mRx->Advance(samples_per_bit);
					}
					else {
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
						bit_counter++;
						//Size - lsb first inversion
						if (mTx->GetBitState() == BIT_HIGH) {
							value = 1;
							value_byte = 1;
							for (U32 j = 0; j < i; j++) {
								value *= 2;
							}
							if (!first_byte) {
								for (U32 j = 0; j < i - 8; j++) {

									value_byte *= 2;
								}
								data_byte += (uint8_t)value_byte;
							}
							else {
								data_byte += (uint8_t)value;
							}
							data += value;
						}
						label = 'SIZE';
						mTx->Advance(samples_per_bit);
					}
				}
				if (mRx->GetSampleNumber() > mTx->GetSampleNumber())
					mTx->AdvanceToAbsPosition(mRx->GetSampleNumber());
				if (transmission_error) {
					state = WAIT;
					noop = 1;
					break;
				}
				size = data;
				ComputeCRC(data_byte);
				if (size == 0) { state = CRC; }
				else { state = DATA; }
				data_byte = 0;
				break;
			}
			case DATA:
			{
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;
				if (Rx_msg)
				{
					if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						state = WAIT;
						noop = 1;
						break;
					}
					mRx->AdvanceToNextEdge();
					mRx->Advance(samples_to_first_center_of_first_data_bit);

					for (U32 i = 0; i < 8; i++)
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						if (collision_detection) //error X
							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
						mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);
						//Data - lsb first inversion
						if (mRx->GetBitState() == BIT_HIGH)
						{
							value = 1;

							for (U32 j = 0; j < i; j++)
							{
								value *= 2;
							}

							data += value;
							data_byte += (uint8_t)value;
						}
						label = data_idx;
						mRx->Advance(samples_per_bit);
					}
				}
				else {
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						state = WAIT;
						noop = 1;
						break;
					}
					mTx->AdvanceToNextEdge();
					mTx->Advance(samples_to_first_center_of_first_data_bit);


					for (U32 i = 0; i < 8; i++)
					{
						//if the time between the current moment and the last transition is more than timeout -> reset
						if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
							transmission_error = 1;
							break;
						}
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
						//Data - lsb first inversion
						if (mTx->GetBitState() == BIT_HIGH)
						{
							value = 1;
							for (U32 j = 0; j < i; j++)
							{
								value *= 2;
							}
							data += value;
							data_byte += (uint8_t)value;
						}
						label = data_idx;
						mTx->Advance(samples_per_bit);
					}
				}
				//reset
				if (mRx->GetSampleNumber() > mTx->GetSampleNumber())
					mTx->AdvanceToAbsPosition(mRx->GetSampleNumber());
				if (transmission_error) {
					state = WAIT;
					noop = 1;
					data_idx = 0;
					break;
				}
				ComputeCRC(data_byte);
				data_idx++;
				data_byte = 0;
				//if data number reach the size or the maximum data size -> terminate the reception
				if (data_idx == size || data_idx == 128) {
					state = CRC;
					data_idx = 0;
				}
				break;
			}
			case CRC:
			{
				starting_sample += (uint64_t)samples_per_bit * 2 - samples_per_bit / 2;
				if (Rx_msg)	//msg in Rx
				{
					if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						state = WAIT;
						noop = 1;
						break;
					}
					mRx->AdvanceToNextEdge();
					mRx->Advance(samples_to_first_center_of_first_data_bit);

					bit_counter = 0;
					for (U32 i = 0; i < 16; i++)
					{
						if ((!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) && i <= 8) {
							ack = 0;
							transmission_error = 1;
							break;
						}
						if (bit_counter == 8)  //if 8 bits sampled, skip Start and Stop bit
						{
							mRx->AdvanceToNextEdge();
							mRx->Advance(samples_to_first_center_of_first_data_bit);
							bit_counter = 0;
						}
						//crc reception
						if (mRx->GetBitState() == BIT_HIGH)
						{
							value = 1;
							for (U32 j = 0; j < i; j++) {

								value *= 2;
							}
							data += value;
						}
						if ((i == 15) && (ack == 1))
						{
							if (collision_detection)		//errorX
								mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
							//Last bit of CRC - Timeout timer is on!
							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);
							tracking = mRx->GetSampleNumber() + samples_per_bit / 2;
							mResults->AddMarker(tracking, AnalyzerResults::Start, mSettings->mTxChannel);
							mResults->AddMarker(tracking, AnalyzerResults::Start, mSettings->mRxChannel);
							if (mRx->GetSampleNumber() > mTx->GetSampleNumber())
								mTx->AdvanceToAbsPosition(mRx->GetSampleNumber());
							tracking += timeout * samples_per_bit;
							if (mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit) || mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
								state = ACK;
							}
							else {
								mResults->AddMarker(tracking, AnalyzerResults::ErrorX, mSettings->mTxChannel);
								mResults->AddMarker(tracking, AnalyzerResults::ErrorX, mSettings->mRxChannel);
								state = WAIT;
							}
						}
						else {
							if (collision_detection)		//errorX
								mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);
						}
						bit_counter++;
						mRx->Advance(samples_per_bit);
					}
				}
				else {	//msg in Tx
					if (!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
						transmission_error = 1;
						state = WAIT;
						noop = 1;
					}
					mTx->AdvanceToNextEdge();
					mTx->Advance(samples_to_first_center_of_first_data_bit);
					

					bit_counter = 0;
					for (U32 i = 0; i < 16; i++)
					{

						if ((!mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) && i <= 8) {		//if no data for timeout period - error
							transmission_error = 1;
							//mTx->AdvanceToNextEdge();
							break;
						}
						if (bit_counter == 8)  //if 8 bits sampled, skip Start and Stop bit
						{
							mTx->AdvanceToNextEdge();
							mTx->Advance(samples_to_first_center_of_first_data_bit);
							bit_counter = 0;
						}
						//crc reception
						if (mTx->GetBitState() == BIT_HIGH)
						{
							value = 1;
							for (U32 j = 0; j < i; j++) {

								value *= 2;
							}
							data += value;
						}
						if ((i == 15) && (ack == 1))
						{
							//Last bit of CRC - Timeout timer is on!
							mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
							tracking = mTx->GetSampleNumber() + samples_per_bit / 2;
							if (mTx->GetSampleNumber() > mRx->GetSampleNumber())    //Rx pointer is left behind
								mRx->AdvanceToAbsPosition(mTx->GetSampleNumber());
							mResults->AddMarker(tracking, AnalyzerResults::Start, mSettings->mTxChannel);			//green symbol for timeout start
							mResults->AddMarker(tracking, AnalyzerResults::Start, mSettings->mRxChannel);			//green symbol for timeout beginning
							tracking += timeout * samples_per_bit;
							if (mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit) || mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
								state = ACK;    //if there is data in less than timeout -> ack found
							}
							else {
								mResults->AddMarker(tracking, AnalyzerResults::ErrorX, mSettings->mTxChannel);
								mResults->AddMarker(tracking, AnalyzerResults::ErrorX, mSettings->mRxChannel);
								state = WAIT;
							}
						}
						else
							mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);

						bit_counter++;
						mTx->Advance(samples_per_bit);
					}
				}
				if (mRx->GetSampleNumber() > mTx->GetSampleNumber())
					mTx->AdvanceToAbsPosition(mRx->GetSampleNumber());			//Tx reach the position of Rx
				if (transmission_error) {
					state = WAIT;
					noop = 1;
					break;
				}
				//crc evaluation
				if (data == crc_val)
					label = 'CRC';
				else {
					label = 'NOT';
					transmission_error = 1;
				}
				if (ack == 0)
					state = WAIT;

				bit_counter = 0;
				break;
			}
			case ACK:
			{
				if (target == source)				//ack to the same channel with the msg
				{
					mTx->AdvanceToNextEdge();
					mTx->Advance(samples_to_first_center_of_first_data_bit);
					starting_sample = mTx->GetSampleNumber() - (samples_per_bit / 2);   //find the position of ack frame

					for (U32 i = 0; i < 8; i++)     //sample the 8 ack bits
					{
						mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);
						if (mTx->GetBitState() == BIT_HIGH) {
							value = 1;
							for (U32 j = 0; j < i; j++)
								value *= 2;

							data += value;
						}
						label = 'ACK';
						mTx->Advance(samples_per_bit);
					}
				}
				else			//ack to the other channel => if msg in Rx -> ack to Tx / if msg in Tx -> ack to Rx
				{
					if (Rx_msg)
					{
						if (target == 0) {		//if target = 0 and message to Rx -> ack to Rx
							mRx->AdvanceToNextEdge();
							mRx->Advance(samples_to_first_center_of_first_data_bit);
							starting_sample = mRx->GetSampleNumber() - (samples_per_bit / 2);
							for (U32 i = 0; i < 8; i++)
							{
								mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);
								if (mRx->GetBitState() == BIT_HIGH) {		//ack value - lsb inversion
									value = 1;
									for (U32 j = 0; j < i; j++)
										value *= 2;

									data += value;
								}
								label = 'ACK';

								mRx->Advance(samples_per_bit);
							}
						}
						else {		//ack to Tx channel
							mTx->AdvanceToNextEdge();
							mTx->Advance(samples_per_bit / 2);
							starting_sample = mTx->GetSampleNumber() - (samples_per_bit / 2);
							for (U32 i = 0; i < 8; i++)
							{
								mResults->AddMarker(mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel);

								if (mTx->GetBitState() == BIT_HIGH) {
									value = 1;
									for (U32 j = 0; j < i; j++)
										value *= 2;

									data += value;
								}
								label = 'ACK';

								mTx->Advance(samples_per_bit);
							}
							if (mTx->GetSampleNumber() > mRx->GetSampleNumber())
								mRx->AdvanceToAbsPosition(mTx->GetSampleNumber());
							mTx->AdvanceToNextEdge();
						}
					}
					else	//ack to Rx channel
					{
						mRx->AdvanceToAbsPosition(mTx->GetSampleNumber());
						mRx->AdvanceToNextEdge();
						mRx->Advance(samples_to_first_center_of_first_data_bit);	//find ack position
						starting_sample = mRx->GetSampleNumber() - (samples_per_bit / 2);
						for (U32 i = 0; i < 8; i++)
						{

							mResults->AddMarker(mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel);

							bit_counter++;
							if (mRx->GetBitState() == BIT_HIGH) {		//ack value
								value = 1;
								for (U32 j = 0; j < i; j++)
									value *= 2;

								data += value;
							}
							label = 'ACK';
							timer++;

							mRx->Advance(samples_per_bit);
						}
						if (mRx->GetSampleNumber() > mTx->GetSampleNumber())
							mTx->AdvanceToAbsPosition(mRx->GetSampleNumber());			//Tx reach the position of Rx
					}
				}
				
				if (!Rx_msg && mRx->GetSampleNumber() < mTx->GetSampleNumber())
					mRx->AdvanceToAbsPosition(mTx->GetSampleNumber());

				mResults->AddMarker(tracking, AnalyzerResults::Stop, mSettings->mTxChannel);
				mResults->AddMarker(tracking, AnalyzerResults::Stop, mSettings->mRxChannel);
				state = WAIT;
				ack = 0;
				bit_counter = 0;
				break;
			}
			default:
			{
				state = WAIT;
				break;
			}
			}

			if (collision_detection && Rx_msg && state == WAIT) {			//end of message
				ack = 0;
			}
			if (!noop) {			//if noop=1 ->send no frame
				Frame frame;
				frame.mData1 = label;
				frame.mData2 = data;
				frame.mFlags = 0;
				frame.mStartingSampleInclusive = starting_sample;

				if (Rx_msg)
					frame.mEndingSampleInclusive = mRx->GetSampleNumber() - samples_per_bit / 2;
				else
					frame.mEndingSampleInclusive = mTx->GetSampleNumber() - samples_per_bit / 2;

				mResults->AddFrame(frame);
				mResults->CommitResults();
				ReportProgress(frame.mEndingSampleInclusive);
			}
			/*This state is the default state. It is enabled in case of no data, transmission_error, ack wait, end of msg, etc
			and it waits until the next msg|*/
			if (state == WAIT)
			{
				while (1)
				{
					CheckIfThreadShouldExit();
					if (transmission_error)			//This is the handling of a transmission_error in case CRC is not good! Wait until we find a no data timeout period
					{
						if (Rx_msg)
						{
							if ((mRx->GetSampleNumber() > mTx->GetSampleNumber()) && (mRx->GetSampleNumber() - mTx->GetSampleNumber() > timeout * samples_per_bit))
								mTx->AdvanceToNextEdge();
							if (!mRx->WouldAdvancingCauseTransition(timeout * samples_per_bit)) {
								
								mRx->AdvanceToNextEdge();
								if ((mRx->GetSampleNumber() > mTx->GetSampleNumber()) && (mRx->GetSampleNumber() - mTx->GetSampleNumber() > timeout * samples_per_bit)) {
									while (mTx->WouldAdvancingCauseTransition(timeout * samples_per_bit))
										mTx->AdvanceToNextEdge();
									mTx->AdvanceToNextEdge();
								}
									
								if (mRx->GetSampleNumber() < mTx->GetSampleNumber()) {			//if we found a msg in Rx earlier Rx_msg
									if (mTx->GetSampleNumber() - mRx->GetSampleNumber() < timeout * samples_per_bit)	//if we also have a Tx msg close to Rx ->collision
										collision_detection = 1;
									else collision_detection = 0;
										Rx_msg = 1;
								}
								else
									Rx_msg = 0;
								state = PROTOCOL;			//Restart
								
								//if (!noop)
								mRx->Advance(samples_to_first_center_of_first_data_bit);
								bit_counter = 0;
								ack = 0;
								break;
							}
							else {
								state = WAIT;
								mRx->AdvanceToNextEdge();
								continue;
							}
						}
						else			//same for Tx msg
						{
							if (mRx->GetSampleNumber() < mTx->GetSampleNumber())
								mRx->AdvanceToNextEdge();
							if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
								mTx->AdvanceToNextEdge();
					
								if (mRx->GetSampleNumber() < mTx->GetSampleNumber()) {
									if (mTx->GetSampleNumber() - mRx->GetSampleNumber() < timeout * samples_per_bit)
										collision_detection = 1;
									else collision_detection = 0;
									Rx_msg = 1;
								}
								else {
									Rx_msg = 0;
								}
								state = PROTOCOL;
								//if (!noop)
								mRx->Advance(samples_to_first_center_of_first_data_bit);
								
								bit_counter = 0;
								ack = 0;
								break;
							}
							else {
								state = WAIT;
								mTx->AdvanceToNextEdge();
								continue;
							}
						}
					}
					//Initialization of the next message - Reset state machine
					if (!Rx_msg) {
						if (mRx->GetSampleNumber() < mTx->GetSampleNumber() && mTx->GetSampleNumber() - mRx->GetSampleNumber() > timeout * samples_per_bit)
							mRx->AdvanceToAbsPosition(mTx->GetSampleNumber());
						mTx->AdvanceToNextEdge();
						
					}
					if (!noop && !collision_detection) {				//Normally, in Tx msg Rx is left behind, so we advance to the next msg
						mRx->AdvanceToNextEdge();
					}
					if (mTx->GetSampleNumber() > mRx->GetSampleNumber())		//data found in Rx? ->Rx msg
					{
						if (mTx->GetSampleNumber() - mRx->GetSampleNumber() < timeout * samples_per_bit)		//Data found in Tx also -> collision
							collision_detection = 1;
						else collision_detection = 0;

						Rx_msg = 1;
						starting_sample += samples_per_bit;
						if (mRx->GetBitState() == BIT_LOW)		//reset
						{
							mRx->Advance(samples_to_first_center_of_first_data_bit);
							state = PROTOCOL;
							break;
						}
					}
					else {
						if (mRx->GetSampleNumber() - mTx->GetSampleNumber() > timeout * samples_per_bit) {		//In case of a previous error if we still have data we wait
							Rx_msg = 0;
							transmission_error = 1;
							continue;
						}
						//We finally can go to the next msg
						collision_detection = 0;
						Rx_msg = 0;
						starting_sample += samples_per_bit;
						if (mTx->GetBitState() == BIT_LOW)
						{
							mRx->Advance(samples_to_first_center_of_first_data_bit);
							state = PROTOCOL;
							break;
						}
					}
				}
			}
		}
	}
}

/*Function to compute CRC value - Called every 8 bits*/
void ComputeCRC(uint8_t data) {

	uint16_t dbyte = data;
	crc_val ^= dbyte << 8;
	for (uint8_t j = 0; j < 8; j++)
	{
		uint16_t mix = crc_val & 0x8000;
		crc_val = (crc_val << 1);
		if (mix)
			crc_val = crc_val ^ 0x0007;
	}
}

bool LuosAnalyzer::NeedsRerun()
{
	return false;
}

U32 LuosAnalyzer::GenerateSimulationData(U64 minimum_sample_index, U32 device_sample_rate, SimulationChannelDescriptor** simulation_channels)
{
	if (mSimulationInitilized == false)
	{
		mSimulationDataGenerator.Initialize(GetSimulationSampleRate(), mSettings.get());
		mSimulationInitilized = true;
	}

	return mSimulationDataGenerator.GenerateSimulationData(minimum_sample_index, device_sample_rate, simulation_channels);
}

U32 LuosAnalyzer::GetMinimumSampleRateHz()
{
	return mSettings->mBitRate * 4;
}

const char* LuosAnalyzer::GetAnalyzerName() const
{
	return "Luos";
}

const char* GetAnalyzerName()
{
	return "Luos";
}

Analyzer* CreateAnalyzer()
{
	return new LuosAnalyzer();
}

void DestroyAnalyzer(Analyzer* analyzer)
{
	delete analyzer;
}
