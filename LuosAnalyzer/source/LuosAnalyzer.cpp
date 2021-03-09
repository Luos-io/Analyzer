#include "LuosAnalyzer.h"
#include "LuosAnalyzerSettings.h"
#include <AnalyzerChannelData.h>
#include <stdlib.h>
#include <stdint.h>

LuosAnalyzer::LuosAnalyzer()
:	Analyzer2(),
	mSettings( new LuosAnalyzerSettings() ),
	mSimulationInitilized( false )
{
	SetAnalyzerSettings( mSettings.get() );
}


U16 crc_val;
U8 ONE_WIRE;

LuosAnalyzer::~LuosAnalyzer()
{
	KillThread();
}

void LuosAnalyzer::SetupResults()
{
	mResults.reset( new LuosAnalyzerResults( this, mSettings.get() ) );
	SetAnalyzerResults( mResults.get() );
	mResults->AddChannelBubblesWillAppearOn( mSettings->mTxChannel );

	//if user does not define Rx, we have one_wire config
	if (mSettings->mRxChannel == UNDEFINED_CHANNEL)
		ONE_WIRE = 1;
	else
	{
		ONE_WIRE = 0;
		mResults->AddChannelBubblesWillAppearOn( mSettings->mRxChannel );
	}

}

void LuosAnalyzer::WorkerThread()
{


	mSampleRateHz = GetSampleRate();
	mTx = GetAnalyzerChannelData( mSettings->mTxChannel );

	if( mTx->GetBitState() == BIT_LOW )
		mTx->AdvanceToNextEdge();

	//Timeout = 2*10*(1sec/baudrate)
	U64 timeout = 2 * 10 * (1000000/mSettings->mBitRate);

	U32 samples_per_bit = mSampleRateHz / mSettings->mBitRate;
	U32 samples_to_first_center_of_first_data_bit = U32( 1.5 * double( mSampleRateHz ) / double( mSettings->mBitRate ) );
	U8 bit_counter=0, data_byte = 0;
	U16 size, data_idx = 0, target, source, timer = 0;
	bool ack = 0, found_ack = 0, ack_done=0, Rx_msg = 0, collision_detection = 0, first_byte=0;

	U32 state=PROTOCOL;							//initialization of state machine
	//Initial Position of the first bit -- falling edge -- beginning of the start bit
	mTx->AdvanceToNextEdge();

	//used for aknowledgement time tracking
	U64 tracking = mTx->GetSampleNumber();

	//Process for one_wire config
	if (ONE_WIRE)
	{

	for( ; ; )
	{
		U64 label, data = 0;													//frames' info
		U64 value = 0, value_byte = 0;								//data & crc calculation helpers
		U8 dd=0;																			//
		U64 starting_sample = mTx->GetSampleNumber();	//points to the beginning of a frame
		bool transmission_error=0, noop=0;						//reset and collision notifiers

		//state machine for the transmission-reception of a message
		switch (state) {
			case PROTOCOL:
			{
				mTx->Advance( samples_to_first_center_of_first_data_bit );
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					data_idx=0;
					break;
				}

				data_byte=0;
				crc_val = 0xFFFF; 	//initial value of crc
				starting_sample += samples_per_bit; 	//skip the start bit
				for(U32 i=0; i<4; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
						mTx->AdvanceToNextEdge();
						break;
					}
					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
					bit_counter++;

					//store the 4 bits protocol value (lsb transformation)
					if( mTx->GetBitState() == BIT_HIGH ) {
						value = 1;
						for (U32 j=0; j<i; j++) {
							value *=2;
						}
						data += value;
						data_byte += value;
					}
					label = 'PROT';
					mTx->Advance(samples_per_bit);
				}

				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error=0;
					noop = 1;
					break;
				}

				first_byte = 1;
				state = TARGET;
				break;
			}
			case TARGET:
			{
				starting_sample -= samples_per_bit/2;

				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				for(U32 i=0; i<12; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
						mTx->AdvanceToNextEdge();
						break;
					}
					//if 8 bits are sampled, skip the stop and start bit
					if (bit_counter==8) {
						dd = dd << 4;
						data_byte += dd;
						ComputeCRC(data_byte);
						mTx->AdvanceToNextEdge();
						mTx->Advance( samples_to_first_center_of_first_data_bit );
						data_byte = 0;
						dd=0;
						bit_counter = 0;
						first_byte =0;
					}
					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );

					bit_counter++;

					//target value calculation + lsb first inversion
					if( mTx->GetBitState() == BIT_HIGH ) {
						value = 1;
						value_byte = 1;
						for (U32 j=0; j<i; j++) {
							value *=2;
						}
						if (!first_byte) {
							for (U32 j=0; j<i-4; j++) {
								value_byte *=2;
							}
							dd += value_byte;
						}
						else {
							dd += value;
						}
						data += value;
					}
					label = 'TRGT';
					mTx->Advance( samples_per_bit);
				}
				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error=0;
					noop = 1;
					break;
				}
				data_byte = dd;
				ComputeCRC(data_byte);
				data_byte=0;
				state = TARGET_MODE;
				break;
			}
			case TARGET_MODE:
			{
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				starting_sample += samples_per_bit*2 - samples_per_bit/2;			//skip start & stop bit
				mTx->AdvanceToNextEdge();
				mTx->Advance( samples_to_first_center_of_first_data_bit );

				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				bit_counter=0;
				for(U32 i=0; i<4; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
						mTx->AdvanceToNextEdge();
						break;
					}

					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
					bit_counter++;
					//target mode data calculation - lsb first inversion
					if( mTx->GetBitState() == BIT_HIGH ) {
						value = 1;
						for (U32 j=0; j<i; j++) {

							value *=2;
						}
						data += value;
						data_byte += value;
					}
					label = 'MODE';
					//sample next bit
					mTx->Advance( samples_per_bit);

				}

				// if mode = IDACK | NODEIDACK -> ack notifier is ON
				if ( data==1 || data==6 ) {
					ack=1;
				}
				else {
					ack=0;
				}

				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error=0;
					noop = 1;
					break;
				}
				state = SOURCE;
				first_byte = 1;
				break;
			}
			case SOURCE:
			{
				starting_sample -= samples_per_bit/2;

				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				for(U32 i=0; i<12; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
						mTx->AdvanceToNextEdge();
						break;
					}
					//when 8 bits sampled -> skip stop and start bit
					if (bit_counter==8) {
						dd = dd << 4;
						data_byte += dd;
						ComputeCRC(data_byte);
						mTx->AdvanceToNextEdge();
						mTx->Advance( samples_to_first_center_of_first_data_bit );
						bit_counter = 0;
						data_byte=0;
						dd=0;
						first_byte = 0;
					}
					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
					bit_counter++;

					//source value computation - lsb first inversion
					if( mTx->GetBitState() == BIT_HIGH ) {
						value = 1;
						value_byte = 1;
						for (U32 j=0; j<i; j++) {
							value *=2;
						}
						if (!first_byte)
						{
							for (U32 j=0; j<i-4; j++)
							{
								value_byte *=2;
							}
							dd += value_byte;
						}
						else {
							dd += value;
						}
						data += value;
					}
					label = 'SRC';
					mTx->Advance( samples_per_bit);

				}
				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error=0;
					bit_counter=0;
					noop = 1;
					break;
				}
				data_byte = dd;
				ComputeCRC(data_byte);
				data_byte=0;
				source = data;
				state=CMD;
				break;
			}
			case CMD:
			{
				starting_sample += samples_per_bit*2 - samples_per_bit/2;		//skip start & stop bit
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				mTx->AdvanceToNextEdge();
				mTx->Advance( samples_to_first_center_of_first_data_bit );

				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				bit_counter = 0;
				for(U32 i=0; i<8; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
						mTx->AdvanceToNextEdge();
						break;
					}
					//let's put a dot exactly where we sample this bit:
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
					bit_counter++;
					//cmd computation - lsb first inversion
					if( mTx->GetBitState() == BIT_HIGH ) {
						value = 1;
						for (U32 j=0; j<i; j++) {
							value *=2;
						}
						data += value;
						data_byte +=value;
					}
					label = 'CMD';
					mTx->Advance(samples_per_bit);

				}
				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error=0;
					noop = 1;
					break;
				}
				state = SIZE;
				first_byte = 1;
				ComputeCRC( data_byte);
				data_byte =0;
				break;
			}
			case SIZE:
			{
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				starting_sample += samples_per_bit*2 - samples_per_bit/2;
				mTx->AdvanceToNextEdge();
				mTx->Advance( samples_to_first_center_of_first_data_bit );
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				bit_counter = 0;
				for(U32 i=0; i<16; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
						mTx->AdvanceToNextEdge();
						break;
					}
					//when 8 bits are sampled ->skip stop and start bit
					if (bit_counter==8) {
						ComputeCRC(data_byte);
						mTx->AdvanceToNextEdge();
						mTx->Advance( samples_to_first_center_of_first_data_bit );
						bit_counter = 0;
						data_byte=0;
						first_byte = 0;
					}
				//let's put a dot exactly where we sample this bit:
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
					bit_counter++;
					if( mTx->GetBitState() == BIT_HIGH ) {
						value = 1;
						value_byte = 1;
						for (U32 j=0; j<i; j++) {

							value *=2;
						}
						if (!first_byte) {
							for (U32 j=0; j<i-8; j++) {

								value_byte *=2;
							}
							data_byte += value_byte;
						}
						else {
							data_byte += value;
						}
						data += value;
					}
					label = 'SIZE';
					mTx->Advance(samples_per_bit);
				}
				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error=0;
					noop = 1;
					break;
				}
				size = data;
				ComputeCRC(data_byte);
				if (size==0) state = CRC;				//if no data -> go to crc state
				else state = DATA;
				data_byte =0;
				break;
			}
			case DATA:
			{
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				starting_sample += samples_per_bit*2 - samples_per_bit/2;
				mTx->AdvanceToNextEdge();
				mTx->Advance( samples_to_first_center_of_first_data_bit );
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					noop=1;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					data_idx=0;
					break;
				}

				for (U32 i=0; i<8; i++)
				{
					//if there's no edge for the duration of timeout -> reset
					if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
						transmission_error = 1;
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
						mTx->AdvanceToNextEdge();
						break;
					}
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );

					//data computation - lsb first inversion
					if( mTx->GetBitState() == BIT_HIGH )
					{
						value = 1;
						for (U32 j=0; j<i; j++)
							value *=2;

						data += value;
						data_byte += value;
					}
					label = data_idx;
					mTx->Advance( samples_per_bit);
				}
				//reset
				if (transmission_error) {
					state = WAIT;
					transmission_error=0;
					noop = 1;
					data_idx=0;
					break;
				}
				ComputeCRC(data_byte);
				data_idx++;
				data_byte=0;
				//if data finished -> go to next state, else continue with data comp
				if (data_idx == size || data_idx == 128) {
					state = CRC;
					data_idx=0;
				}
				break;
			}
			case CRC:
			{
				//if there's no edge for the duration of timeout -> reset
				if (!mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
					state = WAIT;
					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
					mTx->AdvanceToNextEdge();
					noop=1;
					break;
				}
				starting_sample += samples_per_bit*2 - samples_per_bit/2;
				mTx->AdvanceToNextEdge();
				mTx->Advance( samples_to_first_center_of_first_data_bit );
				bit_counter = 0;
				for (U32 i=0; i<16; i++)
				{
					if ( bit_counter == 8 )  //if 8 bits sampled -> skip stop and start bit
					{
						tracking = mTx->GetSampleNumber();
						mTx->AdvanceToNextEdge();
						mTx->Advance( samples_to_first_center_of_first_data_bit );
						bit_counter = 0;
					}
					//crc computation - lsb first inversion
					if( mTx->GetBitState() == BIT_HIGH )
					{
						value = 1;
						for (U32 j=0; j<i; j++) {

							value *=2;
						}
						data += value;
					}

					if ( (i == 15) && (ack == 1) )
					{
						//Last bit of CRC - Timeout timer is on!
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
						tracking = mTx->GetSampleNumber() + samples_per_bit/2;
						mResults->AddMarker( tracking, AnalyzerResults::Start, mSettings->mTxChannel );
						timer = 0;
					}
					else
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );

					bit_counter++;
					mTx->Advance( samples_per_bit);
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
					data_idx=0;
					break;
				}
				bit_counter=0;
				state = WAIT;
				break;
			}
			case ACK:
			{
				//we enter this case after the wait state and only if we find and ack before timeout
				mTx->Advance(samples_per_bit/2);
				for(U32 i=0; i<8; i++)
				{
					//tracking and timer measure the time until the timeout
				  if ( timer < timeout )
						tracking+=samples_per_bit;

					mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );

					bit_counter++;
					//ack computation - lsb first inversion
					if( mTx->GetBitState() == BIT_HIGH ) {
						value = 1;
						for (U32 j=0; j<i; j++)
							value *=2;

						data += value;
					}
					label = 'ACK';
					timer++;
					mTx->Advance(samples_per_bit);
				}
				state = WAIT;
				ack_done=1;
				ack=0;
				bit_counter=0;
				break;
			}
		}
		if (noop) ack=0;

		//Set Frame Data.
		Frame frame;
		frame.mData1 = label;
		frame.mData2 = data;
		frame.mFlags = 0;
		frame.mStartingSampleInclusive = starting_sample;
		frame.mEndingSampleInclusive = mTx->GetSampleNumber() - samples_per_bit/2;

		//send a frame only if there is not a transmission_error
		if (!noop) {
			mResults->AddFrame( frame );
			mResults->CommitResults();
			ReportProgress( frame.mEndingSampleInclusive );
		}
		//wait state is used when we wait for an ack or for a new message
		if (state==WAIT)
		{
			while (1)
			{
				bit_counter=0;
				//Case where CRC was not good - in wait state until it finds a new msg - timeout period without a new msg
				if (transmission_error)
				{
					if (mTx->WouldAdvancingCauseTransition(timeout*samples_per_bit)) {
						mTx->AdvanceToNextEdge();
						continue;
					}
					mTx->AdvanceToNextEdge();
					state = PROTOCOL;
					ack = 0;
					found_ack=0;
					break;
				}
				//case where we wait for an ack --> Timer is ON
				else if (ack) {
					for (U32 i=0 ; i < timeout; i++)
					{
						if( mTx->GetBitState() == BIT_LOW )
						{
							mTx->AdvanceToNextEdge();
							timer++;
							found_ack=1;
							break;
						}
						if (timer != timeout-1)
 							mTx->Advance( samples_per_bit);
						else
							mTx->Advance( samples_per_bit/2);
						tracking = mTx->GetSampleNumber();
						starting_sample+=samples_per_bit;
						timer++;
					}
					//ack reception before timeout failed
					if (!found_ack)
						mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
				}
				//if ack is found, go to state ACK
				if (found_ack)
				{
					state=ACK;
					ack=0;
					found_ack=0;
					break;
				}
				//when timeout ends after the end of the ack transmission
				if (ack_done && timer < timeout)
				{
					timer++;
					tracking += samples_per_bit;
					continue;
				}
				else if (ack_done && timer==timeout)
				{
					mResults->AddMarker( tracking + samples_per_bit/2 , AnalyzerResults::Stop, mSettings->mTxChannel);
					timer = 0;
					ack_done=0;
				}
				ack=0;

				if (!noop)
					mTx->AdvanceToNextEdge();

				starting_sample+=samples_per_bit;
				if( mTx->GetBitState() == BIT_LOW )
				{
					state=PROTOCOL;
					break;
				}
			}
		}
	}
	}
	else {																								//Rx and Tx Channels

		//Initial Position in Rx Channel
		mRx = GetAnalyzerChannelData( mSettings->mRxChannel );
		if( mRx->GetBitState() == BIT_LOW )
			mRx->AdvanceToNextEdge();
		mRx->AdvanceToNextEdge();
		mRx->Advance( samples_to_first_center_of_first_data_bit );

		for( ; ; )
		{
			U64 label, data = 0, received_data=0;							//frames' info
			U64 value = 0, value_byte = 0;										//data & crc calculation helpers
			U8 dd=0, dd2=0;																		//
			bool transmission_error = 0, noop=0;							//reset and collision notifiers
			U64 starting_sample=0;														//points to the beginning of a frame

			//if Rx is 1 then msg in Rx, else msg in Tx

			//find the beginning of a frame, depending on the channel we have a full msg
			if (Rx_msg)
				starting_sample = mRx->GetSampleNumber();
			else
				starting_sample = mTx->GetSampleNumber();

			switch (state) {
				case PROTOCOL:
				{
					if (mRx->GetSampleNumber() < mTx->GetSampleNumber() && mTx->GetSampleNumber() - mRx->GetSampleNumber() < timeout*samples_per_bit) {
						collision_detection = 1;
						Rx_msg=1;
					}
					data_byte=0;
					ack=0;
					crc_val = 0xFFFF;											//crc initial value
					if (!Rx_msg)													//when data exist in channel Tx, move the Tx pointer
						mTx->Advance( samples_to_first_center_of_first_data_bit );
					else
						starting_sample -= samples_per_bit/2; //skip the start bit

					for(U32 i=0; i<4; i++)
					{
						if (Rx_msg)				//msg in Rx
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mRx->GetSampleNumber() - starting_sample > timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );

							if (collision_detection)			//add error X
							{
								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
								mTx->Advance(samples_per_bit);
							}
							//protocol value computation -> lsb first inversion
							if( mRx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								data += value;
								data_byte += value;
							}
							label = 'PROT';
							mRx->Advance(samples_per_bit);
						}
						else {
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mTx->GetSampleNumber() - starting_sample > timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							//let's put a dot exactly where we sample this bit:
							mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
							//square - bit received by Rx
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Square, mSettings->mRxChannel );
							//protocol value computation - lsb first inversion
							if( mTx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								data += value;
								data_byte += value;
							}
							//protocol value in Rx channel - lsb first inversion
							if( mRx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								received_data += value;
							}
							label = 'PROT';
							mTx->Advance(samples_per_bit);
							mRx->Advance(samples_per_bit);
						}
						bit_counter++;
					}
					//reset
					if (transmission_error) {
						state = WAIT;
						noop = 1;
						bit_counter = 0;
						transmission_error=0;
						break;
					}
					//if data in Tx and Rx not equal -> collision detection
					if (!Rx_msg && received_data!=data) {
						collision_detection=1;
						Rx_msg=1;
						data_byte = received_data;
					}
					first_byte = 1;
					state = TARGET;
					break;
				}
				case TARGET:
				{
					starting_sample -= samples_per_bit/2;
					for(U32 i=0; i<12; i++)
					{
						//if 8 bits are sampled, skip the stop and start bit
						if (bit_counter==8) {
							dd = dd << 4;
							data_byte += dd;
							ComputeCRC(data_byte);
							if (!Rx_msg||collision_detection)				//when data exist in channel Tx, move the Tx pointer
							{
								mTx->AdvanceToNextEdge();
								mTx->Advance( samples_to_first_center_of_first_data_bit );
							}
							mRx->AdvanceToNextEdge();
							mRx->Advance( samples_to_first_center_of_first_data_bit );
							data_byte = 0;
							dd = 0;
							bit_counter = 0;
							first_byte = 0;
						}

						if (Rx_msg) {
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mRx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							if (collision_detection)		//if collision ad an errorX to Tx
								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );
						}
						else
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mTx->GetSampleNumber() - starting_sample > timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Square, mSettings->mRxChannel );
						}
						bit_counter++;
						if (Rx_msg)
						{
							//calculate target id - lsb first inversion
							if( mRx->GetBitState() == BIT_HIGH ) {
								value = 1;
								value_byte = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								if (!first_byte) {
									for (U32 j=0; j<i-4; j++) {
										value_byte *=2;
									}
									dd += value_byte;
								}
								else {
									dd += value;
								}
								data += value;
							}
						}
						else
						{
							//calculate target id in Tx - lsb first inversion
							if( mTx->GetBitState() == BIT_HIGH ) {
								value = 1;
								value_byte = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								if (!first_byte) {
									for (U32 j=0; j<i-4; j++) {
										value_byte *=2;
									}
									dd += value_byte;
								}
								else {
									dd += value;
								}
								data += value;
							}
							//calculate target id in Rx - lsb first inversion
							if( mRx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								if (!first_byte) {
									for (U32 j=0; j<i-4; j++) {
										value_byte *=2;
									}
									dd2 += value_byte;
								}
								else {
									dd2 += value;
								}
								received_data += value;
							}
						}
						//if Rx and Tx data are not equal -> collision detection
						//after a collision detection, the data of Rx are monitored
						if (!Rx_msg && received_data!=data) {
							collision_detection=1;
							data = received_data;
							Rx_msg=1;
							dd = dd2;
						}
						label = 'TRGT';
						if (!Rx_msg||collision_detection)
							mTx->Advance( samples_per_bit);
						mRx->Advance( samples_per_bit);
					}
					//reset
					if (transmission_error) {
						state = WAIT;
						transmission_error=0;
						noop = 1;
						break;
					}
					data_byte = dd;
					ComputeCRC(data_byte);
					target = data;
					data_byte=0;
					state = TARGET_MODE;
					break;
				}
				case TARGET_MODE:
				{
					starting_sample += samples_per_bit*2 - samples_per_bit/2;
					if (!Rx_msg||collision_detection)							//when data exist in channel Tx, move the Tx pointer
					{
						mTx->AdvanceToNextEdge();
						mTx->Advance( samples_to_first_center_of_first_data_bit );
					}
					mRx->AdvanceToNextEdge();
					mRx->Advance( samples_to_first_center_of_first_data_bit );
					bit_counter=0;
					for(U32 i=0; i<4; i++)
					{
						bit_counter++;
						if (Rx_msg)		//msg in Rx channel
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mRx->GetSampleNumber() - starting_sample > timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							if (collision_detection)
							{
								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
								mTx->Advance( samples_per_bit);
							}
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );
							//Target mode computation - lsb first inversion
							if( mRx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++) {

									value *=2;
								}
								data += value;
								data_byte += value;
							}
							label = 'MODE';
							mRx->Advance( samples_per_bit);
						}
						else
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mTx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Square, mSettings->mRxChannel );
							//Target mode computation - lsb first inversion
							if( mTx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++) {

									value *=2;
								}
								data += value;
								data_byte += value;
							}
							//Target mode received in Rx channel
							if( mRx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								received_data += value;
							}
							label = 'MODE';
							mTx->Advance( samples_per_bit);
							mRx->Advance( samples_per_bit);
						}
					}
					//reset
					if (transmission_error) {
						state = WAIT;
						transmission_error=0;
						noop = 1;
						bit_counter=0;
						break;
					}
					//when target = IDACK | NODEIDACK, ack notifier is ON
					if ( data==1 || data==6 ) {
						ack=1;
					}
					state = SOURCE;
					first_byte = 1;
					break;
				}
				case SOURCE:
				{
					starting_sample -= samples_per_bit/2;
					for(U32 i=0; i<12; i++)
					{
						//if 8 bits are sampled, skip the stop and start bit
						if (bit_counter==8) {
							dd = dd << 4;
							data_byte += dd;
							ComputeCRC(data_byte);
							if (!Rx_msg)	//when data in Tx, move Tx pointer
							{
								mTx->AdvanceToNextEdge();
								mTx->Advance( samples_to_first_center_of_first_data_bit );
							}
							mRx->AdvanceToNextEdge();
							mRx->Advance( samples_to_first_center_of_first_data_bit );
							bit_counter = 0;
							data_byte=0;
							dd=0;
							first_byte = 0;
						}
						bit_counter++;
						if (Rx_msg)		//msg in Rx
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mRx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							if (collision_detection)	//error X
								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );
							//Source id computation - lsb first inversion
							if( mRx->GetBitState() == BIT_HIGH ) {
								value = 1;
								value_byte = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								if (!first_byte)
								{
									for (U32 j=0; j<i-4; j++)
									{
										value_byte *=2;
									}
									dd += value_byte;
								}
								else {
									dd += value;
								}
								data += value;
							}
							label = 'SRC';
							mRx->Advance( samples_per_bit);
						}
						else
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mTx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Square, mSettings->mRxChannel );
							//Source id sent from Tx - lsb first inversion
							if( mTx->GetBitState() == BIT_HIGH ) {
								value = 1;
								value_byte = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								if (!first_byte)
								{
									for (U32 j=0; j<i-4; j++)
									{
										value_byte *=2;
									}
									dd += value_byte;
								}
								else {
									dd += value;
								}
								data += value;
							}
							label = 'SRC';
							mTx->Advance( samples_per_bit);
							mRx->Advance( samples_per_bit);
						}
					}
					//reset
					if (transmission_error) {
						state = WAIT;
						transmission_error=0;
						noop = 1;
						break;
					}
					data_byte = dd;
					ComputeCRC(data_byte);
					data_byte=0;
					source = data;
					state=CMD;
					break;
				}
				case CMD:
				{

					starting_sample += samples_per_bit*2 - samples_per_bit/2;
					if (!Rx_msg)		//when data in Tx , move Tx pointer
					{
						mTx->AdvanceToNextEdge();
						mTx->Advance( samples_to_first_center_of_first_data_bit );
					}
					mRx->AdvanceToNextEdge();
					mRx->Advance( samples_to_first_center_of_first_data_bit );
					bit_counter = 0;
					for(U32 i=0; i<8; i++)
					{
						//let's put a dot exactly where we sample this bit:
						bit_counter++;
						if (Rx_msg)
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mRx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							if (collision_detection) //error X
								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );
							//cmd computation - lsb first inversion
							if( mRx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								data += value;
								data_byte +=value;
							}
							label = 'CMD';
							mRx->Advance(samples_per_bit);
						}
						else
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mTx->GetSampleNumber() - starting_sample > timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );

							if( mTx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								data += value;
								data_byte +=value;
							}
							label = 'CMD';
							mTx->Advance(samples_per_bit);
							mRx->Advance(samples_per_bit);
						}
					}
					if (transmission_error) {
						state = WAIT;
						transmission_error=0;
						noop = 1;
						break;
					}
					state = SIZE;
					first_byte = 1;
					ComputeCRC( data_byte);
					data_byte =0;
					break;
				}
				case SIZE:
				{
					starting_sample += samples_per_bit*2 - samples_per_bit/2;
					if (Rx_msg)
					{
						mRx->AdvanceToNextEdge();
						mRx->Advance( samples_to_first_center_of_first_data_bit );
					}
					else
					{
						mTx->AdvanceToNextEdge();
						mTx->Advance( samples_to_first_center_of_first_data_bit );
					}
					bit_counter = 0;
					for(U32 i=0; i<16; i++)
					{
						//if 8 bits are sampled, skip the stop and start bit
						if (bit_counter==8) {
							ComputeCRC(data_byte);
							if (Rx_msg)				//if msg in Rx, move Rx pointer
							{
								mRx->AdvanceToNextEdge();
								mRx->Advance( samples_to_first_center_of_first_data_bit );
							}
							else {						//if msg in Tx, move Tx pointer
								mTx->AdvanceToNextEdge();
								mTx->Advance( samples_to_first_center_of_first_data_bit );
							}
							bit_counter = 0;
							data_byte=0;
							first_byte = 0;
						}

						if (Rx_msg) {				//msg in Rx
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mRx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							if (collision_detection)
								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );
							bit_counter++;
							//Size - lsb first inversion
							if( mRx->GetBitState() == BIT_HIGH ) {
								value = 1;
								value_byte = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								if (!first_byte) {
									for (U32 j=0; j<i-8; j++) {

										value_byte *=2;
									}
									data_byte += value_byte;
								}
								else {
									data_byte += value;
								}
								data += value;
							}
							label = 'SIZE';
							mRx->Advance(samples_per_bit);
						}
						else {
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mTx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
							bit_counter++;
							//Size - lsb first inversion
							if( mTx->GetBitState() == BIT_HIGH ) {
								value = 1;
								value_byte = 1;
								for (U32 j=0; j<i; j++) {
									value *=2;
								}
								if (!first_byte) {
									for (U32 j=0; j<i-8; j++) {

										value_byte *=2;
									}
									data_byte += value_byte;
								}
								else {
									data_byte += value;
								}
								data += value;
							}
							label = 'SIZE';
							mTx->Advance(samples_per_bit);
						}
					}
					if (transmission_error) {
						state = WAIT;
						transmission_error=0;
						noop = 1;
						break;
					}
					size = data;
					ComputeCRC(data_byte);
					if (size==0) {state = CRC;}
					else {state = DATA;}

					data_byte =0;
					break;
				}
				case DATA:
				{
					starting_sample += samples_per_bit*2 - samples_per_bit/2;

					if (Rx_msg)
					{
						mRx->AdvanceToNextEdge();
						mRx->Advance( samples_to_first_center_of_first_data_bit );

						for (U32 i=0; i<8; i++)
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mRx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							if (collision_detection) //error X
								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
							mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );
							//Data - lsb first inversion
							if( mRx->GetBitState() == BIT_HIGH )
							{
								value = 1;

								for (U32 j=0; j<i; j++)
								{
									value *=2;
								}

								data += value;
								data_byte += value;
							}
							label = data_idx;
							mRx->Advance( samples_per_bit);
						}
					}
					else {
						mTx->AdvanceToNextEdge();
						mTx->Advance( samples_to_first_center_of_first_data_bit );

						for (U32 i=0; i<8; i++)
						{
							//if the time between the current moment and the last transition is more than timeout -> reset
							if (mTx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
							//Data - lsb first inversion
							if( mTx->GetBitState() == BIT_HIGH )
							{
								value = 1;
								for (U32 j=0; j<i; j++)
								{
									value *=2;
								}
								data += value;
								data_byte += value;
							}
							label = data_idx;
							mTx->Advance( samples_per_bit);
						}
					}
					//reset
					if (transmission_error) {
						state = WAIT;
						transmission_error=0;
						noop = 1;
						data_idx=0;
						break;
					}
					ComputeCRC(data_byte);
					data_idx++;
					data_byte=0;
					//if data number reach the size or the maximum data size -> terminate the reception
					if (data_idx == size || data_idx == 128) {
						state = CRC;
						data_idx=0;
					}
					break;
				}
				case CRC:
				{
					starting_sample += samples_per_bit*2 - samples_per_bit/2;
					if (Rx_msg)	//msg in Rx
					{
						mRx->AdvanceToNextEdge();
						mRx->Advance( samples_to_first_center_of_first_data_bit );

						bit_counter = 0;
						for (U32 i=0; i<16; i++)
						{
							if (mRx->GetSampleNumber() - starting_sample >= timeout*samples_per_bit) {
								transmission_error = 1;
								break;
							}
							if ( bit_counter == 8 )  //if 8 bits sampled, skip Start and Stop bit
							{
								mRx->AdvanceToNextEdge();
								mRx->Advance( samples_to_first_center_of_first_data_bit );
								bit_counter = 0;
							}
							//crc reception
							if( mRx->GetBitState() == BIT_HIGH )
							{
								value = 1;
								for (U32 j=0; j<i; j++) {

									value *=2;
								}
								data += value;
							}
							if ( (i == 15) && (ack == 1) )
							{
								if (collision_detection)		//errorX
									mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
								//Last bit of CRC - Timeout timer is on!
								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );
								tracking = mRx->GetSampleNumber() + samples_per_bit/2;
								mResults->AddMarker( tracking, AnalyzerResults::Start, mSettings->mTxChannel );
								mResults->AddMarker( tracking, AnalyzerResults::Start, mSettings->mRxChannel );
								timer = 0;
							}
							else{
								if (collision_detection)		//errorX
									mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel );
									mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );
							}
							bit_counter++;
							mRx->Advance( samples_per_bit);
						}
					}
					else {																							 	//msg in Tx
						mTx->AdvanceToNextEdge();
						mTx->Advance( samples_to_first_center_of_first_data_bit );

						bit_counter = 0;
						for (U32 i=0; i<16; i++)
						{
							if (mTx->GetSampleNumber() - starting_sample > timeout*samples_per_bit) {		//if no data for timeout period - error
								transmission_error = 1;
								break;
							}
							if ( bit_counter == 8 )  //if 8 bits sampled, skip Start and Stop bit
							{
								mTx->AdvanceToNextEdge();
								mTx->Advance( samples_to_first_center_of_first_data_bit );
								bit_counter = 0;
							}
							//crc reception
							if( mTx->GetBitState() == BIT_HIGH )
							{
								value = 1;
								for (U32 j=0; j<i; j++) {

									value *=2;
								}
								data += value;
							}
							if ( (i == 15) && (ack == 1) )
							{
								//Last bit of CRC - Timeout timer is on!
								mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );
								tracking = mTx->GetSampleNumber() + samples_per_bit/2;
								mRx->AdvanceToAbsPosition(mTx->GetSampleNumber());
								mResults->AddMarker( tracking, AnalyzerResults::Start, mSettings->mTxChannel );			//green symbol for timeout start
								mResults->AddMarker( tracking, AnalyzerResults::Start, mSettings->mRxChannel );			//green symbol for timeout beginning
								timer = 0;
							}
							else
								mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );

							bit_counter++;
							mTx->Advance( samples_per_bit);
						}
					}
					if (transmission_error) {
						state = WAIT;
						transmission_error=0;
						noop=1;
						break;
					}
					//crc evaluation
					if (data == crc_val)
						label = 'CRC';
					else {
						label = 'NOT';
						transmission_error = 1;
					}

					if (collision_detection && Rx_msg) {			//end of message
						collision_detection = 0;
						ack=0;
						mTx->AdvanceToAbsPosition(mRx->GetSampleNumber());
						mTx->AdvanceToNextEdge();
					}
					bit_counter=0;
					state = WAIT;
					break;
				}
				case ACK:
				{
					if (target==source)				//ack to the same channel with the msg
					{
						mTx->Advance(samples_per_bit/2);
						tracking += samples_per_bit/2;
						for(U32 i=0; i<8; i++)
						{
							if ( timer < timeout )
								tracking+=samples_per_bit;

							mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );

							bit_counter++;
							if( mTx->GetBitState() == BIT_HIGH ) {
								value = 1;
								for (U32 j=0; j<i; j++)
									value *=2;

								data += value;
							}
							label = 'ACK';
							timer++;

							mTx->Advance(samples_per_bit);
						}
					}
					else			//ack to the other channel => if msg in Rx -> ack to Tx / if msg in Tx -> ack to Rx
					{
						if (Rx_msg)
						{
							if (target==0) {		//if target = 0 and message to Rx -> ack to Rx
								mRx->Advance(samples_per_bit/2);
								for(U32 i=0; i<8; i++)
								{
									if ( timer < timeout )			//is timeout already? if not advance to next position
										tracking+=samples_per_bit;

									mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );

									bit_counter++;
									if( mRx->GetBitState() == BIT_HIGH ) {		//ack value - lsb inversion
										value = 1;
										for (U32 j=0; j<i; j++)
											value *=2;

										data += value;
									}
									label = 'ACK';
									timer++;

									mRx->Advance(samples_per_bit);
								}
							}
							else {		//ack to Tx channel
								mTx->Advance(samples_per_bit/2);
								for(U32 i=0; i<8; i++)
								{
									if ( timer < timeout )		//is timeout already? if not advance to next position
										tracking+=samples_per_bit;

									mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mTxChannel );

									bit_counter++;
									if( mTx->GetBitState() == BIT_HIGH ) {
										value = 1;
										for (U32 j=0; j<i; j++)
											value *=2;

										data += value;
									}
									label = 'ACK';
									timer++;

									mTx->Advance(samples_per_bit);
								}

								mRx->AdvanceToAbsPosition(mTx->GetSampleNumber());
								mTx->AdvanceToNextEdge();
							}
						}
						else	//ack to Rx channel
						{
							mRx->Advance(samples_per_bit/2);	//find ack position
							tracking += samples_per_bit/2;		//timeout tracking
							for(U32 i=0; i<8; i++)
							{
								if ( timer < timeout )				//is timeout already? if not advance to next position
									tracking+=samples_per_bit;

								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mRxChannel );

								bit_counter++;
								if( mRx->GetBitState() == BIT_HIGH ) {		//ack value
									value = 1;
									for (U32 j=0; j<i; j++)
										value *=2;

									data += value;
								}
								label = 'ACK';
								timer++;

								mRx->Advance(samples_per_bit);
							}
							mTx->AdvanceToAbsPosition(mRx->GetSampleNumber());			//Tx reach the position of Rx
						}
					}
					state = WAIT;
					ack_done=1;					//ack happened
					ack=0;
					bit_counter=0;
					break;
				}
			}

			Frame frame;
			frame.mData1 = label;
			frame.mData2 = data;
			frame.mFlags = 0;
			frame.mStartingSampleInclusive = starting_sample;

			if (Rx_msg)
				frame.mEndingSampleInclusive = mRx->GetSampleNumber() - samples_per_bit/2;
			else
				frame.mEndingSampleInclusive = mTx->GetSampleNumber() - samples_per_bit/2;
			if (!noop) {			//if noop=1 ->send no frame
				mResults->AddFrame( frame );
				mResults->CommitResults();
				ReportProgress( frame.mEndingSampleInclusive );
			}
			/*This state is the default state. It is enabled in case of no data, transmission_error, ack wait, end of msg, etc
			and it waits until the next msg|*/
			if (state==WAIT)
			{
				while (1)
				{
					if (transmission_error)			//This is the handling of a transmission_error in case CRC is not good! Wait until we find a no data timeout period
					{
						if (Rx_msg)
						{
							tracking =  mRx->GetSampleNumber();
							mRx->AdvanceToNextEdge();
							//in case Tx is left in the previous msg
							if ( (mRx->GetSampleNumber() > mTx->GetSampleNumber()) && (mRx->GetSampleNumber() - mTx->GetSampleNumber() >= timeout*samples_per_bit))
								mTx->AdvanceToNextEdge();
							if (mRx->GetSampleNumber() - tracking >= timeout*samples_per_bit)	//when no data for timeout seconds
							{
								if (mRx->GetSampleNumber() < mTx->GetSampleNumber()) {			//if we found a msg in Rx earlier Rx_msg
									if (mTx->GetSampleNumber() - mRx->GetSampleNumber()  <= timeout*samples_per_bit)	//if we also have a Tx msg close to Rx ->collision
										collision_detection = 1;
									else collision_detection = 0;
									Rx_msg = 1;
								}
								else
									Rx_msg = 0;
								state = PROTOCOL;			//Restart
								if (!noop)
									mRx->Advance(samples_to_first_center_of_first_data_bit);
								bit_counter=0;
								ack = 0;
								found_ack=0;
								break;
							}
							else {		//error msg still hasnt finished - loop again
								state = WAIT;
								continue;
							}
						}
						else			//same for Tx msg
						{
							tracking =  mTx->GetSampleNumber();

							mTx->AdvanceToNextEdge();
							if (mRx->GetSampleNumber() < mTx->GetSampleNumber())
								mRx->AdvanceToNextEdge();

							if (mTx->GetSampleNumber() - tracking >= timeout*samples_per_bit)
							{
								if (mRx->GetSampleNumber() < mTx->GetSampleNumber()) {
									if (mTx->GetSampleNumber() - mRx->GetSampleNumber()  < timeout*samples_per_bit)
										collision_detection = 1;
									else collision_detection = 0;
									Rx_msg = 1;
								}
								else
									Rx_msg=0;

								state = PROTOCOL;
								if (!noop)
									mRx->Advance(samples_to_first_center_of_first_data_bit);
								bit_counter=0;
								ack = 0;
								found_ack=0;
								break;
							}
							else {
								state= WAIT;
								continue;
							}
						}
					}
					else if (ack) {			//Case when we wait for an ack - We wait until we find data, while timer is ON.

						for (U32 i=0 ; i < timeout; i++)
						{
							if (target == 0 && Rx_msg)			//ack in Rx
							{
								if( mRx->GetBitState() == BIT_LOW )
								{
									mRx->AdvanceToNextEdge();
									timer++;
									found_ack=1;
									break;
								}
								if (timer != timeout-1)
									mRx->Advance( samples_per_bit);
								else
									mRx->Advance( samples_per_bit/2);
								tracking = mRx->GetSampleNumber();
								starting_sample+=samples_per_bit;
								timer++;
							}
							else if ( target == source )			//ack in the same channel with the msg
							{
								if (Rx_msg)
								{
									if( mRx->GetBitState() == BIT_LOW )
									{
										mRx->AdvanceToNextEdge();
										timer++;
										found_ack=1;
										break;
									}
									if (timer != timeout-1)
										mRx->Advance( samples_per_bit);
									else
										mRx->Advance( samples_per_bit/2);
									tracking = mRx->GetSampleNumber();
									starting_sample+=samples_per_bit;
									timer++;
								}
								else
								{
									mRx->Advance(1);
									if( mTx->GetBitState() == BIT_LOW )
									{
										mTx->AdvanceToNextEdge();
										timer++;
										found_ack=1;
										break;
									}
									if (timer != timeout-1)
										mTx->Advance( samples_per_bit);
									else
										mTx->Advance( samples_per_bit/2);
									tracking = mTx->GetSampleNumber();
									starting_sample+=samples_per_bit;
									timer++;
								}
							}
							else		//ack in the other channel
							{
								if (Rx_msg)
								{
									//data found before the end of the timeout
									if(( mTx->GetBitState() == BIT_LOW ) && ( mTx->GetSampleNumber() - mRx->GetSampleNumber() < timeout*samples_per_bit))
									{
										found_ack = 1;
										mTx->AdvanceToNextEdge();
									}
									else	// ack reception failure
										found_ack = 0;

									mRx->AdvanceToAbsPosition(mTx->GetSampleNumber());
									break;

								}
								else
								{
									if( mRx->GetBitState() == BIT_LOW )
									{
										mRx->AdvanceToNextEdge();
										mTx->AdvanceToAbsPosition(mRx->GetSampleNumber());
										tracking+=samples_per_bit;
										timer++;
										found_ack=1;
										break;
									}
									if (timer != timeout-1)
										mRx->Advance( samples_per_bit);
									else
										mRx->Advance( samples_per_bit/2);
									tracking = mRx->GetSampleNumber();
									starting_sample+=samples_per_bit;
									timer++;
								}

							}
						}
						//ack reception failure
						if (!found_ack)
						{
							if (target==source)
								mResults->AddMarker( mTx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mTxChannel);
							else
								mResults->AddMarker( mRx->GetSampleNumber(), AnalyzerResults::ErrorX, mSettings->mRxChannel);
						}
					}
					if (found_ack)		//We found Ack - process continued to the ACK state
					{
						state=ACK;
						ack=0;
						found_ack=0;
						break;
					}
					/*when timeout ends after the end of the ack transmission
					timer continues to increase, in order to show the end of the timeout period*/
					if (ack_done && timer < timeout)
					{
						timer++;
						tracking += samples_per_bit;
						continue;
					}
					else if (ack_done && timer==timeout)		//Good case scenario - end of the timeout after the end of the ack
					{
						mResults->AddMarker( tracking, AnalyzerResults::Stop, mSettings->mTxChannel);
						mResults->AddMarker( tracking, AnalyzerResults::Stop, mSettings->mRxChannel);
						timer = 0;
						ack_done = 0;
					}
					ack=0;
					//Initialization of the next message - Reset state machine
					if (!Rx_msg)
						mTx->AdvanceToNextEdge();
					if (!noop) {				//Normally, in Tx msg Rx is left behind, so we advance to the next msg
						mRx->AdvanceToNextEdge();
						mRx->Advance( samples_to_first_center_of_first_data_bit );
					}
					if (mRx->GetSampleNumber() < mTx->GetSampleNumber())		//data found in Rx? ->Rx msg
					{
						if (mTx->GetSampleNumber() - mRx->GetSampleNumber()  < timeout*samples_per_bit)		//Data found in Tx also -> collision
							collision_detection = 1;
						else collision_detection = 0;

						Rx_msg = 1;
						starting_sample+=samples_per_bit;
						if( mRx->GetBitState() == BIT_LOW )		//reset
						{
							state=PROTOCOL;
							break;
					 	}
					}
					else {
						noop=1;
						if (mRx->GetSampleNumber() - mTx->GetSampleNumber() >= timeout*samples_per_bit) {		//In case of a previous error if we still have data we wait
							Rx_msg=0;
							transmission_error = 1;
							continue;
						}
						//We finally can go to the next msg
						collision_detection = 0;
						Rx_msg = 0;
						starting_sample+=samples_per_bit;
						if( mTx->GetBitState() == BIT_LOW )
						{
							state=PROTOCOL;
							break;
						}
					}
				}
			}
		}
	}
}

/*Function to compute CRC value - Called every 8 bits*/
void ComputeCRC (uint8_t data) {

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

U32 LuosAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate, SimulationChannelDescriptor** simulation_channels )
{
	if( mSimulationInitilized == false )
	{
		mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
		mSimulationInitilized = true;
	}

	return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
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

void DestroyAnalyzer( Analyzer* analyzer )
{
	delete analyzer;
}
