/*
 	This module contains the implementation of a base class from which the actual CAN bus
	profile can be derived.


    (c) 2022 Christian Schmidmer, use is subject to MIT license
*/

#include <arduino.h>
#include "SimpleCAN.h"
#include "ThreadSafeQueue.h"

#define RX_QUEUE_SIZE	16		// Max number of received CAN messages stored max in Rx queue

class CanRxMessage
{
	public:
		SimpleCanRxHeader SCHeader;
		uint8_t Data[8];
};

SafeQueue<CanRxMessage> RxQueue(RX_QUEUE_SIZE);


// This is the message hadler for the profile!
static  void HandleCanMessage(SimpleCanRxHeader rxHeader, uint8_t *rxData, void* userData)
{
    SimpleCANProfile* pClass = (SimpleCANProfile*)userData;
    pClass->HandleCanMessage(rxHeader, rxData);
}



SimpleCANProfile::SimpleCANProfile(SimpleCan* _pCan)
{
	Can1 = _pCan;
}


void SimpleCANProfile::Init(CanIDFilter IDFilterFunc)
{
	Serial.println(Can1->Init(SCCanSpeed::Mbit1, IDFilterFunc) == CAN_OK
		? "CAN: initialized."
		: "CAN: error when initializing.");


	SCCanStatus rc=CAN_OK;

	#if 0
	-> This should go into the application code, since it is specific to the CAN IDs used by the application! 
	// Configure Rx filter
	FilterDefinition FilterConfig;
	FilterConfig.IdType = CAN_STDID;
	FilterConfig.FilterIndex = 0;
	FilterConfig.FilterType = CAN_FILTER_MASK;
	FilterConfig.FilterConfig = CAN_FILTER_TO_RXFIFO0;
	FilterConfig.FilterID1 = 0x0;
	FilterConfig.FilterID2 = 0x700;
	rc = Can1->ConfigFilter(&FilterConfig);
	#endif

	if (rc==CAN_OK) rc = Can1->ConfigGlobalFilter();
	if (rc==CAN_OK) rc = Can1->ActivateNotification(8, ::HandleCanMessage, this);			
	if (rc!=CAN_OK)	Serial.println("CAN initialization error!");

	if (rc==CAN_OK) Serial.println(Can1->Start() == CAN_OK
		? "CAN: started."
		: "CAN: error when starting.");
}


void SimpleCANProfile::CANSendText(const char* Text, int CanID)
{
	Can1->SendMessage((uint8_t*)Text,  strlen(Text), CanID);
}


void SimpleCANProfile::CANSendFloat(float Val, int CanID)
{
	Can1->SendMessage((uint8_t*)&Val,  sizeof(Val), CanID);
}


void SimpleCANProfile::CANSendFloat(float Val1, float Val2, int CanID)
{
    uint8_t Buffer[2*sizeof(float)];
    memcpy(Buffer, &Val1, sizeof(float));
    memcpy(Buffer+ sizeof(float), &Val2, sizeof(float));
	Can1->SendMessage(Buffer,  2*sizeof(float), CanID);
}


void SimpleCANProfile::CANSendInt(int32_t Val, int CanID)
{
	Can1->SendMessage((uint8_t*)&Val,  sizeof(int), CanID);
}


void SimpleCANProfile::CANSendInt(int32_t Val1, int32_t Val2, int CanID)
{
    uint8_t Buffer[2*sizeof(int32_t)];
    memcpy(Buffer, &Val1, sizeof(int32_t));
    memcpy(Buffer+ sizeof(int32_t), &Val2, sizeof(int32_t));
	Can1->SendMessage(Buffer,  2*sizeof(int32_t), CanID);
}


int SimpleCANProfile::CANGetInt(const uint8_t* pData)
{
	int Val;
	memcpy(&Val, pData, sizeof(int));
	return Val;
}


void SimpleCANProfile::CANGetInt(const uint8_t* pData, int32_t* pInt1, int32_t* pInt2)
{
	memcpy(pInt1, pData, sizeof(int));
	memcpy(pInt2, pData+sizeof(int), sizeof(int));
}


void SimpleCANProfile::CANGetString(const uint8_t* pData, char* pChar, const int MaxLen)
{
	memcpy(pChar, pData, MaxLen);
	pChar[MaxLen]= 0;
}


float SimpleCANProfile::CANGetFloat(const uint8_t* pData)
{
	float Val;
	memcpy(&Val, pData, sizeof(float));
	return Val;
}


void SimpleCANProfile::CANGetFloat(const uint8_t* pData, float* pVal1, float* pVal2)
{
	float Val;
	memcpy(pVal1, pData, sizeof(float));
	memcpy(pVal2, pData+sizeof(float), sizeof(float));
}


//--------------------------------------------------------------------------------------
// Implementation of the basic recieve handler, which is common for all platforms

RxHandlerBase::RxHandlerBase(uint16_t dataLength)
{
	_rxData = new byte[dataLength];
	_rxDataLength = dataLength;
}


void RxHandlerBase::SetProfileCallback(uint16_t dataLength, RxCallback callback, void* userData)
{
	ProfileCallback = callback;
	ProfileClass = userData;
}


RxHandlerBase::~RxHandlerBase()
{
	delete[] _rxData; 
}


// This function is called by the actual low level ISR when a new frame is received.
// ISR, absolutely no printing to serial!!
void RxHandlerBase::Notify(/*...*/)
{
	CanRxMessage Msg;

    // Copy the received message into the _rxData buffer
	if (!CANReadFrame(&Msg.SCHeader, Msg.Data, 8))
	{
		// Error_Handler();
		Serial.println("Error returned from ReadFrame!");
		delay(100);
	}

    // Let the hardware know the frame has been read.
	ReleaseRcvBuffer();

	if (RxQueue.NumElements<16)
		RxQueue.Enqueue(Msg);
}


bool RxHandlerBase::Loop()
{
	bool rc = false;

	// If a callback was registered, convert the header info and call the callback function.
	if (ProfileCallback != NULL)
	{
		CanRxMessage Msg;
 		if (RxQueue.Dequeue(&Msg))
		{
			// Serial.println("Read message from buffer");
			// --- Convert the header and call the user provided RX handler.
			ProfileCallback(Msg.SCHeader, Msg.Data, ProfileClass);
		}
	}
	else Serial.println("RxHandlerBase::Loop(): No callback set!");
	
	return rc;
}

