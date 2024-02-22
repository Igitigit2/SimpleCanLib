/*
 	This module contains the implementation of a base class from which the actual CAN bus
	profile can be derived.


    (c) 2022 Christian Schmidmer, use is subject to MIT license
*/

#include <Arduino.h>
#include "SimpleCAN.h"


SafeQueue<CanRxMessage> SimpleCan::RxQueue(RX_QUEUE_SIZE);
SafeQueue<CANTxMessage> SimpleCan::TxQueue(TX_QUEUE_SIZE);
bool SimpleCan::BlinkOnActivity=false;


// Set a filter to accept all incoming messages
SCCanStatus SimpleCan::AcceptAllMessages()
{
	SCCanStatus rc=CAN_OK;
	// Configure Rx filter
	FilterDefinition FilterConfig;
	FilterConfig.IdType = CAN_STDID;
	FilterConfig.FilterIndex = 0;
	FilterConfig.FilterType = CAN_FILTER_MASK;
	FilterConfig.FilterConfig = CAN_FILTER_TO_RXFIFO0;
	FilterConfig.FilterID1 = 0x0;
	FilterConfig.FilterID2 = 0x0;
	rc = ConfigFilter(&FilterConfig);
	return rc;
}

// Set a filter to deny all incoming messages (except for CAN ID 0xFFFF)
SCCanStatus SimpleCan::DenyAllMessages()
{
	SCCanStatus rc=CAN_OK;
	// Configure Rx filter
	FilterDefinition FilterConfig;
	FilterConfig.IdType = CAN_STDID;
	FilterConfig.FilterIndex = 0;
	FilterConfig.FilterType = CAN_FILTER_MASK;
	FilterConfig.FilterConfig = CAN_FILTER_TO_RXFIFO0;
	FilterConfig.FilterID1 = 0xffff;
	FilterConfig.FilterID2 = 0xffff;
	rc = ConfigFilter(&FilterConfig);
	return rc;
}

bool SimpleCan::SendMessage(const uint8_t* pData, int NumBytes, int CanID, bool UseEFF)
{
	// Skip command if sender ID is disabled.
	// if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return true; 

	// Skip sending if disabled.
	if (DoNotSend) return true;

	// Serial.printf("CAN: Queueing message with ID 0x%x, %d messages in TX queue.\n", CanID, TxQueue.NumElements);

	CANTxMessage Msg;
	Msg.CanID = CanID;
	Msg.EFF = UseEFF;
	Msg.Size = NumBytes;
	Msg.RTR = false;
	memcpy(Msg.Data, pData, NumBytes);
	if (TxQueue.NumElements<TX_QUEUE_SIZE)
		TxQueue.Enqueue(Msg);
	else 
	{
		Serial.println("CAN Error: Tx buffer overrun");
		
		// Kick sending of messages from buffer anyway.
		TriggerSending();
		return false;
	}
	
	// Kick sending of messages from buffer.
	
	TriggerSending();

	return true;
}


// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
bool SimpleCan::RequestMessage(int NumBytes, int CanID, bool UseEFF)
{
	// Skip command if sender ID is disabled.
	// if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return true; 

	// Skip sending if disabled.
	if (DoNotSend) return true;

	// PrintLog("CAN: Queueing RTR message with ID 0x%x, %d messages in TX queue.\n", CanID, TxQueue.NumElements);

	CANTxMessage Msg;
	Msg.CanID = CanID;
	Msg.EFF = UseEFF;
	Msg.Size = NumBytes;
	Msg.RTR = true;
	if (TxQueue.NumElements<TX_QUEUE_SIZE)
		TxQueue.Enqueue(Msg);
	else 
	{
		Serial.println("CAN Error: Tx buffer overrun");
		
		// Kick sending of messages from buffer anyway.
		TriggerSending();
		return false;
	}
	
	// Kick sending of messages from buffer.
	TriggerSending();

	return true;
}

void SimpleCan::DisableBlinkOnActivity() 
{
	BlinkOnActivity=false;
};

void SimpleCan::EnableBlinkOnActivity() 
{
	BlinkOnActivity=true;
	#ifdef LED_BUILTIN
		pinMode(LED_BUILTIN, OUTPUT);
	#endif	
};

//***********************************************************************
//***********************************************************************
//***********************************************************************
// Profile starts here

// This is the message hadler for the profile!
static  void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData, void* userData)
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


	if (rc==CAN_OK) rc = Can1->ConfigGlobalFilter();
	if (rc==CAN_OK) rc = Can1->AcceptAllMessages();
	if (rc==CAN_OK) rc = Can1->ActivateNotification(8, ::HandleCanMessage, this);			
	if (rc!=CAN_OK)	Serial.println("CAN initialization error!");

	if (rc==CAN_OK) Serial.println(Can1->Start() == CAN_OK
		? "CAN: started."
		: "CAN: error when starting.");
}


void SimpleCANProfile::CANSendText(const char* Text, const int CanID)
{
	Can1->SendMessage((uint8_t*)Text,  strlen(Text), CanID);
}


void SimpleCANProfile::CANSendFloat(const float Val, const int CanID)
{
	Can1->SendMessage((uint8_t*)&Val,  sizeof(Val), CanID);
}


void SimpleCANProfile::CANSendFloat(const float Val1, const float Val2, const int CanID)
{
    uint8_t Buffer[2*sizeof(float)];
    memcpy(Buffer, &Val1, sizeof(float));
    memcpy(Buffer+ sizeof(float), &Val2, sizeof(float));
	Can1->SendMessage(Buffer,  2*sizeof(float), CanID);
}


void SimpleCANProfile::CANSendInt(const int32_t Val, const int CanID)
{
	Can1->SendMessage((uint8_t*)&Val,  sizeof(int), CanID);
}


void SimpleCANProfile::CANSendInt(const int32_t Val1, const int32_t Val2, const int CanID)
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
// Implementation of the basic recerive handler, which is common for all platforms

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

	Msg.Error = RxErrorFlag;
	if (SimpleCan::RxQueue.NumElements<16)
		SimpleCan::RxQueue.Enqueue(Msg);
}

void RxHandlerBase::CANBusACtivityDetected()
{
	#ifdef LED_BUILTIN
	if (SimpleCan::BlinkOnActivity)
		digitalToggle(LED_BUILTIN);
	#endif
}

bool RxHandlerBase::Loop()
{
	bool rc = false;

	// If a callback was registered, convert the header info and call the callback function.
	if (ProfileCallback != NULL)
	{
		CanRxMessage Msg;
 		if (1 && SimpleCan::RxQueue.Dequeue(&Msg))
		{
			// Serial.println("Read message from buffer");
			// --- Convert the header and call the user provided RX handler.
			if (Msg.Error) Serial.println("Dequeue msg with error");
			ProfileCallback(Msg.SCHeader, Msg.Data, ProfileClass);
		}
	}
	else Serial.println("RxHandlerBase::Loop(): No callback set!");
	
	return rc;
}

