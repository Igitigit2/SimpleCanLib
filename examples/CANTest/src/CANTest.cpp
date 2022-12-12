/*

    Demo code for portable CAN bus library
   
	(c) 2022 Christian Schmidmer, use is subject to MIT license
*/

#include <Arduino.h>


#include <Arduino.h>
#include <math.h>
#include "SimpleCAN.h"
#include "PingPongCANProfile.h"


// Broker class which handles information received from CAN bus.
// These functions are called from within CANPingPong::Can1->Loop() whenever there are new CAN messages in the queue.
// CAN IDs you want to react to must be specified in the CAN profile class (PingPongNotificationsFromCAN in this example).
// CANPingPong::Can1->Loop()   ->  CANPingPong::HandleCanMessage() ->  RxFromCAN::ReceivedXxxx()
// The reason for having this broker class is that it allows us to completely separate the profile definition from any code
// which is related to e.g. motor control. this means, a single header file is sufficient to define the profile and the same header file
// can be used on both sides of the CAN bus without having to include all the stuff which may be required on one side only.
class RxFromCAN : public PingPongNotificationsFromCAN
{
    public:
		RxFromCAN() : ReceivedID(-1), ReceivedFloatVal(1.0f), RTR(false) {};

        void ReceivedPong(const char* pText)
        {
			Serial.printf("Received: %s\n", pText);
 			ReceivedID = CANID_PP_PONG;
        };

        void ReceivedPing(const char* pText)
        {
			Serial.printf("Received: %s\n", pText);
			ReceivedID = CANID_PP_PING;
        };

        void ReceivedFloat(const float Val)
        {
			Serial.printf("Rcvd float: %.3f\n", Val);
			ReceivedFloatVal = Val;
			ReceivedID = CANID_PP_FLOAT;
        };

		void ReceivedRequestInt()
		{
			Serial.printf("Received: RTR\n");
			ReceivedID = CANID_PP_RTRINT;
			RTR = true;
		};

		void ReceivedInt(int Val)
		{
			Serial.printf("Rcvd int: %d\n", Val);
			ReceivedID = CANID_PP_RTRINT;
		};

		int ReceivedID;		
		bool RTR = true;
		float ReceivedFloatVal;
};



// Instatiation of the class which receives messages from the CAN bus.
// This class depends on your application!
RxFromCAN CANBroker;

// The actual CAN bus class, which handles all communication.
CANPingPong CANDevice(CreateCanLib(), &CANBroker);

int MyDeviceID=0;

void setup() 
{
	Serial.begin(BAUDRATE);
	delay(3000);
	while (!Serial);
	Serial.println("Started");

	// Create a random device ID to avoid conflicts on the CAN bus.
	MyDeviceID = random(1, 255);

	CANDevice.Init();
	// Set bus termination on/off (may not be available on all platforms).
	if (CAN_OK!=CANDevice.Can1->SetBusTermination(true))
		Serial.println("Setting CAN bus termination via software not possible");
	
	Serial.printf("Setup done, random device ID is %d\n", MyDeviceID);
}

void loop()
{
	static uint32_t LastAction=millis();
	static uint32_t LastFloatAction=millis();
	static uint32_t LastRTR=millis();

	int RandWait = random(-500, 1000);

	// Test of regular messages:
	// What is sent next to the CAN bus depends on what was received last. 
	// When a PING was received, send a PONG and vice versa.
	// To get the whole thing started, a PONG is sent every 5s without having received anything.
	// This is just for testing. Usually you would invoke actions for incomming messages
	// directly in the broker class.
	if ((CANBroker.ReceivedID==CANID_PP_PING && LastAction+1000<millis()) || (LastAction+RandWait+5000<millis()) )
	{
		Serial.println("Sending Pong");
		CANDevice.CANSendText("Pong", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PONG));
		LastAction=millis();

		// Make sure we don't react twice to the same message.
		CANBroker.ReceivedID = -1;		
	}
	else if (CANBroker.ReceivedID==CANID_PP_PONG && LastAction+RandWait+1000<millis())
	{
		Serial.println("Sending Ping");
		CANDevice.CANSendText("Ping", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PING));
		LastAction=millis();

		// Make sure we don't react twice to the same message.
		CANBroker.ReceivedID = -1;		
	}
	else if (CANBroker.ReceivedID==CANID_PP_RTRINT && CANBroker.RTR)
	{
		// React to an RTR request message. The reply should be the number "1234". If something else is 
		// received, check the byte order used by the devices!
		CANBroker.RTR=false;
		CANDevice.CANSendInt(1234, PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_RTRINT));

		// Make sure we don't react twice to the same message.
		CANBroker.ReceivedID = -1;		
	}


	// Every 3s just send a float value. This can be used to check if all devices on 
	// the bus use the same floating point number representation and byte order.
	if (LastFloatAction+RandWait+3000<millis() )
	{
		float NewVal = CANBroker.ReceivedFloatVal*2.5;
		if (NewVal==0) NewVal=1.0f;
		if (NewVal>1000000) NewVal=-1.0;
		if(NewVal<-1000000) NewVal = 1.0;

		Serial.printf("Sending: %.3f\n", NewVal);
		CANDevice.CANSendFloat(NewVal, PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_FLOAT));
		LastFloatAction=millis();
	}

	// Test of RTR messages
	// Every 5s request an int value. Response should be the number 1234 in binary form.
	if (LastRTR+RandWait+5000<millis() )
	{
		Serial.printf("Request int\n");
		CANDevice.CANRequestInt(MyDeviceID);
		LastRTR=millis();
	}

	// Get some statistics on bus errors.
	static int LastTxErrors=0;
	static int LastRxErrors=0;
	static int LastOtherErrors = 0;
	static uint32_t LastStatus = 0;
	uint32_t Status = 0;
	char StatusStr[MAX_STATUS_STR_LEN]={0};

	CANDevice.Can1->GetStatus(&Status, StatusStr);
	if (CANDevice.Can1->GetTxErrors()!=LastTxErrors || CANDevice.Can1->GetRxErrors()!=LastRxErrors || CANDevice.Can1->GetOtherErrors()!=LastOtherErrors || LastStatus!=Status)
	{
		LastTxErrors = CANDevice.Can1->GetTxErrors();
		LastRxErrors = CANDevice.Can1->GetRxErrors();
		LastOtherErrors = CANDevice.Can1->GetOtherErrors();
		LastStatus = Status;

		Serial.printf("New Status=%s, RxErrors=%d, TxErrors=%d, Other=%d\n", StatusStr, LastTxErrors, LastRxErrors, LastOtherErrors);
	}

	// Update message queues.
	CANDevice.Can1->Loop();
}
