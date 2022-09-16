/*

    Demo code for portable CAN bus library

	This code uses the button on the B-G431B-ESC1 board to issue a reset.
	On ESP32 platforms, OTA updates are activated (but I did not get it to work reliably under platformio!).
    
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

void setup() 
{
	Serial.begin(BAUDRATE);
	delay(3000);
	Serial.println("Started");

	CANDevice.Init();
	// Set bus termination on/off (may not be available on all platforms).
	CANDevice.Can1->SetBusTermination(true);
	Serial.println("Setup done");
}

void loop()
{
	static uint32_t LastAction=millis();
	static uint32_t LastFloatAction=millis();
	static uint32_t LastRTR=millis();

	// Test of regular messages:
	// What is sent next to the CAN bus depends on what was received last. 
	// When a PING was received, send a PONG and vice versa.
	// To get the whole thing started, a PONG is sent every 5s without having received anything.
	// This just for testing. Usually you would activate actions for incomming messages
	// directly in the broker.
	if (CANBroker.ReceivedID==CANID_PP_PING  || (LastAction+5000<millis()) )
	{
		CANDevice.CANSendText("Pong", CANID_PP_PONG);
		LastAction=millis();
	}
	else if (CANBroker.ReceivedID==CANID_PP_PONG)
	{
		CANDevice.CANSendText("Ping", CANID_PP_PING);
		LastAction=millis();
	}
	else if (CANBroker.ReceivedID==CANID_PP_RTRINT && CANBroker.RTR)
	{
		CANBroker.RTR=false;
		CANDevice.CANSendInt(1234, CANID_PP_RTRINT);
	}

	// Make sure we don't react twice to the same message.
	CANBroker.ReceivedID = -1;		


	// Test of RTR messages
	// Every 3s send a float value
	if (LastFloatAction+3000<millis() )
	{
		float NewVal = CANBroker.ReceivedFloatVal*2.5;
		if (NewVal==0) NewVal=1.0f;
		if (NewVal>1000000) NewVal=-1.0;
		if(NewVal<-1000000) NewVal = 1.0;

		Serial.printf("Sending: %.3f\n", NewVal);
		CANDevice.CANSendFloat(NewVal, CANID_PP_FLOAT);
		LastFloatAction=millis();
	}

	// Every 5s request an int value
	if (LastRTR+5000<millis() )
	{
		Serial.printf("Request int\n");
		CANDevice.CANRequestInt();
		LastRTR=millis();
	}

	// Update message queues.
	CANDevice.Can1->Loop();
}
