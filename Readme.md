# SimpleCAN - A simple cross platform library for CAN bus communication

## Introduction

SimpleCAN is library intended to run on MCUs with integrated CAN controller. It is intended to separate the low level, MCU specific code from the actual application. You can define "CAN profiles" which use the same code on different MCUs and you can have an ESP32 communicate with an STM32 CPU, both compiling the same code, just using platform specific versions of this library. The library has so far been tested on ESP32 Devkit 4 boards fro AZDelivery and the STM32 dev board B-G431B-ESC1 from STM. 

## What you should know before reading

Some basic understanding of CAN bus concepts is expected in order to understand and best use this library.

## Some definitions and concepts
CAN bus as such is completely ignorant of any application layer. It only defines how data packets are transmitted between endpoints. All data packets are broadcasted and there is no distinction between a master and subordinated devices on the bus. All devices may send at any time and all devices can receive all data packets. In order to identify the content of packets, some bits are used as an identifier, the so-called CAN ID. it is important to understand that CAN IDs are application specific and they do not per se represent addresses of devices (there is no such concept on CAN bus). However, since CAN IDs can be freely selected, it is of course possible to use some of the ID's bits as a kind of address. By assigning a meaning to individual CAN IDs you start creating what in the context of SimpleCAN is called an application specific CAN profile. For example you could say, that CAN id 35 followed by a number x requests all motors to run at x rpm. reversely, each motor could report its temperature by using CAN ID 40 + m, where m is a motor specific index. Such a concept also allows the implementation of client - server concepts on an application level.


## Installation
1. Clone this archive to your PlatformIO Libraries folder.
2. Assuming a standard PlatformIO installation, copy the folder 
	xxx\PlatformIO\Projects\Libraries\SimpleCanLib\examples\CANTest 
	to 
	xxx\PlatformIO\Projects\
3. Open PlatformIO and add the folder \PlatformIO\Projects\CANTest to your projects.
4. Compile CANTest for your specific boards and download it to two boards.
5. Connect the CAN bus of the two boards and start them up.
6. If serial monitor is connected to one of the two boards, you should see reports of incoming and sent messages.


## How it works
At the basis is the class SimpleCan (SimpleCAN.h), which defines an interface for all platform specific implementations. Through this interface all low level communication is handled. If you ever derive your own platform specific class from this interface, make sure that all routines are thread safe and do not expose any Interupt service routines (which aren't thread safe...). For the ESP32 and STM32 implementations, thread safe queues are used to accomplish this.

On top of the low level routines sits the CAN profile. The base class for application specific profiles is always SimpleCANProfile (SimpleCAN.h). The low level SimpleCan derived object is a member of this class. SimpleCANProfile provides functions for sending basic data types over the CAN bus and it contains an abstract handler SimpleCANProfile::HandleCanMessage() for incomming messages, which must be implemented by your own profile class.

The example profile CANPingPong is derived from SimpleCANProfile and shows how to do this. As you can see, depending on the CAN ID, the implementation for HandleCanMessage() simply dispatches messages to other functions which are defined by an abstract class PingPongNotificationsFromCAN. A pointer to a class derived from PingPongNotificationsFromCAN is passed to the constructor of CANPingPong.

In the main code (CANTest.cpp), very few things are required. most important is the class RxFromCAN, which implements the interface defined by PingPongNotificationsFromCAN. All incomming messages which are handled by the profile end up here. In the example, we simply print the received data, but in a real application, you would of course do something more sophisticated.

Two important objects are declared in the main program, which are:
	RxFromCAN CANBroker, which is the object handling incomming messages, and
	CANPingPong CANDevice(CreateCanLib(), &CANBroker), which implements our CAN profile (and all the rest).

In the loop function messages are sent to the CAN bus by calling any of the CANDevice.CANSendXxxx() functions provided by our CAN profile. Most important however is that the loop functions calls CANDevice.Can1->Loop()
regularly in order to pump messages through the queues.

The reason for using the broker mechanism and the abstract classes is that I wanted to have the entire profile definition (in my case SimpleFOC running on an STM32 CPU and a higher level controller on an ESP32) in the same profile definition header file. Without the abstract classes and the broker this would have meant to have all the SimpleFOC stuff included on the ESP32 as well, something I clearly wantd to avoid. Of course, if you don't need/want all this, you can work directly on the lower level classes, e.g. SimpleCANProfile.

## How to use SimpleCAN in your own application
In order to use SimpleCAN in your own applications, you need to: 
1. Define your own CAN profile
2. Implement the code for your profile.
3. Implement the code for your broker class derived from the abstract definition in your profile.
4. Declare instances of your broker class and CAN profile class in the main program.
5. Call ProfileClass.Can1->Loop() regularly from your own loop function.
In the following sections we will outline those steps.
6. Make sure to have the correct -D options for the compiler and your platform. You are safe if you use the ones from the CANTest project's platformio.ini file..

### Define your own CAN profile
The definition of a CAN profile is closely tied to the actual application and there are very few restrictions of what you may do, apart from the low level CAN restrictions, especially the size of CAN IDs, speed and the size of messages. Considerations you should start with are:
- Do I need to address individual devices? 
- Can I use the same profile and broker classes on all devices or do I need different profile classes and brokers which share the same CAN IDs?
For our PingPong example the answers are easy, all devices are identical, so we only need one set of profile and broker class and we do not need specific addresses for devices (but they must be unique nevertheless). We simply start by defining a few CAN IDs which we will use as descriptors for the content of messages:


````
#define CANID_PP_PING    1       // Message is "Ping"
#define CANID_PP_PONG    2       // Message is "Pong"
#define CANID_PP_FLOAT   3       // Message is a floating point value
#define CANID_PP_RTRINT  4       // Request an int from the client
````

The CAN bus defintion requires that no two devices can send messages with identical CAN IDs (otherwise bus arbitration may fail, which you might be willing to tolerate...). Therefore, in the example, each device is assigned a random device ID at startup. This Device ID is or'ed together with the above CAN IDs before sending messages. To make this easier, a few macros are defined in the profile header file:

````
#define PP_MAKE_CAN_ID(Device, Message)     ((Device<<8) | Message) 
#define PP_GET_MESSAGE_ID(CanID)            (CanID & 0xff)
#define PP_GET_DEVICE_ID(CanID)             (CanID>>8)
````

For example, if a device sends the message with ID CANID_PP_PING, the message will consist of the characters 'Ping'. The next step is to define the interface of the broker class, which reacts to incoming messages. It needs one pure virtual function for each message:

````
class PingPongNotificationsFromCAN
{
    public:
        virtual void ReceivedPing(const char* pText)=0;
        virtual void ReceivedPong(const char* pText)=0;
        virtual void ReceivedFloat(float Val)=0;
        virtual void ReceivedRequestInt()=0;
        virtual void ReceivedInt(int Val)=0;
};
````
And finally, we need to write our profile class, which needs to be derived from SimpleCANProfile. It has three main tasks, linking the broker, dispatching incoming messages to the broker and providing methods to send specific messages. In the constructor we link the broker with the profile, we implement the HandleCanMessage() function, which dispatches received messages to the broker class object. If the send functions provided by the base class SimpleCANProfile are sufficient for the project, those two are all you need. If not, you need to implement your own code for sending messages. If you follow the examples in SimpleCANProfile, that should be easy. Some data types however may require further considerations, see the section on float values below.


````
class CANPingPong : public SimpleCANProfile
{
    public:

        CANPingPong(SimpleCan* pCan, PingPongNotificationsFromCAN* _pRxCommands) : SimpleCANProfile(pCan)
        {
            pRxCommands = _pRxCommands;
        }

		void  CANRequestInt(int DeviceID)
        {
            Can1->RequestMessage(2, PP_MAKE_CAN_ID(DeviceID, CANID_PP_RTRINT));            
        }

        void HandleCanMessage(SimpleCanRxHeader rxHeader, uint8_t *rxData)
        {
            #define MAX_STRLEN  16
            
            char Str[MAX_STRLEN];
            float Val=0;
            switch(PP_GET_MESSAGE_ID(rxHeader.Identifier))
            {
                case CANID_PP_PING:
                    CANGetString(rxData, Str, min(MAX_STRLEN-1, (int)rxHeader.DataLength));
                    pRxCommands->ReceivedPing(Str);
                    break;
                case CANID_PP_PONG:
                    CANGetString(rxData, Str, min(MAX_STRLEN, (int)rxHeader.DataLength));
                    pRxCommands->ReceivedPong(Str);
                    break;
                case CANID_PP_FLOAT:
                    Val = CANGetFloat(rxData);
                    pRxCommands->ReceivedFloat(Val);
                    break;
                case CANID_PP_RTRINT:
                    if (rxHeader.RxFrameType==CAN_REMOTE_FRAME)
                        pRxCommands->ReceivedRequestInt();
                    else
                    {
                        int ValI = CANGetInt(rxData);
                        pRxCommands->ReceivedInt(ValI);
                    }
                    break;
                default:
                    Serial.printf("y:0x%x DLC=0x%x ", rxHeader.Identifier, rxHeader.DataLength);
            } 
        }

    private:
        PingPongNotificationsFromCAN* pRxCommands;
};
````

Now, the profile definition is complete, safe it as a header file and include it in your main program. The next step is then to implement the broker class, which is derived from the PingPongNotificationsFromCAN class which we defined in our profile header file above. This class is pretty straight forward, just implement what you expect your program to do upon reveiving a certain message from the CAN bus. Note that these functions are called whenever you call CANDevice.Can1->Loop() (see further down, we'll come to that soon) in your program and that these functions are already completely decoupled from any interrupt service routines. Our example broker looks like this:


````
#include "SimpleCAN.h"
#include "PingPongCANProfile.h"

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
````


It is time now to create some objects which allow us to use the CAN bus. What we need is simply an instance of our broker class and an instance of the profile class. Note that you have to adjust the Tx/Rx pin names according to your needs! It goes like this:
````
// A *unique* ID for each device on the CAN bus.
int MyDeviceID=3;

// Instantiation of the class which receives messages from the CAN bus.
// This class depends on your application!
RxFromCAN CANBroker;

// The actual CAN bus class, which handles all communication.
// Insert the correct pin names here!
CANPingPong CANDevice(CreateCanLib(A_CAN_TX, A_CAN_RX), &CANBroker);

````
In the above code make sure that your ID is really unique for all devices on the CAN bus. Also note that the total length of CAN IDs must not exceed 11 bits!
The example code uses a simple macro to combine the device specific ID with the message IDs from the profile. 

These two objects can now be used in our setup and loop functions. In setup() we initialize SimipleCAN. If we activate the bus termination here as well depends on your bus topology and wether your device supports enabling/disabling bus termination by software (B-G431B-ESC1 does, ESP32 typically does not).
````
void setup() 
{
	...
	
	CANDevice.Init();

	// Set bus termination on/off (may not be available on all platforms).
	CANDevice.Can1->SetBusTermination(true);
}
````
In the loop function finally we have to update the message queues on a regular basis. All incomming messages will be placed in the input queue automatically by the ISR and in the same way, all messages ready to be sent will be sent automatically, independent from  calling CANDevice.Can1->Loop(), but your broker class won't be called if you don't call this function.
````
void loop()
{

	... 
	
	// Update message queues.
	CANDevice.Can1->Loop();
}
````

So far so good, we have everything set up to receive messages now and we have everything in place to send messages, but how do we send? That is easy, we simply call any of the send functions that we declared in our profile class. The example below will send the text "Ping", whenever it received "Pong". In addition, all 5 seconds a "Pong" will be sent without having received anything, since we need to get the game started somehow.

````
void loop()
{
	static uint32_t LastAction=millis();

	if (CANBroker.ReceivedID==CANID_PP_PING  || (LastAction+5000<millis()) )
	{
		CANDevice.CANSendText("Pong", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PONG));
		LastAction=millis();
	}
	else if (CANBroker.ReceivedID==CANID_PP_PONG)
	{
		CANDevice.CANSendText("Ping", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PING));
		LastAction=millis();
	}

	// Make sure we don't react twice to the same message.
	CANBroker.ReceivedID = -1;		

	CANDevice.Can1->Loop();
}
````

That's it! It looks far more complicated than it actually is. If you follow this tutorial step by step, it is really easy since all the required functions are very basic. You don't need to deal with anything CAN specific anymore, just focus on your own application. Most of the code presented here is directly taken from the PinPong example in the examples folder of the library, but a bit simplified. The example handles more messages than just "Ping" and "Pong" and it also demonstrates how to use RTR frames. 


### Float values in messages sent across CAN bus between different platforms
The binary representation of floating point numbers is machine and compiler specific. Most compilers today use IEEE number formats, so the main issue when exchanging float numbers between different CPUs in binary form will be the byte order. Luckily, STM32 and ESP32 use exactly the same single precision float format, so they can simply be sent as four bytes. For other CPUs, this is not necessarily the case and numbers may need to be packed differently.

### STM32 specific
For RTR frames to work you need to apply the following patch to the Arduino STM32 framework:

Note: In xxx\.platformio\packages\framework-arduinoststm32\system\Drivers\STM32G4xx_HAL_Driver\Src\stm32g4xx_hal_fdcan.c apply the following change (the if clause around the loop):

````	
Line 3493:
    for (ByteCounter = 0; ByteCounter < DLCtoBytes[pTxHeader->DataLength >> 16U]; ByteCounter += 4U)
    {
      *TxAddress = (((uint32_t)pTxData[ByteCounter + 3U] << 24U) |
                    ((uint32_t)pTxData[ByteCounter + 2U] << 16U) |
                    ((uint32_t)pTxData[ByteCounter + 1U] << 8U)  |
                    (uint32_t)pTxData[ByteCounter]);
      TxAddress++;
    }
````
-> Change to:
````
  if (!(pTxHeader->TxFrameType&&FDCAN_REMOTE_FRAME))
  {
    for (ByteCounter = 0; ByteCounter < DLCtoBytes[pTxHeader->DataLength >> 16U]; ByteCounter += 4U)
    {
      *TxAddress = (((uint32_t)pTxData[ByteCounter + 3U] << 24U) |
                    ((uint32_t)pTxData[ByteCounter + 2U] << 16U) |
                    ((uint32_t)pTxData[ByteCounter + 1U] << 8U)  |
                    (uint32_t)pTxData[ByteCounter]);
      TxAddress++;
    }
  }
````

## Limitations
For the sake of platfom independence, the SimpleCAN library supports the least common denominator between ESP32 and STM32 only at the moment. This means for example, no FDCAN and very simple filters only (oh, good point, I haven't covered filters in my description yet, but, maybe I'll wait for them to become more elaborate...). Of course, you cannot expect SimpleCAN to implement something, which the underlying hardware does not support (e.g. bus termination).


## License
The MIT License (MIT)

Copyright (c) 2017, Thomas Barth, barth-dev.de (ESP32_CAN_regdef.h, ESP32_CAN.h)

Copyright (c) 202, Christian Schmidmer

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.



## Credits
The STM32 implementation was strongly inspired by the discussions and proposals on the SimpleFOC forum, especially by work provided by erwin74, JorgeMaker and Owen_Williams of the SimpleFOC community (see this thread: https://community.simplefoc.com/t/can-bus-support/407/21). The code versions provided by erwin74 and Owen_Williams were the basis for this derived work which attempts to be more modular and portable. 
The ESP32 specific implementation is based on previous work by Thomas Barth, barth-dev.de, especially his definition of the ESP32 CAN controller registeres is used in SimpleCAN.


