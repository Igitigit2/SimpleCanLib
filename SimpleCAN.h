/*

    Portable CAN bus library

    Interface, clases and enums for a platform independent CAN bus library.  

	A platform specific implementation of the abstract class SimpleCan as well as an implementation of the class factory CreateCanLib()
	must be provided in a separate module. Classes using this library must first create the object by calling CreateCanLib() and can 
	subsequently use the exposed methods. Everything on top of this interface should be completly platform independent, the implementation
	of this interface is of course not. 

	SimpleCan* pCan = CreateCanLib();
	pCan->init(Mbit1);
	...

	This code was strongly inspired by the discussions and proposals on the SimplFOC forum, especially
	by work provided by erwin74, JorgeMaker and Owen_Williams of the SimpleFOC community (see this thread: https://community.simplefoc.com/t/can-bus-support/407/21).
	The code versions provided by erwin74 and Owen_Williams were the basis for this derived work which attempts
	to be more modular and portable.

	(c) 2022 Christian Schmidmer, use is subject to MIT license
*/

#pragma once
#include "arduino.h"

#define RX_QUEUE_SIZE	16
#define TX_QUEUE_SIZE	8

#if !defined _STM32_DEF_ && !defined _ESP32_
	// Depending on the board you compile this for, define one (!) of the following two.
	// For platformio this should be one of the -D options in the platformio.ini file. This section
	// is for Arduino only and not required on pülatformio.

	// #define _STM32_DEF_
	// #define _ESP32_
#endif

#if defined(_ESP32_)
	#include "freertos/FreeRTOS.h"
	#include "freertos/queue.h"
#endif
#include "ThreadSafeQueue.h"


enum SCCanSpeed
{
	Mbit1 = 10,
	Kbit500 = 20,
	Kbit250 = 40,
	Kbit125 = 80	
};


enum SCCanStatus
{
  CAN_OK       = 0x00U,
  CAN_ERROR    = 0x01U,
  CAN_BUSY     = 0x02U,
  CAN_TIMEOUT  = 0x03U,
  CAN_UNSUPPORTED = 0x04U
};

enum SCIdType
{
	CAN_STDID = 0,
	CAN_EXTID = 1
};

enum SCCanType
{
	CAN_CLASSIC	= 0,
	CAN_FDFCAN	= 1
};


enum SCFrameType
{
	CAN_DATA_FRAME		= 0,
	CAN_REMOTE_FRAME	= 1
};


// Filter types
enum SCFilterType
{
	CAN_FILTER_RANGE = 0,			// Range filter from FilterID1 to FilterID2 (STM32)                        
	CAN_FILTER_DUAL = 1,			// Dual ID filter for FilterID1 or FilterID2 (STM32) 
	CAN_FILTER_MASK = 2,			// Classic filter: FilterID1 = filter, FilterID2 = mask (STM32, ESP32)
	CAN_FILTER_RANGE_NO_EIDM = 3	// Range filter from FilterID1 to FilterID2, EIDM mask not applied (STM32)
};


enum SCFilterConfig
{
	CAN_FILTER_DISABLE = 0,			// Disable filter element (STM32) 
	CAN_FILTER_TO_RXFIFO0 = 1,		// Store in Rx FIFO 0 if filter matches (STM32) 
	CAN_FILTER_TO_RXFIFO1 = 2,		// Store in Rx FIFO 1 if filter matches (STM32)
	CAN_FILTER_REJECT = 3,			// Reject ID if filter matches (STM32)
	CAN_FILTER_HP = 4,				// Set high priority if filter matches (STM32)
	CAN_FILTER_TO_RXFIFO0_HP = 5,	// Set high priority and store in FIFO 0 if filter matches(STM32)
	CAN_FILTER_TO_RXFIFO1_HP = 6	// Set high priority and store in FIFO 1 if filter matches(STM32) 
};


class FilterDefinition
{
	public:

		uint32_t FilterIndex;      /*!< Specifies the filter which will be initialized (ESP32 supports only 1!).
										This parameter must be a number between:
										- 0 and (SRAMCAN_FLS_NBR-1), if IdType is FDCAN_STANDARD_ID
										- 0 and (SRAMCAN_FLE_NBR-1), if IdType is FDCAN_EXTENDED_ID */

		SCFilterType FilterType;       /*!< Specifies the filter type.
										This parameter can be a value of @ref FDCAN_filter_type.
										The value FDCAN_FILTER_RANGE_NO_EIDM is permitted
										only when IdType is FDCAN_EXTENDED_ID.                    */

		SCFilterConfig FilterConfig;     /*!< Specifies the filter configuration (STM32).
										This parameter can be a value of @ref FDCAN_filter_config */

		SCIdType IdType;           		// SCIdType: Specifies the identifier type of the received message (standard / extended)
		uint32_t FilterID1;        		// Specifies the filter identification 1. 
										// (11 Bits for Std frame, ESP32 could use 16. For ext frames 29 bits. Only the IDs are compared.)
		uint32_t FilterID2;       		// Specifies the filter identification 2 or mask.
										// (11 Bits for Std frame, ESP32 could use 16. For ext frames 29 bits. Only the IDs are compared.)
};


// Class with information on the received CAN header.
// This is passed on to the callback function whenever a message is received.
class SimpleCanRxHeader
{
	public:
		uint32_t Identifier;            // Specifies the identifier.

		SCIdType IdType;                // SCIdType: Specifies the identifier type of the received message (standard / extended)
	
		uint32_t RxFrameType;           // SCFrameType: data frame / remote frame

		uint32_t DataLength;            // Specifies the received payload length in bytes.

		uint32_t Format;              	// SCCanType: Classic CAN / FDCAN frame

		int32_t RxTimestamp;           	// Specifies the timestamp counter value captured on start of frame reception [0 .. 0xFFFF]. -1 if unsupported.

		int32_t FilterIndex;           	// Specifies the index of matching Rx acceptance filter element. -1 if unsupported.

		int32_t IsFilterMatchingFrame; 	// Specifies whether the accepted frame did not match any Rx filter.
										//	Acceptance of non-matching frames may be enabled via
										//	HAL_FDCAN_ConfigGlobalFilter().
										// 	0=No match, 1=match, -1 unsupported	
};



class CANTxMessage
{
	public:
		int CanID;
		uint8_t Data[8];
		int Size;
		bool EFF;
		bool RTR;
};


class CanRxMessage
{
	public:
		SimpleCanRxHeader SCHeader;
		uint8_t Data[8];
};



//--------------------------------------------------------------------------------------
// Abstract class which must be implemented by a platform specific version.



// Callback function which should be user supplied and which will be called for each
// incomming message which passes the filters.
// The void* is exactly the value which was set as userData when calling ActivateNotification().
typedef void(*RxCallback)(SimpleCanRxHeader rxHeader, uint8_t *rxData, void* userData);


// Platform specific implementations must also provide one class derived from RxHandlerBase
// and implement the methods CANReadFrame() and ReleaseRcvBuffer().
class RxHandlerBase
{
	public:
		RxHandlerBase(uint16_t dataLength);
		~RxHandlerBase();
		void SetProfileCallback(uint16_t dataLength, RxCallback callback, void* userData);
		void Notify(/*FDCAN_HandleTypeDef *hfdcan*/);		// Platform dependent!
		RxCallback ProfileCallback;
		void* ProfileClass;		// Pointer available for callback routine to do with it whatever it likes.

		// Called for every frame which was received from the CAN bus. 
		virtual bool CANReadFrame(SimpleCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen)=0;

		// Called whenever a frame was read and CANReadFrame() finished or the frame can be discarded otherwise.
		// Used to signal the hardware that the frame data are not required anymore.
		virtual void ReleaseRcvBuffer()=0;
		
		bool Loop();

		uint8_t *_rxData;
		uint16_t _rxDataLength;
};


// Pointer to function which returns true when a CAN ID is valid and false if any commands send to it shall be ignored.
typedef bool (*CanIDFilter) (int CanID);		

// This is the "driver interface", which provides low level access to the CAN bus.
class SimpleCan 
{
	public:
		//*************************************************************************************
		//*** Pure virtual methods, which require hardware specific implementation ************
		
		// Initialize the CAN controller
		virtual SCCanStatus Init(SCCanSpeed speed, CanIDFilter IDFilterFunc=0)=0;

		// Register/deregister a callback function for message received events.
		// The notification handler is platform specific, that is why it's needed here.
		// These functions may be overloaded if required.
		virtual SCCanStatus ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData)=0;
		virtual SCCanStatus DeactivateNotification()=0;

		// Set bus termination on/off (may not be available on all platforms).
		// Default is on.
		virtual SCCanStatus SetBusTermination(bool On)=0;

		// Start and stop all activities. Changing acceptance filters requires stop()
		// before doing so on some platforms.
		virtual SCCanStatus Start()=0;
		virtual SCCanStatus Stop()=0;

		// Modify the global filter to reject everything which is not matching the other filters and to accept all remote frames. 
		virtual SCCanStatus ConfigGlobalFilter()=0;

		// Modify the acceptance filter. This may be forbidden while the controller is active.
		virtual SCCanStatus ConfigFilter(FilterDefinition *filterDef)=0;

		// Start sending messages from the queue to the CAN bus, until the TX queue is emty.
		virtual bool TriggerSending()=0;


		//*****************************************************
		//*** Methods with default implementation *************
		
		// Send a message to the TX FIFO and trigger sending to CAN bus.
		// If UseEFF is set, extended frame format will be used, standard frame format otherwise.
		// Returns true on success.
		virtual bool SendMessage(const uint8_t* pData, int NumBytes, int CanID, bool UseEFF=false);


		// Send an RTR frame to TX Fifo and trigger sending to CAN bus
		// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
		// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
		// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
		virtual bool RequestMessage(int NumBytes, int CanID, bool UseEFF=false);

		// Set a filter to accept all incoming messages
		virtual SCCanStatus AcceptAllMessages(); 

		// Set a filter to deny all incoming messages (except for CAN ID 0xFFFF)
		SCCanStatus DenyAllMessages();

		// Return the number of registered transmit errors or -1 if not available.
		virtual int GetTxErrors() {return -1;};
		// Return the number of registered receive errors or -1 if not available.
		virtual int GetRxErrors() {return -1;};
		// Return the number of registered other CAN bus errors or -1 if not available.
		virtual int GetOtherErrors() {return -1;};

		// If a message is waiting in the queue, then get it and dispatch it.
		virtual bool Loop()=0;

		// Read the status of the device. Result is platform depending.
		// NOTE: Str must be 0 or have space for at least MAX_STATUS_STR_LEN chars. 
		#define MAX_STATUS_STR_LEN 64
		virtual SCCanStatus GetStatus(uint32_t* Status, char* Str) {return CAN_UNSUPPORTED;};


		static SafeQueue<CanRxMessage> RxQueue;
		static SafeQueue<CANTxMessage> TxQueue;
};


// Class factory which creates a SimpleCan object for the correct platform.
// There must be exactly one such function available.
SimpleCan* CreateCanLib();


//*******************************************************************************************************
//*****    Helper class to simplify implementation of profiles   ****************************************
//*****    ** Usage is optional! **                              ****************************************
//*******************************************************************************************************


// Every CAN profile must be derived from this classd and at least implement HandleCanMessage().
// HandleCanMessage() will be called for every received CAN message.
class SimpleCANProfile
{
    public:
		// This is, where you finaly receive the CAN messages in your own code!
        virtual void HandleCanMessage(SimpleCanRxHeader rxHeader, uint8_t *rxData)=0;

        SimpleCANProfile(SimpleCan* pCan);
        virtual void  Init(CanIDFilter IDFilterFunc=0);
        virtual void  CANSendText(const char* Text, int CanID);
        virtual void  CANSendFloat(float Val1, int CanID);
        virtual void  CANSendFloat(float Val1, float Val2, int CanID);
        virtual void  CANSendInt(int32_t Val, int CanID);
		virtual void  CANSendInt(int32_t Val1, int32_t Val2, int CanID);
        virtual float CANGetFloat(const uint8_t* pData);
        virtual int   CANGetInt(const uint8_t* pData);
		virtual void  CANGetInt(const uint8_t* pData, int32_t* pInt1, int32_t* pInt2);
        virtual void  CANGetFloat(const uint8_t* pData, float* pf1, float* pf2);
        virtual void  CANGetString(const uint8_t* pData, char* pChar, int MaxLen);      
        SimpleCan* Can1;     
};
