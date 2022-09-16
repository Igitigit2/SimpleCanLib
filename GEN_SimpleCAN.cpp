
/*

    Portable CAN bus library - implementation for STM B-G431B-ESC1 board.

	This module implements the SimpleCan interface for a specific platorm.

	This code was strongly inspired by the discussions and proposals on the SimplFOC forum, especially
	by work provided by erwin74, JorgeMaker and Owen_Williams of the SimpleFOC community (see this thread: https://community.simplefoc.com/t/can-bus-support/407/21).
	The code versions provided by erwin74 and Owen_Williams were the basis for this derived work which attempts
	to be more modular and portable. Actually, most of the code in this specific module is their work.

	(c) 2022 Christian Schmidmer, use is subject to MIT license
*/

#include <arduino.h>
#include "SimpleCan.h"

// #define GENERIC_PLATFORM

#if defined(GENERIC_PLATFORM)

// Utility function to convert filter definition types from SimpleCan to stm32 definition.
SCCanStatus SC2STM32_Filter(FilterDefinition *SCFilter/*, FDCAN_FilterTypeDef*  STMFilter*/)
{
#if 0
	STMFilter->FilterIndex = SCFilter->FilterIndex; 
	STMFilter->FilterID1 = SCFilter->FilterID1; 
	STMFilter->FilterID2 = SCFilter->FilterID2; 


	switch(SCFilter->IdType)
	{
		case CAN_STDID: STMFilter->IdType = FDCAN_STANDARD_ID; break;
		case CAN_EXTID: STMFilter->IdType = FDCAN_EXTENDED_ID; break;
		default: return CAN_UNSUPPORTED;
	}

	switch(SCFilter->FilterType)
	{
		case CAN_FILTER_RANGE: STMFilter->FilterType = FDCAN_FILTER_RANGE; break;
		case CAN_FILTER_DUAL: STMFilter->FilterType = FDCAN_FILTER_DUAL; break;
		case CAN_FILTER_MASK: STMFilter->FilterType = FDCAN_FILTER_MASK; break;
		case CAN_FILTER_RANGE_NO_EIDM: STMFilter->FilterType = FDCAN_FILTER_RANGE_NO_EIDM; break;
		default: return CAN_UNSUPPORTED;
	}

	switch(SCFilter->FilterConfig)
	{
		case CAN_FILTER_DISABLE: STMFilter->FilterConfig = FDCAN_FILTER_DISABLE; break;
		case CAN_FILTER_TO_RXFIFO0: STMFilter->FilterConfig = FDCAN_FILTER_TO_RXFIFO0; break;
		case CAN_FILTER_TO_RXFIFO1: STMFilter->FilterConfig = FDCAN_FILTER_TO_RXFIFO1; break;
		case CAN_FILTER_REJECT: STMFilter->FilterConfig = FDCAN_FILTER_REJECT; break;
		case CAN_FILTER_HP: STMFilter->FilterConfig = FDCAN_FILTER_HP; break;
		case CAN_FILTER_TO_RXFIFO0_HP: STMFilter->FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP; break;
		case CAN_FILTER_TO_RXFIFO1_HP: STMFilter->FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP; break;
		default: return CAN_UNSUPPORTED;
	}
#endif
	return CAN_OK;
}


#if 1
class RxHandler
{
	public:
		RxHandler(uint16_t dataLength, RxCallback callback, void* userData);
		~RxHandler();
		void SetRxHandler(uint16_t dataLength, RxCallback callback, void* userData);
		void notify(/*FDCAN_HandleTypeDef *hfdcan*/);
		RxCallback _callback;
		void* _userData;		// Pointer available for callback routine to do with it whatever it likes.

	private:
		uint8_t *_rxData;
};

RxHandler::RxHandler(uint16_t dataLength, RxCallback callback, void* userData)
{
	_rxData = new byte[dataLength];
	_callback = callback;
	_userData = userData;
}

void RxHandler::SetRxHandler(uint16_t dataLength, RxCallback callback, void* userData)
{
	if (_rxData) delete[] _rxData;
	_rxData = new byte[dataLength];
	_callback = callback;
	_userData = userData;
}

RxHandler::~RxHandler()
{
	delete[] _rxData; 
}

// This function is called by the actual low level CAN routines when a new frame is received.
void RxHandler::notify(/*...*/)
{
    // ---
	if (1 /* GetMessage() */)
	{
		// Error_Handler();
	}


	if (_callback != NULL)
	{
		// --- Convert the header and call the user provided RX handler.
		SimpleCanRxHeader SCHeader;
		_callback(SCHeader, _rxData, _userData);
	}
	else Serial.println("SimpleCan_B_g431B::RxHandler::notify(): No callback set!");
}

static RxHandler Can1RxHandler(8, 0, 0);			// Preferably this should be allocated by the HAL, just paramtereized here!
#endif



class SimpleCan_ESP32_DevC : public SimpleCan
{
	public:
		SimpleCan_ESP32_DevC();
		void SendMessage(const uint8_t* pData, int NumBytes, int CanID);
		void SetBusTermination(bool On);
		SCCanStatus init(SCCanSpeed speed);
		SCCanStatus configFilter(FilterDefinition *filterDef);
		SCCanStatus configGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt);
		SCCanStatus configGlobalFilter();
		SCCanStatus activateNotification(uint16_t dataLength, RxCallback callback, void* userData);
		SCCanStatus deactivateNotification();
		SCCanStatus start();
		SCCanStatus stop();

		static RxHandler *_rxHandler;		        
};



RxHandler* SimpleCan_ESP32_DevC::_rxHandler=nullptr;	// Presumably this must be static because of IRQs????



SimpleCan* CreateCanLib()
{
	return (SimpleCan*) new SimpleCan_ESP32_DevC;
}

SimpleCan_ESP32_DevC::SimpleCan_ESP32_DevC()
{
        // ---
}

void SimpleCan_ESP32_DevC::SetBusTermination(bool On)
{
	// ---
}

SCCanStatus SimpleCan_ESP32_DevC::start(void)
{
    // ---
	return CAN_OK;
}

SCCanStatus SimpleCan_ESP32_DevC::stop(void)
{
	// ---
	return CAN_OK;
}

SCCanStatus SimpleCan_ESP32_DevC::init(SCCanSpeed speed)
{
	// ---
	Serial.println("SimpleCan_ESP32_DevC::Init()"); delay(200);
	
	return CAN_OK;
}

SCCanStatus SimpleCan_ESP32_DevC::configFilter(FilterDefinition *SCFilter)
{
	// ---
	return CAN_OK;
}

SCCanStatus SimpleCan_ESP32_DevC::configGlobalFilter()
{
	// ---
	return CAN_OK;
}

// Set user provided callback which receives the converted messages and activate notifications.
SCCanStatus SimpleCan_ESP32_DevC::activateNotification(uint16_t dataLength, RxCallback callback, void* userData)
{
	if (_rxHandler != NULL)
	{
		return CAN_ERROR;
	}

	_rxHandler = &Can1RxHandler;
	_rxHandler->SetRxHandler(dataLength, callback, userData);
	return CAN_OK;
}

SCCanStatus SimpleCan_ESP32_DevC::deactivateNotification()
{
	_rxHandler = NULL;
	// DeactivateNotificationsLL();
	return CAN_OK;
}


void SimpleCan_ESP32_DevC::SendMessage(const uint8_t* pData, int NumBytes, int CanID)
{
	// Serial.printf("CAN: Sending %d bytes with ID 0x%x (%d/%d)\n", NumBytes, CanID, MOTOR_FROM_CAN_ID(CanID), BASEID_FROM_ID(CanID));
}
#endif