
/*

    Portable CAN bus library - implementation for STM B-G431B-ESC1 board.

	This module implements the SimpleCan interface for a specific platorm.

	This code was strongly inspired by the discussions and proposals on the SimplFOC forum, especially
	by work provided by erwin74, JorgeMaker and Owen_Williams of the SimpleFOC community (see this thread: https://community.simplefoc.com/t/can-bus-support/407/21).
	The code versions provided by erwin74 and Owen_Williams were the basis for this derived work which attempts
	to be more modular and portable. Actually, most of the code in this specific module is their work.

	(c) 2022 Christian Schmidmer, use is subject to MIT license
*/


/*
	Note: In xxx\.platformio\packages\framework-arduinoststm32\system\Drivers\STM32G4xx_HAL_Driver\Src\stm32g4xx_hal_fdcan.c apply the following change:

	Line 3493
    for (ByteCounter = 0; ByteCounter < DLCtoBytes[pTxHeader->DataLength >> 16U]; ByteCounter += 4U)
    {
      *TxAddress = (((uint32_t)pTxData[ByteCounter + 3U] << 24U) |
                    ((uint32_t)pTxData[ByteCounter + 2U] << 16U) |
                    ((uint32_t)pTxData[ByteCounter + 1U] << 8U)  |
                    (uint32_t)pTxData[ByteCounter]);
      TxAddress++;
    }

-> Change to:

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

*/


#include <arduino.h>
#include "SimpleCan.h"


// #define _STM32_DEF_

#if defined(_STM32_DEF_)

#ifdef Error_Handler
#undef Error_Handler

void Error_Handler(int Code=-1)
{
	Serial.printf("\nError SimpleCan_B_g431B %d\n", Code);
}
#endif


#define HALSTATUS2CANSTATUS(s) (static_cast<SCCanStatus>(s))


// Utility function to convert filter definition types from SimpleCan to stm32 definition.
SCCanStatus SC2STM32_Filter(FilterDefinition *SCFilter, FDCAN_FilterTypeDef*  STMFilter)
{
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
	return CAN_OK;
}



class RxHandlerSTM32 : public RxHandlerBase
{
	public:
		// Note: The constructor here is a must on order to initialize the base class. 
		RxHandlerSTM32(uint16_t dataLength) : RxHandlerBase(dataLength) {};
		bool CANReadFrame(SimpleCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen);		
 		void ReleaseRcvBuffer();		

	private:
		FDCAN_RxHeaderTypeDef _rxHeader;
};



class SimpleCan_B_g431B : public SimpleCan
{
	public:
		SimpleCan_B_g431B();
		bool SendMessage(const uint8_t* pData, int NumBytes, int CanID, bool UseEFF=false);

		// WARNING: RTR FRAMES ARE COMPLETELY UNTESTED!!
		// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
		// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
		// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
		bool RequestMessage(int NumBytes, int CanID, bool UseEFF=false);

		void SetBusTermination(bool On);
		SCCanStatus Init(SCCanSpeed speed, CanIDFilter IDFilterFunc=0);
		SCCanStatus ConfigFilter(FilterDefinition *filterDef);
		SCCanStatus ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt);
		SCCanStatus ConfigGlobalFilter();
		SCCanStatus ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData);
		SCCanStatus DeactivateNotification();
		SCCanStatus Start();
		SCCanStatus Stop();
		SCCanStatus AddMessageToTxFifoQ(FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData);
		bool Loop();

		static FDCAN_HandleTypeDef _hfdcan1;
		uint8_t TxData[8];
		FDCAN_TxHeaderTypeDef TxHeader;

		static RxHandlerSTM32 *RxHandlerP;		

	private:
		static CanIDFilter SendIDFilterFunc;
};


CanIDFilter SimpleCan_B_g431B::SendIDFilterFunc;

static RxHandlerSTM32 Can1RxHandler(8);			// Preferably this should be allocated by the HAL, just paramtereized here!
RxHandlerSTM32* SimpleCan_B_g431B::RxHandlerP=nullptr;	// Presumably this must be static because of IRQs????


SimpleCan* CreateCanLib()
{
	return (SimpleCan*) new SimpleCan_B_g431B;
}



static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// Copy max _rxDataLength bytes from received frame to _rxData. 
bool RxHandlerSTM32::CANReadFrame(SimpleCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen)
{
	if (HAL_FDCAN_GetRxMessage(&SimpleCan_B_g431B::_hfdcan1, FDCAN_RX_FIFO0, &_rxHeader, pData) != HAL_OK)		//!!! Data length not checked!!!
	{
		Error_Handler(1);
		return false;
	}

	if (ProfileCallback != NULL)
	{
		// Convert the header
		SCHeader->Identifier = _rxHeader.Identifier;
		SCHeader->DataLength = DLCtoBytes[_rxHeader.DataLength>>16];			
		SCHeader->RxTimestamp = _rxHeader.RxTimestamp;
		SCHeader->FilterIndex = _rxHeader.FilterIndex;
		SCHeader->IsFilterMatchingFrame = _rxHeader.IsFilterMatchingFrame;

		SCHeader->IdType   = (_rxHeader.IdType==FDCAN_EXTENDED_ID) ? SCIdType::CAN_EXTID : SCIdType::CAN_STDID;
		SCHeader->Format = (_rxHeader.FDFormat==FDCAN_CLASSIC_CAN) ? SCCanType::CAN_CLASSIC : SCCanType::CAN_FDFCAN;
		SCHeader->RxFrameType = (_rxHeader.RxFrameType==FDCAN_DATA_FRAME) ? SCFrameType::CAN_DATA_FRAME : SCFrameType::CAN_REMOTE_FRAME;
	}
	else return false;

	return true;
}


// Let the hardware know the frame has been read.
 void RxHandlerSTM32::ReleaseRcvBuffer()
{
	// Nothing to do for STM32;	
}



// will be called from: HAL_FDCAN_Init
extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
extern "C" void FDCAN1_IT0_IRQHandler();
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

FDCAN_HandleTypeDef SimpleCan_B_g431B::_hfdcan1 = { };

SimpleCan_B_g431B::SimpleCan_B_g431B()
{
	if (_hfdcan1.Instance != NULL)
	{
		Error_Handler(2);
	}

	_hfdcan1.Instance = FDCAN1;

	pinMode(A_CAN_SHDN, OUTPUT);
	pinMode(A_CAN_TERM, OUTPUT);

	// Bus termination is on by default.
	digitalWrite(A_CAN_TERM, HIGH);
	
	SendIDFilterFunc = 0;
}

void SimpleCan_B_g431B::SetBusTermination(bool On)
{
	digitalWrite(A_CAN_TERM, On ? HIGH : LOW);
}

SCCanStatus SimpleCan_B_g431B::Start(void)
{
	digitalWrite(A_CAN_SHDN, LOW);
	return HALSTATUS2CANSTATUS(HAL_FDCAN_Start(&_hfdcan1));
}

SCCanStatus SimpleCan_B_g431B::Stop(void)
{
	digitalWrite(A_CAN_SHDN, HIGH);
	return HALSTATUS2CANSTATUS(HAL_FDCAN_Stop(&_hfdcan1));
}

SCCanStatus SimpleCan_B_g431B::Init(SCCanSpeed speed, CanIDFilter IDFilterFunc)
{
	Serial.println("SimpleCan_B_g431B::Init()"); delay(200);

	if (IDFilterFunc) SendIDFilterFunc = IDFilterFunc;

	FDCAN_InitTypeDef *init = &_hfdcan1.Init;
	
	init->ClockDivider = FDCAN_CLOCK_DIV1;
	init->FrameFormat = FDCAN_FRAME_FD_BRS;
	init->Mode = FDCAN_MODE_NORMAL;
	init->AutoRetransmission = DISABLE;
	init->TransmitPause = ENABLE;
	init->ProtocolException = DISABLE;
	
	// 1 MBit: NominalPrescaler = 10
	// see: http://www.bittiming.can-wiki.info/
	// 170MHz / Prescaler / SyncJumpWith / (TimeSeg1 + TimeSeg2 + SyncSeg) 
	// SyncSeg = SYNC_SEG = 1, TimeSeg1 = PROP_SEG + PHASE_SEG1, TimeSeg2 = PHASE_SEG2
	init->NominalPrescaler = (uint16_t) speed;
	init->NominalSyncJumpWidth = 1;
	init->NominalTimeSeg1 = 14;
	init->NominalTimeSeg2 = 2;

	init->DataPrescaler = 1;
	init->DataSyncJumpWidth = 4;
	init->DataTimeSeg1 = 5;
	init->DataTimeSeg2 = 4;
	init->StdFiltersNbr = 1;
	init->ExtFiltersNbr = 1;
	init->TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	
	return HALSTATUS2CANSTATUS(HAL_FDCAN_Init(&_hfdcan1));
}


SCCanStatus SimpleCan_B_g431B::ConfigFilter(FilterDefinition *SCFilter)
{
	// Convert the filter definition
	FDCAN_FilterTypeDef  STMFilter;

	SCCanStatus rc = SC2STM32_Filter(SCFilter, &STMFilter);
	if (CAN_OK!=rc) return rc;
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ConfigFilter(&_hfdcan1, &STMFilter));
}



// Specify what to do with non-matching frames.
SCCanStatus SimpleCan_B_g431B::ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, nonMatchingStd, nonMatchingExt, rejectRemoteStd, rejectRemoteExt));
}


// Modify the global filter to reject everything which is not matching the other filters and acceopt all remote frames. 
SCCanStatus SimpleCan_B_g431B::ConfigGlobalFilter()
{
	return ConfigGlobalFilter(FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}


SCCanStatus SimpleCan_B_g431B::ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData)
{
	if (RxHandlerP != NULL)
	{
		return HALSTATUS2CANSTATUS(HAL_ERROR);
	}
#if USE_HAL_FDCAN_REGISTER_CALLBACKS
	// This would make interrupt handling much easier and cleaner.
	// But how to activate it on Arduino platform?
	//_hfdcan1.Instance->RxFifo0Callback = ...;
#endif
	RxHandlerP = &Can1RxHandler;
	RxHandlerP->SetProfileCallback(dataLength, callback, userData);
	Serial.println("CAN: notifications activated");
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ActivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
}

SCCanStatus SimpleCan_B_g431B::DeactivateNotification()
{
	RxHandlerP = NULL;
	return HALSTATUS2CANSTATUS(HAL_FDCAN_DeactivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
}

SCCanStatus SimpleCan_B_g431B::AddMessageToTxFifoQ(FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData)
{
	// Serial.printf(F("Sending CAN message ID=%d\n"), pTxHeader->Identifier);
	return HALSTATUS2CANSTATUS(HAL_FDCAN_AddMessageToTxFifoQ(&_hfdcan1, pTxHeader, pTxData));
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan == NULL || hfdcan->Instance != FDCAN1)
	{
		return;
	}

	RCC_PeriphCLKInitTypeDef periphClkInit = { };
	
	HAL_RCCEx_GetPeriphCLKConfig(&periphClkInit);
	
	// Initializes the peripherals clocks
	periphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_FDCAN;
	periphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&periphClkInit) != HAL_OK)
	{
		Error_Handler(3);
	}
	
	// Peripheral clock enable
	__HAL_RCC_FDCAN_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	// FDCAN1 GPIO Configuration
	// PA11 ------> FDCAN1_RX
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PB9 ------> FDCAN1_TX 
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// FDCAN1 interrupt Init
	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

void FDCAN1_IT0_IRQHandler(void)
{
	HAL_FDCAN_IRQHandler(&SimpleCan_B_g431B::_hfdcan1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	// Serial.println("cb");
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (SimpleCan_B_g431B::RxHandlerP == NULL)
		{
			return;
		}
		SimpleCan_B_g431B::RxHandlerP->Notify(/*hfdcan*/);			// handle!!
	}
}


bool SimpleCan_B_g431B::SendMessage(const uint8_t* pData, int NumBytes, int CanID, bool UseEFF)
{
	// Serial.printf("CAN: Sending %d bytes with ID 0x%x\n", NumBytes, CanID);

	// Skip command if sender ID is disabled.
	if ( SendIDFilterFunc && !SendIDFilterFunc(CanID) ) 
		return true; 

	bool TxOk=true;
	TxHeader.Identifier = CanID;
	TxHeader.IdType = UseEFF ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
	switch(NumBytes)
	{
		case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
		case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
		case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
		case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
		case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
		case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
		case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
		case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
		case 8: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
		default: Serial.print("CAN: Invalid message length!\n"); 
			TxOk = false;
	}

	if (TxOk)
	{
		memcpy(TxData, pData, NumBytes);
		TxOk = (AddMessageToTxFifoQ(&TxHeader, TxData) == CAN_OK);
	}
	if (!TxOk)
		Serial.print("CAN: sending message failed!\n");
	
	return true;
}


// WARNING: THIS IS COMPLETELY UNTESTED!!
// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
bool SimpleCan_B_g431B::RequestMessage(int NumBytes, int CanID, bool UseEFF)
{
	// Skip command if sender ID is disabled.
	if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return true; 

	bool TxOk=true;
	TxHeader.Identifier = CanID;
	TxHeader.IdType = UseEFF ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
	switch(NumBytes)
	{
		case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
		case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
		case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
		case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
		case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
		case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
		case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
		case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
		case 8: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
		default: Serial.print("CAN: Invalid message length!\n"); 
			TxOk = false;
	}

	if (TxOk)
	{
		//memcpy(TxData, pData, NumBytes);
		TxOk = (AddMessageToTxFifoQ(&TxHeader, TxData) == CAN_OK);
	}

	if (!TxOk)
		Serial.print("CAN: sending message failed!\n");

	return true;
}


bool SimpleCan_B_g431B::Loop()
{
	return RxHandlerP->Loop();
}

#endif