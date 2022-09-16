
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
#ifdef NO_LOGLIB
	#define PrintLnLog  Serial.println
	#define PrintLog    Serial.printf
#else
	include "Logging.h"
#endif
#include "SimpleCan.h"

// #define _ESP32_

#if defined(_ESP32_)


#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_intr_alloc.h"        // "esp_intr.h"
#include "soc/dport_reg.h"
#include <math.h>

#include "driver/gpio.h"

#include "ESP32_CAN_regdef.h"
#include "ThreadSafeQueue.h"

#define TX_QUEUE_SIZE	8		// Max number of mesages stored in Tx buffer

class CANTxMessage
{
	public:
		int CanID;
		uint8_t Data[8];
		int Size;
		bool EFF;
		bool RTR;
};

SafeQueue<CANTxMessage> TxQueue(TX_QUEUE_SIZE);


static  void CAN_isr(void *arg_p);

class RxHandlerESP32 : public RxHandlerBase
{
	public:
		// Note: The constructor here is a must on order to initialize the base class. 
		RxHandlerESP32(uint16_t dataLength) : RxHandlerBase(dataLength) {};
		 bool CANReadFrame(SimpleCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen);
		 void ReleaseRcvBuffer();
};



// Copy max _rxDataLength bytes from received frame to _rxData. 
bool RxHandlerESP32::CANReadFrame(SimpleCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen)
{
	// Frame read buffer
	CAN_frame_t __frame;

    //check if we have a queue. If not, operation is aborted.
    if (ProfileCallback == NULL)
	{
        return false;
    }

	// Get Frame info record
	__frame.FIR.U=MODULE_CAN->MBX_CTRL.FCTRL.FIR.U;

	SCHeader->RxTimestamp = -1;			// Unusupported...
	SCHeader->FilterIndex = -1;
	SCHeader->IsFilterMatchingFrame = -1;
	SCHeader->RxFrameType = __frame.FIR.B.RTR==CAN_RTR ? SCFrameType::CAN_REMOTE_FRAME : SCFrameType::CAN_DATA_FRAME;
	SCHeader->DataLength = __frame.FIR.B.DLC;
	SCHeader->Format = SCCanType::CAN_CLASSIC;

    // Check if this is a standard or extended CAN frame
    if(__frame.FIR.B.FF==CAN_frame_std)
	{
 	   	// Standard frame
        //Get Message ID and header info
        SCHeader->Identifier = _CAN_GET_STD_ID;
		SCHeader->IdType = SCIdType::CAN_STDID;

		// if (SCHeader->RxFrameType!=SCFrameType::CAN_DATA_FRAME)
		//	Serial.print(" RTR ");

        // Deep copy data bytes
        if (SCHeader->RxFrameType==SCFrameType::CAN_DATA_FRAME)
			for(int i=0; i<SCHeader->DataLength && i<MaxDataLen; i++)
				pData[i]=MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[i];
    }
    else
	{
		// Extended frame
        // Get Message ID and header info
        SCHeader->Identifier = _CAN_GET_EXT_ID;
		SCHeader->IdType = SCIdType::CAN_EXTID;

        //Deep copy data bytes
        if (SCHeader->RxFrameType==SCFrameType::CAN_DATA_FRAME)
			for(int i=0; i<SCHeader->DataLength && i<MaxDataLen; i++)
				pData[i]=MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[i];
    }

	// PrintLnLog("CRF end"); delay(50);
	return true;
}


// Let the hardware know the frame has been read.
void RxHandlerESP32::ReleaseRcvBuffer()
{
	MODULE_CAN->CMR.B.RRB = 1;	
}

static RxHandlerESP32 Can1RxHandler(8);			// Preferably this should be allocated by the HAL, just paramtereized here!

class SimpleCan_ESP32_DevC : public SimpleCan
{
	public:
		SimpleCan_ESP32_DevC(gpio_num_t _TxPin=GPIO_NUM_5, gpio_num_t _RxPin=GPIO_NUM_35);

		bool SendMessage(const uint8_t* pData, int NumBytes, int CanID, bool UseEFF=false);
		static void SendNextMessageFromQueue();

		// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
		// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
		// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
		bool RequestMessage(int NumBytes, int CanID, bool UseEFF=false);
		static void SendRequestMessage(int NumBytes, int CanID, bool UseEFF=false);


		void SetBusTermination(bool On);
		SCCanStatus Init(SCCanSpeed speed, CanIDFilter IDFilterFunc=0);
		SCCanStatus ConfigFilter(FilterDefinition *filterDef);
		SCCanStatus ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt);
		SCCanStatus ConfigGlobalFilter();
		SCCanStatus ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData);
		SCCanStatus DeactivateNotification();
		SCCanStatus Start();
		SCCanStatus Stop();
		bool Loop();

		int GetTxErrors(); 
		int GetRxErrors(); 
		int GetOtherErrors();

		static RxHandlerESP32 *RxHandlerP;	
		static int CANBusErrors;	        

	private:

		gpio_num_t RxPin;
		gpio_num_t TxPin;
		static CanIDFilter SendIDFilterFunc;
};

// static members
CanIDFilter SimpleCan_ESP32_DevC::SendIDFilterFunc;
int SimpleCan_ESP32_DevC::CANBusErrors;


RxHandlerESP32* SimpleCan_ESP32_DevC::RxHandlerP=nullptr;	// Presumably this must be static because of IRQs????

static  void CAN_isr(void *arg_p)
{
	// PrintLnLog("+");

    // Read interrupt status and clear flags
    uint32_t interrupt = MODULE_CAN->IR.U;

    // Handle TX complete interrupt
    if (interrupt & __CAN_IRQ_TX) 
	{

    	// Try sending another message from the queue.
		SimpleCan_ESP32_DevC::SendNextMessageFromQueue();
    }

    // Handle RX frame available interrupt
    if (interrupt & __CAN_IRQ_RX)
    {
		// Serial.print("i");
		while (MODULE_CAN->RMC.B.RMC>0)
		{
			// Serial.print("+");
			if (SimpleCan_ESP32_DevC::RxHandlerP == NULL)
			{
				// Let the hardware know the frame has been read.
				// Normaly the rxHandler should do this, since it can release the buffer earlier than this routine.
				// But if there is no handler, we have to do it here...  
				MODULE_CAN->CMR.B.RRB = 1;	
				PrintLnLog("ERROR: No receive handler installed!");
				return;
			}
			SimpleCan_ESP32_DevC::RxHandlerP->Notify();
		}
		return;
	}
	

 #if 0
    // Handle error interrupts.
    if (interrupt & __CAN_IRQ_ERR)
		PrintLnLog("CAN ERROR: __CAN_IRQ_ERR");

    if (interrupt & __CAN_IRQ_DATA_OVERRUN)
		PrintLnLog("CAN ERROR: __CAN_IRQ_DATA_OVERRUN");

    if (interrupt & __CAN_IRQ_ERR_PASSIVE)
		PrintLnLog("CAN ERROR: __CAN_IRQ_ERR_PASSIVE");

    if (interrupt & __CAN_IRQ_ARB_LOST)
		; // Don'tz print, this happens often! PrintLnLog("CAN ERROR: __CAN_IRQ_ARB_LOST");

    if (interrupt & __CAN_IRQ_BUS_ERR)
		PrintLnLog("CAN ERROR: __CAN_IRQ_BUS_ERR");
#endif
    // Handle error interrupts.
    if (interrupt & __CAN_IRQ_ERR						//0x4
                      | __CAN_IRQ_DATA_OVERRUN			//0x8
                      | __CAN_IRQ_WAKEUP				//0x10
                      | __CAN_IRQ_ERR_PASSIVE			//0x20
                      | __CAN_IRQ_ARB_LOST				//0x40
                      | __CAN_IRQ_BUS_ERR				//0x80
	)
	{
    	/*Error handler*/
		SimpleCan_ESP32_DevC::CANBusErrors++;
    }
}



SimpleCan* CreateCanLib()
{
	return (SimpleCan*) new SimpleCan_ESP32_DevC;
}

SimpleCan_ESP32_DevC::SimpleCan_ESP32_DevC(gpio_num_t _TxPin, gpio_num_t _RxPin)
{
    RxPin = _RxPin;
    TxPin = _TxPin;
	
	SendIDFilterFunc = 0;
	CANBusErrors = 0;
}

void SimpleCan_ESP32_DevC::SetBusTermination(bool On)
{
	// ---
}

bool SimpleCan_ESP32_DevC::Loop()
{
	return RxHandlerP->Loop();
}

SCCanStatus SimpleCan_ESP32_DevC::Start(void)
{
    //Showtime. Release Reset Mode.
	PrintLnLog("CAN (ESP32): Start");
    MODULE_CAN->MOD.B.RM = 0;
	return CAN_OK;
}

SCCanStatus SimpleCan_ESP32_DevC::Stop(void)
{
	//enter reset mode
	MODULE_CAN->MOD.B.RM = 1;
	return CAN_OK;
}

SCCanStatus SimpleCan_ESP32_DevC::Init(SCCanSpeed speed, CanIDFilter IDFilterFunc)
{
	PrintLog("CAN (ESP32): initializing, Tx=%d, Rx=%d, Speed=%d\n", TxPin, RxPin, speed);

	if (IDFilterFunc) SendIDFilterFunc = IDFilterFunc;

	//Time quantum
	double __tq;

    //enable module
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);

    //configure TX pin
    gpio_set_level(TxPin, 1);
    gpio_set_direction(TxPin,GPIO_MODE_OUTPUT);
    gpio_matrix_out(TxPin,CAN_TX_IDX,0,0);
    gpio_pad_select_gpio(TxPin);

    //configure RX pin
	gpio_set_direction(RxPin,GPIO_MODE_INPUT);
	gpio_matrix_in(RxPin,CAN_RX_IDX,0);
	gpio_pad_select_gpio(RxPin);

    //set to PELICAN mode
	MODULE_CAN->CDR.B.CAN_M=0x1;			// 0=Basic CAN, 1=PeliCAN

	//synchronization jump width is the same for all baud rates
	MODULE_CAN->BTR0.B.SJW		=0x1;

	//TSEG2 is the same for all baud rates
	MODULE_CAN->BTR1.B.TSEG2	=0x1;

	// Select time quantum and set TSEG1
	// -> https://www.esacademy.com/en/library/calculators/sja1000-timing-calculator.html
	// -> Set clock = 240MHz
	__tq = 0.125;
	switch(speed){
		case SCCanSpeed::Mbit1:
			MODULE_CAN->BTR0.B.BRP	=0x4;
			MODULE_CAN->BTR1.B.TSEG1=0x4;
			break;
		case SCCanSpeed::Kbit500:
			MODULE_CAN->BTR0.B.BRP	=0x4;
			MODULE_CAN->BTR1.B.TSEG1=0xc;
			break;
		case SCCanSpeed::Kbit250:
			MODULE_CAN->BTR0.B.BRP	=0x9;
			MODULE_CAN->BTR1.B.TSEG1=0xc;
			break;
		case SCCanSpeed::Kbit125:
			MODULE_CAN->BTR0.B.BRP	=0x4;		//59
			MODULE_CAN->BTR1.B.TSEG1=0xc;
			break;

		default:
			// speed interpreted as baudrate! 
			MODULE_CAN->BTR1.B.TSEG1	=0xc;
			__tq = ((float)1000/speed) / 16;
	}

	//set baud rate prescaler
	MODULE_CAN->BTR0.B.BRP=(uint8_t)round((((APB_CLK_FREQ * __tq) / 2) - 1)/1000000)-1;
	PrintLog("CAN: BTR0.B.BRP=%d\n", (int)(MODULE_CAN->BTR0.B.BRP));

    /* Set sampling
     * 1 -> triple; the bus is sampled three times; recommended for low/medium speed buses     (class A and B) where filtering spikes on the bus line is beneficial
     * 0 -> single; the bus is sampled once; recommended for high speed buses (SAE class C)*/
    MODULE_CAN->BTR1.B.SAM	= 0x0; // 0x1;

    //enable all interrupts
    MODULE_CAN->IER.U = 0xff;

 
#if 1
	// Accept all
    MODULE_CAN->MBX_CTRL.ACC.CODE[0] = 0;
    MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0;
    MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0;
    MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0;
    MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff;
#else
	// Disable all
    MODULE_CAN->MBX_CTRL.ACC.CODE[0] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0x0;
    MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0x0;
    MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0x0;
    MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0x0;
#endif

    //set to normal mode
    MODULE_CAN->OCR.B.OCMODE=__CAN_OC_NOM;

    //clear error counters
    MODULE_CAN->TXERR.U = 0;
    MODULE_CAN->RXERR.U = 0;
    (void)MODULE_CAN->ECC;

    //clear interrupt flags
    (void)MODULE_CAN->IR.U;

    //install CAN ISR
    esp_intr_alloc(ETS_CAN_INTR_SOURCE,0,CAN_isr,NULL,NULL);

    //Showtime. Release Reset Mode.
    // Do not start it yet! Start();

    return CAN_OK;
}


// See https://www.nxp.com/docs/en/data-sheet/SJA1000.pdf pp.44
SCCanStatus SimpleCan_ESP32_DevC::ConfigFilter(FilterDefinition *SCFilter)
{

	// For ESP32: Mask bit==1 ignores the bit. Acceptance code spans ID,RTR and first two data bytes (std frame).
	// For SimpleCAN: Mask bit==1 requires the bit to match.  Acceptance code spans ID only.

	if (SCFilter->FilterIndex>0) return CAN_UNSUPPORTED;

	if (SCFilter->IdType==CAN_STDID && SCFilter->FilterType==CAN_FILTER_MASK)
	{
		PrintLnLog("CAN: setting filter single ID & mask, Std Frame");
		uint32_t Code = SCFilter->FilterID1; 
		uint32_t Mask = ~SCFilter->FilterID2;		// Lowest 4 bits unused, 5th would be RTR 
		MODULE_CAN->MBX_CTRL.ACC.CODE[0] = Code >> 3; 
		MODULE_CAN->MBX_CTRL.ACC.CODE[1] = (Code<<5) & 0xff; 

		MODULE_CAN->MBX_CTRL.ACC.MASK[0] = Mask >> 3; 
		MODULE_CAN->MBX_CTRL.ACC.MASK[1] = Mask<<5 | 0x1f; 
		MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff; 
		MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff; 
		MODULE_CAN->MOD.B.AFM = 1;
	}
	else if (SCFilter->IdType==CAN_STDID && SCFilter->FilterType==CAN_FILTER_DUAL)
	{
		PrintLnLog("CAN: Unsupported filter mode");
		// Same as single, but with CODE/MASK[2/3] in addition.
		// ACR3 / AMR3 is shared between the two filters.
	    return CAN_UNSUPPORTED;
	}
	else if (SCFilter->IdType==CAN_EXTID && SCFilter->FilterType==CAN_FILTER_MASK)
	{
		PrintLnLog("CAN: setting filter single ID & mask, Ext Frame");
		uint32_t Code = SCFilter->FilterID1; 
		uint32_t Mask = SCFilter->FilterID2;		
		MODULE_CAN->MBX_CTRL.ACC.CODE[0] = Code >> 21; 
		MODULE_CAN->MBX_CTRL.ACC.CODE[1] = (Code>>13) & 0xff; 
		MODULE_CAN->MBX_CTRL.ACC.CODE[2] = (Code>>5)  & 0xff; 
		MODULE_CAN->MBX_CTRL.ACC.CODE[3] = (Code<<3) & 0xff;

		MODULE_CAN->MBX_CTRL.ACC.MASK[0] = Mask >> 21; 
		MODULE_CAN->MBX_CTRL.ACC.MASK[1] = (Mask>>13) & 0xff; 
		MODULE_CAN->MBX_CTRL.ACC.MASK[2] = (Mask>>5)  & 0xff; 
		MODULE_CAN->MBX_CTRL.ACC.MASK[3] = (Mask<<3) | 0x07;
	}
	else
	{
		PrintLnLog("CAN: Unsupported filter mode");
		return CAN_UNSUPPORTED;
	}

	return CAN_OK;
}


SCCanStatus SimpleCan_ESP32_DevC::ConfigGlobalFilter()
{
	// ---
	return CAN_OK;
}

#if 1
// Set user provided callback which receives the converted messages and activate notifications.

//:: Called by SimpleCANProfile::Init() with: SimpleCan->ActivateNotification(8, ::HandleCanMessage, SimpleCANProfile);	
// userData is a pointer to the SimpleCanProfile class instance.
// callback is the method of the SimpleCanProfile HandleCanMessage() method.
// The ESP342 implementation only sets the callback, but STM32 for example, also enables the notifications by accessing the hardware.
SCCanStatus SimpleCan_ESP32_DevC::ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData)
{
	if (RxHandlerP != NULL)
	{
		return CAN_ERROR;
	}

	RxHandlerP = &Can1RxHandler;		// This is the static object inside SimpleCan_ESP32_DevC
	RxHandlerP->SetProfileCallback(dataLength, callback, userData);
	return CAN_OK;
}


SCCanStatus SimpleCan_ESP32_DevC::DeactivateNotification()
{
	RxHandlerP = NULL;		
	return CAN_OK;
}


#endif

bool SimpleCan_ESP32_DevC::SendMessage(const uint8_t* pData, int NumBytes, int CanID, bool UseEFF)
{
	// Skip command if sender ID is disabled.
	if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return true; 

	// PrintLog("CAN: Queueing message with ID 0x%x, %d messages in TX queue.\n", CanID, TxQueue.NumElements);

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
		PrintLnLog("CAN Error: Tx buffer overrun");
		
		// Kick sending of messages from buffer anyway.
		if(MODULE_CAN->SR.B.TBS)
			SendNextMessageFromQueue();
		return false;
	}
	
	// Kick sending of messages from buffer.
	
	if(MODULE_CAN->SR.B.TBS)
		SendNextMessageFromQueue();

	return true;
}


// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
bool SimpleCan_ESP32_DevC::RequestMessage(int NumBytes, int CanID, bool UseEFF)
{
	// Skip command if sender ID is disabled.
	if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return true; 

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
		PrintLnLog("CAN Error: Tx buffer overrun");
		
		// Kick sending of messages from buffer anyway.
		if(MODULE_CAN->SR.B.TBS)
			SendNextMessageFromQueue();
		return false;
	}
	
	// Kick sending of messages from buffer.
	if(MODULE_CAN->SR.B.TBS)
		SendNextMessageFromQueue();

	return true;
}

// Static, also called from ISR!!!!
void SimpleCan_ESP32_DevC::SendNextMessageFromQueue()
{
	CANTxMessage Msg;
	if (!TxQueue.Dequeue(&Msg))
		// Nothing to send...
		return;

	// PrintLog("CAN (ESP32): Sending %d bytes with ID 0x%x\n", NumBytes, CanID);

	if (Msg.RTR)
	{
		// The queued message was an RTR
		SendRequestMessage(Msg.Size, Msg.CanID, Msg.EFF);
	}
	else
	{
		// PrintLnLog("#");
		uint8_t i;
		CAN_FIR_t Fir;

		// Serial.print("T");

		Fir.U = 0;
		Fir.B.RTR = CAN_no_RTR;
		Fir.B.DLC = Msg.Size;

		if(!Msg.EFF)
		{
			// Standard frame
			Fir.B.FF = CAN_frame_std;
		
			//Write message ID
			_CAN_SET_STD_ID(Msg.CanID);

			// Copy the frame data to the hardware
			for(i=0; i<Msg.Size; i++)
				MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[i] = Msg.Data[i];
		}
		else
		{
			// Extended frame
			Fir.B.FF = CAN_frame_ext;

			// Write message ID
			_CAN_SET_EXT_ID(Msg.CanID);

			// Copy the frame data to the hardware
			for(i=0; i<Msg.Size; i++)
				MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[i]=Msg.Data[i];
		}

		// Transmit frame
		MODULE_CAN->MBX_CTRL.FCTRL.FIR.U = Fir.U;
		MODULE_CAN->CMR.B.TR=1;
//		MODULE_CAN->CMR.B.SRX=1;
	}
}



void SimpleCan_ESP32_DevC::SendRequestMessage(int NumBytes, int CanID, bool UseEFF)
{
	// PrintLog("CAN (ESP32): Sending %d bytes with ID 0x%x\n", NumBytes, CanID);

	// Skip command if sender ID is disabled.
	if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return; 

	uint8_t i;
	CAN_FIR_t Fir;

	// Serial.print("T");

	Fir.U = 0;
	Fir.B.RTR = CAN_RTR;
	Fir.B.DLC = NumBytes;

	if(!UseEFF)
	{
		// Standard frame
		Fir.B.FF = CAN_frame_std;
	
		//Write message ID
		_CAN_SET_STD_ID(CanID);
	}
	else
	{
		// Extended frame
		Fir.B.FF = CAN_frame_ext;

		// Write message ID
		_CAN_SET_EXT_ID(CanID);
	}

    // Transmit frame
	MODULE_CAN->MBX_CTRL.FCTRL.FIR.U = Fir.U;
    MODULE_CAN->CMR.B.TR=1;
}

int SimpleCan_ESP32_DevC::GetTxErrors() 
{
	return (int)(MODULE_CAN->TXERR.B.TXERR);
}

int SimpleCan_ESP32_DevC::GetRxErrors() 
{
	return (int)(MODULE_CAN->RXERR.B.RXERR);
}

int SimpleCan_ESP32_DevC::GetOtherErrors() 
{
//	return CANBusErrors;
	return MODULE_CAN->SR.U;
}

#endif