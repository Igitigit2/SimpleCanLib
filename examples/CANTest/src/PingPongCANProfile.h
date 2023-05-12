
//*******************************************************************************************************
//******   Very simple Test profile (Ping Pong)   *******************************************************
//*******************************************************************************************************

// Message IDs 
#define CANID_PP_PING    1       // Message is "Ping"
#define CANID_PP_PONG    2       // Message is "Pong"
#define CANID_PP_FLOAT   3       // Message is a floating point value
#define CANID_PP_RTRINT  4       // Request an int from the client

// The actually used CAN ID consists of 8 bits as defined by the CANID_PP_XXX message IDs,
// or'ed together with an 8 bit device identifier. This is required to avoid that two devices will try 
// bus arbitration with identical CAN IDs, which is not allowed. For the demo program,
// the device IDs are generated randomly at start up.
#define PP_MAKE_CAN_ID(Device, Message)     ((Device<<8) | Message) 
#define PP_GET_MESSAGE_ID(CanID)            (CanID & 0xff)
#define PP_GET_DEVICE_ID(CanID)             (CanID>>8)

class PingPongNotificationsFromCAN
{
    public:
        virtual void ReceivedPing(const char* pText)=0;
        virtual void ReceivedPong(const char* pText)=0;
        virtual void ReceivedFloat(float Val)=0;
        virtual void ReceivedRequestInt()=0;
        virtual void ReceivedInt(int Val)=0;
};

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

        void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData)
        {
            // Serial.println("@");
            #ifdef _STM32_DEF_  
                digitalToggle(LED_BUILTIN);
            #endif

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

