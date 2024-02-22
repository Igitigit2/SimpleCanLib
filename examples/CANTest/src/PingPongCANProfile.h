
//*******************************************************************************************************
//******   Very simple Test profile (Ping Pong)   *******************************************************
//*******************************************************************************************************

// Message IDs 
#define CANID_PP_PING    1       // Message is "Ping"
#define CANID_PP_PONG    2       // Message is "Pong"
#define CANID_PP_FLOAT   3       // Message is a floating point value
#define CANID_PP_RTRINT  4       // Request an int from the client

// The actually used CAN ID consists of 4 bits as defined by the CANID_PP_XXX message IDs,
// or'ed together with a 7 bit device identifier (the total ID length must fit into 11 bits!). 
// This is required to avoid that two devices will try 
// bus arbitration with identical CAN IDs, which is not allowed. For the demo program,
// the device IDs are generated randomly at start up.
#define PP_MAKE_CAN_ID(Device, Message)     ((Device<<4) | Message) 
#define PP_GET_MESSAGE_ID(CanID)            (CanID & 0xf)
#define PP_GET_DEVICE_ID(CanID)             (CanID>>4)      // Max ID = 127!

class PingPongNotificationsFromCAN
{
    public:
        virtual void ReceivedPing(const int Device, const char* pText)=0;
        virtual void ReceivedPong(const int Device, const char* pText)=0;
        virtual void ReceivedFloat(const int Device, float Val)=0;
        virtual void ReceivedRequestInt(const int Device)=0;
        virtual void ReceivedInt(const int Device, int Val)=0;
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

            #define MAX_STRLEN  16
            
            char Str[MAX_STRLEN];
            float Val=0;
            int Device = PP_GET_DEVICE_ID(rxHeader.Identifier);
            switch(PP_GET_MESSAGE_ID(rxHeader.Identifier))
            {
                case CANID_PP_PING:
                    CANGetString(rxData, Str, min(MAX_STRLEN-1, (int)rxHeader.DataLength));
                    pRxCommands->ReceivedPing(Device, Str);
                    break;
                case CANID_PP_PONG:
                    CANGetString(rxData, Str, min(MAX_STRLEN, (int)rxHeader.DataLength));
                    pRxCommands->ReceivedPong(Device, Str);
                    break;
                case CANID_PP_FLOAT:
                    Val = CANGetFloat(rxData);
                    pRxCommands->ReceivedFloat(Device, Val);
                    break;
                case CANID_PP_RTRINT:
                    if (rxHeader.RxFrameType==CAN_REMOTE_FRAME)
                        pRxCommands->ReceivedRequestInt(Device);
                    else
                    {
                        int ValI = CANGetInt(rxData);
                        pRxCommands->ReceivedInt(Device, ValI);
                    }
                    break;
                default:
                    Serial.printf("y:0x%x DLC=0x%x ", rxHeader.Identifier, rxHeader.DataLength);
            } 
        }

    private:
        PingPongNotificationsFromCAN* pRxCommands;
};

