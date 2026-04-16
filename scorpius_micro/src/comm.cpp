/* We use a custom packet made with uint8 with this format
HEAD 0xAA
Data_length
Data ...
Checksum
TAIL 0xBB
*/

#include "comm.h"
#include "state_machine.h"

static const uint8_t HEAD = 0xAA;
static const uint8_t TAIL = 0xBB;

static HardwareSerial* commSerial = &Serial;
static volatile int len = 0;
static uint8_t dataBuf[PACKET_MAX_LEN];
static uint8_t dataPos = 0;
static int checksum = 0;

static const uint8_t PACKET_QUEUE_CAPACITY = 4;

// Circular buffer for packet RX
static uint8_t packetQueueIn[PACKET_QUEUE_CAPACITY][PACKET_MAX_LEN];
static uint8_t packetQueueInLen[PACKET_QUEUE_CAPACITY];
static volatile uint8_t packetQueueHead = 0;
static volatile uint8_t packetQueueTail = 0;
static volatile uint8_t packetQueueCount = 0;

// Queue for packet TX
static sPacketOut packetQueueOut[PACKET_QUEUE_CAPACITY];
static volatile uint8_t packetQueueOutCount = 0U;

enum eParserState
{
    WAIT_HEAD,
    READ_LEN,
    READ_DATA,
    READ_CHECKSUM,
    READ_TAIL
};

static eParserState serialStateMachine = WAIT_HEAD;

void commProcess()
{
    uint8_t processedPackets = 0;
    while (commSerial->available() && processedPackets < MAX_PROCESSED_PER_LOOP)
    {
        uint8_t b = (uint8_t)commSerial->read();

        switch (serialStateMachine)
        {
            case WAIT_HEAD:
                if (b == HEAD)
                {
                    serialStateMachine = READ_LEN;
                    dataPos = 0;
                    checksum = 0;
                }
                break;

            case READ_LEN:
                len = b;
                if (len == 0 || len > PACKET_MAX_LEN)
                {
                    COMM_DEBUG("Invalid packet length: ");
                    COMM_DEBUG(len);
                    static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_LENGTH_RECEIVED)};
                    commSendNow(eSerialMsgType::ERROR, errPayload, 1);
                    serialStateMachine = WAIT_HEAD;
                }
                else
                {
                    serialStateMachine = READ_DATA;
                }

                break;

            case READ_DATA:
                dataBuf[dataPos] = b;
                ++dataPos;
                checksum += b;

                if (dataPos >= len)
                {
                    serialStateMachine = READ_CHECKSUM;
                }

                break;

            case READ_CHECKSUM:
            {
                uint8_t expected = (uint8_t)(checksum & 0xFF);
                if (b == expected)
                {
                    serialStateMachine = READ_TAIL;
                }
                else
                {
                    // checksum mismatch -> discard
                    COMM_DEBUG("Checksum mismatch: got=");
                    COMM_DEBUG((int)b);
                    COMM_DEBUG(" expected=");
                    COMM_DEBUG((int)expected);

                    static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::CHECKSUM_MISMATCH)};
                    commSendEnqueue(eSerialMsgType::ERROR, errPayload, 1);
                    serialStateMachine = WAIT_HEAD;
                }
                break;
            }
            case READ_TAIL:
                if (b == TAIL)
                {
                    if (packetQueueCount >= PACKET_QUEUE_CAPACITY)
                    {
                        COMM_DEBUG("Dropped a packet");
                        static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::PACKET_DROPPED)};
                        commSendEnqueue(eSerialMsgType::ERROR, errPayload, 1);
                    }
                    else
                    {
                        memcpy(packetQueueIn[packetQueueTail], dataBuf, size_t(len));
                        packetQueueInLen[packetQueueTail] = static_cast<uint8_t>(len);
                        packetQueueTail = (packetQueueTail + 1) % PACKET_QUEUE_CAPACITY;
                        packetQueueCount++;
                    }
                }

                serialStateMachine = WAIT_HEAD;
                dataPos = 0;
                checksum = 0;
                len = 0;
                ++processedPackets;
                break;

            default:
                serialStateMachine = WAIT_HEAD;
                dataPos = 0;
                checksum = 0;
                break;
        }
    }
}

eSerialMsgType commConsume(sAngles& angles_)
{
    if (packetQueueCount == 0)
    {
        return eSerialMsgType::UNKNOWN;
    }

    uint8_t currentIndex = packetQueueHead;
    uint8_t currentLen = packetQueueInLen[currentIndex];
    uint8_t packetData[PACKET_MAX_LEN];
    memcpy(packetData, packetQueueIn[currentIndex], currentLen);
    packetQueueHead = (packetQueueHead + 1) % PACKET_QUEUE_CAPACITY;
    packetQueueCount--;

    size_t index = 0;

    eSerialMsgType type = eSerialMsgType::UNKNOWN;

    switch ((eSerialMsgType)packetData[index++])
    {
        case eSerialMsgType::COMMAND:
            if (currentLen != COMMAND_EXPECTED_LEN)
            {
                COMM_DEBUG("commConsume: bad len=");
                COMM_DEBUG(currentLen);
                static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_LENGTH_RECEIVED)};
                commSendNow(eSerialMsgType::ERROR, errPayload, 1);
                return type;
            }

            angles_.vert_a = (int8_t)packetData[index++];
            angles_.vert_b = (int8_t)packetData[index++];
            angles_.vert_c = (int8_t)packetData[index++];
            angles_.vert_d = (int8_t)packetData[index++];
            angles_.vert_e = (int8_t)packetData[index++];
            angles_.vert_f = (int8_t)packetData[index++];
            angles_.hori_a = (int8_t)packetData[index++];
            angles_.hori_b = (int8_t)packetData[index++];
            angles_.hori_c = (int8_t)packetData[index++];
            angles_.hori_d = (int8_t)packetData[index++];
            angles_.hori_e = (int8_t)packetData[index++];
            angles_.hori_f = (int8_t)packetData[index++];
            angles_.tail = (int8_t)packetData[index++];
            type = eSerialMsgType::COMMAND;
            break;

        case eSerialMsgType::STATE:
        {
            if (currentLen != STATE_EXPECTED_LEN)
            {
                COMM_DEBUG(currentLen);
                static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_LENGTH_RECEIVED)};
                commSendNow(eSerialMsgType::ERROR, errPayload, 1);
                return type;
            }

            uint8_t uState = packetData[index++];
            if (uState < static_cast<uint8_t>(eStates::HOME) || uState >= static_cast<uint8_t>(eStates::eLast))
            {
                COMM_DEBUG("Invalid state received");
                COMM_DEBUG(uState);
                static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_STATE_RECEIVED)};
                commSendNow(eSerialMsgType::ERROR, errPayload, 1);
                return type;
            }
            controllerStateMachine = static_cast<eStates>(uState);
            type = eSerialMsgType::STATE;
            break;
        }

        case eSerialMsgType::INFO:
        [[fallthrough]]
        case eSerialMsgType::ERROR:
        [[fallthrough]]
        case eSerialMsgType::HEARTBEAT:
        [[fallthrough]]
        case eSerialMsgType::UNKNOWN:
        [[fallthrough]]
        default:
            COMM_DEBUG("Unsupported serial msg type");
            static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_MSG_TYPE_RECEIVED)};
            commSendNow(eSerialMsgType::ERROR, errPayload, 1);
            return type;
    }

    if (type == eSerialMsgType::COMMAND)
    {
        COMM_DEBUG("Received angles ok");
    }
    return type;
}

static bool comm_writePacket(uint8_t msgType_, const uint8_t* msgContent_, uint8_t msgContentLen_)
{
    if (!commSerial)
    {
        return false;
    }

    uint8_t writeLength = 1 + msgContentLen_;
    uint8_t checksum = msgType_;

    commSerial->write(HEAD);
    commSerial->write(writeLength);
    commSerial->write(msgType_);

    for (uint8_t i = 0; i < msgContentLen_; ++i)
    {
        commSerial->write(msgContent_[i]);
        checksum += msgContent_[i];
    }

    commSerial->write(checksum);
    commSerial->write(TAIL);
    return true;
}

bool commSendNow(eSerialMsgType msgType_, const uint8_t* msgContent_, uint8_t contentLength_)
{
    if (!commSerial)
    {
        return false;
    }

    uint8_t expectedLen;
    const uint8_t* contentPtr = nullptr;

    switch (msgType_)
    {
        case eSerialMsgType::INFO:
        [[fallthrough]]
        case eSerialMsgType::ERROR:
            expectedLen = 1;
            if (msgContent_ == nullptr || contentLength_ != 1)
            {
                return false;
            }
            contentPtr = msgContent_;
            break;

        case eSerialMsgType::HEARTBEAT:
            expectedLen = 0;
            contentPtr = nullptr;
            break;

        default:
            // convert unknown/CMD into ERROR(0x06)
            msgType_ = eSerialMsgType::ERROR;
            static const uint8_t invalidPayload[1] = {static_cast<uint8_t>(eErrorCode::TRIED_TO_SEND_INVALID_COMMAND)};
            expectedLen = 1;
            contentPtr = invalidPayload;
            break;
    }

    return comm_writePacket(static_cast<uint8_t>(msgType_), contentPtr, expectedLen);
}

void commSendEnqueue(eSerialMsgType msgType_, const uint8_t* msgContent_, uint8_t contentLength_)
{
    const uint8_t* contentPtr = nullptr;

    switch (msgType_)
    {
        case eSerialMsgType::INFO:
        [[fallthrough]]
        case eSerialMsgType::ERROR:
            if (msgContent_ == nullptr || contentLength_ != 1)
            {
                msgType_ = eSerialMsgType::ERROR;
                static const uint8_t invalidPayload[1] = {static_cast<uint8_t>(eErrorCode::TRIED_TO_SEND_INVALID_COMMAND)};
                contentPtr = invalidPayload;
                contentLength_ = 1;
            }
            else
            {
                contentPtr = msgContent_;
            }
            break;

        case eSerialMsgType::HEARTBEAT:
            contentPtr = nullptr;
            contentLength_ = 0;
            break;

        default:
            msgType_ = eSerialMsgType::ERROR;
            static const uint8_t invalidPayload[1] = {static_cast<uint8_t>(eErrorCode::TRIED_TO_SEND_INVALID_COMMAND)};
            contentPtr = invalidPayload;
            contentLength_ = 1;
            break;
    }

    if (packetQueueOutCount == PACKET_QUEUE_CAPACITY)
    {
        commSendDequeueAll();
    }

    sPacketOut& packet = packetQueueOut[packetQueueOutCount++];
    packet.type = msgType_;
    packet.len = contentLength_;
    if (contentPtr != nullptr && contentLength_ > 0)
    {
        memcpy(packet.data, contentPtr, contentLength_);
    }
}

void commSendDequeueAll()
{
    for (uint8_t idx = 0; idx < packetQueueOutCount; ++idx)
    {
        comm_writePacket(static_cast<uint8_t>(packetQueueOut[idx].type), packetQueueOut[idx].data, packetQueueOut[idx].len);
    }
    packetQueueOutCount = 0;
}

void commHeartbeat(void)
{
    commSendNow(eSerialMsgType::HEARTBEAT, nullptr, 0);
}

bool commPacketReady()
{
    return packetQueueCount != 0;
}

void commInit(HardwareSerial& serial_)
{
    commSerial = &serial_;
}

void commDebug_impl(const char* msg_)
{
    if (!ENABLE_DEBUG || commSerial == nullptr)
    {
        return;
    }
    commSerial->println(msg_);
}
void commDebug_impl(int v_)
{
    if (!ENABLE_DEBUG || commSerial == nullptr)
    {
        return;
    }
    commSerial->println(v_);
}