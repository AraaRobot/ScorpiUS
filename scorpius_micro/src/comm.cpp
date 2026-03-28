/* We use a custom packet made with uint8 with this format
HEAD 0xAA
Data_length
Data ...
Checksum
TAIL 0xBB
*/

#include "comm.h"

#define COMM_EXPECTED_LEN 13

static const uint8_t HEAD = 0xAA;
static const uint8_t TAIL = 0xBB;

static HardwareSerial* commSerial = &Serial;
static volatile int len = 0;
static uint8_t dataBuf[COMM_EXPECTED_LEN];
static uint8_t packet[COMM_EXPECTED_LEN];
static uint8_t dataPos = 0;
static int checksum = 0;
static volatile bool packetReady = false;
static volatile int packetLen = 0;

enum eParserState
{
    WAIT_HEAD,
    READ_LEN,
    READ_DATA,
    READ_CHECKSUM,
    READ_TAIL
};

static eParserState state = WAIT_HEAD;

void comm_process()
{
    while (commSerial->available())
    {
        uint8_t b = (uint8_t)commSerial->read();

        switch (state)
        {
            case WAIT_HEAD:
                if (b == 0xAA)
                {
                    state = READ_LEN;
                    dataPos = 0;
                    checksum = 0;
                }
                break;

            case READ_LEN:
                len = b;
                if (len == 0 || len > COMM_EXPECTED_LEN)
                {
                    COMM_DEBUG("Invalid packet length: ");
                    COMM_DEBUG(len);
                    static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_LENGTH_RECEIVED)};
                    comm_send(eSerialMsgType::ERROR, errPayload, 1);
                    state = WAIT_HEAD;
                }
                else
                {
                    state = READ_DATA;
                }

                break;

            case READ_DATA:
                dataBuf[dataPos] = b;
                ++dataPos;
                checksum += b;

                if (dataPos >= len)
                {
                    state = READ_CHECKSUM;
                }

                break;

            case READ_CHECKSUM:
            {
                uint8_t expected = (uint8_t)(checksum & 0xFF);
                if (b == expected)
                {
                    state = READ_TAIL;
                }
                else
                {
                    // checksum mismatch -> discard
                    COMM_DEBUG("Checksum mismatch: got=");
                    COMM_DEBUG((int)b);
                    COMM_DEBUG(" expected=");
                    COMM_DEBUG((int)expected);

                    static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::CHECKSUM_MISMATCH)};
                    comm_send(eSerialMsgType::ERROR, errPayload, 1);
                    state = WAIT_HEAD;
                }
                break;
            }
            case READ_TAIL:
                if (b == 0xBB)
                {
                    if (packetReady)
                    {
                        COMM_DEBUG("Dropped a packet");

                        static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::PACKET_DROPPED)};
                        comm_send(eSerialMsgType::ERROR, errPayload, 1);
                    }
                    memcpy(packet, dataBuf, size_t(len));
                    packetLen = len;
                    packetReady = true;
                }

                state = WAIT_HEAD;
                dataPos = 0;
                checksum = 0;
                len = 0;
                break;

            default:
                state = WAIT_HEAD;
                dataPos = 0;
                checksum = 0;
                break;
        }
    }
}

bool comm_consume(sAngles& angles_)
{
    if (!packetReady)
    {
        return false;
    }

    if (packetLen != COMM_EXPECTED_LEN)  // Can be change to a switch case if there a more message types in the future
    {
        COMM_DEBUG("comm_consume: bad len=");
        COMM_DEBUG(packetLen);
        static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_LENGTH_RECEIVED)};
        comm_send(eSerialMsgType::ERROR, errPayload, 1);
        packetReady = false;
        state = WAIT_HEAD;
        return false;
    }

    size_t index = 0;

    if ((int8_t)packet[index++] != static_cast<uint8_t>(eSerialMsgType::COMMAND))
    {
        COMM_DEBUG("Unknown serial msg type");
        static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_MSG_TYPE_RECEIVED)};
        comm_send(eSerialMsgType::ERROR, errPayload, 1);
        packetReady = false;
        state = WAIT_HEAD;
        len = 0;
        return false;
    }

    angles_.vert_a = (int8_t)packet[index++];
    angles_.vert_b = (int8_t)packet[index++];
    angles_.vert_c = (int8_t)packet[index++];
    angles_.vert_d = (int8_t)packet[index++];
    angles_.vert_e = (int8_t)packet[index++];
    angles_.vert_f = (int8_t)packet[index++];
    angles_.hori_a = (int8_t)packet[index++];
    angles_.hori_b = (int8_t)packet[index++];
    angles_.hori_c = (int8_t)packet[index++];
    angles_.hori_d = (int8_t)packet[index++];
    angles_.hori_e = (int8_t)packet[index++];
    angles_.hori_f = (int8_t)packet[index++];

    packetReady = false;
    state = WAIT_HEAD;
    len = 0;
    COMM_DEBUG("Received angles ok");
    return true;
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

bool comm_send(eSerialMsgType msgType_, const uint8_t* msgContent_, uint8_t contentLength_)
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

void comm_heartbeat(void)
{
    comm_send(eSerialMsgType::HEARTBEAT, nullptr, 0);
}

void comm_init(HardwareSerial& serial)
{
    commSerial = &serial;
}

void comm_debug_impl(const char* msg)
{
    if (!ENABLE_DEBUG || commSerial == nullptr)
    {
        return;
    }
    commSerial->println(msg);
}
void comm_debug_impl(int v)
{
    if (!ENABLE_DEBUG || commSerial == nullptr)
    {
        return;
    }
    commSerial->println(v);
}