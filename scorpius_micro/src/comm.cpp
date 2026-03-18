/* We use a custom packet made with uint8 with this format
HEAD 0xAA
Data_length
Data ...
Checksum
TAIL 0xBB
*/

#include "comm.h"

#define COMM_EXPECTED_LEN 12

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
                    commSerial->print("Invalid packet length: ");
                    commSerial->print(len);
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
                    commSerial->print("Checksum mismatch: got=");
                    commSerial->print((int)b);
                    commSerial->print(" expected=");
                    commSerial->println((int)expected);
                    state = WAIT_HEAD;
                }
                break;
            }
            case READ_TAIL:
                if (b == 0xBB)
                {
                    if (packetReady)
                    {
                        commSerial->print("Dropped a packet");
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

void comm_consume(sAngles& angles_)
{
    if (!packetReady)
    {
        return;
    }

    if (packetLen != COMM_EXPECTED_LEN)  // Can be change to a switch case if there a more message types in the future
    {
        commSerial->print("comm_consume: bad len=");
        commSerial->println(len);
        packetReady = false;
        state = WAIT_HEAD;
        return;
    }

    size_t index = 0;
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
}

void comm_init(HardwareSerial& serial)
{
    commSerial = &serial;
}