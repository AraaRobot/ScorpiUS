/* We use a custom packet made with uint8 with this format
HEAD 0xAA
Data_lenght
Data ...
Checksum
TAIL 0xBB
*/

#include "comm.h"

#define COMM_MAX_SERVO 16

static HardwareSerial* commSerial = &Serial;
static int len = 0;
static uint8_t dataBuf[COMM_MAX_SERVO];
static uint8_t dataPos = 0;
static int checksum = 0;
static bool packetReady = false;

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
                if (len == 0)
                {
                    state = READ_CHECKSUM;
                }
                else if (len <= COMM_MAX_SERVO)
                {
                    state = READ_DATA;
                }
                else
                {
                    state = WAIT_HEAD;
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

            case READ_TAIL:
                if (b == 0xBB)
                {
                    packetReady = true;
                }
                else
                {
                    state = WAIT_HEAD;
                }
                break;

            default:
                state = WAIT_HEAD;
                break;
        }
    }
}

void comm_init(HardwareSerial& serial)
{
    commSerial = &serial;
}