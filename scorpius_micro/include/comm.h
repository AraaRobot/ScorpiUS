#ifndef COMM_H
#define COMM_H

#include <Arduino.h>

#ifndef ENABLE_DEBUG
#define ENABLE_DEBUG 0
#endif

#if ENABLE_DEBUG
#define COMM_DEBUG(x) commDebug_impl(x)
#else
#define COMM_DEBUG(x) ((void)0)
#endif

#define MAX_PROCESSED_PER_LOOP 3
#define PACKET_MAX_LEN 16
#define COMMAND_EXPECTED_LEN 13
#define STATE_EXPECTED_LEN 2
struct sAngles
{
    int8_t vert_a = 0;
    int8_t vert_b = 0;
    int8_t vert_c = 0;
    int8_t vert_d = 0;
    int8_t vert_e = 0;
    int8_t vert_f = 0;
    int8_t hori_a = 0;
    int8_t hori_b = 0;
    int8_t hori_c = 0;
    int8_t hori_d = 0;
    int8_t hori_e = 0;
    int8_t hori_f = 0;
};

enum class eSerialMsgType : uint8_t
{
    COMMAND = 0x00,
    INFO = 0x01,
    ERROR = 0x02,
    HEARTBEAT = 0x03,
    STATE = 0x04,
    UNKNOWN = 0xFF
};

enum class eErrorCode : uint8_t
{
    INVALID_LENGTH_RECEIVED = 0x01,
    CHECKSUM_MISMATCH = 0x02,
    PACKET_DROPPED = 0x03,
    INVALID_MSG_TYPE_RECEIVED = 0x04,
    INVALID_COMMAND_RECEIVED = 0x05,
    TRIED_TO_SEND_INVALID_COMMAND = 0x06,
    INVALID_SERVO_ID = 0x07,
    INVALID_STATE_RECEIVED = 0x08
};

enum class eInfoCode : uint8_t
{
    INIT_COMPLETE = 0x01,
    SERVOS_HOMED = 0x02
};

struct sPacketOut
{
    eSerialMsgType type;
    uint8_t len;
    uint8_t data[PACKET_MAX_LEN];
};

void commInit(HardwareSerial& serial_);
void commProcess();
bool commPacketReady();
eSerialMsgType commConsume(sAngles& angles_);
bool commSendNow(eSerialMsgType msgType_, const uint8_t* msgContent_, uint8_t contentLength_);
void commHeartbeat(void);
void commSendEnqueue(eSerialMsgType msgType_, const uint8_t* msgContent_, uint8_t contentLength_);
void commSendDequeueAll();

void commDebug_impl(const char* msg_);
void commDebug_impl(int v_);

#endif  // COMM_H