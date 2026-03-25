#ifndef COMM_H
#define COMM_H

#include <Arduino.h>

#ifndef ENABLE_DEBUG
#define ENABLE_DEBUG 0
#endif

#if ENABLE_DEBUG
#define COMM_DEBUG(x) comm_debug_impl(x)
#else
#define COMM_DEBUG(x) ((void)0)
#endif

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
    HEARTBEAT = 0x03
};

enum class eErrorCode : uint8_t
{
    INVALID_LENGTH_RECEIVED = 0x01,
    CHECKSUM_MISMATCH = 0x02,
    PACKET_DROPPED = 0x03,
    INVALID_MSG_TYPE_RECEIVED = 0x04,
    INVALID_COMMAND_RECEIVED = 0x05,
    TRIED_TO_SEND_INVALID_COMMAND = 0x06,
    INVALID_SERVO_ID = 0x07
};

enum class eInfoCode: uint8_t
{
    INIT_COMPLETE = 0x01,
    SERVOS_HOMED = 0x02
};

void comm_init(HardwareSerial& serial);
void comm_process();
bool comm_consume(sAngles& angles_);
bool comm_send(uint8_t msgType_, const uint8_t* msgContent_, uint8_t contentLength_);

void comm_debug_impl(const char* msg);
void comm_debug_impl(int v);

#endif  // COMM_H