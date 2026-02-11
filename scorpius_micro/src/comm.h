#ifndef COMM_H
#define COMM_H

#include <Arduino.h>

struct sAngles
{
    uint8_t vert_a = 0;
    uint8_t vert_b = 0;
    uint8_t vert_c = 0;
    uint8_t vert_d = 0;
    uint8_t vert_e = 0;
    uint8_t vert_f = 0;
    uint8_t hori_a = 0;
    uint8_t hori_b = 0;
    uint8_t hori_c = 0;
    uint8_t hori_d = 0;
    uint8_t hori_e = 0;
    uint8_t hori_f = 0;
};

void comm_init(HardwareSerial& serial);
void comm_process();


#endif // COMM_H