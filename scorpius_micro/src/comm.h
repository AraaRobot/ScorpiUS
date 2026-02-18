#ifndef COMM_H
#define COMM_H

#include <Arduino.h>

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

void comm_init(HardwareSerial& serial);
void comm_process();
void comm_consume(sAngles& angles_);


#endif // COMM_H