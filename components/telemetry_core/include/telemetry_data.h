#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <stdint.h>

typedef struct __attribute__((packed))
{
    uint16_t header;     // 0xCDAB
    uint32_t timeStamp;
    float pitch;
    float roll;
    float altitude;
    uint8_t status;
    uint8_t checksum;
} FlyData_t;

#endif // TELEMETRY_DATA_H