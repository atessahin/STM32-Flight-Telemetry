//define struct
#include <stdint.h>

typedef struct __attribute__((packed))
{
    uint32_t timeStamp;
    float pitch;
    float roll;
    float altitude;
    uint8_t status;
} FlyData_t;
void copyStruct(FlyData_t *D,uint8_t *buff);