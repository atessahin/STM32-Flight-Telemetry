//define struct
#include <stdint.h>

typedef struct __attribute__((packed))
{
	uint16_t header;
    uint32_t timeStamp;
    float pitch;
    float roll;
    float altitude;
    uint8_t status;
    uint8_t checksum;
} FlyData_t;
void copyStruct(FlyData_t *D,uint8_t *buff);
