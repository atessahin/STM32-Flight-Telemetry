//paketleme fonksiyonu
#include "telemetry.h"
void copyStruct(FlyData_t *D,uint8_t *buff)
{
    uint8_t *src = (uint8_t*)D;
    for (int i = 0; i < sizeof(FlyData_t); i++)
    {
        buff[i]=src[i];
    }
    
}
