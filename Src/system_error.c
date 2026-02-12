
#include "system_error.h"
#include "stm32f4xx.h"
#include "delay.h"
void System_Error_Handler(int error_code)
{
	while(1)
	{
		for (int i = 0; i < error_code; i++)
		{
			GPIOA->BSRR = (1U << 10);//LED ON
			delay_ms(1000);
			GPIOA->BSRR = (1U << (10+16));//LED OFF
			delay_ms(500);
		}
		delay_ms(2000);
	}

}
/*
2 Kere,IMU (MPU6050),Sensör bulunamadı veya bozuk.,mpu6050_init() içinde 0x68 kontrolünde.
3 Kere,Barometre (BMP280),Basınç sensörü yanıt vermiyor.,bmp280_init() içinde 0x60 kontrolünde.
 */
