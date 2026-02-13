#include "stm32f4xx.h"
#include "system_clock.h"

void gpioConfig();
void i2cConfig();

void boardInit()
{
	systemClockConfig();
	gpioConfig();
}


void gpioConfig()
{
	RCC->AHB1ENR |= (1<<1); // Enable GPIOB
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA

	// Configure PB6 SCL and PB7 SDA for Alternate Function
	GPIOB->MODER &= ~((3<<12) | (3<<14)); // PB6, PB7 clear MODE bits
	GPIOB->MODER |=  ((2<<12) | (2<<14)); // Set to Alternate Function Mode

	// Set output type to Open-Drain
	GPIOB->OTYPER |= (1<<6) | (1<<7);

	// Configure Pull-up resistors
	GPIOB->PUPDR &= ~((3<<12) | (3<<14));
	GPIOB->PUPDR |=  (1<<12) | (1<<14); // Set to Pull-up

	// Select Alternate Function 4 AF4 for I2C1
	GPIOB->AFR[0] &= ~((0xF<<24) | (0xF<<28)); // Clear AF bits
	GPIOB->AFR[0] |=  ((4<<24) | (4<<28)); // Set AF4 for PB6/PB7

	//USART2 TX (PA2)

	// Set PA2 mode to Alternate Function (10)
	GPIOA->MODER &= ~(3 << 4);
	GPIOA->MODER |=  (2 << 4);

	// Select AF7 (USART2) for PA2 (AFRL[11:8])
	GPIOA->AFR[0] &= ~(0xF << 8);
	GPIOA->AFR[0] |=  (0x7 << 8);

	// Set PA2 output speed to High
	GPIOA->OSPEEDR |= (3 << 4);


	//USART2 RX (PA3)

	// Set PA3 mode to Alternate Function (10)
	GPIOA->MODER &= ~(3 << 6);
	GPIOA->MODER |=  (2 << 6);

	// Select AF7 (USART2) for PA3 (AFRL[15:12])
	GPIOA->AFR[0] &= ~(0xF << 12);
	GPIOA->AFR[0] |=  (0x7 << 12);


	//LED (PA10) for debug

	// Set PA10 as General Purpose Output (01)
	GPIOA->MODER &= ~(3 << 20);
	GPIOA->MODER |=  (1 << 20);

}

void i2cConfig()
{
    RCC->APB1ENR |= (1<<21); // Enable I2C1 clock

    // Software reset
    I2C1->CR1 |= (1<<15);
    I2C1->CR1 &= ~(1<<15);

    I2C1->CR1 &= ~(1<<0);  // Disable Peripheral

    // Set timing registers for 100kHz Standard Mode
    I2C1->CR2 = 42;    // PCLK1 frequency in MHz
    I2C1->CCR = 210;   // Clock Control Register
    I2C1->TRISE = 43;  // Rise Time Register

    I2C1->CR1 |= (1<<10);  // Enable Acknowledge
    I2C1->CR1 |= (1<<0);   // Enable Peripheral

    for(volatile int i=0;i<50000;i++);
}

//FOR USART2_TX STREAM_6
void uart2Config()
{
	RCC->APB1ENR|=RCC_APB1ENR_USART2EN ;

	USART2->CR1 &= ~USART_CR1_UE;
	/*USART_BRR =Fck / Baud
	 *
	 42,000,000 / 9,600 = 4375

	 Mantissa = 4375
	 Fraction = 0

	 BRR = (Mantissa << 4) | Fraction

	 Mantissa << 4 = 4375 × 16 = 70000
	 Fraction        = 0
	 --------------------------------
	 BRR             = 70000 (decimal)


	70000 (decimal) = 0x11170

	USART2->BRR = 0x1170;

	 */
    USART2->BRR  = 0x1117;

	USART2->CR1 &= ~USART_CR1_M;

	USART2->CR1 &= ~USART_CR1_PCE;

	USART2->CR1|=USART_CR1_RE;

	USART2->CR1|=USART_CR1_TE;

	USART2->CR3|=USART_CR3_DMAR;

	USART2->CR3|=USART_CR3_DMAT;

	USART2->CR1|=USART_CR1_UE;
}

/*

| Setting                          | What does it provide?                      | Why is it enabled/disabled?              |
| -------------------------------- | ------------------------------------------ | ---------------------------------------- |
| **Circular mode**                | Restarts from the beginning after transfer | For continuous data streaming            |
| **Peripheral increment disable** | Keeps the peripheral address fixed         | For fixed-address registers like DR      |
| **Memory increment enable**      | Increments memory pointer each transfer    | To write data sequentially into a buffer |

 */
void dmaInit()
{
	RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
	DMA1_Stream6->CR&= ~DMA_SxCR_EN;
	while(DMA1_Stream6->CR & DMA_SxCR_EN);
	DMA1_Stream6->CR|= DMA_SxCR_DIR_0;

	DMA1_Stream6->CR &= ~DMA_SxCR_CIRC;
	DMA1_Stream6->CR &= ~DMA_SxCR_PINC;
	DMA1_Stream6->CR|=DMA_SxCR_MINC;
	DMA1_Stream6->CR &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE);
	DMA1_Stream6->CR|=(DMA_SxCR_CHSEL_2);
	DMA1_Stream6->CR|=(DMA_SxCR_PL_0);
	DMA1_Stream6->PAR = (uint32_t)&(USART2->DR);

}

