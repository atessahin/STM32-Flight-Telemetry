#include "dma_bus.h"
#include "stm32f4xx.h"

void dmaUsart2Transmit(uint8_t *dataBuffer, uint16_t size)
{
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while(DMA1_Stream6->CR & DMA_SxCR_EN);
    DMA1->HIFCR |= (1 << 21) | (1 << 20);
    DMA1_Stream6->NDTR = size;
    DMA1_Stream6->M0AR = (uint32_t)dataBuffer;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}
