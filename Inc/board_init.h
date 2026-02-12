#ifndef BOARD_INIT_H
#define BOARD_INIT_H

void gpioConfig(void);
void i2cConfig(void);
void boardInit(void);
void uart2Config();
void dmaInit();

#endif // BOARD_INIT_H
