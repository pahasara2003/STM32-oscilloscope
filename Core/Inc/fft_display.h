#ifndef FFT_DISPLAY_H
#define FFT_DISPLAY_H

#include "main.h"
#include "ILI9341_STM32_Driver.h"
#include "arm_math.h"

extern SPI_HandleTypeDef hspi1;
extern volatile uint8_t spi_dma_done;
extern uint16_t displayed_data1[];
extern uint16_t displayed_data2[];

void FFT_Init(void);
void Plot_FFT(void);
void Draw_FFT_Labels();

#endif