#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "config.h"
#include "stdio.h"
int fputc(int ch, FILE *f);
	
#define system_delay_ms HAL_Delay
void spi_write_8bit_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data);
void spi_write_8bit_registers(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t* data, uint16_t len);
uint8_t spi_read_8bit_register(SPI_HandleTypeDef *hspi, uint8_t reg);
void spi_read_8bit_registers(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t* data, uint16_t len);
	
#ifdef __cplusplus
}
#endif