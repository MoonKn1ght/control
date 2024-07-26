#include "hal.h"

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

void spi_write_8bit_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data){
    uint8_t temp = reg;
    HAL_SPI_Transmit(hspi, &temp, 1, 0xffff);
    temp = data;
    HAL_SPI_Transmit(hspi, &temp, 1, 0xffff);
}

void spi_write_8bit_registers(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t* data, uint16_t len){
    uint8_t temp = reg;
    HAL_SPI_Transmit(hspi, &temp, 1, 0xffff);
    HAL_SPI_Transmit(hspi, data, len, 0xffff);
}

uint8_t spi_read_8bit_register(SPI_HandleTypeDef *hspi, uint8_t reg){
    uint8_t temp = reg;
    HAL_SPI_Transmit(hspi, &temp, 1, 0xffff);
    HAL_SPI_Receive(hspi, &temp, 1, 0xffff);
    return temp;
}

void spi_read_8bit_registers(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t* data, uint16_t len){
    uint8_t temp = reg;
    HAL_SPI_Transmit(hspi, &temp, 1, 0xffff);
    HAL_SPI_Receive(hspi, data, len, 0xffff);
}