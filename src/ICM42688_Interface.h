#pragma once

#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "ICM42688_RegMap.h"


typedef struct __InterfaceDescriptor
{
	bool busy;
} ICM42688_Interface_t;


void ICM42688_SPI_InterfaceInit(spi_host_device_t host, int miso, int mosi, int sck, int cs, int sck_freq);

void ICM42688_SPI_readRegister(uint8_t reg, uint8_t *buf);
void ICM42688_SPI_writeRegister(uint8_t reg, uint8_t data);
void ICM42688_SPI_readFIFO(ICM42688_Interface_t *local, uint8_t *buf, size_t quan);

void ICM42688_I2C_readRegister(uint8_t reg, uint8_t *buf);
void ICM42688_I2C_writeRegister(uint8_t reg, uint8_t data);