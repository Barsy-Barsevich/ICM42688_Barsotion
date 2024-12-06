#pragma once

#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "ICM42688_RegMap.h"


void ICM42688_spiInit(int miso, int mosi, int sck, int cs);

void ICM42688_readRegister(uint8_t reg, uint8_t *buf);
void ICM42688_writeRegister(uint8_t reg, uint8_t data);

void ICM42688_readRegAG(int32_t *raw);