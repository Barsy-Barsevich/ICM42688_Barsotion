#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "ICM42688_Barsotion.h"
#include "ICM42688_Interface.h"

#include "ICM42688_RegMap.h"
#include "hal/spi_types.h"
#include "pin_defs.h"

ICM42688_t hicm;




void app_main(void)
{
    printf("Hello from app_main!\n");
    sleep(1);
    
    ICM42688_Config_t icm_cfg = {
		.protocol = Hardware_SPI,
		.spi.host = SPI3_HOST,
		.spi.miso_pin = MISO_PINNUM,
		.spi.mosi_pin = MOSI_PINNUM,
		.spi.sck_pin = CLK_PINNUM,
		.spi.cs_pin = CS_PINNUM,
		.spi.sck_freq = 1000000,
		.accel.enable = ENABLE_XA | ENABLE_YA | ENABLE_ZA,
		.accel.mode = ACCEL_LN_MODE,
		.accel.odr = ACCEL_ODR_6p25HZ,
		.accel.fs_sel = ACCEL_16G_COEF,
		.gyro.enable = ENABLE_XG | ENABLE_YG | ENABLE_ZG,
		.gyro.mode = GYRO_LN_MODE,
		.gyro.odr = GYRO_ODR_12p5HZ,
		.gyro.fs_sel = GYRO_FS_SEL_2000DPS,
		.fifo.mode = FIFO_STOP_ON_FULL_MODE,
		.fifo.watermark = 0xFFF,
	};
    
    ICM42688_Init(&hicm, &icm_cfg);
//    hicm.readRegister = ICM42688_SPI_readRegister;
	//hicm.accel_coef = 16. / 32768.;
	
	static uint8_t buf[5000] = {0};
    
    while (1)
    {
//		printf("Reading who_am_i:");
//		int32_t raw[6];
//		//hicm.read_reg(ICM_0_WHO_AM_I, &dummy);
//		ICM42688_readRegAG(&hicm, raw);
//		//printf(" %d, %d, %d, %d, %d, %d\n", (int)raw[0], (int)raw[1], (int)raw[2], (int)raw[3], (int)raw[4], (int)raw[5]);
//		//printf(" %04lX, %ld\n", (uint32_t)raw[0], raw[0]);
//		ICM42688_calculateGyro(&hicm, raw);
//		ICM42688_calculateAccel(&hicm, raw+3);
//		printf("%ld, %ld, %ld, %f, %f, %f\n", raw[3], raw[4], raw[5], hicm.accel.x, hicm.accel.y, hicm.accel.z);
//		
		ICM42688_SPI_readFIFO(hicm.interface, buf, 20);
		
		for (size_t i=0; i<20; i++)
		{
			printf("%02X ", buf[i]);
		}
		printf("\n");

		uint8_t dummy[2];
		dummy[0] = 0;
		hicm.readRegister(ICM_0_FIFO_COUNTH, dummy);
		hicm.readRegister(ICM_0_FIFO_COUNTL, dummy+1);
		printf("Count: %u\t", (uint16_t)dummy[0]<<8 | dummy[1]);
//		
//		ICM42688_SPI_readFIFO(hicm.interface, buf, 20);
//		
//		hicm.readRegister(ICM_0_FIFO_LOST_PKT1, dummy);
//		hicm.readRegister(ICM_0_FIFO_LOST_PKT0, dummy+1);
//		printf("Lost: %u\n", (uint16_t)dummy[0]<<8 | dummy[1]);
		
		
		
		vTaskDelay(10);
	}
}
