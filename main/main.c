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
		.spi.sck_freq = 10000000,
		.accel.enable = ENABLE_XA | ENABLE_YA | ENABLE_ZA,
		.accel.mode = ACCEL_LN_MODE,
		.accel.odr = ACCEL_ODR_50HZ,
		.accel.fs_sel = ACCEL_16G_COEF,
		.gyro.enable = ENABLE_XG | ENABLE_YG | ENABLE_ZG,
		.gyro.mode = GYRO_LN_MODE,
		.gyro.odr = GYRO_ODR_50HZ,
		.gyro.fs_sel = GYRO_FS_SEL_2000DPS,
		.fifo.mode = FIFO_STOP_ON_FULL_MODE,
		.fifo.watermark = 20,
	};
    
    ICM42688_Init(&hicm, &icm_cfg);
//    hicm.readRegister = ICM42688_SPI_readRegister;
	//hicm.accel_coef = 16. / 32768.;
	
	while (1)
	{
		while (!ICM42688_FIFO_THS_IRQ_Check(&hicm));
		uint8_t dummy[2];
		dummy[0] = 0;
		hicm.readRegister(ICM_0_FIFO_COUNTH, dummy);
		hicm.readRegister(ICM_0_FIFO_COUNTL, dummy+1);
		printf("%u\t", (uint16_t)dummy[0]<<8 | dummy[1]);
		int32_t raw[6];
		ICM42688_readFIFO(&hicm, raw);
		ICM42688_calculateAccel(&hicm, raw+3);
		ICM42688_calculateGyro(&hicm, raw);
		//printf("%ld, %ld, %ld, %f, %f, %f\n", raw[0], raw[1], raw[2], hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
		printf("%f, %f, %f\n", hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
		
	}
}
