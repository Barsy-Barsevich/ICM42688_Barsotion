#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "ICM42688_Barsotion.h"
#include "ICM42688_Interface.h"

#include "ICM42688_RegMap.h"
#include "pin_defs.h"

ICM42688_t hicm;




void app_main(void)
{
    printf("Hello from app_main!\n");
    sleep(1);
    
    ICM42688_Config_t icm_cfg = {
		.accel.enable = ENABLE_XA | ENABLE_YA | ENABLE_ZA,
		.accel.mode = ACCEL_LN_MODE,
		.accel.odr = ACCEL_ODR_2KHZ,
		.accel.fs_sel = ACCEL_16G_COEF,
		.gyro.enable = ENABLE_XG | ENABLE_YG | ENABLE_ZG,
		.gyro.mode = GYRO_LN_MODE,
		.gyro.odr = GYRO_ODR_2KHZ,
		.gyro.fs_sel = GYRO_FS_SEL_2000DPS,
	};
    
    ICM42688_SPI_InterfaceInit(MISO_PINNUM, MOSI_PINNUM, CLK_PINNUM, CS_PINNUM);
    ICM42688_Init(&hicm, &icm_cfg);
//    hicm.readRegister = ICM42688_SPI_readRegister;
	//hicm.accel_coef = 16. / 32768.;
    
    while (1)
    {
		printf("Reading who_am_i:");
		int32_t raw[6];
		//hicm.read_reg(ICM_0_WHO_AM_I, &dummy);
		ICM42688_readRegAG(&hicm, raw);
		//printf(" %d, %d, %d, %d, %d, %d\n", (int)raw[0], (int)raw[1], (int)raw[2], (int)raw[3], (int)raw[4], (int)raw[5]);
		//printf(" %04lX, %ld\n", (uint32_t)raw[0], raw[0]);
		ICM42688_calculateGyro(&hicm, raw);
		ICM42688_calculateAccel(&hicm, raw+3);
		printf("%ld, %ld, %ld, %f, %f, %f\n", raw[3], raw[4], raw[5], hicm.accel.x, hicm.accel.y, hicm.accel.z);
		
		vTaskDelay(1);
	}
}
