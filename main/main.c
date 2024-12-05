#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "ICM42688_Barsotion.h"
#include "ICM42688_Interface.h"

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
		.gyro.enable = ENABLE_XG | ENABLE_YG | ENABLE_ZG,
		.gyro.mode = GYRO_LN_MODE,
		.gyro.odr = GYRO_ODR_2KHZ,
	};
    
    ICM42688_spiInit(MISO_PINNUM, MOSI_PINNUM, CLK_PINNUM, CS_PINNUM);
    //ICM42688_Init(&hicm, &icm_cfg);
    hicm.read_reg = ICM42688_readRegister;
    
    while (1)
    {
		printf("Reading who_am_i:");
		uint8_t dummy;
		hicm.read_reg(ICM_0_WHO_AM_I, &dummy);
		printf(" %02X\n", dummy);
		vTaskDelay(10);
	}
}
