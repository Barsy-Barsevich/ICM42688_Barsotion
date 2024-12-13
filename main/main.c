#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "ICM42688_Barsotion.h"
#include "ICM42688_Interface.h"

#include "ICM42688_RegMap.h"
#include "hal/gpio_types.h"
#include "hal/spi_types.h"
#include "pin_defs.h"

ICM42688_t hicm;




//void app_main(void)
//{
//    printf("Hello from app_main!\n");
//    sleep(1);
//    
//    ICM42688_Config_t icm_cfg = {
//		.protocol = Hardware_SPI
//		.spi.host = SPI3_HOST,
//		.spi.miso_pin = MISO_PINNUM,
//		.spi.mosi_pin = MOSI_PINNUM,
//		.spi.sck_pin = CLK_PINNUM,
//		.spi.cs_pin = CS_PINNUM,
//		.spi.sck_freq = 10000000,
//		.accel.enable = ENABLE_XA | ENABLE_YA | ENABLE_ZA,
//		.accel.mode = ACCEL_LN_MODE,
//		.accel.odr = ACCEL_ODR_200HZ,
//		.accel.fs_sel = ACCEL_16G_COEF,
//		.gyro.enable = ENABLE_XG | ENABLE_YG | ENABLE_ZG,
//		.gyro.mode = GYRO_LN_MODE,
//		.gyro.odr = GYRO_ODR_200HZ,
//		.gyro.fs_sel = GYRO_FS_SEL_2000DPS,
//		.fifo.mode = FIFO_STREAM_MODE,
//		.fifo.watermark = 20,
//	};
//    
//    ICM42688_Init(&hicm, &icm_cfg);
////    hicm.readRegister = ICM42688_SPI_readRegister;
//	//hicm.accel_coef = 16. / 32768.;
//	
//	while (1)
//	{
//		while (!ICM42688_FIFO_THS_IRQ_Check(&hicm));
//		uint8_t dummy[2];
//		dummy[0] = 0;
//		hicm.readRegister(ICM_0_FIFO_COUNTH, dummy);
//		hicm.readRegister(ICM_0_FIFO_COUNTL, dummy+1);
//		printf("%u\t", (uint16_t)dummy[0]<<8 | dummy[1]);
//		int32_t raw[6];
//		ICM42688_readFIFO(&hicm, raw);
//		ICM42688_calculateAccel(&hicm, raw+3);
//		ICM42688_calculateGyro(&hicm, raw);
//		//printf("%ld, %ld, %ld, %f, %f, %f\n", raw[0], raw[1], raw[2], hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
//		printf("%f, %f, %f\n", hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
//		
//	}
//}

volatile bool anm;

static void IRAM_ATTR ANMtrap_handler(void* arg)
{
	uint8_t dummy[2];
	hicm.readRegister(ICM_0_INT_STATUS, dummy);
	hicm.readRegister(ICM_0_FIFO_COUNTH, dummy);
	hicm.readRegister(ICM_0_FIFO_COUNTL, dummy+1);
	//printf("%u\t", (uint16_t)dummy[0]<<8 | dummy[1]);
	int32_t raw[6];
	ICM42688_readFIFO(&hicm, raw);
	ICM42688_calculateAccel(&hicm, raw+3);
	ICM42688_calculateGyro(&hicm, raw);
	//printf("%ld, %ld, %ld, %f, %f, %f\n", raw[0], raw[1], raw[2], hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
//	printf("%f, %f, %f\n", hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
	anm = true;
}


void IRAM_ATTR task0(void *pvParameters)
{
	
}


void app_main() 
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
		.accel.odr = ACCEL_ODR_12p5HZ,
		.accel.fs_sel = ACCEL_16G_COEF,
		.gyro.enable = ENABLE_XG | ENABLE_YG | ENABLE_ZG,
		.gyro.mode = GYRO_LN_MODE,
		.gyro.odr = GYRO_ODR_12p5HZ,
		.gyro.fs_sel = GYRO_FS_SEL_2000DPS,
		.fifo.mode = FIFO_STREAM_MODE,
		.fifo.watermark = 20,
		.interrupt.fifo_ths_int_clear = FIFO_THS_INT_CLEAR_ON_STATUS_BIT_READ,
		.interrupt.tdeassert_dis = INT_TDEASSERT_ENABLE,
		.interrupt.tpulse_duration = INT_TPULSE_100US,
		.interrupt.int1.drive_circuit = INT1_PUSH_PULL,
		.interrupt.int1.fifo_ths_en = true,
	};
    
  	// Настраиваем вывод для кнопки
  	gpio_set_direction(GINT1_PINNUM, GPIO_MODE_INPUT);
  	gpio_set_pull_mode(GINT1_PINNUM, GPIO_PULLDOWN_ONLY);

  	// Устанавливаем сервис GPIO ISR service
  	esp_err_t err = gpio_install_isr_service(0);
  	if (err == ESP_ERR_INVALID_STATE)
  	{
    	ESP_LOGW("ISR", "GPIO isr service already installed");
  	};

  	// Регистрируем обработчик прерывания на нажатие кнопки
  	gpio_isr_handler_add(GINT1_PINNUM, ANMtrap_handler, NULL);
  	// Устанавливаем тип события для генерации прерывания - по низкому уровню
  	gpio_set_intr_type(GINT1_PINNUM, GPIO_INTR_LOW_LEVEL);
  	// Разрешаем использование прерываний
  	gpio_intr_enable(GINT1_PINNUM);
  	
  	ICM42688_Init(&hicm, &icm_cfg);
  	
  	anm = false;
  	
  	while (1)
  	{
		if (anm)
		{
			printf("%f, %f, %f\n", hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
			anm = false;
		}
	}
}
