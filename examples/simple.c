
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>

#include "../src/ICM42688_Barsotion.h"
#include "../main/pin_defs.h"


#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "hal/spi_types.h"



#include "portmacro.h"

ICM42688_t hicm;
static QueueHandle_t button_queue = NULL;

volatile bool anm;
int32_t raw[6];


static void IRAM_ATTR IMU_IRQ_handler(void* arg)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;
	bool ready = true;
	xResult = xQueueSendFromISR(button_queue, &ready, &xHigherPriorityTaskWoken);
	// Если высокоприоритетная задача ждет этого события, переключаем управление
  	if (xResult == pdPASS) {
    	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  	};
}


void IRAM_ATTR IMU_IRQ_process(void *pvParameters)
{
	bool res;
	while (1)
	{
		xQueueReceive(button_queue, &res, portMAX_DELAY);
		if (res)
		{
			uint8_t dummy[2];
			hicm.readRegister(ICM_0_INT_STATUS, dummy);
			hicm.readRegister(ICM_0_FIFO_COUNTH, dummy);
			hicm.readRegister(ICM_0_FIFO_COUNTL, dummy+1);
			//printf("%u\t", (uint16_t)dummy[0]<<8 | dummy[1]);
			ICM42688_readFIFO(&hicm, raw);
			ICM42688_calculateAccel(&hicm, raw+3);
			ICM42688_calculateGyro(&hicm, raw);
			
			/* Здесь можно делать все, что угодно, связанное с обработкой данных */
			//printf("%ld, %ld, %ld, %f, %f, %f\n", raw[0], raw[1], raw[2], hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
			//printf("%f, %f, %f\n", hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
			printf("%f, %f, %f\n", hicm.accel.x, hicm.accel.y, hicm.accel.z);
		}
	}
}


static void IMU_Init()
{
	ICM42688_Config_t icm_cfg = {
		.protocol = Hardware_SPI,
		.spi.host = SPI3_HOST,
		.spi.miso_pin = IMU_MISO_PINNUM,
		.spi.mosi_pin = IMU_MOSI_PINNUM,
		.spi.sck_pin = IMU_SCK_PINNUM,
		.spi.cs_pin = IMU_CS_PINNUM,
		.spi.sck_freq = 5000000,
		.accel.enable = ENABLE_XA | ENABLE_YA | ENABLE_ZA,
		.accel.mode = ACCEL_LN_MODE,
		.accel.odr = ACCEL_ODR_200HZ,
		.accel.scale = ACCEL_16G_COEF,
		.gyro.enable = ENABLE_XG | ENABLE_YG | ENABLE_ZG,
		.gyro.mode = GYRO_LN_MODE,
		.gyro.odr = GYRO_ODR_200HZ,
		.gyro.scale = GYRO_FS_SEL_2000DPS,
		.fifo.mode = FIFO_STREAM_MODE,
		.fifo.watermark = 20,
		.interrupt.cfg.fifo_ths_int_clear = FIFO_THS_INT_CLEAR_ON_STATUS_BIT_READ,
		.interrupt.cfg.tdeassert_dis = INT_TDEASSERT_ENABLE,
		.interrupt.cfg.tpulse_duration = INT_TPULSE_8US,
		.interrupt.int1.drive_circuit = PUSH_PULL,
		.interrupt.int1.fifo_ths_en = false,
	};
	
	button_queue = xQueueCreate(32, sizeof(bool));
	// Запускаем задачу управления светодиодом
  	xTaskCreatePinnedToCore(IMU_IRQ_process, "imu", 4096, NULL, 10, NULL, 0);
    
  	// Настраиваем вывод для кнопки
  	gpio_set_direction(IMU_INT_PINNUM, GPIO_MODE_INPUT);
  	gpio_set_pull_mode(IMU_INT_PINNUM, GPIO_PULLDOWN_ONLY);

  	// Устанавливаем сервис GPIO ISR service
  	esp_err_t err = gpio_install_isr_service(0);
  	if (err == ESP_ERR_INVALID_STATE)
  	{
    	ESP_LOGW("ISR", "GPIO isr service already installed");
  	};

  	// Регистрируем обработчик прерывания на нажатие кнопки
  	gpio_isr_handler_add(IMU_INT_PINNUM, IMU_IRQ_handler, NULL);
  	// Устанавливаем тип события для генерации прерывания - по низкому уровню
  	// Важно генерить прерывания именно по отрицательному фронту, а не по
  	// низкому уровню, ибо иначе будет interrupt wdt timeout error
  	gpio_set_intr_type(IMU_INT_PINNUM, GPIO_INTR_NEGEDGE);
  	// Разрешаем использование прерываний
  	gpio_intr_enable(IMU_INT_PINNUM);
  	
  	ICM42688_Init(&hicm, &icm_cfg);
	
	ICM42688_gyroAntiAliasFilterEnable(&hicm);
	ICM42688_gyroNotchFilterEnable(&hicm);
  	
	ICM42688_calibrateGyro(&hicm);
	
	ICM42688_INT_Channel_Config_t int1_cfg = {
		.fifo_ths_en = true,
		.drive_circuit = PUSH_PULL,
	};
	ICM42688_setINT1Config(&hicm, &int1_cfg);
}


void app_main() 
{
	printf("Hello from app_main!\n");
    sleep(1);
    //calibrateGyro();
    
    IMU_Init();
  	
  	while (1);
}
