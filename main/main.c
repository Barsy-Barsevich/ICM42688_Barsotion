#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>

#include "ICM42688_Barsotion.h"
#include "ICM42688_Interface.h"

#include "ICM42688_RegMap.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "hal/spi_types.h"
#include "pin_defs.h"
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
			//printf("%ld, %ld, %ld, %f, %f, %f\n", raw[0], raw[1], raw[2], hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
			//printf("%f, %f, %f\n", hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
			printf("%f, %f, %f\n", hicm.accel.x, hicm.accel.y, hicm.accel.z);
		}
	}
}

static float vReal[4096];
static float vImag[4096];

#include "microFFT.h"
void spectreGyro()
{
	const int iter_number = 4096;
	const int freq = 4000;
	ICM42688_FIFO_MODE_t fifo_mode = hicm.fifo_mode;
	ICM42688_GYRO_ODR_t odr = hicm.gyro_odr;
	ICM42688_setFIFOMode(&hicm, FIFO_BYPASS_MODE);
	ICM42688_setGyroODR(&hicm, GYRO_ODR_4KHZ);
	ICM42688_flushFIFO(&hicm);
	size_t counter = 0;
	int32_t raw_data[6];
//	float vReal[iter_number];
//	float vImag[iter_number];
//	float* vReal = (float*)malloc(iter_number);
//	float* vImag = (float*)malloc(iter_number);
	ICM42688_setFIFOMode(&hicm, FIFO_STREAM_MODE);
	
	while (counter < iter_number)
	{
		if (ICM42688_FIFO_THS_IRQ_Check(&hicm))
		{
			ICM42688_readFIFO(&hicm, raw_data);
			ICM42688_calculateGyro(&hicm, raw_data);
			vReal[counter] = hicm.gyro.x;
			vImag[counter] = 0.0;
			counter += 1;
		}
	}
	
	FFT_Init(vReal, vImag, iter_number, freq);
	
	FFT_Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Recommended use for high frequencies. Relative to Sampling Frequency. In this case: 70Hz
    FFT_Compute(FFT_FORWARD);
    FFT_ComplexToMagnitude();
    
    for (size_t i=0; i<(iter_number>>1); i++)
    {
		float abscissa = (((float)i * freq) / iter_number);
		if ((i % 8) == 1) printf("%f    %f\n", abscissa, vReal[i]);
	}
	
	ICM42688_setFIFOMode(&hicm, FIFO_BYPASS_MODE);
	ICM42688_setGyroODR(&hicm, odr);
	ICM42688_flushFIFO(&hicm);
	ICM42688_setFIFOMode(&hicm, fifo_mode);
}


void app_main() 
{
	printf("Hello from app_main!\n");
    sleep(1);
    //calibrateGyro();
    
    ICM42688_Config_t icm_cfg = {
		.protocol = Hardware_SPI,
		.spi.host = SPI3_HOST,
		.spi.miso_pin = MISO_PINNUM,
		.spi.mosi_pin = MOSI_PINNUM,
		.spi.sck_pin = CLK_PINNUM,
		.spi.cs_pin = CS_PINNUM,
		.spi.sck_freq = 50000000,
		.accel.enable = ENABLE_XA | ENABLE_YA | ENABLE_ZA,
		.accel.mode = ACCEL_LN_MODE,
		.accel.odr = ACCEL_ODR_12p5HZ,
		.accel.scale = ACCEL_16G_COEF,
		.gyro.enable = ENABLE_XG | ENABLE_YG | ENABLE_ZG,
		.gyro.mode = GYRO_LN_MODE,
		.gyro.odr = GYRO_ODR_12p5HZ,
		.gyro.scale = GYRO_FS_SEL_2000DPS,
		.fifo.mode = FIFO_STREAM_MODE,
		.fifo.watermark = 20,
		.interrupt.cfg.fifo_ths_int_clear = FIFO_THS_INT_CLEAR_ON_STATUS_BIT_READ,
		.interrupt.cfg.tdeassert_dis = INT_TDEASSERT_ENABLE,
		.interrupt.cfg.tpulse_duration = INT_TPULSE_8US,
		.interrupt.int1.drive_circuit = PUSH_PULL,
		.interrupt.int1.fifo_ths_en = false,
	};
	
	//calibrateGyro();
	
	button_queue = xQueueCreate(32, sizeof(bool));
	// Запускаем задачу управления светодиодом
  	xTaskCreatePinnedToCore(IMU_IRQ_process, "imu", 4096, NULL, 10, NULL, 0);
    
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
  	gpio_isr_handler_add(GINT1_PINNUM, IMU_IRQ_handler, NULL);
  	// Устанавливаем тип события для генерации прерывания - по низкому уровню
  	// Важно генерить прерывания именно по отрицательному фронту, а не по
  	// низкому уровню, ибо иначе будет interrupt wdt timeout error
  	gpio_set_intr_type(GINT1_PINNUM, GPIO_INTR_NEGEDGE);
  	// Разрешаем использование прерываний
  	gpio_intr_enable(GINT1_PINNUM);
  	
  	ICM42688_Init(&hicm, &icm_cfg);
  	
	//calibrateGyro();
	ICM42688_calibrateGyro(&hicm);
	
	spectreGyro();
	
	ICM42688_INT_Channel_Config_t int1_cfg = {
		.fifo_ths_en = true,
		.drive_circuit = PUSH_PULL,
	};
	//ICM42688_setINT1Config(&hicm, &int1_cfg);
	ICM42688_setGyroODR(&hicm, GYRO_ODR_200HZ);
	ICM42688_setAccelODR(&hicm, ACCEL_ODR_200HZ);
  	
  	anm = false;
  	
  	while (1)
  	{
//		if (anm)
//		{
//			ICM42688_calculateAccel(&hicm, raw+3);
//			ICM42688_calculateGyro(&hicm, raw);
//			printf("%f, %f, %f\n", hicm.gyro.x, hicm.gyro.y, hicm.gyro.z);
//			anm = false;
//		}
	}
}