#include "ICM42688_Barsotion.h"


void ICM42688_Init(ICM42688_t *hicm, ICM42688_Config_t *cfg)
{
	/* SPI init */
	hicm->writeRegister = ICM42688_SPI_writeRegister;
	hicm->readRegister = ICM42688_SPI_readRegister;
	ICM42688_SPI_InterfaceInit(cfg->spi.host, cfg->spi.miso_pin,
		 cfg->spi.mosi_pin, cfg->spi.sck_pin, cfg->spi.cs_pin, cfg->spi.sck_freq);
	
	//Сделать по умолчанию оси выключенными
	/* PWR_MGMT0 */
	uint8_t pwr_mgmt = IDLE_MODE_DISABLE;
	if (cfg->accel.enable != 0) pwr_mgmt |= cfg->accel.mode;
	if (cfg->gyro.enable != 0) pwr_mgmt |= cfg->gyro.mode;
	hicm->writeRegister(ICM_0_PWR_MGMT0, pwr_mgmt);
	
	/* GYRO_CONFIG0 */
	uint8_t gyro_config0 = cfg->gyro.fs_sel | cfg->gyro.odr;
	hicm->writeRegister(ICM_0_GYRO_CONFIG0, gyro_config0);
	switch (cfg->gyro.fs_sel)
	{
		case GYRO_FS_SEL_2000DPS: hicm->gyro_scale = 2000; break;
		case GYRO_FS_SEL_1000DPS: hicm->gyro_scale = 1000; break;
		case GYRO_FS_SEL_500DPS: hicm->gyro_scale = 500; break;
		case GYRO_FS_SEL_250DPS: hicm->gyro_scale = 250; break;
		case GYRO_FS_SEL_125DPS: hicm->gyro_scale = 125; break;
		case GYRO_FS_SEL_62p5DPS: hicm->gyro_scale = 62.5; break;
		case GYRO_FS_SEL_31p25DPS: hicm->gyro_scale = 31.25; break;
		case GYRO_FS_SEL_15p625DPS: hicm->gyro_scale = 15.625; break;
		default: hicm->gyro_scale = 2000; break;
	}
	hicm->gyro_scale /= 32768.;
	/* ACCEL_CONFIG0 */
	uint8_t accel_config0 = cfg->accel.fs_sel | cfg->accel.odr;
	hicm->writeRegister(ICM_0_ACCEL_CONFIG0, accel_config0);
	switch (cfg->accel.fs_sel)
	{
		case ACCEL_FS_SEL_16G: hicm->accel_scale = 16.; break;
		case ACCEL_FS_SEL_8G: hicm->accel_scale = 8.; break;
		case ACCEL_FS_SEL_4G: hicm->accel_scale = 4.; break;
		case ACCEL_FS_SEL_2G: hicm->accel_scale = 2.; break;
		default: hicm->accel_scale = 16.;
	}
	hicm->accel_scale /= 32768.;
}


/**
 * @brief Asyncronous reading gyro & accel raw data from registers
 */
void ICM42688_readRegAG(ICM42688_t *hicm, int32_t *raw)
{
	uint8_t dummy;
	int16_t pre;
	hicm->readRegister(ICM_0_GYRO_DATA_X1, &dummy);
	pre = (int16_t)dummy << 8;
	hicm->readRegister(ICM_0_GYRO_DATA_X0, &dummy);
	pre |= (int16_t)dummy;
	raw[0] = (int32_t)pre;
	hicm->readRegister(ICM_0_GYRO_DATA_Y1, &dummy);
	pre = (int16_t)dummy << 8;
	hicm->readRegister(ICM_0_GYRO_DATA_Y0, &dummy);
	pre |= (int16_t)dummy;
	raw[1] = (int32_t)pre;
	hicm->readRegister(ICM_0_GYRO_DATA_Z1, &dummy);
	pre = (int16_t)dummy << 8;
	hicm->readRegister(ICM_0_GYRO_DATA_Z0, &dummy);
	pre |= (int16_t)dummy;
	raw[2] = (int32_t)pre;
	hicm->readRegister(ICM_0_ACCEL_DATA_X1, &dummy);
	pre = (int16_t)dummy << 8;
	hicm->readRegister(ICM_0_ACCEL_DATA_X0, &dummy);
	pre |= (int16_t)dummy;
	raw[3] = (int32_t)pre;
	hicm->readRegister(ICM_0_ACCEL_DATA_Y1, &dummy);
	pre = (int16_t)dummy << 8;
	hicm->readRegister(ICM_0_ACCEL_DATA_Y0, &dummy);
	pre |= (int16_t)dummy;
	raw[4] = (int32_t)pre;
	hicm->readRegister(ICM_0_ACCEL_DATA_Z1, &dummy);
	pre = (int16_t)dummy << 8;
	hicm->readRegister(ICM_0_ACCEL_DATA_Z0, &dummy);
	pre |= (int16_t)dummy;
	raw[5] = (int32_t)pre;
}


/**
 * @brief Calculating gyroscope's data from raw
 */
void ICM42688_calculateGyro(ICM42688_t *hicm, int32_t *raw)
{
	hicm->gyro.x = raw[0] * hicm->gyro_scale - hicm->gyro_bias.x;
	hicm->gyro.y = raw[1] * hicm->gyro_scale - hicm->gyro_bias.y;
	hicm->gyro.z = raw[2] * hicm->gyro_scale - hicm->gyro_bias.z;
}

/**
 * @brief Filtering gyroscope's RMS noise
 */
void ICM42688_filterGyro(ICM42688_t *hicm)
{
	hicm->gyro.x = ICM42688_Filtered(&(hicm->gyro_x_filter), hicm->gyro.x);
	hicm->gyro.y = ICM42688_Filtered(&(hicm->gyro_y_filter), hicm->gyro.y);
	hicm->gyro.z = ICM42688_Filtered(&(hicm->gyro_z_filter), hicm->gyro.z);
}

/**
 * @brief Calculating accelerometer's data from raw
 */
void ICM42688_calculateAccel(ICM42688_t *hicm, int32_t *raw)
{
	hicm->accel.x = raw[0] * hicm->accel_scale;
	hicm->accel.y = raw[1] * hicm->accel_scale;
	hicm->accel.z = raw[2] * hicm->accel_scale;
}