#include "ICM42688_Barsotion.h"
#include "ICM42688_Interface.h"
#include "ICM42688_RegMap.h"
#include <stdint.h>


void ICM42688_Init(ICM42688_t *hicm, ICM42688_Config_t *cfg)
{
	if (cfg->protocol == Hardware_SPI)
	{
		/* SPI init */
		hicm->writeRegister = ICM42688_SPI_writeRegister;
		hicm->readRegister = ICM42688_SPI_readRegister;
		ICM42688_SPI_InterfaceInit(cfg->spi.host, cfg->spi.miso_pin,
			 cfg->spi.mosi_pin, cfg->spi.sck_pin, cfg->spi.cs_pin, cfg->spi.sck_freq);
	}
	else if (cfg->protocol == Hardware_I2C)
	{
		hicm->writeRegister = ICM42688_I2C_writeRegister;
		hicm->readRegister = ICM42688_I2C_readRegister;
	}
	
	//Сделать по умолчанию оси выключенными
	/* PWR_MGMT0 */
	uint8_t pwr_mgmt = IDLE_MODE_DISABLE;
	if (cfg->accel.enable != 0) pwr_mgmt |= cfg->accel.mode;
	if (cfg->gyro.enable != 0) pwr_mgmt |= cfg->gyro.mode;
	hicm->writeRegister(ICM_0_PWR_MGMT0, pwr_mgmt);
	
	if (cfg->fifo.mode != FIFO_BYPASS_MODE)
	{
		hicm->gyro_data_bit = 19;
		hicm->accel_data_bit = 18;
	}
	else
	{
		hicm->gyro_data_bit = 16;
		hicm->accel_data_bit = 16;
	}
	/* Scale & ODR */
	ICM42688_setGyroScale(hicm, cfg->gyro.scale);
	ICM42688_setGyroODR(hicm, cfg->gyro.odr);
	ICM42688_setAccelScale(hicm, cfg->accel.scale);
	ICM42688_setAccelODR(hicm, cfg->accel.odr);
	/* Interrupt */
	ICM42688_setInterruptConfig(hicm, &(cfg->interrupt.cfg));
	ICM42688_setINT1Config(hicm, &(cfg->interrupt.int1));
	ICM42688_setINT2Config(hicm, &(cfg->interrupt.int2));
	/* FIFO */
	ICM42688_flushFIFO(hicm);
	ICM42688_setFIFOMode(hicm, cfg->fifo.mode);
	ICM42688_setFIFOWatermark(hicm, cfg->fifo.watermark);
}

/**
 * @brief
 */
void ICM42688_regBankSelect(ICM42688_t *hicm, uint8_t bank)
{
	if (bank > 4) bank = 4;
	hicm->writeRegister(ICM_0_REG_BANK_SEL, bank);	
}


void ICM42688_readWhoAmI(ICM42688_t *hicm, uint8_t *buf)
{
	hicm->readRegister(ICM_0_WHO_AM_I, buf);
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


void ICM42688_readFIFO(ICM42688_t *hicm, int32_t *raw)
{
	uint8_t buf[20];
	ICM42688_SPI_readFIFO(hicm->interface, buf, 20);
	int16_t pre;
	pre = (int16_t)buf[1]<<8 | (int16_t)buf[2];
	raw[3] = (int32_t)pre << 4;
	pre = (int16_t)buf[3]<<8 | (int16_t)buf[4];
	raw[4] = (int32_t)pre << 4;
	pre = (int16_t)buf[5]<<8 | (int16_t)buf[6];
	raw[5] = (int32_t)pre << 4;
	pre = (int16_t)buf[7]<<8 | (int16_t)buf[8];
	raw[0] = (int32_t)pre << 4;
	pre = (int16_t)buf[9]<<8 | (int16_t)buf[10];
	raw[1] = (int32_t)pre << 4;
	pre = (int16_t)buf[11]<<8 | (int16_t)buf[12];
	raw[2] = (int32_t)pre << 4;
	raw[3] |= (buf[17] & 0xF0) >> 4;
	raw[4] |= (buf[18] & 0xF0) >> 4;
	raw[5] |= (buf[19] & 0xF0) >> 4;
	raw[0] |= (buf[17] & 0x0F);
	raw[1] |= (buf[18] & 0x0F);
	raw[2] |= (buf[19] & 0x0F);
	raw[3] >>= 1;
	raw[4] >>= 1;
	raw[5] >>= 1;
	raw[0] >>= 1;
	raw[1] >>= 1;
	raw[2] >>= 1;
}


/**
 * @brief Calculating gyroscope's data from raw
 */
void ICM42688_calculateGyro(ICM42688_t *hicm, int32_t *raw)
{
	hicm->gyro.x = raw[0] * hicm->gyro_coef + hicm->gyro_bias.x;
	hicm->gyro.y = raw[1] * hicm->gyro_coef + hicm->gyro_bias.y;
	hicm->gyro.z = raw[2] * hicm->gyro_coef + hicm->gyro_bias.z;
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
	hicm->accel.x = raw[0] * hicm->accel_coef;
	hicm->accel.y = raw[1] * hicm->accel_coef;
	hicm->accel.z = raw[2] * hicm->accel_coef;
}


bool ICM42688_UI_FSYNC_IRQ_Check(ICM42688_t *hicm)
{
	uint8_t dummy;
	hicm->readRegister(ICM_0_INT_STATUS, &dummy);
	return (dummy & (1<<ICM_INT_STATUS_UI_FSYNC_INT)) != 0;
}
bool ICM42688_PLL_RDY_IRQ_Check(ICM42688_t *hicm)
{
	uint8_t dummy;
	hicm->readRegister(ICM_0_INT_STATUS, &dummy);
	return (dummy & (1<<ICM_INT_STATUS_PLL_RDY_INT)) != 0;
}
bool ICM42688_RESET_DONE_IRQ_Check(ICM42688_t *hicm)
{
	uint8_t dummy;
	hicm->readRegister(ICM_0_INT_STATUS, &dummy);
	return (dummy & (1<<ICM_INT_STATUS_RESET_DONE_INT)) != 0;
}
bool ICM42688_DATA_RDY_IRQ_Check(ICM42688_t *hicm)
{
	uint8_t dummy;
	hicm->readRegister(ICM_0_INT_STATUS, &dummy);
	return (dummy & (1<<ICM_INT_STATUS_DATA_RDY_INT)) != 0;
}
bool ICM42688_FIFO_THS_IRQ_Check(ICM42688_t *hicm)
{
	uint8_t dummy;
	hicm->readRegister(ICM_0_INT_STATUS, &dummy);
	return (dummy & (1<<ICM_INT_STATUS_FIFO_THS_INT)) != 0;
}
bool ICM42688_FIFO_FULL_IRQ_Check(ICM42688_t *hicm)
{
	uint8_t dummy;
	hicm->readRegister(ICM_0_INT_STATUS, &dummy);
	return (dummy & (1<<ICM_INT_STATUS_FIFO_FULL_INT)) != 0;
}
bool ICM42688_AGC_RDY_IRQ_IRQ_Check(ICM42688_t *hicm)
{
	uint8_t dummy;
	hicm->readRegister(ICM_0_INT_STATUS, &dummy);
	return (dummy & (1<<ICM_INT_STATUS_AGC_RDY_INT)) != 0;
}


void ICM42688_flushFIFO(ICM42688_t *hicm)
{
	uint8_t sig_path_res;
	hicm->readRegister(ICM_0_SIGNAL_PATH_RESET, &sig_path_res);
	sig_path_res |= 1<<1;
	hicm->writeRegister(ICM_0_SIGNAL_PATH_RESET, sig_path_res);
}


void ICM42688_setGyroODR(ICM42688_t *hicm, ICM42688_GYRO_ODR_t odr)
{
	hicm->gyro_odr = odr;
	uint8_t gyro_cfg0;
	hicm->readRegister(ICM_0_GYRO_CONFIG0, &gyro_cfg0);
	gyro_cfg0 = (gyro_cfg0 & 0xF0) | odr;
	hicm->writeRegister(ICM_0_GYRO_CONFIG0, gyro_cfg0);
}


void ICM42688_setAccelODR(ICM42688_t *hicm, ICM42688_ACCEL_ODR_t odr)
{
	hicm->accel_odr = odr;
	uint8_t acc_cfg0;
	hicm->readRegister(ICM_0_ACCEL_CONFIG0, &acc_cfg0);
	acc_cfg0 = (acc_cfg0 & 0xF0) | odr;
	hicm->writeRegister(ICM_0_ACCEL_CONFIG0, acc_cfg0);
}


void ICM42688_setGyroScale(ICM42688_t *hicm, ICM42688_GYRO_FS_SEL_t scale)
{
	hicm->gyro_scale = scale;
	uint8_t gyro_cfg0;
	hicm->readRegister(ICM_0_GYRO_CONFIG0, &gyro_cfg0);
	gyro_cfg0 = (gyro_cfg0 & 0x1F) | scale;
	hicm->writeRegister(ICM_0_GYRO_CONFIG0, gyro_cfg0);
	switch (scale)
	{
		case GYRO_FS_SEL_2000DPS: hicm->gyro_coef = 2000; break;
		case GYRO_FS_SEL_1000DPS: hicm->gyro_coef = 1000; break;
		case GYRO_FS_SEL_500DPS: hicm->gyro_coef = 500; break;
		case GYRO_FS_SEL_250DPS: hicm->gyro_coef = 250; break;
		case GYRO_FS_SEL_125DPS: hicm->gyro_coef = 125; break;
		case GYRO_FS_SEL_62p5DPS: hicm->gyro_coef = 62.5; break;
		case GYRO_FS_SEL_31p25DPS: hicm->gyro_coef = 31.25; break;
		case GYRO_FS_SEL_15p625DPS: hicm->gyro_coef = 15.625; break;
		default: hicm->gyro_coef = 2000; break;
	}
	hicm->gyro_coef *= powf(2.0, -(float)hicm->gyro_data_bit);
}


void ICM42688_setAccelScale(ICM42688_t *hicm, ICM42688_ACCEL_FS_SEL_t scale)
{
	hicm->accel_scale = scale;
	uint8_t acc_cfg0;
	hicm->readRegister(ICM_0_ACCEL_CONFIG0, &acc_cfg0);
	acc_cfg0 = (acc_cfg0 & 0x1F) | scale;
	hicm->writeRegister(ICM_0_ACCEL_CONFIG0, acc_cfg0);
	switch (scale)
	{
		case ACCEL_FS_SEL_16G: hicm->accel_coef = 16.; break;
		case ACCEL_FS_SEL_8G: hicm->accel_coef = 8.; break;
		case ACCEL_FS_SEL_4G: hicm->accel_coef = 4.; break;
		case ACCEL_FS_SEL_2G: hicm->accel_coef = 2.; break;
		default: hicm->accel_coef = 16.;
	}
	hicm->accel_coef *= powf(2.0, -(float)hicm->accel_data_bit);
}


void ICM42688_setInterruptConfig(ICM42688_t *hicm, ICM42688_INT_Config_t *cfg)
{
	/* INT_CONFIG0 */
	uint8_t int_cfg0 = 0;
	int_cfg0 |= cfg->drdy_int_clear;
	int_cfg0 |= cfg->fifo_ths_int_clear;
	int_cfg0 |= cfg->fifo_full_int_clear;
	hicm->writeRegister(ICM_0_INT_CONFIG0, int_cfg0);
	/* INT_CONFIG1 */
	uint8_t int_cfg1 = 0;
	int_cfg1 |= cfg->tpulse_duration;
	int_cfg1 |= cfg->tdeassert_dis;
	int_cfg1 |= 0<<ICM_INT_CONFIG1_INT_ASYNC_RESET;
	hicm->writeRegister(ICM_0_INT_CONFIG1, int_cfg1);
}


void ICM42688_setINT1Config(ICM42688_t *hicm, ICM42688_INT_Channel_Config_t *ch)
{
	/* INT_CONFIG */
	uint8_t int_cfg = 0;
	hicm->readRegister(ICM_0_INT_CONFIG, &int_cfg);
	int_cfg <<= ICM_INT_CONFIG_INT2;
	int_cfg |= ch->mode;
	int_cfg |= ch->drive_circuit;
	int_cfg |= ch->polarity;
	hicm->writeRegister(ICM_0_INT_CONFIG, int_cfg);
	/* INT_SOURCE0 */
	uint8_t int_src0 = 0;
	if (ch->ui_fsync_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_UI_FSYNC_INT1_EN;
	if (ch->pll_rdy_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_PLL_RDY_INT1_EN;
	if (ch->reset_done_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_RESET_DONE_INT1_EN;
	if (ch->ui_drdy_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_UI_DRDY_INT1_EN;
	if (ch->fifo_ths_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_FIFO_THS_INT1_EN;
	if (ch->fifo_full_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_FIFO_FULL_INT1_EN;
	if (ch->ui_agc_rdy_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_UI_AGC_RDY_INT1_EN;
	hicm->writeRegister(ICM_0_INT_SOURCE0, int_src0);
	/* INT_SOURCE1 */
	uint8_t int_src1 = 0;
	if (ch->i3c_err_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_I3C_PROTOCOL_ERROR_INT1_EN;
	if (ch->smd_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_SMD_INT1_EN;
	if (ch->wom_x_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_WOM_X_INT1_EN;
	if (ch->wom_y_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_WOM_Y_INT1_EN;
	if (ch->wom_z_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_WOM_Z_INT1_EN;
	hicm->writeRegister(ICM_0_INT_SOURCE1, int_src1);
}


void ICM42688_setINT2Config(ICM42688_t *hicm, ICM42688_INT_Channel_Config_t *ch)
{
		/* INT_CONFIG */
	uint8_t int_cfg;
	hicm->readRegister(ICM_0_INT_CONFIG, &int_cfg);
	int_cfg |= ch->mode;
	int_cfg |= ch->drive_circuit;
	int_cfg |= ch->polarity;
	hicm->writeRegister(ICM_0_INT_CONFIG, int_cfg);
	/* INT_SOURCE3 */
	uint8_t int_src3 = 0;
	if (ch->ui_fsync_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_UI_FSYNC_INT1_EN;
	if (ch->pll_rdy_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_PLL_RDY_INT1_EN;
	if (ch->reset_done_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_RESET_DONE_INT1_EN;
	if (ch->ui_drdy_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_UI_DRDY_INT1_EN;
	if (ch->fifo_ths_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_FIFO_THS_INT1_EN;
	if (ch->fifo_full_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_FIFO_FULL_INT1_EN;
	if (ch->ui_agc_rdy_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_UI_AGC_RDY_INT1_EN;
	hicm->writeRegister(ICM_0_INT_SOURCE3, int_src3);
	/* INT_SOURCE1 */
	uint8_t int_src4 = 0;
	if (ch->i3c_err_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_I3C_PROTOCOL_ERROR_INT1_EN;
	if (ch->smd_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_SMD_INT1_EN;
	if (ch->wom_x_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_WOM_X_INT1_EN;
	if (ch->wom_y_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_WOM_Y_INT1_EN;
	if (ch->wom_z_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_WOM_Z_INT1_EN;
	hicm->writeRegister(ICM_0_INT_SOURCE4, int_src4);
}


void ICM42688_setFIFOMode(ICM42688_t *hicm, ICM42688_FIFO_MODE_t mode)
{
	/* FIFO_CONFIG */
	uint8_t fifo_cfg = 0;
	fifo_cfg |= mode;
	hicm->writeRegister(ICM_0_FIFO_CONFIG, fifo_cfg);
	/* FIFO_CONFIG1 */
	uint8_t fifo_cfg1 = 0;
	fifo_cfg1 |= 0<<ICM_FIFO_CONFIG1_FIFO_RESUME_PARTIAL_RD;
	fifo_cfg1 |= 0<<ICM_FIFO_CONFIG1_FIFO_WM_GT_TH;
	fifo_cfg1 |= 1<<ICM_FIFO_CONFIG1_FIFO_HIRES_EN;
	fifo_cfg1 |= 0<<ICM_FIFO_CONFIG1_FIFO_TMST_FSYNC_EN;
	fifo_cfg1 |= 1<<ICM_FIFO_CONFIG1_FIFO_TEMP_EN;
	fifo_cfg1 |= 1<<ICM_FIFO_CONFIG1_FIFO_GYRO_EN;
	fifo_cfg1 |= 1<<ICM_FIFO_CONFIG1_FIFO_ACCEL_EN;
	hicm->writeRegister(ICM_0_FIFO_CONFIG1, fifo_cfg1);
}


void ICM42688_setFIFOWatermark(ICM42688_t *hicm, uint16_t watermark)
{
	/* FIFO_CONFIG2-3 */
	hicm->writeRegister(ICM_0_FIFO_CONFIG2, watermark & 0xFF);
	hicm->writeRegister(ICM_0_FIFO_CONFIG3, (watermark>>8)&0x0F);
}