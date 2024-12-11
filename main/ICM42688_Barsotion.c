#include "ICM42688_Barsotion.h"
#include "ICM42688_RegMap.h"


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
	
	
	/* INT_CONFIG */
	uint8_t int_cfg = 0;
	int_cfg |= cfg->interrupt.int1.mode;
	int_cfg |= cfg->interrupt.int1.drive_circuit;
	int_cfg |= cfg->interrupt.int1.polarity;
	int_cfg |= cfg->interrupt.int2.mode;
	int_cfg |= cfg->interrupt.int2.drive_circuit;
	int_cfg |= cfg->interrupt.int2.polarity;
	hicm->writeRegister(ICM_0_INT_CONFIG, int_cfg);
	/* INT_CONFIG0 */
	uint8_t int_cfg0 = 0;
	int_cfg0 |= cfg->interrupt.drdy_int_clear;
	int_cfg0 |= cfg->interrupt.fifo_ths_int_clear;
	int_cfg0 |= cfg->interrupt.fifo_full_int_clear;
	hicm->writeRegister(ICM_0_INT_CONFIG0, int_cfg0);
	/* INT_CONFIG1 */
	uint8_t int_cfg1 = 0;
	int_cfg1 |= cfg->interrupt.tpulse_duration;
	int_cfg1 |= cfg->interrupt.tdeassert_dis;
	int_cfg1 |= 0<<ICM_INT_CONFIG1_INT_ASYNC_RESET;
	hicm->writeRegister(ICM_0_INT_CONFIG1, int_cfg1);
	/* INT_SOURCE0 */
	uint8_t int_src0 = 0;
	if (cfg->interrupt.int1.ui_fsync_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_UI_FSYNC_INT1_EN;
	if (cfg->interrupt.int1.pll_rdy_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_PLL_RDY_INT1_EN;
	if (cfg->interrupt.int1.reset_done_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_RESET_DONE_INT1_EN;
	if (cfg->interrupt.int1.ui_drdy_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_UI_DRDY_INT1_EN;
	if (cfg->interrupt.int1.fifo_ths_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_FIFO_THS_INT1_EN;
	if (cfg->interrupt.int1.fifo_full_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_FIFO_FULL_INT1_EN;
	if (cfg->interrupt.int1.ui_agc_rdy_en)
		int_src0 |= 1<<ICM_INT_SOURCE0_UI_AGC_RDY_INT1_EN;
	hicm->writeRegister(ICM_0_INT_SOURCE0, int_src0);
	/* INT_SOURCE1 */
	uint8_t int_src1 = 0;
	if (cfg->interrupt.int1.i3c_err_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_I3C_PROTOCOL_ERROR_INT1_EN;
	if (cfg->interrupt.int1.smd_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_SMD_INT1_EN;
	if (cfg->interrupt.int1.wom_x_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_WOM_X_INT1_EN;
	if (cfg->interrupt.int1.wom_y_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_WOM_Y_INT1_EN;
	if (cfg->interrupt.int1.wom_z_en)
		int_src1 |= 1<<ICM_INT_SOURCE1_WOM_Z_INT1_EN;
	hicm->writeRegister(ICM_0_INT_SOURCE1, int_src1);
	/* INT_SOURCE3 */
	uint8_t int_src3 = 0;
	if (cfg->interrupt.int2.ui_fsync_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_UI_FSYNC_INT1_EN;
	if (cfg->interrupt.int2.pll_rdy_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_PLL_RDY_INT1_EN;
	if (cfg->interrupt.int2.reset_done_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_RESET_DONE_INT1_EN;
	if (cfg->interrupt.int2.ui_drdy_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_UI_DRDY_INT1_EN;
	if (cfg->interrupt.int2.fifo_ths_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_FIFO_THS_INT1_EN;
	if (cfg->interrupt.int2.fifo_full_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_FIFO_FULL_INT1_EN;
	if (cfg->interrupt.int2.ui_agc_rdy_en)
		int_src3 |= 1<<ICM_INT_SOURCE3_UI_AGC_RDY_INT1_EN;
	hicm->writeRegister(ICM_0_INT_SOURCE3, int_src3);
	/* INT_SOURCE1 */
	uint8_t int_src4 = 0;
	if (cfg->interrupt.int2.i3c_err_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_I3C_PROTOCOL_ERROR_INT1_EN;
	if (cfg->interrupt.int2.smd_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_SMD_INT1_EN;
	if (cfg->interrupt.int2.wom_x_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_WOM_X_INT1_EN;
	if (cfg->interrupt.int2.wom_y_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_WOM_Y_INT1_EN;
	if (cfg->interrupt.int2.wom_z_en)
		int_src4 |= 1<<ICM_INT_SOURCE4_WOM_Z_INT1_EN;
	hicm->writeRegister(ICM_0_INT_SOURCE4, int_src4);
	
	
	/* SIGNAL_PATH_RESET */
	uint8_t sig_path_res = 1<<1;
	hicm->writeRegister(ICM_0_SIGNAL_PATH_RESET, sig_path_res);
	/* FIFO_CONFIG */
	uint8_t fifo_cfg = 0;
	fifo_cfg |= cfg->fifo.mode;
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
	printf("Podstava: %02X\n", fifo_cfg1);
	hicm->writeRegister(ICM_0_FIFO_CONFIG1, fifo_cfg1);
	/* FIFO_CONFIG2-3 */
	//hicm->writeRegister(ICM_0_FIFO_CONFIG2, cfg->fifo.watermark & 0xFF);
	//hicm->writeRegister(ICM_0_FIFO_CONFIG3, (cfg->fifo.watermark>>8)&0x0F);
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