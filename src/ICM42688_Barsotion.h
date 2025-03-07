#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "ICM42688_RegMap.h"
#include "ICM42688_Interface.h"


#define FIFO_HEADER_MSG					7
#define FIFO_HEADER_ACCEL				6
#define FIFO_HEADER_GYRO				5
#define FIFO_HEADER_20					4
#define FIFO_HEADER_TIMESTAMP_FSYNC		2
#define FIFO_HEADER_ODR_ACCEL			1
#define FIFO_HEADER_ODR_GYRO			0

#define GYRO_ZRO_VARvsTEMP				0.005


#ifdef __cplusplus
extern "C" {
#endif

typedef enum __ICM42688_Interfaces
{
	Hardware_SPI,
	Hardware_I2C,
} ICM42688_InterfaceProtocol_t;


//==============================================================================
//--<Config>--------------------------------------------------------------------
typedef struct __ICM42688_InterruptConfig
{
	ICM42688_UI_DRDY_INT_CLEAR_t drdy_int_clear;
	ICM42688_FIFO_THS_INT_CLEAR_t fifo_ths_int_clear;
	ICM42688_FIFO_FULL_INT_CLEAR_t fifo_full_int_clear;
	ICM42688_INT_TPULSE_DURATION_t tpulse_duration;
	ICM42688_INT_TDEASSERT_DISABLE_t tdeassert_dis;
} ICM42688_INT_Config_t;


typedef struct __ICM42688_InterruptChannelConfig
{
	int pin;
	ICM42688_INT_MODE_t mode;
	ICM42688_INT_DRIVE_CIRCUIT_t drive_circuit;
	ICM42688_INT_POLARITY_t polarity;
	bool ui_fsync_en;
	bool pll_rdy_en;
	bool reset_done_en;
	bool ui_drdy_en;
	bool fifo_ths_en;
	bool fifo_full_en;
	bool ui_agc_rdy_en;
	bool i3c_err_en;
	bool smd_en;
	bool wom_z_en;
	bool wom_y_en;
	bool wom_x_en;
} ICM42688_INT_Channel_Config_t;


typedef struct __ICM42688_Config
{
	ICM42688_InterfaceProtocol_t protocol;
	ICM42688_Interface_t interface_descriptor;
	struct __SPI {
		int miso_pin;
		int mosi_pin;
		int sck_pin;
		int cs_pin;
		spi_host_device_t host;
		int sck_freq;
	} spi;
	struct __I2C {
		int scl_pin;
		int sda_pin;
		uint8_t address;
	} i2c;
	struct {
		ICM42688_ACCEL_ENABLE_t enable;
		ICM42688_ACCEL_MODE_t mode;
		ICM42688_ACCEL_FS_SEL_t scale;
		ICM42688_ACCEL_ODR_t odr;
		ICM42688_ACCEL_UI_FILT_BW_t ui_filt_bw;
		ICM42688_ACCEL_UI_FILT_ORD_t ui_filt_ord;
		ICM42688_ACCEL_DEC2_M2_ORD_t dec2_m2_ord;
	} accel;
	struct {
		ICM42688_GYRO_ENABLE_t enable;
		ICM42688_GYRO_MODE_t mode;
		ICM42688_GYRO_FS_SEL_t scale;
		ICM42688_GYRO_ODR_t odr;
		ICM42688_GYRO_UI_FILT_BW_t ui_filt_bw;
		ICM42688_GYRO_UI_FILT_ORD_t ui_filt_ord;
		ICM42688_GYRO_DEC2_M2_ORD_t dec2_m2_ord;
	} gyro;
	struct __Interrupt
	{
		ICM42688_INT_Config_t cfg;
		ICM42688_INT_Channel_Config_t int1;
		ICM42688_INT_Channel_Config_t int2;
	} interrupt;
	struct {
		ICM42688_FIFO_MODE_t mode;
		uint16_t watermark;
	} fifo;
} ICM42688_Config_t;


//==============================================================================
//--<Parameters>----------------------------------------------------------------

typedef struct __ICM42688_XYZ
{
	float x;
	float y;
	float z;
} ICM42688_XYZ_t;

typedef struct __Filter
{
    float err_measure;
	float err_estimate;
	float q;
	float last_estimate;
} ICM42688_Filter_t;

typedef struct 
{
	ICM42688_XYZ_t gyro_bias;
    ICM42688_XYZ_t gyro_eps;
    ICM42688_XYZ_t accel_eps;
    float calib_temperature;
    ICM42688_XYZ_t gyro_drift_sign;
} ICM42688_Par_t;


//==============================================================================
//--<Descriptor>----------------------------------------------------------------

typedef struct __ICM42688_Descriptor
{
    uint8_t address;
    //interface
    ICM42688_Interface_t *interface;
    //coefficients
    float _gyro_coef;
    float _accel_coef;
    size_t _gyro_data_bit;
    size_t _accel_data_bit;
    ICM42688_ACCEL_ODR_t accel_odr;
    ICM42688_GYRO_ODR_t gyro_odr;
    ICM42688_ACCEL_FS_SEL_t accel_scale;
    ICM42688_GYRO_FS_SEL_t gyro_scale;
    ICM42688_FIFO_MODE_t fifo_mode;
    //output data
    ICM42688_XYZ_t accel;
    ICM42688_XYZ_t gyro;
    float accel_total;
    float temperature;
    //bias
    ICM42688_Par_t _par;
    //filters
    ICM42688_Filter_t _gyro_x_filter;
    ICM42688_Filter_t _gyro_y_filter;
    ICM42688_Filter_t _gyro_z_filter;
    ICM42688_Filter_t _accel_x_filter;
    ICM42688_Filter_t _accel_y_filter;
    ICM42688_Filter_t _accel_z_filter;
    
    void (*writeRegister) (uint8_t addr, uint8_t data);
    void (*readRegister) (uint8_t reg, uint8_t *buf);
} ICM42688_t;


//==============================================================================
//--<Functions>-----------------------------------------------------------------

void ICM42688_Init(ICM42688_t *hicm, ICM42688_Config_t *cfg);
void ICM42688_regBankSelect(ICM42688_t *hicm, uint8_t bank);
void ICM42688_flushFIFO(ICM42688_t *hicm);
/* Readings */
void ICM42688_readWhoAmI(ICM42688_t *hicm, uint8_t *buf);
void ICM42688_readRegAG(ICM42688_t *hicm, int32_t *raw);
void ICM42688_readFIFO(ICM42688_t *hicm, int32_t *raw);
/* Calculations */
void ICM42688_calculateGyro(ICM42688_t *hicm, int32_t *raw);
void ICM42688_calculateAccel(ICM42688_t *hicm, int32_t *raw);
/* Settings */
void ICM42688_setGyroODR(ICM42688_t *hicm, ICM42688_GYRO_ODR_t odr);
void ICM42688_setAccelODR(ICM42688_t *hicm, ICM42688_ACCEL_ODR_t odr);
void ICM42688_setGyroScale(ICM42688_t *hicm, ICM42688_GYRO_FS_SEL_t scale);
void ICM42688_setAccelScale(ICM42688_t *hicm, ICM42688_ACCEL_FS_SEL_t scale);
void ICM42688_setInterruptConfig(ICM42688_t *hicm, ICM42688_INT_Config_t *cfg);
void ICM42688_setINT1Config(ICM42688_t *hicm, ICM42688_INT_Channel_Config_t *ch);
void ICM42688_setINT2Config(ICM42688_t *hicm, ICM42688_INT_Channel_Config_t *ch);
void ICM42688_setFIFOMode(ICM42688_t *hicm, ICM42688_FIFO_MODE_t mode);
void ICM42688_setFIFOWatermark(ICM42688_t *hicm, uint16_t watermark);
void ICM42688_setGyroUIFiltBandwidth(ICM42688_t *hicm, ICM42688_GYRO_UI_FILT_BW_t bw);
void ICM42688_setGyroUIFiltOrder(ICM42688_t *hicm, ICM42688_GYRO_UI_FILT_ORD_t ord);

void ICM42688_gyroAntiAliasFilterEnable(ICM42688_t *hicm);
void ICM42688_gyroAntiAliasFilterDisable(ICM42688_t *hicm);
void ICM42688_gyroNotchFilterEnable(ICM42688_t *hicm);
void ICM42688_gyroNotchFilterDisable(ICM42688_t *hicm);
void ICM42688_gyroSetAAF_DELT(ICM42688_t *hicm, uint8_t delt);
void ICM42688_gyroSetAAF_DELTSQR(ICM42688_t *hicm, uint16_t deltsqr);
void ICM42688_gyroSetAAF_BITSHIFT(ICM42688_t *hicm, uint8_t bitshift);
/* Filtering */
void _ICM42688_setFilterParameters(ICM42688_Filter_t *channel, float mea_e, float est_e, float q);
float _ICM42688_Filtered(ICM42688_Filter_t *channel, float value);
void ICM42688_filterInit(ICM42688_t *hicm, float cycle_time);
void ICM42688_filterGyro(ICM42688_t *hicm);
void ICM42688_filterAccel(ICM42688_t *hicm);
/* Calibration */
void ICM42688_calibrateGyro(ICM42688_t *hicm);
/* IRQ flag status check */
bool ICM42688_UI_FSYNC_IRQ_Check(ICM42688_t *hicm);
bool ICM42688_PLL_RDY_IRQ_Check(ICM42688_t *hicm);
bool ICM42688_RESET_DONE_IRQ_Check(ICM42688_t *hicm);
bool ICM42688_DATA_RDY_IRQ_Check(ICM42688_t *hicm);
bool ICM42688_FIFO_THS_IRQ_Check(ICM42688_t *hicm);
bool ICM42688_FIFO_FULL_IRQ_Check(ICM42688_t *hicm);
bool ICM42688_AGC_RDY_IRQ_IRQ_Check(ICM42688_t *hicm);

#ifdef __cplusplus
}
#endif
