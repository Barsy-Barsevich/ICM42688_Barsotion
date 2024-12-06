#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "ICM42688_RegMap.h"
#include "ICM42688_Interface.h"


typedef enum __ICM42688_Interfaces
{
	Hardware_SPI,
	Hardware_I2C,
} ICM42688_Interface_t;

typedef struct __ICM42688_CONFIG
{
	ICM42688_Interface_t interface;
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
		ICM42688_ACCEL_FS_SEL_t fs_sel;
		ICM42688_ACCEL_ODR_t odr;
		ICM42688_ACCEL_UI_FILT_BW_t ui_filt_bw;
		ICM42688_ACCEL_UI_FILT_ORD_t ui_filt_ord;
		ICM42688_ACCEL_DEC2_M2_ORD_t dec2_m2_ord;
	} accel;
	struct {
		ICM42688_GYRO_ENABLE_t enable;
		ICM42688_GYRO_MODE_t mode;
		ICM42688_GYRO_FS_SEL_t fs_sel;
		ICM42688_GYRO_ODR_t odr;
		ICM42688_GYRO_UI_FILT_BW_t ui_filt_bw;
		ICM42688_GYRO_UI_FILT_ORD_t ui_filt_ord;
		ICM42688_GYRO_DEC2_M2_ORD_t dec2_m2_ord;
	} gyro;
} ICM42688_Config_t;


typedef struct
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


typedef struct __ICM42688_Descriptor
{
    uint8_t address;
    //interface
    void* interface;
    //coefficients
    float accel_coef;
    float gyro_coef;
    float gyro_scale;
    float accel_scale;
    //output data
    ICM42688_XYZ_t accel;
    ICM42688_XYZ_t gyro;
    //bias
    ICM42688_XYZ_t gyro_bias;
    ICM42688_XYZ_t gyro_eps;
    
    ICM42688_Filter_t gyro_x_filter;
    ICM42688_Filter_t gyro_y_filter;
    ICM42688_Filter_t gyro_z_filter;
    
    void (*writeRegister) (uint8_t addr, uint8_t data);
    void (*readRegister) (uint8_t reg, uint8_t *buf);
} ICM42688_t;







void ICM42688_Init(ICM42688_t *hicm, ICM42688_Config_t *cfg);
void ICM42688_readRegAG(ICM42688_t *hicm, int32_t *raw);

void ICM42688_calculateGyro(ICM42688_t *hicm, int32_t *raw);
void ICM42688_filterGyro(ICM42688_t *hicm);
void ICM42688_calculateAccel(ICM42688_t *hicm, int32_t *raw);



void ICM42688_setFilterParameters(ICM42688_Filter_t *channel, float mea_e, float est_e, float q);
float ICM42688_Filtered(ICM42688_Filter_t *channel, float value);





