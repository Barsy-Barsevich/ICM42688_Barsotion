# Methods descriptions

- [Enums](#Enums)
- [Structures](#Structures)
- [Methods](#Methods)
	- [Readings](#Readings)
	- [Calculations](#Calculations)
	- [Settings](#Gyroscope-&-accelerometer-settings)
	- [Interrupt settings](#Interrupt-settings)
	- [FIFO settings](#FIFO-settings)
	- [Filter settings](#Filter-settings)
	- [Filtering](#Filtering)
	- [Calibration](#Calibration)
	- [IRQ flag status check](#IRQ-flag-status-check)

## Enums
#### `ICM42688_Interface_t`
- `bool busy`
#### `ICM42688_InterfaceProtocol_t`
- `Hardware_SPI`
- `Hardware_I2C`
#### `ICM42688_SPI_MODE_t`
- `SPI_MODE_0_3`
- `SPI_MODE_1_2`
#### `ICM42688_SLEW_RATE_t`
- `SLEW_RATE_20NS_60NS`
- `SLEW_RATE_12NS_36NS`
- `SLEW_RATE_6NS_18NS`
- `SLEW_RATE_4NS_12NS`
- `SLEW_RATE_2NS_6NS`
- `SLEW_RATE_LESS_THAN_2NS`
#### `ICM42688_INT_MODE_t`
- `PULSED_MODE`
- `LATCHED_MODE`
#### `ICM42688_INT_DRIVE_CIRCUIT_t`
- `OPEN_DRAIN` - interrupt signals open drain circuit;
- `PUSH_PULL` - interrupt signals push pull circuit
#### `ICM42688_INT_POLARITY_t`
- `ACTIVE_LOW` - interrupt signals active low level;
- `ACTIVE_HIGH` - interrupt signals active high level
#### `ICM_TEMP_DIS_t`
- `TEMP_SENS_ENABLE` - enable temperature sensor;
- `TEMP_SENS_DISABLE` - disable temperature sensor
#### `ICM_IDLE_t`
- `IDLE_MODE_DISABLE`
- `IDLE_MODE_ENABLE`
#### `ICM42688_GYRO_MODE_t`
- `GYRO_OFF_MODE` - disable gyro;
- `GYRO_STANDBY_MODE`
- `GYRO_LN_MODE` - gyro low-noise mode
#### `ICM42688_ACCEL_MODE_t`
- `ACCEL_OFF_MODE` - disable accel;
- `ACCEL_STANDBY_MODE`
- `ACCEL_LP_MODE` - accel low-power mode;
- `ACCEL_LN_MODE` - accel low-noise mode
#### `ICM42688_GYRO_FS_SEL_t`
- `GYRO_FS_SEL_2000DPS`
- `GYRO_FS_SEL_1000DPS`
- `GYRO_FS_SEL_500DPS`
- `GYRO_FS_SEL_250DPS`
- `GYRO_FS_SEL_125DPS`
- `GYRO_FS_SEL_62p5DPS`
- `GYRO_FS_SEL_31p25DPS`
- `GYRO_FS_SEL_15p625DPS`
#### `ICM42688_GYRO_ODR_t`
- `GYRO_ODR_32KHZ`
- `GYRO_ODR_16KHZ`
- `GYRO_ODR_8KHZ`
- `GYRO_ODR_4KHZ`
- `GYRO_ODR_2KHZ`
- `GYRO_ODR_1KHZ`
- `GYRO_ODR_500HZ`
- `GYRO_ODR_200HZ`
- `GYRO_ODR_100HZ`
- `GYRO_ODR_50HZ`
- `GYRO_ODR_25HZ`
- `GYRO_ODR_12p5HZ`
#### `ICM42688_ACCEL_FS_SEL_t`
- `ACCEL_FS_SEL_16G`
- `ACCEL_FS_SEL_8G`
- `ACCEL_FS_SEL_4G`
- `ACCEL_FS_SEL_2G`
#### `ICM42688_ACCEL_ODR_t`
- `ACCEL_ODR_32KHZ`
- `ACCEL_ODR_16KHZ`
- `ACCEL_ODR_8KHZ`
- `ACCEL_ODR_4KHZ`
- `ACCEL_ODR_2KHZ`
- `ACCEL_ODR_1KHZ`
- `ACCEL_ODR_500HZ`
- `ACCEL_ODR_200HZ`
- `ACCEL_ODR_100HZ`
- `ACCEL_ODR_50HZ`
- `ACCEL_ODR_25HZ`
- `ACCEL_ODR_12p5HZ`
- `ACCEL_ODR_6p25HZ`
- `ACCEL_ODR_3p125HZ`
- `ACCEL_ODR_1p5625HZ`
#### `ICM42688_TEMP_FILT_BW_t`
- `DLPF_BW_4000HZ_LATENCY_0p125MS`
- `DLPF_BW_170HZ_LATENCY_1MS`
- `DLPF_BW_82HZ_LATENCY_2MS`
- `DLPF_BW_40HZ_LATENCY_4MS`
- `DLPF_BW_20HZ_LATENCY_8MS`
- `DLPF_BW_10HZ_LATENCY_16MS`
- `DLPF_BW_5HZ_LATENCY_32MS`
#### `ICM42688_GYRO_UI_FILT_ORD_t`
- `GYRO_UI_FILT_1ST_ORDER`
- `GYRO_UI_FILT_2ND_ORDER`
- `GYRO_UI_FILT_3RD_ORDER`
#### `ICM42688_GYRO_DEC2_M2_ORD_t`
- `GYRO_DEC2_M2_FILT_RESERVED`
- `GYRO_DEC2_M2_FILT_3RD_ORDER`
#### `ICM42688_ACCEL_UI_FILT_BW_t`
- `ACCEL_BW_ODR_DIV2` - accel bandwidth ODR/2 (LN-mode)
- `ACCEL_BW_MAX_400HZ_ODR_DIV2` - accel bandwidth max(400Hz, ODR/2) (LN-mode)
- `ACCEL_BW_MAX_400HZ_ODR_DIV4` - accel bandwidth max(400Hz, ODR/4) (LN-mode)
- `ACCEL_BW_MAX_400HZ_ODR_DIV5` - accel bandwidth max(400Hz, ODR/5) (LN-mode)
- `ACCEL_BW_MAX_400HZ_ODR_DIV8` - accel bandwidth max(400Hz, ODR/8) (LN-mode)
- `ACCEL_BW_MAX_400HZ_ODR_DIV10` - accel bandwidth max(400Hz, ODR/10) (LN-mode)
- `ACCEL_BW_MAX_400HZ_ODR_DIV16` - accel bandwidth max(400Hz, ODR/16) (LN-mode)
- `ACCEL_BW_MAX_400HZ_ODR_DIV20` - accel bandwidth max(400Hz, ODR/20) (LN-mode)
- `ACCEL_BW_MAX_400HZ_ODR_DIV40` - accel bandwidth max(400Hz, ODR/40) (LN-mode)
- `ACCEL_BW_MAX_200HZ_8xODR_DIV2` - accel bandwidth ODR/2 (LN-mode)
- `ACCEL_BW_1X_AVG_FILTER` - (LP-mode)
- `ACCEL_BW_16X_AVG_FILTER` - (LP-mode)
- `ACCEL_BW_DEFAULT` = `ACCEL_BW_MAX_400HZ_ODR_DIV4`
#### `ICM42688_FIFO_MODE_t`
- `FIFO_BYPASS_MODE`
- `FIFO_STREAM_MODE`
- `FIFO_STOP_ON_FULL_MODE`
#### `ICM42688_GYRO_UI_FILT_BW_t`
- `GYRO_BW_ODR_DIV2` - ODR/2
- `GYRO_BW_MAX_400HZ_ODR_DIV2` - the maximum of (400Hz) and ODR/2
- `GYRO_BW_MAX_400HZ_ODR_DIV4` - the maximum of (400Hz) and ODR/4
- `GYRO_BW_MAX_400HZ_ODR_DIV5` - the maximum of (400Hz) and ODR/5
- `GYRO_BW_MAX_400HZ_ODR_DIV8` - the maximum of (400Hz) and ODR/8
- `GYRO_BW_MAX_400HZ_ODR_DIV10` - the maximum of (400Hz) and ODR/10
- `GYRO_BW_MAX_400HZ_ODR_DIV16` - the maximum of (400Hz) and ODR/16
- `GYRO_BW_MAX_400HZ_ODR_DIV20` - the maximum of (400Hz) and ODR/20
- `GYRO_BW_MAX_400HZ_ODR_DIV40` - the maximum of (400Hz) and ODR/40
- `GYRO_BW_MAX_200HZ_8xODR` - the maximum of (200Hz) and ODR\*8
- `GYRO_BW_DEFAULT` = `GYRO_BW_MAX_400HZ_ODR_DIV4`
#### `ICM42688_ACCEL_UI_FILT_ORD_t`
- `ACCEL_UI_FILT_1ST_ORDER`
- `ACCEL_UI_FILT_2ND_ORDER`
- `ACCEL_UI_FILT_3RD_ORDER`
#### `ICM42688_ACCEL_DEC2_M2_ORD_t`
- `ACCEL_DEC2_M2_RESERVED`
- `ACCEL_DEC2_M2_3RD_ORDER`
#### `ICM42688_TMST_TO_REGS_EN_t`
- `TMST_TO_REGS_DISABLE`
- `TMST_TO_REGS_ENABLE`
#### `ICM42688_TMST_RES_t`
- `TMST_RES_1US`
- `TMST_RES_16US`
- `TMST_RES_RTC`
- `TMST_RES_DEFAULT` = `TMST_RES_1US`
#### `ICM42688_TMST_MODE_t`
- `TMST_DELTA_EN`
- `TMST_FSYNC_EN`
#### `ICM42688_TMST_EN_t`
- `TMST_ENABLE`
- `TMST_DISABLE`
#### `ICM42688_DMP_POWER_SAVE_t`
- `DMP_DISABLE`
- `DMP_ENABLE`
#### `ICM42688_DMP_FLAGS_t`
- `DMP_TAP_ENABLE`
- `DMP_PED_ENABLE`
- `DMP_TILT_ENABLE`
- `DMP_R2W_ENABLE`
#### `ICM42688_DMP_ODR_t`
- `DMP_ODR_25HZ`
- `DMP_ODR_50HZ`
#### `ICM42688_SMD_MODE_t`
- `SMD_DISABLED`
- `SMD_SHORT`
- `SMD_LONG`
#### `ICM42688_WOM_INT_MODE_t`
- `THE_OR_OF_ENABLED_ACCELS`
- `THE_AND_OF_ENABLED_ACCELS`
#### `ICM42688_WOM_MODE_t`
- `COMPARE_WITH_INITIAL_SAMPLE`
- `COMPARE_WITH_PREVIOUS_SAMPLE`
#### `ICM42688_INT_REQUEST_t`
- `UI_FSYNC_IRQ`
- `PLL_RDY_IRQ`
- `RESET_DONE_IRQ`
- `UI_DRDY_IRQ`
- `FIFO_THS_IRQ`
- `FIFO_FULL_IRQ`
- `UI_AGC_RDY_IRQ`
- `I3C_PROTOCOL_ERR_IRQ`
- `SMD_IRQ`
- `WOM_Z_IRQ`
- `WOM_Y_IRQ`
- `WOM_X_IRQ`
#### `ICM42688_ACCEL_ENABLE_t`
- `ENABLE_XA`
- `ENABLE_YA`
- `ENABLE_ZA`
#### `ICM42688_GYRO_ENABLE_t`
- `ENABLE_XG`
- `ENABLE_YG`
- `ENABLE_ZG`
#### `ICM42688_UI_DRDY_INT_CLEAR_t`
- `DRDY_INT_CLEAR_ON_STATUS_BIT_READ`
- `DRDY_INT_CLEAR_ON_SENSOR_REG_READ`
- `DRDY_INT_CLEAR_ON_STATUS_BIT_AND_SENSOR_REG_READ`
#### `ICM42688_FIFO_THS_INT_CLEAR_t`
- `FIFO_THS_INT_CLEAR_ON_STATUS_BIT_READ`
- `FIFO_THS_INT_CLEAR_ON_SENSOR_REG_READ`
- `FIFO_THS_INT_CLEAR_ON_STATUS_BIT_AND_SENSOR_REG_READ`
#### `ICM42688_FIFO_FULL_INT_CLEAR_t`
- `FIFO_FULL_INT_CLEAR_ON_STATUS_BIT_READ`
- `FIFO_FULL_INT_CLEAR_ON_SENSOR_REG_READ`
- `FIFO_FULL_INT_CLEAR_ON_STATUS_BIT_AND_SENSOR_REG_READ`
#### `ICM42688_INT_TPULSE_DURATION_t`
- `INT_TPULSE_100US`
- `INT_TPULSE_8US`
#### `ICM42688_INT_TDEASSERT_DISABLE_t`
- `INT_TDEASSERT_ENABLE`
- `INT_TDEASSERT_DISABLE`
#### `ICM42688_FIFO_MODE_t`
- `FIFO_BYPASS_MODE`
- `FIFO_STREAM_MODE`
- `FIFO_STOP_ON_FULL_MODE`
#### `ICM42688_GYRO_AAF_Enable_t`
- `GYRO_AAF_ENABLE`
- `GYRO_AAF_DISABLE`
#### `ICM42688_GYRO_NF_Enable_t`
- `GYRO_NF_ENABLE`
- `GYRO_NF_DISABLE`
#### `ICM42688_GYRO_UI_FILT_ORD_t`
- `GYRO_UI_FILT_1ST_ORDER`
- `GYRO_UI_FILT_2ND_ORDER`
- `GYRO_UI_FILT_3RD_ORDER`

---
## Structures

#### `ICM42688_INT_Config_t`
- [`ICM42688_UI_DRDY_INT_CLEAR_t`](#ICM42688_UI_DRDY_INT_CLEAR_t)` drdy_int_clear`
- [`ICM42688_FIFO_THS_INT_CLEAR_t`](#ICM42688_FIFO_THS_INT_CLEAR_t)` fifo_ths_int_clear`
- [`ICM42688_FIFO_FULL_INT_CLEAR_t`](#ICM42688_FIFO_FULL_INT_CLEAR_t)` fifo_full_int_clear`
- [`ICM42688_INT_TPULSE_DURATION_t`](#ICM42688_INT_TPULSE_DURATION_t)` tpulse_duration`
- [`ICM42688_INT_TDEASSERT_DISABLE_t`](#ICM42688_INT_TDEASSERT_DISABLE_t)` tdeassert_dis`
#### `ICM42688_INT_Channel_Config_t`
- `int pin`
- [`ICM42688_INT_MODE_t`](#ICM42688_INT_MODE_t)` mode`
- [`ICM42688_INT_DRIVE_CIRCUIT_t`](#ICM42688_INT_DRIVE_CIRCUIT_t)` drive_circuit`
- [`ICM42688_INT_POLARITY_t`](#ICM42688_INT_POLARITY_t)` polarity`
- `bool ui_fsync_en`
- `bool pll_rdy_en`
- `bool reset_done_en`
- `bool ui_drdy_en`
- `bool fifo_ths_en`
- `bool fifo_full_en`
- `bool ui_agc_rdy_en`
- `bool i3c_err_en`
- `bool smd_en`
- `bool wom_z_en`
- `bool wom_y_en`
- `bool wom_x_en`
#### `ICM42688_Config_t`
- [`ICM42688_InterfaceProtocol_t`](#ICM42688_InterfaceProtocol_t)` protocol`
- [`ICM42688_Interface_t`](#ICM42688_Interface_t)` interface_descriptor`
- `spi`
	- `int miso_pin`
	- `int mosi_pin`
	- `int sck_pin`
	- `int cs_pin`
	- `spi_host_device_t host`
	- `int sck_freq`
- `i2c`
	- `int scl_pin`
	- `int sda_pin`
	- `uint8_t address`
- `accel`
	- [`ICM42688_ACCEL_ENABLE_t`](#ICM42688_ACCEL_ENABLE_t)` enable`
	- [`ICM42688_ACCEL_MODE_t`](#ICM42688_ACCEL_MODE_t)` mode`
	- [`ICM42688_ACCEL_FS_SEL_t`](#ICM42688_ACCEL_FS_SEL_t)` scale`
	- [`ICM42688_ACCEL_ODR_t`](#ICM42688_ACCEL_ODR_t)` odr`
	- [`ICM42688_ACCEL_UI_FILT_BW_t`](#ICM42688_ACCEL_UI_FILT_BW_t)` ui_filt_bw`
	- [`ICM42688_ACCEL_UI_FILT_ORD_t`](#ICM42688_ACCEL_UI_FILT_ORD_t)` ui_filt_ord`
	- [`ICM42688_ACCEL_DEC2_M2_ORD_t`](#ICM42688_ACCEL_DEC2_M2_ORD_t)` dec2_m2_ord`
- `gyro`
	- [`ICM42688_GYRO_ENABLE_t`](#ICM42688_GYRO_ENABLE_t)` enable`
	- [`ICM42688_GYRO_MODE_t`](#ICM42688_GYRO_MODE_t)` mode`
	- [`ICM42688_GYRO_FS_SEL_t`](#ICM42688_GYRO_FS_SEL_t)` scale`
	- [`ICM42688_GYRO_ODR_t`](#ICM42688_GYRO_ODR_t)` odr`
	- [`ICM42688_GYRO_UI_FILT_BW_t`](#ICM42688_GYRO_UI_FILT_BW_t) ui_filt_bw`
	- [`ICM42688_GYRO_UI_FILT_ORD_t`](#ICM42688_GYRO_UI_FILT_ORD_t)` ui_filt_ord`
	- [`ICM42688_GYRO_DEC2_M2_ORD_t`](#ICM42688_GYRO_DEC2_M2_ORD_t)` dec2_m2_ord`
- `interrupt`
	- [`ICM42688_INT_Config_t`](#ICM42688_INT_Config_t)` cfg`
	- [`ICM42688_INT_Channel_Config_t`](#ICM42688_INT_Channel_Config_t)` int1`
	- [`ICM42688_INT_Channel_Config_t`](#ICM42688_INT_Channel_Config_t)` int2`
- `fifo`
	- [`ICM42688_FIFO_MODE_t`](#ICM42688_FIFO_MODE_t)` mode`
	- `uint16_t watermark`
#### `ICM42688_XYZ_t`
- `float x`
- `float y`
- `float z`
#### `ICM42688_Filter_t`
- `float err_measure`
- `float err_estimate`
- `float q`
- `float last_estimate`
#### `ICM42688_Par_t`
- [`ICM42688_XYZ_t`](#ICM42688_XYZ_t)` gyro_bias`
- [`ICM42688_XYZ_t`](#ICM42688_XYZ_t)` gyro_eps`
- [`ICM42688_XYZ_t`](#ICM42688_XYZ_t)` accel_eps`
- `float calib_temperature`
- [`ICM42688_XYZ_t`](#ICM42688_XYZ_t)` gyro_drift_sign`
#### `ICM42688_t`
- `uint8_t address`
- [`ICM42688_Interface_t`](#ICM42688_Interface_t)` *interface`
- `float _gyro_coef`
- `float _accel_coef`
- `size_t _gyro_data_bit`
- `size_t _accel_data_bit`
- [`ICM42688_ACCEL_ODR_t`](#ICM42688_ACCEL_ODR_t)` accel_odr`
- [`ICM42688_GYRO_ODR_t`](#ICM42688_GYRO_ODR_t)` gyro_odr`
- [`ICM42688_ACCEL_FS_SEL_t`](#ICM42688_ACCEL_FS_SEL_t)` accel_scale`
- [`ICM42688_GYRO_FS_SEL_t`](#ICM42688_GYRO_FS_SEL_t)` gyro_scale`
- [`ICM42688_FIFO_MODE_t`](#ICM42688_FIFO_MODE_t)` fifo_mode`
- [`ICM42688_XYZ_t`](#ICM42688_XYZ_t)` accel`
- [`ICM42688_XYZ_t`](#ICM42688_XYZ_t)` gyro`
- `float accel_total`
- `float temperature`
- [`ICM42688_Par_t`](#ICM42688_Par_t)` _par`
- [`ICM42688_Filter_t`](#ICM42688_Filter_t)` _gyro_x_filter`
- [`ICM42688_Filter_t`](#ICM42688_Filter_t)` _gyro_y_filter`
- [`ICM42688_Filter_t`](#ICM42688_Filter_t)` _gyro_z_filter`
- [`ICM42688_Filter_t`](#ICM42688_Filter_t)` _accel_x_filter`
- [`ICM42688_Filter_t`](#ICM42688_Filter_t)` _accel_y_filter`
- [`ICM42688_Filter_t`](#ICM42688_Filter_t)` _accel_z_filter`
- `void (*writeRegister) (uint8_t addr, uint8_t data)`
- `void (*readRegister) (uint8_t reg, uint8_t *buf)`

---
## Methods
#### `void ICM42688_Init(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_Config_t`](#ICM42688_Config_t)` *cfg)`
#### `void ICM42688_regBankSelect(`[`ICM42688_t`](#ICM42688_t)` *hicm, uint8_t bank)`
#### `void ICM42688_flushFIFO(`[`ICM42688_t`](#ICM42688_t)` *hicm)`
---
### Readings
#### `void ICM42688_readWhoAmI(`[`ICM42688_t`](#ICM42688_t)` *hicm, uint8_t *buf)`
Reading ``WHO_AM_I`` register value.
#### `void ICM42688_readRegAG(`[`ICM42688_t`](#ICM42688_t)` *hicm, int32_t *raw)`
Reading gyroscope's and accelerometer data via data registers. The methode uses a single byte reading function.
#### `void ICM42688_readFIFO(`[`ICM42688_t`](#ICM42688_t)` *hicm, int32_t *raw)`
Reading gyroscope's and accelerometer's data via FIFO. The FIFO must be initialized before. The methode means that the FIFO packet is 20-byte size and contains 20-bit gyroscope's and accelerometer's extentions.

---
### Calculations
#### `void ICM42688_calculateGyro(`[`ICM42688_t`](#ICM42688_t)` *hicm, int32_t *raw)`
Converts raw gyroscope's data into degrees values. Substracts the offsets and corrects results with temperature.
#### `void ICM42688_calculateAccel(`[`ICM42688_t`](#ICM42688_t)` *hicm, int32_t *raw)`
Converts raw accelerometer's data into g-values.

---
### Gyroscope & accelerometer settings
#### `void ICM42688_setGyroODR(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_GYRO_ODR_t`](#ICM42688_GYRO_FS_SEL_t)` odr)`
Sets gyroscope's output data rate. Allowed values:
- `GYRO_ODR_32KHZ`
- `GYRO_ODR_16KHZ`
- `GYRO_ODR_8KHZ`
- `GYRO_ODR_4KHZ`
- `GYRO_ODR_2KHZ`
- `GYRO_ODR_1KHZ`
- `GYRO_ODR_500HZ`
- `GYRO_ODR_200HZ`
- `GYRO_ODR_100HZ`
- `GYRO_ODR_50HZ`
- `GYRO_ODR_25HZ`
- `GYRO_ODR_12p5HZ`

#### `void ICM42688_setAccelODR(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_ACCEL_ODR_t`](#ICM42688_ACCEL_ODR_t)` odr)`
Sets accelerometer's output data rate. Allowed values:
- `ACCEL_ODR_32KHZ`
- `ACCEL_ODR_16KHZ`
- `ACCEL_ODR_8KHZ`
- `ACCEL_ODR_4KHZ`
- `ACCEL_ODR_2KHZ`
- `ACCEL_ODR_1KHZ`
- `ACCEL_ODR_500HZ`
- `ACCEL_ODR_200HZ`
- `ACCEL_ODR_100HZ`
- `ACCEL_ODR_50HZ`
- `ACCEL_ODR_25HZ`
- `ACCEL_ODR_12p5HZ`
- `ACCEL_ODR_6p25HZ`
- `ACCEL_ODR_3p125HZ`
- `ACCEL_ODR_1p5625HZ`

#### `void ICM42688_setGyroScale(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_GYRO_FS_SEL_t`](#ICM42688_GYRO_FS_SEL_t)` scale)`
Sets gyroscope's scale (15.625-2000 deg/s). Allowed values:
- `GYRO_FS_SEL_2000DPS`
- `GYRO_FS_SEL_1000DPS`
- `GYRO_FS_SEL_500DPS`
- `GYRO_FS_SEL_250DPS`
- `GYRO_FS_SEL_125DPS`
- `GYRO_FS_SEL_62p5DPS`
- `GYRO_FS_SEL_31p25DPS`
- `GYRO_FS_SEL_15p625DPS`

#### `void ICM42688_setAccelScale(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_ACCEL_FS_SEL_t`](#ICM42688_ACCEL_FS_SEL_t)` scale)`
Sets accelerometer's scale (2-16 g). Allowed values:
- `ACCEL_FS_SEL_16G`
- `ACCEL_FS_SEL_8G`
- `ACCEL_FS_SEL_4G`
- `ACCEL_FS_SEL_2G`
---
### Interrupt settings
#### `void ICM42688_setInterruptConfig(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_INT_Config_t`](#ICM42688_INT_Config_t)` *cfg)`

#### `void ICM42688_setINT1Config(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_INT_Channel_Config_t`](#ICM42688_INT_Channel_Config_t)` *ch)`

#### `void ICM42688_setINT2Config(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_INT_Channel_Config_t`](#ICM42688_INT_Channel_Config_t)` *ch)`
---
### FIFO settings
### `void ICM42688_setFIFOMode(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_FIFO_MODE_t`](#ICM42688_FIFO_MODE_t)` mode)`
Sets FIFO mode. Allowed values:
- `FIFO_BYPASS_MODE`
- `FIFO_STREAM_MODE`
- `FIFO_STOP_ON_FULL_MODE`

### `void ICM42688_setFIFOWatermark(`[`ICM42688_t`](#ICM42688_t)` *hicm, uint16_t watermark)`
Set FIFO thresfold watermark. If the FIFO contans more or equal bytes than watermark value, FIFO threshold flag is set.

### `void ICM42688_setGyroUIFiltBandwidth(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_GYRO_UI_FILT_BW_t`](#ICM42688_GYRO_UI_FILT_BW_t)` bw)`
Sets gyro's UI filter bandwidth. Allowed values:
- `GYRO_BW_ODR_DIV2` - ODR/2
- `GYRO_BW_MAX_400HZ_ODR_DIV2` - the maximum of (400Hz) and ODR/2
- `GYRO_BW_MAX_400HZ_ODR_DIV4` - the maximum of (400Hz) and ODR/4
- `GYRO_BW_MAX_400HZ_ODR_DIV5` - the maximum of (400Hz) and ODR/5
- `GYRO_BW_MAX_400HZ_ODR_DIV8` - the maximum of (400Hz) and ODR/8
- `GYRO_BW_MAX_400HZ_ODR_DIV10` - the maximum of (400Hz) and ODR/10
- `GYRO_BW_MAX_400HZ_ODR_DIV16` - the maximum of (400Hz) and ODR/16
- `GYRO_BW_MAX_400HZ_ODR_DIV20` - the maximum of (400Hz) and ODR/20
- `GYRO_BW_MAX_400HZ_ODR_DIV40` - the maximum of (400Hz) and ODR/40
- `GYRO_BW_MAX_200HZ_8xODR` - the maximum of (200Hz) and ODR\*8

### `void ICM42688_setGyroUIFiltOrder(`[`ICM42688_t`](#ICM42688_t)` *hicm, `[`ICM42688_GYRO_UI_FILT_ORD_t`](#ICM42688_GYRO_UI_FILT_ORD_t)` ord)`
Allowed values:
- `GYRO_UI_FILT_1ST_ORDER`
- `GYRO_UI_FILT_2ND_ORDER`
- `GYRO_UI_FILT_3RD_ORDER`

---
### Filter settings
#### `void ICM42688_gyroAntiAliasFilterEnable(`[`ICM42688_t`](#ICM42688_t)` *hicm)`

#### `void ICM42688_gyroAntiAliasFilterDisable(`[`ICM42688_t`](#ICM42688_t)` *hicm)

#### `void ICM42688_gyroNotchFilterEnable(`[`ICM42688_t`](#ICM42688_t)` *hicm)`

#### `void ICM42688_gyroNotchFilterDisable(`[`ICM42688_t`](#ICM42688_t)` *hicm)

#### `void ICM42688_gyroSetAAF_DELT(`[`ICM42688_t`](#ICM42688_t)` *hicm, uint8_t delt)`

#### `void ICM42688_gyroSetAAF_DELTSQR(`[`ICM42688_t`](#ICM42688_t)` *hicm, uint16_t deltsqr)`

#### `void ICM42688_gyroSetAAF_BITSHIFT(`[`ICM42688_t`](#ICM42688_t)` *hicm, uint8_t bitshift)`

---
### Filtering
#### `void ICM42688_filterInit(`[`ICM42688_t`](#ICM42688_t)` *hicm, float cycle_time)`
Kalman filter initialization

#### `void ICM42688_filterGyro(`[`ICM42688_t`](#ICM42688_t)` *hicm)`
Filter 3 gyro axes

#### `void ICM42688_filterAccel(`[`ICM42688_t`](#ICM42688_t)` *hicm)`
Filter 3 accel axes

---
### Calibration
#### `void ICM42688_calibrateGyro(`[`ICM42688_t`](#ICM42688_t)` *hicm)`

---
### IRQ flag status check
#### `bool ICM42688_UI_FSYNC_IRQ_Check(`[`ICM42688_t`](#ICM42688_t)` *hicm)`

#### `bool ICM42688_PLL_RDY_IRQ_Check(`[`ICM42688_t`](#ICM42688_t)` *hicm)`

#### `bool ICM42688_RESET_DONE_IRQ_Check(`[`ICM42688_t`](#ICM42688_t)` *hicm)`

#### `bool ICM42688_DATA_RDY_IRQ_Check(`[`ICM42688_t`](#ICM42688_t)` *hicm)`

#### `bool ICM42688_FIFO_THS_IRQ_Check(`[`ICM42688_t`](#ICM42688_t)` *hicm)`

#### `bool ICM42688_FIFO_FULL_IRQ_Check(`[`ICM42688_t`](#ICM42688_t)` *hicm)`

#### `bool ICM42688_AGC_RDY_IRQ_IRQ_Check(`[`ICM42688_t`](#ICM42688_t)` *hicm)`
