# Methods descriptions

- [Readings](#Readings)
- [Calculations](#Calculations)
- [Settings](#Gyroscope-&-accelerometer-settings)
- [Interrupt settings](#Interrupt-settings)
- [FIFO settings](#FIFO-settings)
- [Filter settings](#Filter-settings)
- [Filtering](#Filtering)
- [Calibration](#Calibration)
- [IRQ flag status check](#IRQ-flag-status-check)
### `void ICM42688_Init(ICM42688_t *hicm, ICM42688_Config_t *cfg)`
### `void ICM42688_regBankSelect(ICM42688_t *hicm, uint8_t bank)`
### `void ICM42688_flushFIFO(ICM42688_t *hicm)`
-----------
## Readings
### ``void ICM42688_readWhoAmI(ICM42688_t *hicm, uint8_t *buf)``
Reading ``WHO_AM_I`` register value.
### ``void ICM42688_readRegAG(ICM42688_t *hicm, int32_t *raw)``
Reading gyroscope's and accelerometer data via data registers. The methode uses a single byte reading function.
### ``void ICM42688_readFIFO(ICM42688_t *hicm, int32_t *raw)``
Reading gyroscope's and accelerometer's data via FIFO. The FIFO must be initialized before. The methode means that the FIFO packet is 20-byte size and contains 20-bit gyroscope's and accelerometer's extentions.

---
## Calculations
### ``void ICM42688_calculateGyro(ICM42688_t *hicm, int32_t *raw)``
Converts raw gyroscope's data into degrees values. Substracts the offsets and corrects results with temperature.
### ``void ICM42688_calculateAccel(ICM42688_t *hicm, int32_t *raw)``
Converts raw accelerometer's data into g-values.

---
## Gyroscope & accelerometer settings
### ``void ICM42688_setGyroODR(ICM42688_t *hicm, ICM42688_GYRO_ODR_t odr)``
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

### ``void ICM42688_setAccelODR(ICM42688_t *hicm, ICM42688_ACCEL_ODR_t odr)``
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

### ``void ICM42688_setGyroScale(ICM42688_t *hicm, ICM42688_GYRO_FS_SEL_t scale)``
Sets gyroscope's scale (15.625-2000 deg/s). Allowed values:
- `GYRO_FS_SEL_2000DPS`
- `GYRO_FS_SEL_1000DPS`
- `GYRO_FS_SEL_500DPS`
- `GYRO_FS_SEL_250DPS`
- `GYRO_FS_SEL_125DPS`
- `GYRO_FS_SEL_62p5DPS`
- `GYRO_FS_SEL_31p25DPS`
- `GYRO_FS_SEL_15p625DPS`

### ``void ICM42688_setAccelScale(ICM42688_t *hicm, ICM42688_ACCEL_FS_SEL_t scale)``
Sets accelerometer's scale (2-16 g). Allowed values:
- `ACCEL_FS_SEL_16G`
- `ACCEL_FS_SEL_8G`
- `ACCEL_FS_SEL_4G`
- `ACCEL_FS_SEL_2G`

## Interrupt settings
### ``void ICM42688_setInterruptConfig(ICM42688_t *hicm, ICM42688_INT_Config_t *cfg)``
### ``void ICM42688_setINT1Config(ICM42688_t *hicm, ICM42688_INT_Channel_Config_t *ch)``
### ``void ICM42688_setINT2Config(ICM42688_t *hicm, ICM42688_INT_Channel_Config_t *ch)``

## FIFO settings
### `void ICM42688_setFIFOMode(ICM42688_t *hicm, ICM42688_FIFO_MODE_t mode)`
Sets FIFO mode. Allowed values:
- `FIFO_BYPASS_MODE`
- `FIFO_STREAM_MODE`
- `FIFO_STOP_ON_FULL_MODE`

### `void ICM42688_setFIFOWatermark(ICM42688_t *hicm, uint16_t watermark)`
Set FIFO thresfold watermark. If the FIFO contans more or equal bytes than watermark value, FIFO threshold flag is set.
### `void ICM42688_setGyroUIFiltBandwidth(ICM42688_t *hicm, ICM42688_GYRO_UI_FILT_BW_t bw)`
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

### `void ICM42688_setGyroUIFiltOrder(ICM42688_t *hicm, ICM42688_GYRO_UI_FILT_ORD_t ord)`
Allowed values:
- `GYRO_UI_FILT_1ST_ORDER`
- `GYRO_UI_FILT_2ND_ORDER`
- `GYRO_UI_FILT_3RD_ORDER`

## Filter settings
### ``void ICM42688_gyroAntiAliasFilterEnable(ICM42688_t *hicm)``
### ``void ICM42688_gyroAntiAliasFilterDisable(ICM42688_t *hicm)``
### ``void ICM42688_gyroNotchFilterEnable(ICM42688_t *hicm)``
### ``void ICM42688_gyroNotchFilterDisable(ICM42688_t *hicm)``
### ``void ICM42688_gyroSetAAF_DELT(ICM42688_t *hicm, uint8_t delt)``
### ``void ICM42688_gyroSetAAF_DELTSQR(ICM42688_t *hicm, uint16_t deltsqr)``
### ``void ICM42688_gyroSetAAF_BITSHIFT(ICM42688_t *hicm, uint8_t bitshift)``
---
## Filtering
### ``void ICM42688_filterInit(ICM42688_t *hicm, float cycle_time)``
### ``void ICM42688_filterGyro(ICM42688_t *hicm)``
### ``void ICM42688_filterAccel(ICM42688_t *hicm)``
---
## Calibration
### ``void ICM42688_calibrateGyro(ICM42688_t *hicm)``
---
## IRQ flag status check
### ``bool ICM42688_UI_FSYNC_IRQ_Check(ICM42688_t *hicm)``
### ``bool ICM42688_PLL_RDY_IRQ_Check(ICM42688_t *hicm)``
### ``bool ICM42688_RESET_DONE_IRQ_Check(ICM42688_t *hicm)``
### ``bool ICM42688_DATA_RDY_IRQ_Check(ICM42688_t *hicm)``
### ``bool ICM42688_FIFO_THS_IRQ_Check(ICM42688_t *hicm)``
### ``bool ICM42688_FIFO_FULL_IRQ_Check(ICM42688_t *hicm)``
### ``bool ICM42688_AGC_RDY_IRQ_IRQ_Check(ICM42688_t *hicm)``
