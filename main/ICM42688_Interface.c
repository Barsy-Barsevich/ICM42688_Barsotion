#include "ICM42688_Interface.h"
#include "ICM42688_RegMap.h"

/* device */
spi_device_handle_t icm_dev;
/* transs */
spi_transaction_t icm_2byte_trans;


void ICM42688_spiInit(int miso, int mosi, int sck, int cs)
{
	spi_bus_config_t buscfg = {0};
	buscfg.miso_io_num = miso;
    buscfg.mosi_io_num = mosi;
    buscfg.sclk_io_num = sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 262;
    buscfg.isr_cpu_id = 0;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    
	spi_device_interface_config_t icm_devcfg;
    memset(&icm_devcfg, 0, sizeof(spi_device_interface_config_t));
    icm_devcfg.clock_speed_hz = 1000000;
    icm_devcfg.mode = 0;
    icm_devcfg.spics_io_num = cs;
    icm_devcfg.command_bits = 8;
    icm_devcfg.address_bits = 0;
    icm_devcfg.dummy_bits = 0;
    icm_devcfg.clock_source = SPI_CLK_SRC_DEFAULT;
    icm_devcfg.queue_size = 2;
    icm_devcfg.post_cb = NULL;
    icm_devcfg.pre_cb = NULL;
    spi_bus_add_device(SPI2_HOST, &icm_devcfg, &icm_dev);
    
    memset(&icm_2byte_trans, 0, sizeof(spi_transaction_t));
    icm_2byte_trans.length = 8 * 2;
    icm_2byte_trans.rxlength = 8 * 2;
    icm_2byte_trans.flags =  SPI_TRANS_USE_TXDATA |  SPI_TRANS_USE_RXDATA;
}


void ICM42688_readRegister(uint8_t reg, uint8_t *buf)
{
    icm_2byte_trans.cmd = reg | 0x80;
    icm_2byte_trans.tx_data[0] = 0xFF;
    spi_device_polling_transmit(icm_dev, &icm_2byte_trans);
    *buf = icm_2byte_trans.rx_data[0];
}


void ICM42688_writeRegister(uint8_t reg, uint8_t data)
{
    icm_2byte_trans.cmd = reg;
    icm_2byte_trans.tx_data[0] = data;
    spi_device_polling_transmit(icm_dev, &icm_2byte_trans);
}


void ICM42688_readRegAG(int32_t *raw)
{
	uint8_t dummy;
	int16_t pre;
	ICM42688_readRegister(ICM_0_GYRO_DATA_X1, &dummy);
	pre = (int16_t)dummy << 8;
	ICM42688_readRegister(ICM_0_GYRO_DATA_X0, &dummy);
	pre |= (int16_t)dummy;
	raw[0] = (int32_t)pre;
	ICM42688_readRegister(ICM_0_GYRO_DATA_Y1, &dummy);
	pre = (int16_t)dummy << 8;
	ICM42688_readRegister(ICM_0_GYRO_DATA_Y0, &dummy);
	pre |= (int16_t)dummy;
	raw[1] = (int32_t)pre;
	ICM42688_readRegister(ICM_0_GYRO_DATA_Z1, &dummy);
	pre = (int16_t)dummy << 8;
	ICM42688_readRegister(ICM_0_GYRO_DATA_Z0, &dummy);
	pre |= (int16_t)dummy;
	raw[2] = (int32_t)pre;
	ICM42688_readRegister(ICM_0_ACCEL_DATA_X0, &dummy);
	pre = (int16_t)dummy << 8;
	ICM42688_readRegister(ICM_0_ACCEL_DATA_X1, &dummy);
	pre |= (int16_t)dummy;
	raw[3] = (int32_t)pre;
	ICM42688_readRegister(ICM_0_ACCEL_DATA_Y0, &dummy);
	pre = (int16_t)dummy << 8;
	ICM42688_readRegister(ICM_0_ACCEL_DATA_Y1, &dummy);
	pre |= (int16_t)dummy;
	raw[4] = (int32_t)pre;
	ICM42688_readRegister(ICM_0_ACCEL_DATA_Z0, &dummy);
	pre = (int16_t)dummy << 8;
	ICM42688_readRegister(ICM_0_ACCEL_DATA_Z1, &dummy);
	pre |= (int16_t)dummy;
	raw[5] = (int32_t)pre;
}