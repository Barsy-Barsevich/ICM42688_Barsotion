#include "ICM42688_Interface.h"
#include "driver/spi_master.h"
#include "portmacro.h"

/* device */
spi_device_handle_t icm_dev;
/* transactions */
spi_transaction_t icm_2byte_trans;
spi_transaction_t icm_fifo_trans;


void ICM42688_SPI_InterfaceInit(spi_host_device_t host, int miso, int mosi, int sck, int cs, int sck_freq)
{
	spi_bus_config_t buscfg = {0};
	buscfg.miso_io_num = miso;
    buscfg.mosi_io_num = mosi;
    buscfg.sclk_io_num = sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 262;
    buscfg.isr_cpu_id = 0;
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
    
	spi_device_interface_config_t icm_devcfg;
    memset(&icm_devcfg, 0, sizeof(spi_device_interface_config_t));
    icm_devcfg.clock_speed_hz = sck_freq;
    icm_devcfg.mode = 0;
    icm_devcfg.spics_io_num = cs;
    icm_devcfg.command_bits = 8;
    icm_devcfg.address_bits = 0;
    icm_devcfg.dummy_bits = 0;
    icm_devcfg.clock_source = SPI_CLK_SRC_DEFAULT;
    icm_devcfg.queue_size = 2;
    icm_devcfg.post_cb = NULL;
    icm_devcfg.pre_cb = NULL;
    spi_bus_add_device(host, &icm_devcfg, &icm_dev);
    
    memset(&icm_2byte_trans, 0, sizeof(spi_transaction_t));
    icm_2byte_trans.length = 8 * 2;
    icm_2byte_trans.rxlength = 8 * 2;
    icm_2byte_trans.flags =  SPI_TRANS_USE_TXDATA |  SPI_TRANS_USE_RXDATA;
    
    memset(&icm_fifo_trans, 0, sizeof(spi_transaction_t));
    icm_fifo_trans.length = 8 * 2;
    icm_fifo_trans.cmd = ICM_0_FIFO_DATA | 0x80;
    icm_fifo_trans.tx_buffer = NULL;
}


void ICM42688_SPI_readRegister(uint8_t reg, uint8_t *buf)
{
    icm_2byte_trans.cmd = reg | 0x80;
    icm_2byte_trans.tx_data[0] = 0xFF;
    spi_device_polling_transmit(icm_dev, &icm_2byte_trans);
    *buf = icm_2byte_trans.rx_data[0];
}


void ICM42688_SPI_writeRegister(uint8_t reg, uint8_t data)
{
    icm_2byte_trans.cmd = reg;
    icm_2byte_trans.tx_data[0] = data;
    spi_device_polling_transmit(icm_dev, &icm_2byte_trans);
}


void ICM42688_SPI_readFIFO(ICM42688_Interface_t *local, uint8_t *buf, size_t quan)
{
//	if (local->busy) spi_device_polling_end(icm_dev, portMAX_DELAY);
    icm_fifo_trans.cmd = ICM_0_FIFO_DATA | 0x80;
	icm_fifo_trans.length = quan * 8 + 8;
	icm_fifo_trans.rxlength = quan * 8;
	icm_fifo_trans.rx_buffer = buf;
	spi_device_polling_transmit(icm_dev, &icm_fifo_trans);
//	spi_device_polling_start(icm_dev, &icm_fifo_trans, portMAX_DELAY);
//	local->busy = true;
}