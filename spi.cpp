#include "pico/stdlib.h"
#include "spi.h"

SPI::SPI(spi_inst_t *spi, uint8_t miso, uint8_t sck, uint8_t mosi, uint baudrate)
: spi(spi)
{
    spi_init (spi, baudrate);

    gpio_set_function(miso, GPIO_FUNC_SPI);
    gpio_set_function(sck, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
}

SPI::~SPI() {
    spi_deinit(spi);
}

uint8_t SPI::transfer(uint8_t data){
    spi_write_read_blocking (spi, &data, &data, 1);
    return data;
}

uint16_t SPI::transfer16(uint16_t data){
    spi_write16_read16_blocking(spi, &data, &data, 1);
    return data;
    // uint8_t buff[2];
    // buff[0] = data & 0xff;
    // buff[1] = (data & 0xff00) >> 8;
    // spi_write_read_blocking (spi, buff, buff, 2);
    // return ((uint16_t)buff[1] << 8) | buff[0];

}
uint8_t SPI::read(uint8_t txData){
    uint8_t data;
    spi_read_blocking(spi, txData, &data,1);
    return data;
}

void SPI::write(uint8_t data){
    spi_write_blocking(spi, &data, 1);
}

void SPI::write16(uint16_t data){
    spi_write16_blocking(spi,&data,1);
    // uint8_t buff[2];
    // buff[0] = data & 0xff;
    // buff[1] = (data & 0xff00) >> 8;
    // spi_write_blocking(spi, buff, 2);
}

bool SPI::busy() {
    // SPI: SSPSR Register
	// bit 4,  BSY,  PrimeCell SSP busy flag, RO: 0 SSP is idle. 1 SSP is 
	// currently transmitting and/or receiving a frame or the
	// transmit FIFO is not empty
	return (spi_get_hw(spi)->sr & SPI_SSPSR_BSY_BITS) != 0;
}

void SPI::set_format (uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha, spi_order_t order){
    spi_set_format(spi, data_bits, cpol, cpha, order);
}

void SPI::set_slave (bool slave){
    spi_set_slave(spi, slave);
}

size_t SPI::is_writable (){
    return spi_is_writable(spi);
}

size_t SPI::is_readable (){
    return spi_is_readable(spi);
}

int SPI::write_read(const uint8_t *src, uint8_t *dst, size_t len){
    return spi_write_read_blocking(spi, src, dst, len);
}

int SPI::write(const uint8_t *src, size_t len){
    return spi_write_blocking(spi, src, len);
}

int SPI::read(uint8_t repeated_tx_data, uint8_t *dst, size_t len) {
    return spi_read_blocking(spi, repeated_tx_data, dst, len);
}

int SPI::write16_read16(const uint16_t *src, uint16_t *dst, size_t len){
    return spi_write16_read16_blocking(spi, src, dst, len);
}

int SPI::write16(const uint16_t *src, size_t len) {
    return spi_write16_blocking(spi, src, len);
}

int SPI::read16(uint16_t repeated_tx_data, uint16_t *dst, size_t len){
    return spi_read16_blocking(spi, repeated_tx_data,dst, len);
}