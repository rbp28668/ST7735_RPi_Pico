#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "hardware/spi.h"
class SPI {

spi_inst_t * spi;

public:
SPI(spi_inst_t *spi, uint8_t miso, uint8_t sck, uint8_t mosi, uint baudrate);
~SPI();

// Helper functions that are closer to Arduino equivalents.
uint8_t transfer(uint8_t data);
uint16_t transfer16(uint16_t data);
uint8_t read(uint8_t txData = 0);
void write(uint8_t data);
void write16(uint16_t data);

// Additional check to allow activity to be separated (e.g. to set other control bits)
// Needed for some graphics controllers that use seperate control/data pins
bool busy();

// Direct equivalent for PICO SPI
void set_format (uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha, spi_order_t order);
void set_slave (bool slave);
size_t is_writable ();
size_t is_readable ();
int write_read(const uint8_t *src, uint8_t *dst, size_t len);
int write(const uint8_t *src, size_t len);
int read(uint8_t repeated_tx_data, uint8_t *dst, size_t len);
int write16_read16(const uint16_t *src, uint16_t *dst, size_t len);
int write16(const uint16_t *src, size_t len);
int read16(uint16_t repeated_tx_data, uint16_t *dst, size_t len);
};


#endif