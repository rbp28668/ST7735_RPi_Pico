
# ST7735 library for Raspberry Pi Pico

This library supports ST7735 and ST7789 using 4 wire SPI.
Originally Written by Limor Fried/Ladyada for Adafruit Industries.
Taken from Paul Stoffregen's Teensy port at https://github.com/PaulStoffregen/ST7735_t3
Modified for Raspberry Pi Pico by Bruce Porteous


## Connections
Pins are:
- CS - chip select, active low.
- SCL - Serial clock.  Data clocked in on rising edges
- SDA - Serial data.  Transmitted MS bit first in 8 or 16 bit words
- RES - Reset, active low
- DC -  Data/Command - sending a command when 0, data when 1
- BLK - backlight control.

e.g. using ST7735S and SPI 0

- PIN 22, GPIO17 (SPIO-CS) -> CS
- PIN 24, GPIO18 (SPIO-CLK)-> SCL
- PIN 25, GPIO19 (SPIO->TX)-> SDA
- PIN 26, GPIO20 -> RES
- PIN 27, GPIO21 -> DC
- PIN 31, GPIO26 ->  BLK


From demo code and datasheet:

Chip select = active low.
Command set RS is 0, data RS is 1
Data clocked in on rising edge
clock idle is high (may not matter)
Bytes transmitted MS bit first
2 byte data - command is 1, data transmitted high byte first.  CS stays low for complete 16 bit transfer.
Looking at data sheet (and behaviour) you can't transfer a 16 bit quantity as 2 8 bit quanties with CS
being de-asserted (i.e. going high for a bit) in the middle.




To initialize an instance of the display the user has a choice of constructors.  Either use hardware SPI
 object or bit-banged software.


For hardware SPI (spi0 in this case)
`SPI spi(spi0, 16, SCL_PIN, SDA_PIN, 2000000); // CS pin not part of SPI per se as really tied to addressable device`
`ST7735_pico screen(&spi,CS_PIN, DC_PIN,RES_PIN); // Hence CS is included here.`

For software bit-banging
`ST7735_pico screen(CS_PIN, DC_PIN,SDA_PIN, SCL_PIN, RES_PIN);`


