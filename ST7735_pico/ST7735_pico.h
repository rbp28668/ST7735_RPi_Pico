/***************************************************
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef __ST7735_pico_H_
#define __ST7735_pico_H_

#include  <cstdint>
#include  <string.h>
#include <stdlib.h>
#include "../spi.h"

#include "ILI9341_fonts.h"

#define SPI_MODE0 0

#define ST7735_SPICLOCK 24000000
//#define ST7735_SPICLOCK 16000000

// some flags for initR() :(
#define INITR_GREENTAB 0x0
#define INITR_REDTAB   0x1
#define INITR_BLACKTAB 0x2

#define INITR_18GREENTAB    INITR_GREENTAB
#define INITR_18REDTAB      INITR_REDTAB
#define INITR_18BLACKTAB    INITR_BLACKTAB
#define INITR_144GREENTAB   0x1
#define INITR_144GREENTAB_OFFSET   0x4
#define INITR_MINI160x80  0x05
#define INITR_MINI160x80_ST7735S 0x06

#define INIT_ST7789_TABCOLOR 42  // Not used except as a indicator to the code... 

#define ST7735_TFTWIDTH  128
#define ST7735_TFTWIDTH_80     80 // for mini
// for 1.44" display
#define ST7735_TFTHEIGHT_144 128
// for 1.8" display and mini
#define ST7735_TFTHEIGHT_160  160 // for 1.8" and mini display

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF

// Also define them in a non specific ST77XX specific name
#define ST77XX_BLACK      0x0000
#define ST77XX_WHITE      0xFFFF
#define ST77XX_RED        0xF800
#define ST77XX_GREEN      0x07E0
#define ST77XX_BLUE       0x001F
#define ST77XX_CYAN       0x07FF
#define ST77XX_MAGENTA    0xF81F
#define ST77XX_YELLOW     0xFFE0
#define ST77XX_ORANGE     0xFC00
#define ST77XX_PINK       0xF81F

// Map fonts that were modified back to the ILI9341 font
#define ST7735_pico_font_t ILI9341_t3_font_t

// Lets see about supporting Adafruit fonts as well?
#if __has_include(<gfxfont.h>)
  #include <gfxfont.h>
#endif

#ifndef _GFXFONT_H_
#define _GFXFONT_H_

/// Font data stored PER GLYPH
typedef struct {
	uint16_t bitmapOffset;     ///< Pointer into GFXfont->bitmap
	uint8_t  width;            ///< Bitmap dimensions in pixels
    uint8_t  height;           ///< Bitmap dimensions in pixels
	uint8_t  xAdvance;         ///< Distance to advance cursor (x axis)
	int8_t   xOffset;          ///< X dist from cursor pos to UL corner
    int8_t   yOffset;          ///< Y dist from cursor pos to UL corner
} GFXglyph;

/// Data stored for FONT AS A WHOLE
typedef struct { 
	uint8_t  *bitmap;      ///< Glyph bitmaps, concatenated
	GFXglyph *glyph;       ///< Glyph array
	uint8_t   first;       ///< ASCII extents (first char)
    uint8_t   last;        ///< ASCII extents (last char)
	uint8_t   yAdvance;    ///< Newline distance (y axis)
} GFXfont;

#endif // _GFXFONT_H_


#ifndef st7735_swap
#define st7735_swap(a, b) { typeof(a) t = a; a = b; b = t; }
#endif

#define CL(_r,_g,_b) ((((_r)&0xF8)<<8)|(((_g)&0xFC)<<3)|((_b)>>3))


//These enumerate the text plotting alignment (reference datum point)
#define TL_DATUM 0 // Top left (default)
#define TC_DATUM 1 // Top centre
#define TR_DATUM 2 // Top right
#define ML_DATUM 3 // Middle left
#define CL_DATUM 3 // Centre left, same as above
#define MC_DATUM 4 // Middle centre
#define CC_DATUM 4 // Centre centre, same as above
#define MR_DATUM 5 // Middle right
#define CR_DATUM 5 // Centre right, same as above
#define BL_DATUM 6 // Bottom left
#define BC_DATUM 7 // Bottom centre
#define BR_DATUM 8 // Bottom right
//#define L_BASELINE  9 // Left character baseline (Line the 'A' character would sit on)
//#define C_BASELINE 10 // Centre character baseline
//#define R_BASELINE 11 // Right character baseline

 template<class T> 
  const T& min(const T& a, const T& b)
  {
      return (b < a) ? b : a;
  }


  template<class T> 
  const T& max(const T& a, const T& b)
  {
      return (b > a) ? b : a;
  }


class ST7735_pico
{
  SPI* _pspi;

 public:

  ST7735_pico(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST = -1);
  ST7735_pico(SPI* pspi, uint8_t CS, uint8_t RS, uint8_t RST = -1);

  void     initB(void),                             // for ST7735B displays
           initR(uint8_t options = INITR_GREENTAB), // for ST7735R
           setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1),
           pushColor(uint16_t color, bool last_pixel=false),
           fillScreen(uint16_t color),
           drawPixel(int16_t x, int16_t y, uint16_t color),
           drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
           drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
           fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
   inline void fillWindow(uint16_t color) {fillScreen(color);}
  virtual void setRotation(uint8_t r);
  void     invertDisplay(bool i);
  void     setRowColStart(uint16_t x, uint16_t y);
  uint16_t  rowStart() {return _rowstart;}
  uint16_t  colStart() {return _colstart;}

  void setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
    __attribute__((always_inline)) {
        writecommand(ST7735_CASET); // Column addr set
        writedata16(x0+_xstart);   // XSTART 
        writedata16(x1+_xstart);   // XEND
        writecommand(ST7735_RASET); // Row addr set
        writedata16(y0+_ystart);   // YSTART
        writedata16(y1+_ystart);   // YEND
  }

  ////
  	// from Adafruit_GFX.h
	int16_t width(void) const { return _width; };
	int16_t height(void) const { return _height; }
	uint8_t getRotation(void);
	
	void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
	void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
	void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
	void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
	void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
	void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
	void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
	void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
	void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y);
	void inline drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size) 
	    { drawChar(x, y, c, color, bg, size, size);}
	
	static const int16_t CENTER = 9998;
	void setCursor(int16_t x, int16_t y, bool autoCenter=false);
    void getCursor(int16_t *x, int16_t *y);
	void setTextColor(uint16_t c);
	void setTextColor(uint16_t c, uint16_t bg);
    void setTextSize(uint8_t sx, uint8_t sy);
	void inline setTextSize(uint8_t s) { setTextSize(s,s); }
	uint8_t getTextSizeX();
	uint8_t getTextSizeY();
	uint8_t getTextSize();
	void setTextWrap(bool w);
	bool getTextWrap();
	
	//////
	virtual size_t write(uint8_t);		
	virtual size_t write(const uint8_t *buffer, size_t size);
	int16_t getCursorX(void) const { return cursor_x; }
	int16_t getCursorY(void) const { return cursor_y; }
	void setFont(const ILI9341_t3_font_t &f);
    void setFont(const GFXfont *f = NULL);
	void setFontAdafruit(void) { setFont(); }
	void drawFontChar(unsigned int c);
	void drawGFXFontChar(unsigned int c);

    void getTextBounds(const uint8_t *buffer, uint16_t len, int16_t x, int16_t y,
      int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
    void getTextBounds(const char *string, int16_t x, int16_t y,
      int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
    // void getTextBounds(const String &str, int16_t x, int16_t y,
    //   int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
	int16_t strPixelLen(const char * str);

	// added support for drawing strings/numbers/floats with centering
	// modified from tft_ili9341_ESP github library
	// Handle numbers
	int16_t  drawNumber(long long_num,int poX, int poY);
	int16_t  drawFloat(float floatNumber,int decimal,int poX, int poY);   
	// Handle char arrays
	int16_t drawString(const char*  string, int poX, int poY);
	int16_t drawString1(const char string[], int16_t len, int poX, int poY);

	void setTextDatum(uint8_t datum);
	
	// added support for scrolling text area
	// https://github.com/vitormhenrique/ILI9341_t3
	// Discussion regarding this optimized version:
    //http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-%28320x240-TFT-color-display%29-library
	//	
	void setScrollTextArea(int16_t x, int16_t y, int16_t w, int16_t h);
	void setScrollBackgroundColor(uint16_t color);
	void enableScroll(void);
	void disableScroll(void);
	void scrollTextArea(uint8_t scrollSize);
	void resetScrollBackgroundColor(uint16_t color);
	uint16_t readPixel(int16_t x, int16_t y);
	void readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors);

	// setOrigin sets an offset in display pixels where drawing to (0,0) will appear
	// for example: setOrigin(10,10); drawPixel(5,5); will cause a pixel to be drawn at hardware pixel (15,15)
	void setOrigin(int16_t x = 0, int16_t y = 0) { 
		_originx = x; _originy = y; 
		//if (Serial) Serial.printf("Set Origin %d %d\n", x, y);
		updateDisplayClip();
	}
	void getOrigin(int16_t* x, int16_t* y) { *x = _originx; *y = _originy; }

	// setClipRect() sets a clipping rectangle (relative to any set origin) for drawing to be limited to.
	// Drawing is also restricted to the bounds of the display

	void setClipRect(int16_t x1, int16_t y1, int16_t w, int16_t h) 
		{ _clipx1 = x1; _clipy1 = y1; _clipx2 = x1+w; _clipy2 = y1+h; 
			//if (Serial) Serial.printf("Set clip Rect %d %d %d %d\n", x1, y1, w, h);
			updateDisplayClip();
		}
	void setClipRect() {
			 _clipx1 = 0; _clipy1 = 0; _clipx2 = _width; _clipy2 = _height; 
			//if (Serial) Serial.printf("clear clip Rect\n");
			updateDisplayClip(); 
		}	
////
	
  void sendCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes);



  // Pass 8-bit (each) R,G,B, get back 16-bit packed color
  inline uint16_t Color565(uint8_t r, uint8_t g, uint8_t b) {
           return ((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3);
  }
  void setBitrate(uint32_t n);

  // Useful methods added from ili9341_t3 
  void writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pcolors);


 protected:
  uint8_t  tabcolor;

  void     spiwrite(uint8_t),
  		     spiwrite16(uint16_t d),
           writecommand(uint8_t c),
           writecommand_last(uint8_t c),
           writedata(uint8_t d),
           writedata_last(uint8_t d),
           writedata16(uint16_t d),
           writedata16_last(uint16_t d),
           commandList(const uint8_t *addr),
           commonInit(const uint8_t *cmdList, uint8_t mode=SPI_MODE0);
  void charBounds(char c, int16_t *x, int16_t *y,
      			int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy);
  //uint8_t  spiread(void);

  void waitTransmitComplete(void);
  inline void beginSPITransaction();
  inline void endSPITransaction();
  inline void set8Bit();
  inline void set16Bit();
  void delay(uint ms);

  bool is16Bit;

  ////
  int16_t  cursor_x, cursor_y;
	bool 	 _center_x_text = false; 
	bool 	 _center_y_text = false; 
	int16_t  _clipx1, _clipy1, _clipx2, _clipy2;
	int16_t  _originx, _originy;
	int16_t  _displayclipx1, _displayclipy1, _displayclipx2, _displayclipy2;
	bool _invisible = false; 
	bool _standard = true; // no bounding rectangle or origin set. 

	inline void updateDisplayClip() {
		_displayclipx1 = max<int16_t>(0,min<int16_t>( _clipx1+_originx, width()));
		_displayclipx2 = max<int16_t>(0,min<int16_t>( _clipx2+_originx, width()));

		_displayclipy1 = max<int16_t>(0,min<int16_t>(_clipy1+_originy, height()));
		_displayclipy2 = max<int16_t>(0,min<int16_t>(_clipy2+_originy, height()));
		_invisible = (_displayclipx1 == _displayclipx2 || _displayclipy1 == _displayclipy2);
		_standard =  (_displayclipx1 == 0) && (_displayclipx2 == _width) && (_displayclipy1 == 0) && (_displayclipy2 == _height);
	}
	
	int16_t _width, _height;
	int16_t scroll_x, scroll_y, scroll_width, scroll_height;
	bool scrollEnable,isWritingScrollArea; // If set, 'wrap' text at right edge of display

	uint16_t textcolor, textbgcolor, scrollbgcolor;
	uint32_t textcolorPrexpanded, textbgcolorPrexpanded;
	uint8_t textsize_x, textsize_y, rotation, textdatum;
	bool wrap; // If set, 'wrap' text at right edge of display
	const ILI9341_t3_font_t *font;
	// Anti-aliased font support
	uint8_t fontbpp = 1;
	uint8_t fontbppindex = 0;
	uint8_t fontbppmask = 1;
	uint8_t fontppb = 8;
	uint8_t* fontalphalut;
	float fontalphamx = 1;
	
	uint32_t padX;

	// GFX Font support
	const GFXfont *gfxFont = nullptr;
	int8_t _gfxFont_min_yOffset = 0;

	// Opaque font chracter overlap?
	unsigned int _gfx_c_last;
	int16_t   _gfx_last_cursor_x, _gfx_last_cursor_y;
	int16_t	 _gfx_last_char_x_write = 0;
	uint16_t _gfx_last_char_textcolor;
	uint16_t _gfx_last_char_textbgcolor;
	bool gfxFontLastCharPosFG(int16_t x, int16_t y);
	
	void drawFontBits(bool opaque, uint32_t bits, uint32_t numbits, int32_t x, int32_t y, uint32_t repeat);
	void drawFontPixel( uint8_t alpha, uint32_t x, uint32_t y );
	uint32_t fetchpixel(const uint8_t *p, uint32_t index, uint32_t x);


  uint16_t _colstart, _rowstart, _xstart, _ystart, _rot, _screenHeight, _screenWidth;
  
  uint8_t  _cs, _rs, _rst, _sid, _sclk;
  uint8_t pcs_data, pcs_command;
  uint32_t ctar;
  


void HLine(int16_t x, int16_t y, int16_t w, uint16_t color)
	  __attribute__((always_inline)) 
	  {
		#ifdef ENABLE_ST77XX_FRAMEBUFFER
	  	if (_use_fbtft) {
	  		drawFastHLine(x, y, w, color);
	  		return;
	  	}
	  	#endif
	    x+=_originx;
	    y+=_originy;

	    // Rectangular clipping
	    if((y < _displayclipy1) || (x >= _displayclipx2) || (y >= _displayclipy2)) return;
	    if(x<_displayclipx1) { w = w - (_displayclipx1 - x); x = _displayclipx1; }
	    if((x+w-1) >= _displayclipx2)  w = _displayclipx2-x;
	    if (w<1) return;

		setAddr(x, y, x+w-1, y);
		writecommand(ST7735_RAMWR);
		do { writedata16(color); } while (--w > 0);
	  }
	  
	void VLine(int16_t x, int16_t y, int16_t h, uint16_t color)
	  __attribute__((always_inline)) 
	  {
		x+=_originx;
	    y+=_originy;

	    // Rectangular clipping
	    if((x < _displayclipx1) || (x >= _displayclipx2) || (y >= _displayclipy2)) return;
	    if(y < _displayclipy1) { h = h - (_displayclipy1 - y); y = _displayclipy1;}
	    if((y+h-1) >= _displayclipy2) h = _displayclipy2-y;
	    if(h<1) return;

		setAddr(x, y, x, y+h-1);
		writecommand(ST7735_RAMWR);
		do { writedata16(color); } while (--h > 0);
	  }
	  
	/**
	 * Found in a pull request for the Adafruit framebuffer library. Clever!
	 * https://github.com/tricorderproject/arducordermini/pull/1/files#diff-d22a481ade4dbb4e41acc4d7c77f683d
	 * Converts  0000000000000000rrrrrggggggbbbbb
	 *     into  00000gggggg00000rrrrr000000bbbbb
	 * with mask 00000111111000001111100000011111
	 * This is useful because it makes space for a parallel fixed-point multiply
	 * This implements the linear interpolation formula: result = bg * (1.0 - alpha) + fg * alpha
	 * This can be factorized into: result = bg + (fg - bg) * alpha
	 * alpha is in Q1.5 format, so 0.0 is represented by 0, and 1.0 is represented by 32
	 * @param	fg		Color to draw in RGB565 (16bit)
	 * @param	bg		Color to draw over in RGB565 (16bit)
	 * @param	alpha	Alpha in range 0-255
	 **/
	uint16_t alphaBlendRGB565( uint32_t fg, uint32_t bg, uint8_t alpha )
	 __attribute__((always_inline)) {
	 	alpha = ( alpha + 4 ) >> 3; // from 0-255 to 0-31
		bg = (bg | (bg << 16)) & 0b00000111111000001111100000011111;
		fg = (fg | (fg << 16)) & 0b00000111111000001111100000011111;
		uint32_t result = ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
		return (uint16_t)((result >> 16) | result); // contract result
	}
	/**
	 * Same as above, but fg and bg are premultiplied, and alpah is already in range 0-31
	 */
	uint16_t alphaBlendRGB565Premultiplied( uint32_t fg, uint32_t bg, uint8_t alpha )
	 __attribute__((always_inline)) {
		uint32_t result = ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
		return (uint16_t)((result >> 16) | result); // contract result
	}
	  
	void Pixel(int16_t x, int16_t y, uint16_t color)
	  __attribute__((always_inline)) {
	    x+=_originx;
	    y+=_originy;

	  	if((x < _displayclipx1) ||(x >= _displayclipx2) || (y < _displayclipy1) || (y >= _displayclipy2)) return;

	  setAddr(x, y, x, y);
		writecommand(ST7735_RAMWR);
		writedata16(color);
	}

};

// To avoid conflict when also using Adafruit_GFX or any Adafruit library
// which depends on Adafruit_GFX, #include the Adafruit library *BEFORE*
// you #include ST7735_pico.h.
// Warning the implemention of class needs to be here, else the code
// compiled in the c++ file will cause duplicate defines in the link phase. 
#define Adafruit_GFX_Button ST7735_Button
class ST7735_Button {
public:
	ST7735_Button(void) { _gfx = NULL; }
	void initButton(ST7735_pico *gfx, int16_t x, int16_t y,
		uint8_t w, uint8_t h,
		uint16_t outline, uint16_t fill, uint16_t textcolor,
		const char *label, uint8_t textsize_x, uint8_t textsize_y) {
		_x = x;
		_y = y;
		_w = w;
		_h = h;
		_outlinecolor = outline;
		_fillcolor = fill;
		_textcolor = textcolor;
		_textsize_x = textsize_x;
		_textsize_y = textsize_y;
		_gfx = gfx;
		strncpy(_label, label, 9);
		_label[9] = 0;

	}
	void drawButton(bool inverted = false) {
		uint16_t fill, outline, text;

		if (! inverted) {
			fill = _fillcolor;
			outline = _outlinecolor;
			text = _textcolor;
		} else {
			fill =  _textcolor;
			outline = _outlinecolor;
			text = _fillcolor;
		}
		_gfx->fillRoundRect(_x - (_w/2), _y - (_h/2), _w, _h, min(_w,_h)/4, fill);
		_gfx->drawRoundRect(_x - (_w/2), _y - (_h/2), _w, _h, min(_w,_h)/4, outline);
		_gfx->setCursor(_x - strlen(_label)*3*_textsize_x, _y-4*_textsize_y);
		_gfx->setTextColor(text);
		_gfx->setTextSize(_textsize_x, _textsize_y);
		//_gfx->print(_label);
	}

	bool contains(int16_t x, int16_t y) {
		if ((x < (_x - _w/2)) || (x > (_x + _w/2))) return false;
		if ((y < (_y - _h/2)) || (y > (_y + _h/2))) return false;
		return true;
	}

	void press(bool p) {
		laststate = currstate;
		currstate = p;
	}
	bool isPressed() { return currstate; }
	bool justPressed() { return (currstate && !laststate); }
	bool justReleased() { return (!currstate && laststate); }
private:
	ST7735_pico *_gfx;
	int16_t _x, _y;
	uint16_t _w, _h;
	uint8_t _textsize_x, _textsize_y;
	uint16_t _outlinecolor, _fillcolor, _textcolor;
	char _label[10];
	bool currstate, laststate;
};



#endif	 