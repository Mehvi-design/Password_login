#include <ST7735.h>
#include<stdlib.h>
#include<string.h>
int16_t _width;       ///< Display width as modified by current rotation
int16_t _height;      ///< Display height as modified by current rotation
int16_t cursor_x;     ///< x location to start print()ing text
int16_t cursor_y;     ///< y location to start print()ing text
uint8_t rotation;     ///< Display rotation (0 thru 3)
uint8_t _colstart;   ///< Some displays need this changed to offset
uint8_t _rowstart;       ///< Some displays need this changed to offset
uint8_t _xstart;
uint8_t _ystart;

  const uint8_t
  init_cmds1[] = {            // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args: //the wholescreen is refreshed
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
	                           //fosc = oscillator frequency (internal chip clock) 1x2 + 40 → fixed formula divisor, (LINE + 2C + 2D) → time per line
	                          // Effect: Sets the frame rate (usually around 60-80Hz). Too low → flicker. Too high → too much power / heat.
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:// This sets the frame rate when the display is in idle mode (low power / less refreshing)
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode: pixel polarity dot by dot → reduces flicker
      0x01, 0x2C, 0x2D,       //     Line inversion mode: Alternates pixel polarity line by line → reduces power noise
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
  init_cmds2[] = {            // Init for 7735R, part 2 (1.44" display)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127
#endif // ST7735_IS_128X128

#ifdef ST7735_IS_160X80
  init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
    3,                        //  3 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x4F,             //     XEND = 79
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F ,            //     XEND = 159
    ST7735_INVON, 0 },        //  3: Invert colors
#endif

  init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12, //= Gamma Positive Polarity Correction These 16 numbers define how the display maps input
	  0x37, 0x32, 0x29, 0x2d, // brightness levels to actual voltage levels applied to pixels for positive polarity pixels.
	  0x29, 0x25, 0x2B, 0x39,//In other words: they control contrast, brightness curve, and color shading for the positive parts of pixels.
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06, // GMCTRN1 = Gamma Negative Polarity Correction
      0x2E, 0x2C, 0x29, 0x2D,//Same idea as GMCTRP1, but this controls the gamma curve for negative polarity pixels.
      0x2E, 0x2E, 0x37, 0x3F,//LCDs alternate positive/negative voltages on pixels to prevent damage — so both curves are needed.
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay



 volatile uint8_t spi_ready = 1;       // SPI ready flag
 volatile uint8_t dma_active = 0;      // DMA active flag
 uint8_t* dma_buffer = NULL;           // Pointer to current DMA buffer
 uint32_t dma_buffer_size = 0;         // Size of current DMA buffer
//
// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
//
////     if (hspi == &ST7735_SPI_PORT) { //Checks if the SPI peripheral that just finished transmitting is the one connected to your ST7735 display.
////    	 ST7735_Unselect();
////    	 spi_ready = 1; // Set flag when DMA completes
////
////     }
//	 if (hspi->Instance == ST7735_SPI_PORT.Instance) {
//	         ST7735_Unselect();
//	         spi_ready = 1;  // if you're using a flagc
//	         //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
//	     }
// }

 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
     if (hspi->Instance == ST7735_SPI_PORT.Instance) {
         // Clean up DMA state on error
         if (dma_buffer != NULL) {
             free(dma_buffer);
             dma_buffer = NULL;
             dma_buffer_size = 0;
         }
         ST7735_Unselect();
         spi_ready = 1;
         dma_active = 0;

         // Optional: Reset SPI peripheral if needed
         HAL_SPI_DeInit(&ST7735_SPI_PORT);
         HAL_SPI_Init(&ST7735_SPI_PORT);
     }
 }

 void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
     if (hspi->Instance == ST7735_SPI_PORT.Instance) {
         // Free the previous DMA buffer if it exists
         if (dma_buffer != NULL) {
             free(dma_buffer);
             dma_buffer = NULL;
             dma_buffer_size = 0;
         }

         ST7735_Unselect();
         spi_ready = 1;      // SPI is now ready
         dma_active = 0;     // DMA is no longer active
     }
 }

//  volatile uint8_t dma_busy = 0;
//
//  void start_dma_transfer(uint8_t *data, uint16_t size) {
//      dma_busy = 1;
//      HAL_SPI_Transmit_DMA(&hspi1, data, size);
//  }
//
//  void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
//      if (hspi->Instance == SPI1) {
//          dma_busy = 0;
//          // Optionally trigger next transfer here
//      }
//  }

 void ST7735_WaitForDMA(void) {
     uint32_t timeout = 1000;  // Reasonable timeout
     while(dma_active && timeout--) {
         __NOP();  // Wait or yield to RTOS if you're using one
     }
     if (timeout == 0) {
         // Handle timeout error
         dma_active = 0;
         spi_ready = 1;
     }
 }


void ST7735_Select()
{
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);//to select the LCD and start the communication CS=0
}

void ST7735_Unselect()
{
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);//to deselect the LCD set CS=1
}

void ST7735_Reset()
{
    HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_RESET);//first reset =0
    HAL_Delay(5);
    HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET);//reset=1
}
//void ST7735_WriteCommand(uint8_t cmd) {
//
////        // Wait until SPI is free
////        while (!spi_ready) {}
////        spi_ready = 0; // Reset flag
//
//        HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
//        HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, 1,HAL_MAX_DELAY);
//        //HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT, &cmd, 1);
//
////    HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, 1,HAL_MAX_DELAY);
//
//}
//void ST7735_WriteData(uint8_t* buff, size_t buff_size)
//{
//	//wait until SPI is free
////	while(!spi_ready){}
////		spi_ready=0;//reset the flag
//    HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);//to send data DC=1
//    HAL_SPI_Transmit(&ST7735_SPI_PORT, buff, buff_size,HAL_MAX_DELAY);
//	//HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT, buff, buff_size);
//
//}
void ST7735_WriteCommand(uint8_t cmd) {
    ST7735_WaitForDMA();  // Wait for any DMA to complete

    HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, 1, HAL_MAX_DELAY);
}

void ST7735_WriteData(uint8_t* buff, size_t buff_size) {
    ST7735_WaitForDMA();  // Wait for any DMA to complete

    HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
}

void DisplayInit(const uint8_t *addr)//the pointer that points to the series of commands which has the list of commands,arguments total, arguments,optional delay
{
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;//the number of command is stored in numCommand and the pointer is incremented
    while(numCommands--) {// the while loop will follow with command is decremented until total commands is zero
        uint8_t cmd = *addr++;//cmd stores the command and pointer is updated
        ST7735_WriteCommand(cmd);//the command is sent via SPI_DMA

        numArgs = *addr++;//the pointer stores the number of arguments and increments
        // If high bit set, delay follows args
        ms = numArgs & DELAY;//if the delay is calculated by taking AND of number of argument and delay
        numArgs &= ~DELAY;//if delay is same the num argument and the invert of DELAY AND will give zero
        if(numArgs) {//if it is more then 1 write the arguments
            ST7735_WriteData((uint8_t*)addr, numArgs);//send the buffer and buffer's size
            addr += numArgs;//adder pointer is added by numArgs
        }

        if(ms) {
            ms = *addr++;//the pointer is pointed in ms then incremented
            if(ms == 255) ms = 500;//if the ms is 25 update it to 500
            HAL_Delay(ms);
        }
    }
}

void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    // column address set
    ST7735_WriteCommand(ST7735_CASET);//column address
    uint8_t data[] = { 0x00, x0 + _xstart, 0x00, x1 + _xstart };
    ST7735_WriteData(data, sizeof(data));

    // row address set
    ST7735_WriteCommand(ST7735_RASET);// row address
    data[1] = y0 + _ystart;
    data[3] = y1 + _ystart;
    ST7735_WriteData(data, sizeof(data));

    // write to RAM
    ST7735_WriteCommand(ST7735_RAMWR);
}

void ST7735_Init(uint8_t rotation)
{
    ST7735_Select();
    ST7735_Reset();
    DisplayInit(init_cmds1);
    DisplayInit(init_cmds2);
    DisplayInit(init_cmds3);
#if ST7735_IS_160X80
    _colstart = 24;
    _rowstart = 0;
 /*****  IF Doesn't work, remove the code below (before #elif) *****/
    uint8_t data = 0xC0;
    ST7735_Select();
    ST7735_WriteCommand(ST7735_MADCTL);//seeting the command on MADCTL register responsible for rotation
    ST7735_WriteData(&data,1);
    ST7735_Unselect();
//The MADCTL register (also called Memory Data Access Control) controls:
//   The mapping of display RAM memory to screen pixels.
//    The scan direction (X/Y flip or mirror).
//    The rotation (row/column swap, aka transposing X and Y).
//    The color order (RGB vs BGR).
#elif ST7735_IS_128X128
    _colstart = 2;
    _rowstart = 3;
#else
    _colstart = 0;
    _rowstart = 0;
#endif
    ST7735_SetRotation (rotation);
    ST7735_Unselect();

}
//0: default (portrait)
//
//1: landscape (rotated 90°)
//
//2: upside-down portrait (rotated 180°)
//
//3: landscape (rotated 270°)
void ST7735_SetRotation(uint8_t m)
{

  uint8_t madctl = 0;

  rotation = m % 4; // can't be higher than 3

  switch (rotation)
  {
  case 0:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB;//selecting the right madctl bit
      //ST7735_MADCTL_MX → mirror X (flip left-right)
//      ST7735_MADCTL_MY → mirror Y (flip top-bottom)
//      ST7735_MADCTL_MV → swap X and Y (transpose image axes)
//      ST7735_MADCTL_BGR / RGB → color order (Blue-Green-Red or Red-Green-Blue)
      _height = ST7735_HEIGHT;
      _width = ST7735_WIDTH;
      _xstart = _colstart;
      _ystart = _rowstart;
#endif
    break;
  case 1:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB;
      _width = ST7735_HEIGHT;
      _height = ST7735_WIDTH;
    _ystart = _colstart;
    _xstart = _rowstart;
#endif
    break;
  case 2:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_RGB;
      _height = ST7735_HEIGHT;
      _width = ST7735_WIDTH;
    _xstart = _colstart;
    _ystart = _rowstart;
#endif
    break;
  case 3:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB;
      _width = ST7735_HEIGHT;
      _height = ST7735_WIDTH;
    _ystart = _colstart;
    _xstart = _rowstart;
#endif
    break;
  }
  ST7735_Select();
  ST7735_WriteCommand(ST7735_MADCTL);
  ST7735_WriteData(&madctl,1);
  ST7735_Unselect();
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= _width) || (y >= _height))
        return;

    ST7735_Select();

    ST7735_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ST7735_WriteData(data, sizeof(data));

    ST7735_Unselect();
}

void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    ST7735_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                ST7735_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                ST7735_WriteData(data, sizeof(data));
            }
        }
    }
}

//void ST7735_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
//    ST7735_Select();
//
//    while(*str) {
//        if(x + font.width >= _width) {
//            x = 0;
//            y += font.height;
//            if(y + font.height >= _height) {
//                break;
//            }
//
//            if (*str == ' ' || *str == '\r' || *str == '\n') {
//                // skip spaces in the beginning of the new line
//                str++;
//                continue;
//            }
//        }
//
//        ST7735_WriteChar(x, y, *str, font, color, bgcolor);
//        x += font.width;
//        str++;
//    }
//
//    ST7735_Unselect();
//}
void ST7735_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    ST7735_Select();

    while (*str) {
        if (*str == '\r') {
            str++;  // skip carriage return
            continue;
        }

        if (*str == '\n') {
            x = 0;
            y += font.height;
            if (y + font.height >= _height) {
                break;  // no more space on screen
            }
            str++;
            continue;
        }

        if (x + font.width >= _width) {
            x = 0;
            y += font.height;
            if (y + font.height >= _height) {
                break;
            }
        }

        ST7735_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    ST7735_Unselect();
}
void ST7735_WriteStringWithSeparators(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    ST7735_Select();//X COLUMN,Y ROWS
//11 pixels wide(column) 18 pixels height(row)
    while (*str) {
        if (*str == '\r') {
            str++;  // skip carriage return
            continue;
        }
        if (*str == '\n') {
            x = 20;
            y += font.height;
            if (y + font.height >= _height) {
                break;
            }
            str++;
            continue;
        }
        if (*str == '/') {  // treat / as newline
            x = 20;
            y += font.height;
            if (y + font.height >= _height) {
                break;
            }
            str++;
            continue;
        }

        if (x + font.width >= _width) {
            x = 0;
            y += (font.height);
            if (y + font.height >= _height) {
                break;
            }
        }

        ST7735_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    ST7735_Unselect();
}

// Assume you have ST7735 width and height defined globally
// #define ST7735_WIDTH 128
// #define ST7735_HEIGHT 160
void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // Boundary checks
    if ((x >= _width) || (y >= _height)) return;
    if ((x + w - 1) >= _width) w = _width - x;
    if ((y + h - 1) >= _height) h = _height - y;

    // Wait for any previous DMA to complete
    ST7735_WaitForDMA();

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    uint32_t size = w * h;
    uint8_t *buffer = malloc(size * 2);  // 2 bytes per pixel (RGB565)

    if (buffer == NULL) {
        ST7735_Unselect();
        return;
    }

    // Fill buffer with color
    uint8_t high = color >> 8;
    uint8_t low = color & 0xFF;
    for (uint32_t i = 0; i < size; i++) {
        buffer[i * 2] = high;
        buffer[i * 2 + 1] = low;
    }


    // Set DMA state before starting transfer
    spi_ready = 0;
    dma_active = 1;
    dma_buffer = buffer;
    dma_buffer_size = size * 2;

    HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);

    if (HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT, buffer, size * 2) != HAL_OK) {
        // If DMA start fails, clean up
        free(buffer);
        dma_buffer = NULL;
        dma_buffer_size = 0;
        spi_ready = 1;
        dma_active = 0;
        ST7735_Unselect();
    }
}

//void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
//    if ((x >= _width) || (y >= _height)) return;
//    if ((x + w - 1) >= _width) w = _width - x;
//    if ((y + h - 1) >= _height) h = _height - y;
//
//    ST7735_Select();
//    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);
//
//    uint32_t size = w * h;
//    uint8_t *buffer = malloc(size * 2);  // 2 bytes per pixel (RGB565)
//
//    if (buffer == NULL) {
//        ST7735_Unselect();
//        return; // handle malloc failure
//    }
//
//    // Fill buffer with color
//    uint8_t high = color >> 8;
//    uint8_t low = color & 0xFF;
//    for (uint32_t i = 0; i < size; i++) {
//        buffer[i * 2] = high;
//        buffer[i * 2 + 1] = low;
//    }
//
//    HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
//    // Start DMA transfer
//
//    spi_ready=0;//reset the flag
//    //dma_busy=0;
//    if (HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT, buffer, size *2) != HAL_OK) {
//        free(buffer);
//        ST7735_Unselect();
//        return; // handle DMA start failure
//    }
//    //HAL_DMA_PollForTransfer(hspi1.hdmatx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
//
////    // Usage
////    start_dma_transfer(buffer, size*2);
////    // In main loop or elsewhere:
////    if (!dma_busy) {
////        start_dma_transfer(buffer, size);
////    }
//}



//void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
//{
//    if((x >= _width) || (y >= _height)) return;
//    if((x + w - 1) >= _width) w = _width - x;
//    if((y + h - 1) >= _height) h = _height - y;
//
//    ST7735_Select();
//    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);
//
//    uint8_t data[] = { color >> 8, color & 0xFF };
//    HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
//    for(y = h; y > 0; y--) {
//        for(x = w; x > 0; x--) {
//            HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
//        }
//    }
//
//    ST7735_Unselect();
//}

void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= _width) || (y >= _height)) return;
    if((x + w - 1) >= _width) return;
    if((y + h - 1) >= _height) return;

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);
    ST7735_WriteData((uint8_t*)data, sizeof(uint16_t)*w*h);
    ST7735_Unselect();
}

void ST7735_InvertColors(bool invert) {
    ST7735_Select();
    ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
    ST7735_Unselect();
}


