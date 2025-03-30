#ifndef GDEW0097T50_LIB_H
#define GDEW0097T50_LIB_H

#include <Arduino.h>
#include <SPI.h>

#define isEPD_W21_BUSY digitalRead( 12 ) 
#define EPD_W21_RST_0 digitalWrite(  0, LOW )
#define EPD_W21_RST_1 digitalWrite(  0, HIGH )
#define EPD_W21_DC_0  digitalWrite( 15, LOW )
#define EPD_W21_DC_1  digitalWrite( 15, HIGH )
#define EPD_W21_CS_0  digitalWrite(  2, LOW )
#define EPD_W21_CS_1  digitalWrite(  2, HIGH )

#define EPD_WIDTH   88
#define EPD_HEIGHT  184
#define EPD_ARRAY  2024  

void SPI_Write(unsigned char value);
void EPD_W21_WriteCMD(unsigned char command);
void EPD_W21_WriteDATA(unsigned char datas);
void lcd_chkstatus(void);
void EPD_lut(unsigned char LUTsel);
void EPD_Init(void);
void EPD_Init_Part(void);
void EPD_DeepSleep(void);
void EPD_Update(void);
void EPD_WhiteScreen_ALL(const unsigned char *datas);
void EPD_WhiteScreen_White(void);
void EPD_SetRAMValue_BaseMap( const unsigned char * datas);
void EPD_Dis_Part(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_COLUMN,unsigned int PART_LINE);
void EPD_Dis_PartAll(const unsigned char * datas);
void EPD_Dis_Part_RAM(unsigned int x_start,unsigned int y_start,
                      const unsigned char * datas_A,const unsigned char * datas_B,
                      const unsigned char * datas_C,const unsigned char * datas_D,const unsigned char * datas_E,
                      unsigned char num,unsigned int PART_COLUMN,unsigned int PART_LINE);
void EPD_Init_180(void);

#endif



/*
    That's all folks!
*/
