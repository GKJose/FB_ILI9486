#ifndef FB_ILI9486_H
#define FB_ILI9486_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"{
#endif

void main(int argc,char* argv[]);
void gpio_init(void);
void ili9486_init(void);
void ili9486_rotate(int degrees,bool bgr);
void ili9486_write(int mode,uint8_t data);
void ili9486_write_array(int mode,uint8_t *data, uint16_t len);
void sleep_ms(int delay);

#ifdef __cplusplus
} /* extern "C" */
#endif
/*********************
 *      TYPEDEFS
 *********************/
typedef union {
    struct {
        uint16_t blue : 5;
        uint16_t green : 6;
        uint16_t red : 5;
    } ch;
    uint16_t full;
} fbcolor16_t;
/*********************
 *      DEFINES
 *********************/
#define LCD_RST_PIN 8
#define LCD_CS_PIN  9
#define LCD_RS_PIN  7
#define LCD_WR_PIN  0
#define LCD_RD_PIN  2

#define LCD_D7_PIN  21
#define LCD_D6_PIN  30
#define LCD_D5_PIN  14
#define LCD_D4_PIN  13
#define LCD_D3_PIN  12
#define LCD_D2_PIN  3
#define LCD_D1_PIN  23
#define LCD_D0_PIN  22

#define ILI9486_CMD_MODE    0
#define ILI9486_DATA_MODE   1

#define ILI9486_TFTWIDTH 480
#define ILI9486_TFTHEIGHT 320

/* Level 1 Commands -------------- [section] Description */

#define ILI9486_NOP         0x00        /* [8.2.1] No operation/Terminate from frame memory write/read */
#define ILI9486_SWRESET     0x01    /* [8.2.2] Software Reset */
#define ILI9486_RDDIDIF     0x04    /* [8.2.3] Read Display Identification Information */
#define ILI9486_RDNEDSI     0x05    /* [8.2.4] Read Number of the errors on DSI */
#define ILI9486_RDDST       0x09    /* [8.2.5] Read Display Status */
#define ILI9486_RDDPM       0x0A    /* [8.2.6] Read Display Power Mode */
#define ILI9486_RDDMADCTL   0x0B  /* [8.2.7] Read Display MADCTL */
#define ILI9486_RDDCOLMOD   0x0C /* [8.2.8] Read Display Pixel Format */
#define ILI9486_RDDIM       0x0D    /* [8.2.9] Read Display Image Mode */
#define ILI9486_RDDSM       0x0E    /* [8.2.10] Read Display Signal Mode */
#define ILI9486_RDDSDR      0x0F    /* [8.2.11] Read Display Self-Diagnostic Result */
#define ILI9486_SLPIN       0x10    /* [8.2.12] Enter Sleep Mode */
#define ILI9486_SLPOUT      0x11    /* [8.2.13] Exit Sleep Mode */
#define ILI9486_PTLON       0x12    /* [8.2.14] Partial Display Mode ON */
#define ILI9486_NORON       0x13    /* [8.2.15] Normal Display Mode ON */
#define ILI9486_DINVOFF     0x20    /* [8.2.16] Display Inversion OFF */
#define ILI9486_DINVON      0x21    /* [8.2.17] Display Inversion ON */
#define ILI9486_DISPOFF     0x28    /* [8.2.18] Display OFF */
#define ILI9486_DISPON      0x29     /* [8.2.19] Display ON */
#define ILI9486_CASET       0x2A     /* [8.2.20] Column Address Set */
#define ILI9486_PASET       0x2B      /* [8.2.21] Page Address Set */
#define ILI9486_RAMWR       0x2C      /* [8.2.22] Memory Write */
#define ILI9486_RANRD       0x2E      /* [8.2.23] Memory Read */
#define ILI9486_PTLAR       0x30      /* [8.2.24] Partial Area */
#define ILI9486_VSCRDEF     0x33    /* [8.2.25] Vertical Scrolling Definition */
#define ILI9486_TEOFF       0x34    /* [8.2.26] Tearing Effect Line OFF */
#define ILI9486_TEON        0x35    /* [8.2.27] Tearing Effect Line ON */
#define ILI9486_MADCTL      0x36    /* [8.2.28] Memory Access Control */
#define     MADCTL_MY       0x80    /*          MY Row Address Order */
#define     MADCTL_MX       0x40    /*          MX Column Address Order */
#define     MADCTL_MV       0x20    /*          MV Row/Column Exchange */
#define     MADCTL_ML       0x10    /*          ML Vertical Refresh Order */
#define     MADCTL_BGR      0x08    /*          BGR Order   */
#define     MADCTL_RGB      0x00    /*          RGB Order [default] */
#define     MADCTL_MH       0x04    /*          MH Hortizontal Refresh Order */
#define ILI9486_VSCRSADD    0x37    /* [8.2.29] Vertical Scrolling Start Address */
#define ILI9486_IDMOFF      0x38    /* [8.2.30] Idle Mode OFF */
#define ILI9486_IDMON       0x39    /* [8.2.31] Idle Mode ON */
#define ILI9486_PIXSET      0x3A    /* [8.2.32] Pixel Fortmat Set */
#define ILI9486_WRMEMCONT   0x3C    /* [8.2.33] Memory Write Continue */
#define ILI9486_RDMEMCONT   0x3E    /* [8.2.34] Memory Read Continue */
#define ILI9486_SETSCANTE   0x44    /* [8.2.35] Set Tear Scanline */
#define ILI9486_GETSCAN     0x45    /* [8.2.36] Get Scanline */
#define ILI9486_WRDISBV     0x51    /* [8.2.37] Write Display Brightness Value */
#define ILI9486_RDDISBV     0x52    /* [8.2.38] Read Display Brightness Value */
#define ILI9486_WRCTRLD     0x53    /* [8.2.39] Write Control Display */
#define ILI9486_RDCTRLD     0x54    /* [8.2.40] Read Control Display */
#define ILI9486_WRCABC      0x55    /* [8.2.41] Write Content Adaptive Brightness Control Value */
#define ILI9486_RDCABC      0x56    /* [8.2.42] Read Content Adaptive Brightnes Control Value */
#define ILI9486_WRCABCMIN   0x5E    /* [8.2.43] Write CABC Minimum Brightness */
#define ILI9486_RDCABCMIN   0x5F    /* [8.2.44] Read CABC Minimum Brightness */
#define ILI9486_RDFCKS      0xAA    /* [8.2.45] Read First Checksum */
#define ILI9486_RDCONTCHS   0xAF    /* [8.2.46] Read Continue Checksum */
#define ILI9486_RDID1       0xDA    /* [8.2.47] Read ID1 - Manufacturer ID (user) */
#define ILI9486_RDID2       0xDB    /* [8.2.48] Read ID2 - Module/Driver version (supplier) */
#define ILI9486_RDID3       0xDC    /* [8.2.49] Read ID3 - Module/Driver version (user) */

/* Level 2 Commands -------------------[section] Description */

#define ILI9486_IFMODE      0xB0   /* [8.2.50] Interface Mode Control */
#define ILI9486_FRMCTR1     0xB1   /* [8.2.51] Frame Rate Control (In Normal Mode/Full Colors) */
#define ILI9486_FRMCTR2     0xB2   /* [8.2.52] Frame Rate Control (In Idle Mode/8 colors) */
#define ILI9486_FRMCTR3     0xB3   /* [8.2.53] Frame Rate control (In Partial Mode/Full Colors) */
#define ILI9486_INVTR       0xB4   /* [8.2.54] Display Inversion Control */
#define ILI9486_PRCTR       0xB5   /* [8.2.55] Blanking Porch Control */
#define ILI9486_DISCTRL     0xB6   /* [8.2.56] Display Function Control */
#define ILI9486_ETMOD       0xB7   /* [8.2.57] Entry Mode Set */
#define ILI9486_PWCTRL1     0xC0
#define ILI9486_PWCTRL2     0xC1
#define ILI9486_PWCTRL3     0xC2
#define ILI9486_PWCTRL4     0xC3
#define ILI9486_PWCTRL5     0xC4
#define ILI9486_VMCTRL      0xC5
#define ILI9486_CABCCTRL1   0xC6
#define ILI9486_CABCCTRL2   0xC8
#define ILI9486_CABCCTRL3   0xC9
#define ILI9486_CABCCTRL4   0xCA
#define ILI9486_CABCCTRL5   0xCB
#define ILI9486_CABCCTRL6   0xCC
#define ILI9486_CABCCTRL7   0xCD
#define ILI9486_CABCCTRL8   0xCE
#define ILI9486_CABCCTRL9   0xCF
#define ILI9486_NVMWR       0xD0
#define ILI9486_NVMPKEY     0xD1
#define ILI9486_RDNVM       0xD2
#define ILI9486_RDID4       0xD3
#define ILI9486_PGAMCTRL    0xE0
#define ILI9486_NGAMCTRL    0xE1
#define ILI9486_DGAMCTRL1   0xE2
#define ILI9486_DGAMCTRL2   0xE3
#define ILI9486_RDSPICMDSET 0xFB



#endif /* ILI9486_H */