#include "FB_ILI9486.h"

#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <string.h>

#include <linux/fb.h>

#define FBDEV_PATH "/dev/fb0"


#ifndef DIV_ROUND_UP
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#endif

/**********************
 *      STRUCTURES
 **********************/

struct fb_var_info{
    uint32_t xoffset;
    uint32_t yoffset;
    uint32_t xres;
    uint32_t yres;
    int bits_per_pixel;
 };

struct fb_fix_info{
    long int line_length;
    long int smem_len;
};
/**********************
 *  STATIC VARIABLES
 **********************/
static struct fb_var_screeninfo vinfo;
static struct fb_fix_screeninfo finfo;

static char *fbp = 0;
static long int screensize = 0;
static int fbfd = 0;

//---------------------------------
static const char *chipname = "gpiochip0";
static struct gpiod_chip *chip;
static struct gpiod_line *lrst;
static struct gpiod_line *lcs;
static struct gpiod_line *lrs;
static struct gpiod_line *lwr;
static struct gpiod_line *lrd;
static struct gpiod_line_bulk ldata;

static unsigned int ldatapins[] = {LCD_D7_PIN,LCD_D6_PIN,LCD_D5_PIN,LCD_D4_PIN,LCD_D3_PIN,LCD_D2_PIN,LCD_D1_PIN,LCD_D0_PIN};

void main(int argc,char* argv[]){
    fbfd = open(FBDEV_PATH,O_RDWR);
    if(fbfd == -1){
        perror("ERROR: Cannot open framebuffer device!");
        return;
    }
// Get fixed screen information
    if(ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        return;
    }
    // Get variable screen information
    if(ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        return;
    }
    screensize =  finfo.smem_len; //finfo.line_length * vinfo.yres;    

    // Map the device to memory
    fbp = (char *)mmap(0, screensize, PROT_READ, MAP_SHARED, fbfd, 0);
    if((intptr_t)fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        return;
    }
    printf("Start init\n");
    gpio_init();
    ili9486_init();
    ili9486_rotate(180,false);
    printf("Start copying fb to tft\n");
    int32_t act_x1 = 0;
    int32_t act_y1 = 0;
    int32_t act_x2 = ILI9486_TFTWIDTH > (uint32_t)vinfo.xres - 1 ?  (uint32_t)vinfo.xres - 1:ILI9486_TFTWIDTH;
    int32_t act_y2 = ILI9486_TFTHEIGHT > (int32_t)vinfo.yres - 1 ? (int32_t)vinfo.yres - 1 : ILI9486_TFTHEIGHT;
    long int location = 0;
    long int byte_location = 0;
    unsigned char bit_location = 0;
    uint32_t screen_data_size = ((act_y2 - act_y1)*(act_x2-act_x1));
    fbcolor16_t * screen_data = calloc(screen_data_size,sizeof(fbcolor16_t));

    while(true){


        
        /*16 bit per pixel*/
        if(vinfo.bits_per_pixel == 16) {
			
            //Copy framebuffer into pointer array of color pixels
            uint16_t * fbp16 = (uint16_t *)fbp;
            int32_t y;
            for(y = act_y1; y <= act_y2; y++) {
                location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 2;
                memcpy(&screen_data[location],&fbp16[location],(act_x2-act_x1));             
            }
            uint8_t data[4];
            int32_t len = (act_x2 - act_x1 +1) *2;
            int32_t w = (act_x2 - act_x1) + 1 ;
            //Send pixels to tft

            /* window horizontal */
            ili9486_write(ILI9486_CMD_MODE,ILI9486_CASET);
            data[0] = act_x1 >> 8;
            data[1] = act_x1;
            data[2] = act_x2 >> 8;
            data[3] = act_x2;
            ili9486_write_array(ILI9486_DATA_MODE,data,4);

            /*window vertical */
            ili9486_write(ILI9486_CMD_MODE,ILI9486_PASET);
            data[0] = act_y1 >> 8;
            data[1] = act_y1;
            data[2] = act_y2 >> 8;
            data[3] = act_y2;
            ili9486_write_array(ILI9486_DATA_MODE,data,4);

            ili9486_write(ILI9486_CMD_MODE, ILI9486_RAMWR);
             int pixIdx = 0;
            for(y = act_y1; y <= act_y2; y++) {
                 ili9486_write_color_array(&screen_data[pixIdx],len);
                 pixIdx += w;
             }


        }else {
            /*Not supported bit per pixel*/
        }



    }
	gpiod_line_release(lrst);
	gpiod_line_release(lcs);
	gpiod_line_release(lrs);
	gpiod_line_release(lwr);
	gpiod_line_release(lrd);
	
	gpiod_line_release_bulk(&ldata);
	gpiod_chip_close(chip);
	
	free(screen_data);
    close(fbfd);



}
void gpio_init(void){
	
	printf("Init GPIO\n");
    chip = gpiod_chip_open_by_name(chipname);
	
	//Get LCD control pins
	lrst = gpiod_chip_get_line(chip,LCD_RST_PIN);
	lcs = gpiod_chip_get_line(chip,LCD_CS_PIN);
	lrs = gpiod_chip_get_line(chip,LCD_RS_PIN);
	lcs = gpiod_chip_get_line(chip,LCD_CS_PIN);
	lwr = gpiod_chip_get_line(chip,LCD_WR_PIN);
	lrd = gpiod_chip_get_line(chip,LCD_RD_PIN);
	if(lrst == NULL || lcs == NULL || lrs == NULL || lcs == NULL || lwr == NULL || lrd == NULL){
		perror("Failed to get control lines");
	}
	printf("got control lines\n");
	//Set LCD control pins to output, logic low
	if(gpiod_line_request_output(lrst,"consumer",0) == -1){
		perror("Failure to open lrst as output");
	}
	if(gpiod_line_request_output(lcs,"consumer",0) == -1){
		perror("Failure to open lcs as output");
	}
	if(gpiod_line_request_output(lrs,"consumer",0) == -1){
		perror("Failure to open lrs as output");
	}
	if(gpiod_line_request_output(lwr,"consumer",0) == -1){
		perror("Failure to open lwr as output");
	}
	if(gpiod_line_request_output(lrd,"consumer",0) == -1){
		perror("Failure to open lrd as output");
	}
	
	//Get LCD Data pins
	if(gpiod_chip_get_lines(chip,ldatapins,8,&ldata) == -1){
		perror("Failure to get data lines");
	}
	printf("got data lines\n");
	//Set LCD Data pins to output, logic low
	if(gpiod_line_request_bulk_output(&ldata,"consumer",NULL) == -1){
		perror("Failure to open ldata as output");
	}

	printf("end gpio init\n");
	gpiod_line_set_value(lrst,1);

}
void ili9486_init(void){
	printf("Init ILI9486\n");
    uint8_t data[15];

    /* hardware reset */
    gpiod_line_set_value(lrst,0);// RESX - Active low - initializes the chip
	usleep(12000);
	gpiod_line_set_value(lrst,1);
	usleep(12000);

    /* software reset */
    ili9486_write(ILI9486_CMD_MODE,ILI9486_SWRESET);
    sleep_ms(5);
	printf("Display off\n");
    ili9486_write(ILI9486_CMD_MODE,ILI9486_DISPOFF);
    usleep(1000);
    /*startup sequence */
	
	ili9486_write(ILI9486_CMD_MODE,0xF1);
	data[0] = 0x36;
	data[1] = 0x04;
	data[2] = 0x00;
	data[3] = 0x3C;
	data[4] = 0x0F;
	data[5] = 0x8F;
	ili9486_write_array(ILI9486_DATA_MODE,data,6);
	
    ili9486_write(ILI9486_CMD_MODE,0xF2);
    data[0] = 0x18;
    data[1] = 0xA3;
    data[2] = 0x12;
    data[3] = 0x02;
    data[4] = 0xB2;
    data[5] = 0x12;
    data[6] = 0xFF;
    data[7] = 0x10;
    data[8] = 0x00;
    ili9486_write_array(ILI9486_DATA_MODE,data,9);

    ili9486_write(ILI9486_CMD_MODE,0xF8);
    data[0] = 0x21;
    data[1] = 0x04;
    ili9486_write_array(ILI9486_DATA_MODE,data,2);


    ili9486_write(ILI9486_CMD_MODE,0xF9);
    data[0] = 0x00;
    data[1] = 0x08;
    ili9486_write_array(ILI9486_DATA_MODE,data,2);

    ili9486_write(ILI9486_CMD_MODE,0xB4);
    data[0] = 0x00;
    ili9486_write_array(ILI9486_DATA_MODE,data,1);

    ili9486_write(ILI9486_CMD_MODE,ILI9486_PWCTRL2); // 0xC1
    data[0] = 0x41;
    ili9486_write_array(ILI9486_DATA_MODE,data,1);
	
	ili9486_write(ILI9486_CMD_MODE,ILI9486_VMCTRL);
	data[0] = 0x00;
	data[1] = 0x91;
	data[2] = 0x80;
	data[3] = 0x00;
	ili9486_write_array(ILI9486_DATA_MODE,data,4);

    ili9486_write(ILI9486_CMD_MODE,ILI9486_PGAMCTRL);
    data[0] = 0x0F;
    data[1] = 0x1F;
    data[2] = 0x1C;
    data[3] = 0x0C;
    data[4] = 0x0F;
    data[5] = 0x08;
    data[6] = 0x48;
    data[7] = 0x98;
    data[8] = 0x37;
    data[9] = 0x0A;
    data[10] = 0x13;
    data[11] = 0x04;
    data[12] = 0x11;
    data[13] = 0x0D;
    data[14] = 0x00;
    ili9486_write_array(ILI9486_DATA_MODE,data,15);

    ili9486_write(ILI9486_CMD_MODE,ILI9486_NGAMCTRL);
    data[0] = 0x0F;
    data[1] = 0x32;
    data[2] = 0x2E;
    data[3] = 0x0B;
    data[4] = 0x0D;
    data[5] = 0x05;
    data[6] = 0x47;
    data[7] = 0x75;
    data[8] = 0x37;
    data[9] = 0x06;
    data[10] = 0x10;
    data[11] = 0x03;
    data[12] = 0x24;
    data[13] = 0x20;
    data[14] = 0x00;
    ili9486_write_array(ILI9486_DATA_MODE,data,15);   
    
	ili9486_write(ILI9486_CMD_MODE,ILI9486_PIXSET);
	data[0] = 0x55;
	ili9486_write_array(ILI9486_DATA_MODE,data,1);

    ili9486_write(ILI9486_CMD_MODE,ILI9486_SLPOUT);
	
	ili9486_write(ILI9486_CMD_MODE,ILI9486_MADCTL); // 0x36
	data[0] = 0x28;
	ili9486_write_array(ILI9486_DATA_MODE,data,1);
	
    sleep_ms(120);
	
    ili9486_write(ILI9486_CMD_MODE,ILI9486_DISPON);



}
void ili9486_rotate(int degrees,bool bgr){
	printf("Rotate %d\n",degrees);
    uint8_t color_order = MADCTL_RGB;

    if(bgr) color_order = MADCTL_BGR;

    switch(degrees){
        case 270:
            ili9486_write(ILI9486_DATA_MODE,MADCTL_MV | color_order);
            break;
        case 180:
            ili9486_write(ILI9486_DATA_MODE,MADCTL_MY | color_order);
            break;
        case 90:
            ili9486_write(ILI9486_DATA_MODE,MADCTL_MX | MADCTL_MY | MADCTL_MV | color_order);
            break;
        case 0:
            /* fall-through */
        default:
            ili9486_write(ILI9486_DATA_MODE,MADCTL_MX | color_order);
            break;
        
    }
}
void ili9486_write(int mode,uint8_t data){
	//Enable chip
	gpiod_line_set_value(lcs,0);
	// Set D/CX to mode
	gpiod_line_set_value(lrs,mode);

	gpiod_line_set_value(lrd,1);
	gpiod_line_set_value(lwr,0);

        //Write data to 8-bit bus
	//digitalWriteByte(data);
	int bits[] = {data&0x80,data &0x40,data &0x20,data &0x10,data &0x08,data &0x04,data &0x02,data &0x01};
	gpiod_line_set_value_bulk(&ldata,bits);
	usleep(5000);
	gpiod_line_set_value(lwr,1);
	usleep(5000);
	gpiod_line_set_value(lcs,1);
	int lowbits[] = {0,0,0,0,0,0,0,0};
	gpiod_line_set_value_bulk(&ldata,lowbits);
	usleep(5000);
}
void ili9486_write_array(int mode,uint8_t *data, uint16_t len){
    for(uint16_t idx = 0; idx < len; idx++){
        ili9486_write(mode,data[idx]);
    }
}
void ili9486_write_color(fbcolor16_t *color){
	uint8_t msb = (color->ch.red << 3) & ((color->ch.green & 0x20) << 2) & ((color->ch.green & 0x10) << 1) & (color->ch.green & 0x08);
	uint8_t lsb = ((color->ch.green & 0x04) << 5) & ((color->ch.green & 0x02) << 4) & ((color->ch.green & 0x01) << 3) & color->ch.blue;
	ili9486_write(ILI9486_DATA_MODE,msb);
	ili9486_write(ILI9486_DATA_MODE,lsb);
	
}
void ili9486_write_color_array(fbcolor16_t* color_array,uint16_t len){
	for(int idx = 0; idx < len; idx++){
		ili9486_write_color(&color_array[idx]);
	}
}
void sleep_ms(int delay){
    if(delay >= 1000)
        sleep(delay/1000);
    usleep((delay%1000)*1000);
}


