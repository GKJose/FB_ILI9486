#include <FB_ILI9341.h>

#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <wiringPi.h>

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

void main(void){
    fbfd = open(FBDEV_PATH,O_RD);
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

    gpio_init();
    ili9486_init();
    ili9486_rotate(180,false);

    int32_t act_x1 = 0;
    int32_t act_y1 = 0;
    int32_t act_x2 = ILI9486_TFTWIDTH > (uint32_t)vinfo.xres - 1 ?  (uint32_t)vinfo.xres - 1:ILI9486_TFTWIDTH;
    int32_t act_y2 = ILI9486_TFTHEIGHT >     int32_t act_y2 = area->y2 > (int32_t)vinfo.yres - 1 ? (int32_t)vinfo.yres - 1 : ILI9486_TFTHEIGHT;
    long int location = 0;
    long int byte_location = 0;
    unsigned char bit_location = 0;
    uint32_t screen_data_size = sizeof(uint16_t) * ((act_y2 - act_y1)*(act_x2-act_x1));
    fbcolor16_t * screen_data = malloc(screen_data_size);

    while(true){


        
        /*16 bit per pixel*/
        if(vinfo.bits_per_pixel == 16) {
            //Copy framebuffer into pointer array of color pixels
            uint16_t * fbp16 = (uint16_t *)fbp;
            int32_t y;
            for(y = act_y1; y <= act_y2; y++) {
                location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 2;
                memcpy(screen_data[location],&fbp16[location],(act_x2-act_x1)*2);             
            }
            uint8_t data[4];
            int32_t len = (act_x2 - act_x1 +1) * 2;
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
                 ili9486_write_array(ILI9341_DATA_MODE, (uint8_t *)screen_data[pixIdx], len);
                 pixIdx += w;
             }


        }else {
            /*Not supported bit per pixel*/
        }



    }
    close(fbfd);



}
void gpio_init(void){
    
}
void ili9486_init(void){
    uint8_t data[15];

    /* hardware reset */
    digitalWrite(LCD_RST_PIN,HIGH);
    usleep(50);
    digitalWrite(LCD_RST_PIN,LOW);

    /* software reset */
    ili9486_write(ILI9486_CMD_MODE,ILI9486_SWRESET):
    sleep_ms(5);
    ili9486_write(ILI9486_CMD_MODE,ILI9486_DISPOFF);

    /*startup sequence */
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

    
    ili9486_write(ILI9486_CMD_MODE,ILI9486_MADCTL);
    data[0] = 0x08;
    ili9486_write_array(ILI9486_DATA_MODE,data,1);

    ili9486_write(ILI9486_CMD_MODE,0xB4);
    data[0] = 0x00;
    ili9486_write_array(ILI9486_DATA_MODE,data,1);

    ili9486_write(ILI9486_CMD_MODE,ILI9486_DISCTRL);
    data[0] = 0x02;
    data[1] = 0x22;
    ili9486_write_array(ILI9486_DATA_MODE,data,2);

    ili9486_write(ILI9486_CMD_MODE,ILI9486_PWCTRL2);
    data[0] = 0x41;
    ili9486_write_array(ILI9486_DATA_MODE,data,1);

    ili9486_write(ILI9486_CMD_MODE,ILI9486_VMCTRL);
    data[0] = 0x00;
    data[1] = 0x18;
    ili9486_write_array(ILI9486_DATA_MODE,data,2);

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

    ili9486_write(ILI9486_CMD_MODE,ILI9486_SLPOUT);
    sleep_ms(120);
    ili9486_write(ILI9486_CMD_MODE,ILI9486_DISPON);



}
void ili9486_rotate(int degrees,bool bgr){
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
    if(mode == ILI9486_CMD_MODE){

    }else if(mode == ILI9486_DATA_MODE){

    }

        //Write data to 8-bit bus
    digitalWrite(LCD_D7_PIN,data & 0x80);
    digitalWrite(LCD_D6_PIN,data & 0x40);
    digitalWrite(LCD_D5_PIN,data & 0x20);
    digitalWrite(LCD_D4_PIN,data & 0x10);
    digitalWrite(LCD_D3_PIN,data & 0x08);
    digitalWrite(LCD_D2_PIN,data & 0x04);
    digitalWrite(LCD_D1_PIN,data & 0x02);
    digitalWrite(LCD_D0_PIN,data & 0x01);
}
void ili9486_write_array(int mode,uint8_t *data, uint16_t len){
    for(uint16_t idx = 0; idx < len; idx++){
        ili9486_write(mode,data[idx]);
    }
}

void sleep_ms(int delay){
    if(delay >= 1000)
        sleep(delay/1000);
    usleep((delay%1000)*1000);
}


