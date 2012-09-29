/* LGV-5480TVR video4linux driver
 *
 * A lot of this copied from Itai Nahshon's PixelView Combo TV Pro driver.
 * Memory allocation stuff take from Michel Lanners' PlanB v4l driver.
 */ 
 
/*  
    Copyright (C) 2003 Angelo Cano

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/videodev.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/wrapper.h>
#include <linux/delay.h>

#include "bitops.c"
#include "misc.h"

MODULE_AUTHOR ("Angelo Cano <angelo_cano@fastmail.fm>");
MODULE_DESCRIPTION ("lgv-5480tvr v4l device driver");
MODULE_LICENSE ("GPL");

#define pdebug(fmt, args...)  if (debug) printk("lgv_5480tvr: " fmt, ## args)
#define MIN(x,y)  ((x) < (y) ? (x) : (y))
#define MAX(x,y)  ((x) > (y) ? (x) : (y))

#define VID_HARDWARE_LGV_5480TVR 0x66666
#define VPX3225D_ID 0x7230

int lgv_5480tvr_open (struct video_device *dev, int mode);
void lgv_5480tvr_close (struct video_device *dev);
int lgv_5480tvr_ioctl (struct video_device *dev, unsigned int cmd,
                       void *arg);
int lgv_5480tvr_mmap (struct video_device *dev, const char *adr,
                      unsigned long size);
int alloc_mmap_buffer (void);
void free_mmap_buffer (void);
void gd5480_stream_engine_off (void);
void gd5480_disable_external_interrupts (void);
void gd5480_stream_engine_begin (void);

void px0002_on (void);
void px0002_set_source (void);
void px0002_off (void);

uint8_t **mmapBufferPages = NULL;
int mmapBufferNumPages = 0;

int debug = 1;
int users = 0;
unsigned long totalInterrupts = 0;
unsigned long ourInterrupts = 0;
int Capturing = 0;
DECLARE_WAIT_QUEUE_HEAD(capture_wait);


enum { Transferring = 0, TransferDone, TransferError, TransferOff, 
       TransferNeed
};

int streamEngineStatus = TransferOff;
unsigned long maxWidth = 0;
unsigned long maxHeight = 0;
int capW = -1;
int capH = -1;
int bpp = -1;
int displayWidth = 0;
int displayHeight = 0;
int muted = 0;

unsigned char cr_list[] = {
    0x1d, 0x31, 0x32, 0x33, 0x34,
    0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x3b, 0x3c, 0x3d, 0x3e,
    0x4e, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x5b, 0x5c, 0x5d,
    0x5e, 0x5f, 0x60, 0x61, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x6c, 0x6d, 0x6f,
    0x7e,

    0x1E, 0x1F, 0x3F, 0x62, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 
    0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7F,
};

unsigned char gr_list[] = {
    0x0c, 0x0d, 0x17,

    0x1C, 
};

unsigned char cr_val[sizeof(cr_list)];
unsigned char gr_val[sizeof(gr_list)];

unsigned long itt_max_width = 640;
unsigned long itt_max_height = 480;

struct lgv_5480tvr_data {
    struct pci_dev *dev;
    unsigned char *mmio_base;
    unsigned char *fb_base;
    int offset;
    int norm;
    int channel;
    unsigned long freq;
    int video_mode;
    int initialized;
    int clock_doubling;
    int ram_size;
    int capW;
    int capH;
    unsigned int page_table_start_addr;
    int num_page_tables;

    int stream_width;
    int stream_height;
};

struct lgv_5480tvr_data *lgv_5480tvr = NULL;

struct video_device lgv_5480tvr_device = {
    THIS_MODULE,
    "lgv_5480tvr",
    VID_TYPE_CAPTURE | VID_TYPE_TUNER,
    VID_HARDWARE_LGV_5480TVR,
    lgv_5480tvr_open,
    lgv_5480tvr_close,
    NULL,  /* lgv_5480tvr_read, */
    NULL,  /* lgv_5480tvr_write */
    NULL,  /* lgv_5480tvr_poll, */
    lgv_5480tvr_ioctl,
    lgv_5480tvr_mmap,
    /* the rest are NULL */
};

struct video_tuner vidtuner = {
    0,
    "TV",
    16 * 55.25,
    16 * 855.25,
    VIDEO_TUNER_NTSC,
    VIDEO_MODE_NTSC,
    0xffff
};

//FIXME
#if 0
struct {
    double picture_if;
    double f_low;
    double f_low_mid;
    double f_mid_high;
    double f_high;
} tuner_types[] = {
    { 0.00,   0.00,   0.00,   0.00,   0.00}, /* NONE   */
    {38.90,  48.25, 170.00, 450.00, 855.25}, /* 1216 CCIR B/G */
    {42.75,  55.25, 160.00, 454.00, 801.25}, /* 1236 RTMA M/N */
    {38.90,  45.75, 170.00, 450.00, 855.25}, /* 1246 CCIR I   */
    {45.75,  55.25, 169.25, 457.25, 855.25}, /* samsung TCPN9082DA27C NTSC only */
};
#endif

struct {
    unsigned short picture_if;
    unsigned short f_low;
    unsigned short f_low_mid;
    unsigned short f_mid_high;
    unsigned short f_high;
} tuner_types [] = {
    { 16 * 45.75, 16 * 55.25, 16 * 169.25, 16 * 457.25, 16 * 855.25 }
};

unsigned long current_freq;
int tuner_i2c_addr = 0xc0;
int tunerType = 0;  //FIXME

struct video_picture vidpicture;
struct video_buffer vidbuffer;
struct video_window vidwin;
struct video_mmap   vidmmap;

/* i2c */
int gd5480_i2cget (void);
int gd5480_i2cgetclk (void);
void gd5480_i2cset (int clk, int data);

//#define I2CDEBUG

#ifdef I2CDEBUG
  #define I2C_DEBUG(x)  printk(x)
#else
  #define I2C_DEBUG(x)  {}
#endif

#define I2C_GET()           (gd5480_i2cget())
#define I2C_GETCLK()        (gd5480_i2cgetclk())
#define I2C_SET(clk,data)   do { udelay(10); \
                                 gd5480_i2cset(clk, data); \
                                 if(clk) { while(!I2C_GETCLK()); } \
                            } while(0)

/* Following is modified from linux-2.1.99/drivers/char/i2c.c */

void i2c_start (void)
{
    I2C_SET (1, 1);
    I2C_SET (1, 0);
    I2C_SET (0, 0);
    I2C_DEBUG ("i2c: <");
}

void i2c_stop (void)
{
    I2C_SET (0, 0);
    I2C_SET (1, 0);
    I2C_SET (1, 1);
    I2C_DEBUG (">\n");
}

void i2c_one (void)
{
    I2C_SET (0, 1);
    I2C_SET (1, 1);
    I2C_SET (0, 1);
}

void i2c_zero (void)
{
    I2C_SET (0, 0);
    I2C_SET (1, 0);
    I2C_SET (0, 0);
}

int i2c_ack (void)
{
    int ack;
    
    I2C_SET (0, 1);
    I2C_SET (1, 1);
    ack = I2C_GET ();
    I2C_SET (0, 1);
    return ack;
}

int i2c_sendbyte (unsigned char data, int wait_for_ack)
{
    int i, ack;
    
    I2C_SET (0, 0);
    for (i = 7;  i >= 0;  i--)
        (data & (1 << i)) ? i2c_one() : i2c_zero();
    if (wait_for_ack)
        udelay (wait_for_ack);
    ack = i2c_ack ();
#ifdef I2CDEBUG
    printk ("%02x%c ", (int) data, ack ? '-' : '+');
#endif
    return ack;
}

unsigned char i2c_readbyte (int last)
{
    int i;
    unsigned char data = 0;

    I2C_SET (0, 1);
    for (i = 7;  i >= 0;  i--) {
        I2C_SET (1, 1);
        if (I2C_GET())
            data |= (1 << i);
        I2C_SET (0, 1);
    }
    last ? i2c_one() : i2c_zero();
#ifdef I2CDEBUG
    printk ("=%02x%c ", (int) data, last ? '-' :'+');
#endif
    return data;
}

void i2c_write16 (unsigned addr, unsigned reg, unsigned data) 
{
    i2c_start ();
    i2c_sendbyte (addr, 0);
    i2c_sendbyte (reg, 0);
    i2c_sendbyte ((data >> 8) & 0xff, 0);
    i2c_sendbyte (data & 0xff, 0);
    i2c_stop ();
}

unsigned i2c_read16 (unsigned addr, unsigned reg) 
{
    unsigned res1, res2;

    i2c_start ();
    i2c_sendbyte (addr, 0);
    i2c_sendbyte (reg, 0);
    i2c_start ();
    i2c_sendbyte (addr | 1, 0);
    res1 = i2c_readbyte (0);
    res2 = i2c_readbyte (1);
    i2c_stop ();

    return (res1 << 8) | res2;
}

void i2c_write8 (unsigned addr, unsigned reg, unsigned data) 
{
    i2c_start ();
    i2c_sendbyte (addr, 0);
    i2c_sendbyte (reg, 0);
    i2c_sendbyte (data, 0);
    i2c_stop ();
}

int i2c_read8 (unsigned addr, unsigned reg) 
{
    unsigned res;

    i2c_start ();
    if (i2c_sendbyte(addr, 0))
        return -1;
    i2c_sendbyte (reg, 0);
    i2c_start ();
    i2c_sendbyte (addr | 1, 0);
    res = i2c_readbyte (1);
    i2c_stop ();

    return res;
}

int i2c_read (uint8_t addr)
{
    int ret;
    
    i2c_start ();
    if (i2c_sendbyte(addr, 0))
        return -1;
    ret = i2c_readbyte (1);
    i2c_stop ();
    return ret;
}

int i2c_write (uint8_t addr, uint8_t *data, int len)
{
    int ack = -1;

    i2c_start ();
    if (i2c_sendbyte(addr, 0))
        return -1;
    while (len > 0) {
        ack = i2c_sendbyte (*data, 0);
        ++data;
        --len;
    }
    i2c_stop ();
    return ack ? -1 : 0;
}

/**  end i2c **/

void write_reg_pair (uint8_t data,
                     uint8_t reg,
                     unsigned long io_base_offset)
{
    writeb (reg, lgv_5480tvr->mmio_base + io_base_offset);
    writeb (data, lgv_5480tvr->mmio_base + io_base_offset + 1);
}

uint8_t read_reg_pair (uint8_t reg, unsigned long io_base_offset)
{
    uint8_t data;

    writeb (reg, lgv_5480tvr->mmio_base + io_base_offset);
    data = readb (lgv_5480tvr->mmio_base + io_base_offset + 1);

    return data;
}

uint8_t cr_read (uint8_t reg)
{
    return read_reg_pair(reg, 0x14);
}

void cr_write (uint8_t reg, uint8_t data)
{
    write_reg_pair (data, reg, 0x14);
}

uint8_t gr_read (uint8_t reg)
{
    return read_reg_pair(reg, 0x0e);
}

void gr_write (uint8_t reg, uint8_t data)
{
    write_reg_pair (data, reg, 0x0e);
}

uint8_t sr_read (uint8_t reg)
{
    return read_reg_pair(reg, 0x04);
}

void sr_write (uint8_t reg, uint8_t data)
{
    write_reg_pair (data, reg, 0x04);
}

void cr_write_bits (uint8_t reg, int bitStart, int bitEnd, int value)
{
    uint8_t data;

    data = cr_read (reg);
    data = u8WriteBits (data, bitStart, bitEnd, value);
    cr_write (reg, data);
}

void cr_write_bit (uint8_t reg, int bitNum, int value)
{
    uint8_t data;

    data = cr_read (reg);
    data = u8WriteBit (data, bitNum, value);
    cr_write (reg, data);
}

uint8_t cr_read_bits (uint8_t reg, int bitStart, int bitEnd)
{
    return u8ReadBits(cr_read(reg), bitStart, bitEnd);
}

uint8_t cr_read_bit (uint8_t reg, int bitNum)
{
    return u8ReadBit(cr_read(reg), bitNum);
}

void gr_write_bits (uint8_t reg, int bitStart, int bitEnd, int value)
{
    uint8_t data;

    data = gr_read (reg);
    data = u8WriteBits (data, bitStart, bitEnd, value);
    gr_write (reg, data);
}

void gr_write_bit (uint8_t reg, int bitNum, int value)
{
    uint8_t data;

    data = gr_read (reg);
    data = u8WriteBit (data, bitNum, value);
    gr_write (reg, data);
}

unsigned char read_HDR (void)
{
    unsigned char data;

    data = lgv_5480tvr->mmio_base[6];
    data = lgv_5480tvr->mmio_base[6];
    data = lgv_5480tvr->mmio_base[6];
    data = lgv_5480tvr->mmio_base[6];
    data = lgv_5480tvr->mmio_base[6];

    return data;
}

/* gd5480 I2C interface */
int gd5480_i2cget (void)
{
    return (sr_read(0x08) & 0x80);
}

int gd5480_i2cgetclk (void)
{
    return (sr_read(0x08) & 0x04);
}

void gd5480_i2cset (int clk, int data)
{
    sr_write (0x08, 0xfc | (data << 1) | clk);
}

/* vpx3225d i2c interface */
int ittI2cAddr = 0x86;
unsigned decoderId = 0x0;
uint8_t fpReadReg   = 0x26;
uint8_t fpWriteReg  = 0x27;
uint8_t fpDataReg   = 0x28;
uint8_t fpStatusReg = 0x29;

void itt_write_direct (uint8_t reg, uint8_t data)
{
    i2c_write8 (ittI2cAddr, reg, data);
}

int itt_read_direct (uint8_t reg)
{
    return i2c_read8(ittI2cAddr, reg);
}

void itt_write_indirect (uint16_t reg, uint16_t data)
{
    uint8_t status;
    status = i2c_read8 (ittI2cAddr, fpStatusReg);
    i2c_write16 (ittI2cAddr, fpWriteReg, reg);
    status = i2c_read8 (ittI2cAddr, fpStatusReg);
    i2c_write16 (ittI2cAddr, fpDataReg, data);
}


uint16_t itt_read_indirect (uint16_t reg)
{
    uint8_t status;
    uint16_t data;
    
    status = i2c_read8 (ittI2cAddr, fpStatusReg);
    i2c_write16 (ittI2cAddr, fpReadReg, reg);
    status = i2c_read8 (ittI2cAddr, fpStatusReg);
    data = i2c_read16 (ittI2cAddr, fpDataReg);

    return data;
}

/**************************************************/

void gd5480_save (void)
{
    int i;

    for (i = 0;  i < sizeof(cr_list);  i++)
        cr_val[i] = cr_read (cr_list[i]);

    for (i = 0;  i < sizeof(gr_list);  i++)
        gr_val[i] = gr_read (gr_list[i]);
}

void gd5480_restore (void)
{
    int i;

    for (i = 0;  i < sizeof(cr_list);  i++)
        cr_write (cr_list[i], cr_val[i]);

#if 1
    for (i = 0;  i < sizeof(gr_list);  i++)
        gr_write (gr_list[i], gr_val[i]);
#endif
}

int gd5480_uses_gd5480_double_buffering (void)
{
    return cr_read_bit(0x66, Bit6);
}

void gd5480_set_video_capture_buffers_1_and_2_offset (uint32_t addr)
{
    /*  |    10     |     9:2     |  1:0  |
     *  |  CR66[4]  |  CR67[7:0]  |   0   |
     */
    if (u32ReadBits(addr, Bit11, Bit31)) {
        pdebug ("%s():  addr[31:12] != 0  0x%x\n", __FUNCTION__, addr);
    }
    if (u32ReadBits(addr, Bit0, Bit1)) {
        pdebug ("%s():  addr[2:0] != 0  0x%x\n", __FUNCTION__, addr);
    }
    
    cr_write_bit (0x66, Bit4, u32ReadBit(addr, Bit10));
    cr_write_bits (0x67, Bit0, Bit7, u32ReadBits(addr, Bit2, Bit9));
}

void lgv_5480tvr_set_capture_colorspace (int cspace)
{
    uint8_t capture_format;

    switch (cspace) {
    case VIDEO_PALETTE_RGB565:
        capture_format = 0x1;  /* 001 b */
        break;
    case VIDEO_PALETTE_RGB555:
        capture_format = 0x1;  /* 001 b */
        break;
    case VIDEO_PALETTE_YUV422:
        capture_format = 0x0;  /* 000 b */
        break;
    case VIDEO_PALETTE_YUV420P:
        capture_format = 0x6;  /* 110 b */
        break;
#if 0
    case ACCUPAK:
        capture_format = 0x2;  /* 010 b */
        break;
#endif
    default:
        printk ("%s(): FIXME color-space %d not handled\n", __FUNCTION__, 
                cspace);
        return;
    }
    cr_write_bits (0x51, Bit0, Bit2, capture_format);
}

void itt_set_window (int w, int h)
{
    int lines, begin;

    switch (lgv_5480tvr->video_mode) {
    case VIDEO_MODE_NTSC:
        //    case VIDEO_MODE_PALM:  //FIXME PALM ??
        //lines = 525;
        //begin = 21;
        lines = 525;
        begin = 21;
        break;
    case VIDEO_MODE_PAL:
    case VIDEO_MODE_SECAM:
        lines = 625;
        begin = 22;
        break;
    default:
        lines = begin = 0;
        printk ("%s() video mode %d not supported\n", __FUNCTION__,
                lgv_5480tvr->video_mode);
        return;
    }

    if (h > lines - 2 * begin)
        h = lines - 2 * begin;
    if (w > itt_max_width)
        w = itt_max_width;

    itt_write_indirect (0x120, begin);  /* window1 ver begin */
    itt_write_indirect (0x121, lines / 2);  /* w1 ver lines in */
    itt_write_indirect (0x122, h / 2);  /* w1 ver lines out */
    itt_write_indirect (0x123, 0);  /* w1 horiz begin */
    itt_write_indirect (0x124, w); /* w1 horiz length */
    itt_write_indirect (0x125, w);  /* w1 num of pixels */
    //itt_write_indirect (0x140, 0x060);
    itt_write_indirect (0x140, 0x020);  /* latch window 1 */
}

void itt_set_colorspace (int cspace)
{
    return;
}

void itt_set_video_mode (int vmode)
{
    uint8_t val;

    /*FIXME other modes */
    switch (vmode) {
    case VIDEO_MODE_NTSC:
    default:
        val = 0x001;
        break;
    }
    itt_write_indirect (0x150, val);
}

void itt_set_source (int source)
{
    unsigned data;

    data = itt_read_indirect (0x20);

#if 0
    switch (source) {
    case TUNER:
        itt_write_indirect (0x21, 0x001);
        itt_write_indirect (0x20, data & 0xfbf);
        break;
    case VCR:  // svhs
        itt_write_indirect (0x21, 0x406);
        itt_write_indirect (0x20, data | 0x40);
        break;
    }
#endif

    //FIXME using only tuner
    itt_write_indirect (0x21, 0x001);
    itt_write_indirect (0x20, data & 0xfbf);
}


void gd5480_set_capture_buffer1_address (uint32_t addr)
{
    /*  |    21:18    |    17:10    |     9:3     |    2      |     1:0     |
     *  |  CR58[3:0]  |  CR5A[7:0]  |  CR59[7:1]  |  CR59[0]  |  CR5D[3:2]  |
     *
     *                                              ^^^^^^^^     ^^^^^^^^^
     *                   only in display case, otherwise |  2  |  1:0  |     
     *                                                   |  0  |   0   |
     */
    if (u32ReadBits(addr, Bit22, Bit31)) {
        pdebug ("%s()  addr[31:22] != 0  0x%0x\n", __FUNCTION__, addr);
    }
    if (u32ReadBits(addr, Bit0, Bit2)) {
        pdebug ("%s()  addr[2:0] != 0  0x%0x\n", __FUNCTION__, addr);
    }
    cr_write_bits (0x58, Bit0, Bit3, u32ReadBits(addr, Bit18, Bit21));
    cr_write_bits (0x5A, Bit0, Bit7, u32ReadBits(addr, Bit10, Bit17));
    cr_write_bits (0x59, Bit1, Bit7, u32ReadBits(addr, Bit3, Bit9));
    cr_write_bit  (0x59, Bit0,       u32ReadBit (addr, Bit2));
    cr_write_bits (0x5D, Bit2, Bit3, u32ReadBits(addr, Bit0, Bit1));
}

void gd5480_set_capture_buffer2_address (uint32_t addr)
{
    /*  |    21:18    |    17:10    |     9:2     |  1:0  |
     *  |  CR66[3:0]  |  CR65[7:0]  |  CR64[7:0]  |   0   |
     */
    if (u32ReadBits(addr, Bit22, Bit31)) {
        pdebug ("%s()  addr[31:22] != 0  0x%0x\n", __FUNCTION__, addr);
    }
    if (u32ReadBits(addr, Bit0, Bit1)) {
        pdebug ("%s()  addr[1:0] != 0  0x%0x\n", __FUNCTION__, addr);
    }
    cr_write_bits (0x66, Bit0, Bit3, u32ReadBits(addr, Bit18, Bit21));
    cr_write_bits (0x65, Bit0, Bit7, u32ReadBits(addr, Bit10, Bit17));
    cr_write_bits (0x64, Bit0, Bit7, u32ReadBits(addr, Bit2, Bit9));
}

void gd5480_set_capture_dimensions (int x, int y, int width, int height)
{
    pdebug ("%s (%d, %d, %d, %d)\n", __FUNCTION__, x, y, width, height);

    if (lgv_5480tvr->clock_doubling) {
        pdebug ("%s(): using clock doubling\n", __FUNCTION__);
        width /= 2;
    }
    x *= (bpp / 8);
    width *= (bpp / 8);

    /* Video Capture Horizontal Delay
     *  |    10:9     |    8:1      |   0   |
     *  |  CR5F[1:0]  |  CR54[7:0]  |   0   |
     */
    if (u32ReadBits(x, Bit11, Bit31)) {
        pdebug ("%s():  x[31:11] != 0  %d\n", __FUNCTION__, x);
    }
    if (u32ReadBit(x, Bit0)) {
        pdebug ("%s():  x[0] != 0  %d\n", __FUNCTION__, x);
    }
    cr_write_bits (0x5F, Bit0, Bit1, u32ReadBits(x, Bit9, Bit10));
    cr_write_bits (0x54, Bit0, Bit7, u32ReadBits(x, Bit1, Bit8));

    /* Video Capture Vertical Delay
     * If Closed Caption is enabled, CR56 indicates which scanline to
     * capture.  If Teletext (VBI) capture enabled, CR56 is the number of
     * lines of VBI data to capture without shrinking or clipping.
     *
     * |    8      |     7:0     |
     * |  CR58[7]  |  CR56[7:0]  |
     */
    cr_write_bit  (0x58, Bit7,       u32ReadBit (y, Bit8));
    cr_write_bits (0x56, Bit0, Bit7, u32ReadBits(y, Bit0, Bit7));

    /* Video Capture Horizontal Width
     *  |    10     |     9:2     |  1:0  |
     *  |  CR5F[2]  |  CR55[7:0]  |   0   |
     */
    cr_write_bit (0x5F, Bit2, u32ReadBit(width, Bit10));
    cr_write_bits (0x55, Bit0, Bit7, u32ReadBits(width, Bit2, Bit9));

    /* Video Capture Maximum Height
     *  |    9:8      |     7:0     |
     *  |  CR58[5:4]  |  CR57[7:0]  |
     */
    cr_write_bits (0x58, Bit4, Bit5, u32ReadBits(height, Bit8, Bit9));
    cr_write_bits (0x57, Bit0, Bit7, u32ReadBits(height, Bit0, Bit7));
}

void gd5480_capture_on (void)
{
    pdebug ("%s()\n", __FUNCTION__);
    cr_write_bit (0x51, Bit3, 1);
    Capturing = 1;
}

void gd5480_capture_off (void)
{
    pdebug ("%s()\n", __FUNCTION__);
    cr_write_bit (0x51, Bit3, 0);
    Capturing = 0;
}

void itt_on (void)
{
    pdebug ("%s()\n", __FUNCTION__);
    itt_write_direct (0xAA, 0x00);  /* full power mode */
    itt_write_direct (0xF2, 0x0f);  /* enable output */
}

void itt_off (void)
{
    pdebug ("%s()\n", __FUNCTION__);
    itt_write_direct (0xF2, 0x00);  /* disable output */
    itt_write_direct (0xAA, 0x03);  /* switch to low power mode */
}

int tuner_set_tv_frequency (int freq)
{
    unsigned int k;
    uint8_t s[4];

    //pdebug ("%s(): freq : %d\n", __FUNCTION__, freq);

    if (freq >= tuner_types[tunerType].f_low  &&
        freq <= tuner_types[tunerType].f_high)
    {
        k = freq + tuner_types[tunerType].picture_if;
        if (k > 32767) {
            printk ("%s(): huh?  k > 32767\n", __FUNCTION__);
            return 0;
        }
        s[0] = (k >> 8) & 0x7f;
        s[1] = k & 0xff;
        s[2] = 0x8e;
        if (freq < tuner_types[tunerType].f_low_mid)
            s[3] = 0xa0;
        else if (freq > tuner_types[tunerType].f_mid_high)
            s[3] = 0x30;
        else
            s[3] = 0x90;

        i2c_write (tuner_i2c_addr, s, 4);
        for (k = 0;  k < 20;  k++) {
            if (i2c_read(tuner_i2c_addr | 1) & 0x40)
                break;
            udelay (20);
        }
        current_freq = freq;
        return 1;
    }
    else {
        printk ("%s() invalid frequency %d\n", __FUNCTION__, freq);
    }

    return 0;
}

void gd5480_vport_clear_interrupt (void)
{
    cr_write_bit (0x62, Bit0, 1);

    cr_write_bit (0x3F, Bit1, 0);
    cr_write_bit (0x3F, Bit1, 1);
}

#if 1
void gd5480_vport_enable_interrupt (void)
{
    cr_write_bit (0x3F, Bit1, 1);
}
#endif

void gd5480_vport_disable_interrupt (void)
{
    cr_write_bit (0x3F, Bit1, 0);
}

int gd5480_vport_check_interrupt (void)
{
    return cr_read_bit(0x62, Bit0);
}

/* returns 0 if no interrupt occured, otherwise returns buff number */
int gd5480_stream_engine_check_interrupt (void)
{
    return cr_read_bits(0x62, Bit2, Bit3);
}

void gd5480_stream_engine_clear_interrupt (void)
{
    uint8_t data;

    data = cr_read (0x62);
    /* [3:2] get written as is, clearing buffer used bit */
    //    data = u8WriteBits (data, Bit2, Bit3, 0x3  /* 11 binary */ );
    data = u8WriteBit (data, Bit1, 1);  /* overrun */
    cr_write (0x62, data);
}

int gd5480_stream_engine_check_dma_error (void)
{
    uint8_t data, bufTransfered, overrun;

    data = cr_read (0x62);
    bufTransfered = u8ReadBits (data, Bit2, Bit3);
    overrun = u8ReadBit (data, Bit1);

    if (overrun  &&  bufTransfered == 0)
        return 1;
    else
        return 0;
}

void gd5480_stream_engine_clear_dma_error (void)
{
#if 0
    uint8_t data;

    data = cr_read (0x62);

#if 0
    /* buffer ack bits */
    data = u8WriteBits (data, Bit2, Bit3, 0x3  /* 11 binary */ );
#endif

#if 0
    data = u8WriteBits (data, Bit5, Bit7, 0x00);
    data = u8WriteBit (data, Bit4, 0);  /* BitBLT irq complete status */
    data = u8WriteBit (data, Bit0, 0);  /* VPORT VREF irq status */
#endif

    data = u8WriteBit (data, Bit1, 1);  /* ack overrun */
    cr_write (0x62, data);
#endif
}


/******************* pxUnknown *************************/
/* px0002 simply taken from Itai Nahshon's PixelView driver.  I don't
 * understand what it does.
 */
int px0002_i2c_addr = 0x4e;

void px0002_on (void)
{
    pdebug ("%s()\n", __FUNCTION__);
#define w2(reg,data)  i2c_write8(px0002_i2c_addr, reg, data)

    w2 (0x4, 0x32);
    w2 (0x3, 0x3a);
    w2 (0x3, 0x38);
    w2 (0x3, 0x38);
    w2 (0x4, 0x36);
    w2 (0x3, 0x38);
    w2 (0x3, 0x38);
    w2 (0x3, 0x38);
    w2 (0x4, 0x32);
    w2 (0x2, 0x0d);
    w2 (0x1, 0xa0);
    w2 (0x2, 0x0d);
    w2 (0x3, 0x0);
    w2 (0x2, 0x3d);
    w2 (0x3, 0x8);
    w2 (0x4, 0x3b);
    w2 (0x3, 0x38);
    w2 (0x3, 0x3a);
    w2 (0x3, 0x3a);
    w2 (0x04, 0x3b);
    w2 (0x4, 0x33);
    w2 (0x3, 0x32);
    w2 (0x4, 0x37);
}

//FIXME doesn't work
//void px0002_set_source (XtvVideoSource vs)
void px0002_set_source (void)
{
    int val;

    pdebug ("%s()\n", __FUNCTION__);
    val = i2c_read8 (px0002_i2c_addr, 0x04);
    val &= ~0x08;
#if 0
    switch (vs) {
    case TUNER:
        break;
    case VCR:
        val |= 0x08;
        break;
    }
#endif
    i2c_write8 (px0002_i2c_addr, 0x04, val);
}

void px0002_off (void)
{
    int val;
    
    pdebug ("%s()\n", __FUNCTION__);
    val = i2c_read8 (px0002_i2c_addr, 0x04);
    val &= ~0x04;
    i2c_write8 (px0002_i2c_addr, 0x04, val);
}

unsigned long int vportInterrupts = 0;
unsigned long int streamTransferInterrupts = 0;
unsigned long int dmaErrorInterrupts = 0;
unsigned long int dmaErrorTimeoutCount = 0;
enum { unknown = 0, vport, dmaError, streamTransfer };


unsigned int odd = 0;
unsigned int even = 0;
unsigned int buf1 = 0;
unsigned int buf2 = 0;
unsigned int buf3 = 0;

void lgv_5480tvr_irq_handler (int irq, void *dev_id, struct pt_regs *regs)
{
    uint8_t cr62, cr62New;
    uint8_t cr3F;
    uint8_t cr6F;

    totalInterrupts++;
    if (dev_id != lgv_5480tvr)  /* not for us */
        return;

    ourInterrupts++;

#if 0
    /*FIXME just testing interrupts */
    if (totalInterrupts >= 1800) {
        /* shutdown */
        cr_write (0x7F, 0);
        cr_write (0x62, 0xff);
        gd5480_vport_disable_interrupt ();
        gd5480_disable_external_interrupts ();
    }
#endif
    
    cr62 = cr_read (0x62);
    cr62New = 0x00;

    cr3F = cr_read (0x3F);
    cr6F = cr_read (0x6F);

    /* log interrupts */
    if (u8ReadBits(cr62, Bit2, Bit3)) {  /* se transfer complete */
        streamTransferInterrupts++;
        /* ack buffer complete */
        cr62New = u8WriteBits (cr62New, Bit2, Bit3, 0x3);
    }
    if (u8ReadBit(cr62, Bit1)) {  /* se overrun */
        dmaErrorInterrupts++;
        pdebug ("%s():  stream engine overrun\n", __FUNCTION__);
        cr62New = u8WriteBit (cr62New, Bit1, 1);  /* ack */
    }
    if (u8ReadBit(cr62, Bit0)) {  /* vport interrupt */
        vportInterrupts++;
        cr62New = u8WriteBit (cr62New, Bit0, 1);  /* ack */
        cr_write_bit (0x3F, Bit1, 0);
        cr_write_bit (0x3F, Bit1, 1);

        /* field id */
        if (u8ReadBit(cr3F, Bit6))
            odd++;
        else
            even++;

        /* which capture buffer is being used */
        if (gd5480_uses_gd5480_double_buffering()) {
            /*
            pdebug ("%s():  whoops, using gd5480 style double buffering\n",
                    __FUNCTION__);
            */
            switch (u8ReadBits(cr6F, Bit2, Bit3)) {
            case 1:
                buf1++;
                break;
            case 2:
                buf2++;
                break;
            case 3:
                buf3++;
                break;
            default:
                pdebug ("%s():  FIXME shouldn't get here %d\n", 
                        __FUNCTION__, __LINE__);
            }
        }
        else {  /* using gd5446 style double buffering */
            pdebug ("%s(): whoops, gd5446 style double buffering\n",
                    __FUNCTION__);
            switch (u8ReadBit(cr3F, Bit2)) {
            case 0:
                buf1++;
                break;
            case 1:
                buf2++;
                break;
            default:
                pdebug ("%s():  FIXME shouldn't get here %d\n",
                        __FUNCTION__, __LINE__);
            }
        }
                    

#if 0
        if (streamEngineStatus == TransferNeed  &&
            !cr_read_bit(0x3F, Bit6)  /* odd field */)
#endif
        if (streamEngineStatus == TransferNeed  &&  !u8ReadBit(cr3F, Bit6))
        //if (streamEngineStatus == TransferNeed)
        {
            gd5480_stream_engine_begin ();
        }
    }

    cr_write (0x62, cr62New);
    if (u8ReadBits(cr62, Bit2, Bit3)) {  /* se transfer complete */
        //pdebug ("ack stream\n");
#if 1
        /* reset stream engine */
        cr_write (0x7F, 0x01);
        cr_write (0x7F, 0x00);
#endif
        streamEngineStatus = TransferDone;
        wake_up_interruptible (&capture_wait);
    }
}

void gd5480_set_bus_master_page_table_start_address (uint32_t addr)
{
    /* addr: |    21:18    |    17:10    |  9:0  |
     *       |             |             |       |
     *       |  CR1F[3:0]  |  CR1E[7:0]  |   0   |
     */
    if (u32ReadBits(addr, Bit22, Bit31) != 0) {
        printk ("%s():  addr[31:22] must equal 0\n", __FUNCTION__);
    }
    /* note: bits [11:10] don't have to be zero (resolved in hardware by
     * addition with buffer page table offsets), but it makes life easier.  
     */
    if (u32ReadBits(addr, Bit0, Bit11) != 0) {
        printk ("%s():  addr[11:0] must equal 0\n", __FUNCTION__);
    }
    cr_write_bits (0x1F, Bit0, Bit3, u32ReadBits(addr, Bit18, Bit21));
    cr_write_bits (0x1E, Bit0, Bit7, u32ReadBits(addr, Bit10, Bit17));
}

void gd5480_set_stream_engine_buffer1_page_table_offset (uint32_t offset)
{
    /* offset:  |   11:10     |    9:2      |  1:0  |
     *          |             |             |       |
     *          |  CR78[7:6]  |  CR70[7:0]  |   0   |
     */
    if (u32ReadBits(offset, Bit12, Bit31) != 0) {
        printk ("%s():  offset[31:12] must equal 0\n", __FUNCTION__);
    }
    if (u32ReadBits(offset, Bit0, Bit1) != 0) {
        printk ("%s():  offset[1:0] must equal 0\n", __FUNCTION__);
    }
    cr_write_bits (0x78, Bit6, Bit7, u32ReadBits(offset, Bit10, Bit11));
    cr_write_bits (0x70, Bit0, Bit7, u32ReadBits(offset, Bit2,  Bit9));
}

void gd5480_set_stream_engine_buffer2_page_table_offset (uint32_t offset)
{
    /* offset:  |   11:10     |    9:2      |  1:0  |
     *          |             |             |       |
     *          |  CR79[7:6]  |  CR74[7:0]  |   0   |
     */
    if (u32ReadBits(offset, Bit0, Bit1) != 0) {
        printk ("%s():  offset[1:0] must equal 0\n", __FUNCTION__);
    }
    if (offset >= 4096) {
        printk ("%s():  offset must be less than 4096 bytes\n", __FUNCTION__);
    }
    cr_write_bits (0x79, Bit6, Bit7, u32ReadBits(offset, Bit10, Bit11));
    cr_write_bits (0x74, Bit0, Bit7, u32ReadBits(offset, Bit2,  Bit9));
}

void gd5480_set_stream_engine_buffer1_target_address_offset (uint32_t offset)
{
    /* offset:  |   11:10     |    9:2      |  1:0  |
     *          |             |             |       |
     *          |  CR78[5:4]  |  CR71[7:0]  |   0   |
     *
     * added to entry in Stream Engine Buffer1 Table entry to yield system
     * memory address
     */
    if (u32ReadBits(offset, Bit12, Bit31) != 0) {
        printk ("%s():  offset[31:12] must equal 0\n", __FUNCTION__);
    }
    if (u32ReadBits(offset, Bit0, Bit1) != 0) {
        printk ("%s():  offset[1:0] must equal 0\n", __FUNCTION__);
    }
    cr_write_bits (0x78, Bit4, Bit5, u32ReadBits(offset, Bit10, Bit11));
    cr_write_bits (0x71, Bit0, Bit7, u32ReadBits(offset, Bit2,  Bit9));
}

void gd5480_set_stream_engine_buffer2_target_address_offset (uint32_t offset)
{
    /* offset:  |   11:10     |    9:2      |  1:0  |     
     *          |             |             |       |  
     *          |  CR79[5:4]  |  CR75[7:0]  |   0   | 
     *                 
     * added to entry in Stream Engine Buffer2 Table entry to yield system
     * memory address        
     */
    if (u32ReadBits(offset, Bit12, Bit31) != 0) {
        printk ("%s():  offset[31:12] must equal 0\n", __FUNCTION__);
    }
    if (u32ReadBits(offset, Bit0, Bit1) != 0) {
        printk ("%s():  offset[1:0] must equal 0\n", __FUNCTION__);
    }
    cr_write_bits (0x79, Bit4, Bit5, u32ReadBits(offset, Bit10, Bit11));
    cr_write_bits (0x75, Bit0, Bit7, u32ReadBits(offset, Bit2,  Bit9));
}

void gd5480_set_stream_engine_buffer1_source_addr (uint32_t addr)
{
    /* addr:  |   21:18     |   17:10     |    9:2      |  1:0  |
     *        |             |             |             |       |
     *        |  CR78[3:0]  |  CR73[7:0]  |  CR72[7:0]  |   0   |
     */
    if (u32ReadBits(addr, Bit22, Bit31) != 0) {
        printk ("%s():  addr[31:22] must equal 0\n", __FUNCTION__);
    }
    if (u32ReadBits(addr, Bit0, Bit1) != 0) {
        printk ("%s():  addr[1:0] must equal 0\n", __FUNCTION__);
    }
    cr_write_bits (0x78, Bit0, Bit3, u32ReadBits(addr, Bit18, Bit21));
    cr_write_bits (0x73, Bit0, Bit7, u32ReadBits(addr, Bit10, Bit17));
    cr_write_bits (0x72, Bit0, Bit7, u32ReadBits(addr, Bit2,  Bit9));
}

void gd5480_set_stream_engine_buffer2_source_addr (uint32_t addr)
{
    /* addr:  |   21:18     |   17:10     |    9:2      |  1:0  | 
     *        |             |             |             |       |  
     *        |  CR79[3:0]  |  CR77[7:0]  |  CR76[7:0]  |   0   |
     */
    if (u32ReadBits(addr, Bit22, Bit31) != 0) {
        printk ("%s():  addr[31:22] must equal 0\n", __FUNCTION__);
    }
    if (u32ReadBits(addr, Bit0, Bit1) != 0) {
        printk ("%s():  addr[1:0] must equal 0\n", __FUNCTION__);
    }
    cr_write_bits (0x79, Bit0, Bit3, u32ReadBits(addr, Bit18, Bit21));
    cr_write_bits (0x77, Bit0, Bit7, u32ReadBits(addr, Bit10, Bit17));
    cr_write_bits (0x76, Bit0, Bit7, u32ReadBits(addr, Bit2,  Bit9));
}

void gd5480_set_stream_engine_source_buffer_pitch_width (uint32_t pwidth)
{
    /* pitch width:  |    10     |    9:2      |  1:0  |
     *               |           |             |       |
     *               |  CR7E[4]  |  CR7A[7:0]  |   0   |
     *
     * This is the value that is added to the source buffer address after
     * a scanline has been transfered.
     */
    if (u32ReadBits(pwidth, Bit11, Bit31) != 0) {
        printk ("%s():  pwidth[31:11] must equal 0\n", __FUNCTION__);
    }
    if (u32ReadBits(pwidth, Bit0, Bit1) != 0) {
        printk ("%s():  pwidth[1:0] must equal 0\n", __FUNCTION__);
    }
    cr_write_bit (0x7E, Bit4, u32ReadBit(pwidth, Bit10));
    cr_write_bits (0x7A, Bit0, Bit7, u32ReadBits(pwidth, Bit2, Bit9));
}

void gd5480_set_stream_engine_dest_buffer_pitch_width (uint32_t pwidth)
{
    /* pitch width:  |    10     |    9:2      |  1:0  |  
     *               |           |             |       | 
     *               |  CR7E[3]  |  CR7B[7:0]  |   0   |  
     *   
     * This is the value that is added to the destination buffer address after
     * a scanline has been transfered. 
     */
    if (u32ReadBits(pwidth, Bit11, Bit31) != 0) {
        printk ("%s():  pwidth[31:11] must equal 0\n", __FUNCTION__);
    }
    if (u32ReadBits(pwidth, Bit0, Bit1) != 0) {
        printk ("%s():  pwidth[1:0] must equal 0\n", __FUNCTION__);
    }
    cr_write_bit (0x7E, Bit3, u32ReadBit(pwidth, Bit10));
    cr_write_bits (0x7B, Bit0, Bit7, u32ReadBits(pwidth, Bit2, Bit9));
}

void gd5480_set_stream_engine_target_buffer_width (uint32_t width)
{
    /* width:  |    10     |    9:4      |  3:0  |
     *         |           |             |       |
     *         |  CR7E[2]  |  CR7C[7:2]  |   0   |
     *
     * This is the number of bytes to transfer.
     */
    if (u32ReadBits(width, Bit11, Bit31) != 0) {
        printk ("%s():  width[31:11] must equal 0\n", __FUNCTION__);
    }
    if (u32ReadBits(width, Bit0, Bit3) != 0) {
        printk ("%s():  0x%x width[3:0] must equal 0\n", __FUNCTION__, width);
    }
    cr_write_bit (0x7E, Bit2, u32ReadBit(width, Bit10));
    cr_write_bits (0x7C, Bit2, Bit7, u32ReadBits(width, Bit4, Bit9));
}

void gd5480_set_stream_engine_target_buffer_height (uint32_t height)
{
    /* height:  |    9:8      |    7:0      |
     *          |             |             |
     *          |  CR7E[1:0]  |  CR7D[7:0]  |
     *
     * This is the number of scanlines to transfer.
     */
    if (u32ReadBits(height, Bit10, Bit31) != 0) {
        printk ("%s():  height[31:10] must equal 0\n", __FUNCTION__);
    }
    cr_write_bits (0x7E, Bit0, Bit1, u32ReadBits(height, Bit8, Bit9));
    cr_write_bits (0x7D, Bit0, Bit7, u32ReadBits(height, Bit0, Bit7));
}

void gd5480_enable_external_interrupts (void)
{
    gr_write_bit (0x17, Bit2, 1);  /* enable INTA# pin */
}

void gd5480_disable_external_interrupts (void)
{
    gr_write_bit (0x17, Bit2, 0);  /* disable INTA# pin */
}

unsigned int streamEngineTransfersAcknowledged = 0;

void gd5480_stream_engine_transfer_acknowledge (void)
{
    uint8_t data;

    //pdebug ("%s():  FIXME\n", __FUNCTION__);
    streamEngineTransfersAcknowledged++;

#if 1
    data = cr_read (0x62);
    //    data = u8WriteBits (data, Bit2, Bit3, 3 /* 11 binary */);
    //data = u8WriteBit (data, Bit1, 1);
    cr_write (0x62, data);
#endif

    /*FIXME */
}


unsigned int streamEngineBeginCalls = 0;

void gd5480_stream_engine_enable_interrupts (void)
{
    uint8_t data;

    data = 0x00;
    u8WriteBit (data, Bit2, 1);
    cr_write (0x7F, data);
}

void gd5480_stream_engine_begin (void)
{
    uint8_t data;

    //pdebug ("%s():  FIXME\n", __FUNCTION__);
    streamEngineBeginCalls++;

    data = 0x00;
    data = u8WriteBit (data, Bit5, 0);  /* 1: enable yuv422p */
    /* 1: transfer continously, 0: transfer a frame and stop */
    data = u8WriteBit (data, Bit3, 0);
    /* 1: stream engine interrupt generates external interrupt if 
     * INTA# enabled 
     */
    data = u8WriteBit (data, Bit2, 1);  /* 1: enable stream eng interrupt */
    data = u8WriteBit (data, Bit1, 1);  /* 1: stream engine start */
    data = u8WriteBit (data, Bit0, 0);  /* 0: turn off reset */

    cr_write (0x7F, data);
    
    streamEngineStatus = Transferring;
}

void gd5480_stream_engine_off (void)
{
    uint8_t data;

    data = cr_read (0x7F);
    data = u8WriteBit (data, Bit3, 0);  /* single frame transfer mode */
    /* stream engine doesn't enable external interrupt */
    data = u8WriteBit (data, Bit2, 0); 
    
    data = u8WriteBit (data, Bit1, 0);  /* stop stream engine */
    cr_write (0x7F, data);

    streamEngineStatus = TransferOff;
}

int lgv_5480tvr_open (struct video_device *dev, int mode)
{
    //uint8_t data;
    int result;
    //int off;

    pdebug ("%s()\n", __FUNCTION__);
    if (users > 0) {
        printk ("device busy\n");
        return -EBUSY;
    }
    //atomic_inc (&users);
    users++;

    gd5480_disable_external_interrupts ();
    gd5480_stream_engine_off ();
    gd5480_vport_disable_interrupt ();
    
    px0002_off ();
    muted = 1;

    cr_write (0x7F, 0x01);  
    cr_write (0x7F, 0x00);  /* reset stream engine */

    if (((read_HDR() & 0xcf) == 0x4a)  ||  ((gr_read(0x18) & 0x20) != 0)) {
        pdebug ("%s(): using clock doubling\n", __FUNCTION__);
        lgv_5480tvr->clock_doubling = 1;
    }
    else
        lgv_5480tvr->clock_doubling = 0;

    capW = 640;
    capH = 480;
    bpp = 16;

    cr_write_bit (0x66, Bit6, 1);  /* 1: gd5480 double buffering */

    vidpicture.palette = VIDEO_PALETTE_YUV422;
    lgv_5480tvr_set_capture_colorspace (vidpicture.palette);

#if 0
    /* set luminance offset */
    cr_write_bits (0x51, Bit0, Bit2, 0x6);
    gd5480_set_capture_buffer1_address (lgv_5480tvr->offset);
    gd5480_set_capture_buffer2_address (lgv_5480tvr->offset);

    /* set chrominance offset */
    cr_write_bits (0x51, Bit0, Bit2, 0x0  /* 110 b */ );
    /* YUV420P 640x480, 12 bpp, 4 * 640 * 480 / 8 */
    off = lgv_5480tvr->offset + (640 * 480 * 4 / 8);
    gd5480_set_capture_buffer1_address (off);
    gd5480_set_capture_buffer2_address (off);
#endif

    gd5480_set_capture_buffer1_address (lgv_5480tvr->offset);
    gd5480_set_capture_buffer2_address (lgv_5480tvr->offset);


    /*** set up some default values  ***/
    lgv_5480tvr->stream_width = 0;
    lgv_5480tvr->stream_height = 0;

    gd5480_set_stream_engine_buffer1_page_table_offset (0);
    gd5480_set_stream_engine_buffer2_page_table_offset (0);
    gd5480_set_stream_engine_buffer1_target_address_offset (0);
    gd5480_set_stream_engine_buffer2_target_address_offset (0);
    gd5480_set_stream_engine_buffer1_source_addr (lgv_5480tvr->offset);
    gd5480_set_stream_engine_buffer2_source_addr (lgv_5480tvr->offset);

    cr_write_bit (0x51, Bit4, 1);  /* 1: use 5480 shrink */

    cr_write_bits (0x7E, Bit5, Bit7, 0);  /* stream engine no vert scaling */

    itt_write_direct (0xAA, 0x00);  /* full power mode */
    /*FIXME set these in hardware too */
    vidpicture.brightness = 32768;
    vidpicture.hue = 32768;
    vidpicture.colour = 32768;
    vidpicture.contrast = 32768;
    vidpicture.whiteness = 32768;
    vidpicture.depth = 16;  //FIXME
    lgv_5480tvr->video_mode = VIDEO_MODE_NTSC;
    itt_set_window (capW, capH);
    itt_set_colorspace (vidpicture.palette);
    itt_set_video_mode (VIDEO_MODE_NTSC);
    itt_set_source (0);  //FIXME TUNER or VCR
    itt_write_indirect (0x12B, 0xc00);  /* disable window 2 */
    itt_write_indirect (0x126, 0x100);  /* peaking and coring */
    /* output multiplexer, set double clock */
    itt_write_indirect (0x154, 0x200);
    I2C_SET (1, 1);

    lgv_5480tvr->capW = itt_max_width;
    lgv_5480tvr->capH = itt_max_height;

    gd5480_set_capture_dimensions (0, 0, itt_max_width, itt_max_height);
    cr_write (0x6F, 0);  /* disable capture buffers 2 and 3 */
    cr_write (0x5C, 0x1a);  /* alt threshold setting */

    //cr_write (0x50, 0x00);
    /* interlaced capture mode, enable double edge capture clock,
     * alt fields?
     * video capture: falling edge on HREF ends line
     */
    /* 0100 1011 */
    //cr_write (0x50, 0x4b);

    odd = 0;
    even = 0;
    buf1 = 0;
    buf2 = 0;
    buf3 = 0;
    ourInterrupts = 0;
    totalInterrupts = 0;
    vportInterrupts = 0;
    streamTransferInterrupts = 0;
    dmaErrorInterrupts = 0;
    dmaErrorTimeoutCount = 0;
    streamEngineTransfersAcknowledged = 0;
    streamEngineBeginCalls = 0;
    result = request_irq (lgv_5480tvr->dev->irq, lgv_5480tvr_irq_handler,
                          SA_SHIRQ | SA_INTERRUPT, "lgv_5480tvr", lgv_5480tvr);
    if (result) {
        printk ("%s(): couldn't install interrupt handler\n", __FUNCTION__);
        return -EBUSY;
    }

    /* 1: INTA# corresponds to vid port vref   0: vga? */
    cr_write_bit (0x3F, Bit5, 1);
    gd5480_vport_enable_interrupt ();
    gd5480_enable_external_interrupts ();

    cr_write_bit (0x3F, Bit4, 0);  /* 0: disable auto-decimation */

    vidwin.width = lgv_5480tvr->capW;
    vidwin.height = lgv_5480tvr->capH;
    vidwin.x = 100;
    vidwin.y = 100;
    vidwin.chromakey = 0xff;

#if 1
    /*FIXME query card for this shit */
    vidbuffer.width = 1024;
    vidbuffer.height = 768;
    vidbuffer.depth = 16;  //FIXME 
    vidbuffer.bytesperline = (vidbuffer.width * vidbuffer.depth) / 8;  //FIXME
#endif

#if 0
    vidbuffer.width = 0;
    vidbuffer.height = 0;
    vidbuffer.depth = 0;
    vidbuffer.bytesperline = 0;
    vidbuffer.base = 0;
#endif


    
    cr_write (0x50, 0);
    cr_write_bit (0x50, Bit3, 1);  /* 1: enable double edge cap clock */

    /* video pin configuration: 11 b vid cap: falling edge on HREF ends line */
    cr_write_bits (0x50, Bit0, Bit1, 0x3  /* 11 b */);

    /* capture all fields */
    cr_write_bit (0x50, Bit7, 0);
    cr_write_bit (0x50, Bit2, 0);

    cr_write_bit (0x50, Bit6, 1);  /* 1: interlaced capture */
    cr_write_bit (0x50, Bit5, 0);  /* 1: invert capture clock */

    cr_write_bit (0x58, Bit6, 1);  /* 1: reverse odd field sense */

    //    cr_write_bits (0x6F, Bit0, Bit1, 0x2  /* 10 b */ );
    
    cr_write_bit (0x6F, Bit1, 1);  /* enable capture buffer 2 */
    tuner_set_tv_frequency (3476);  //FIXME channel 23
    gd5480_stream_engine_enable_interrupts ();

    pdebug ("CR5C  0x%x\n", cr_read(0x5C));
    MOD_INC_USE_COUNT;
    return 0;
}

/* ioctl 0x5401 == read */

void lgv_5480tvr_close (struct video_device *dev)
{
    //atomic_dec (&users);
    users--;

    pdebug ("%s():\n", __FUNCTION__);

    pdebug ("%s(): capW:%d  capH:%d\n", __FUNCTION__, capW, capH);

    px0002_off ();
    muted = 1;

    gd5480_disable_external_interrupts ();
    gd5480_stream_engine_off ();
    gd5480_vport_disable_interrupt ();
    free_irq (lgv_5480tvr->dev->irq, lgv_5480tvr);
    gd5480_capture_off ();
    itt_off ();
    cr_write (0x7F, 0x1);  /* reset stream engine */

    pdebug ("interrupts %ld [vport %ld] [stream %ld] [dmaErr %ld] / %ld\n",
            ourInterrupts, vportInterrupts,
            streamTransferInterrupts, dmaErrorInterrupts, totalInterrupts);
    pdebug ("%d streamEngineBeginCalls\n", streamEngineBeginCalls);
    pdebug ("odd: %d   even: %d\n", odd, even);
    pdebug ("buf1: %d   buf2: %d   buf3: %d\n", buf1, buf2, buf3);
    pdebug ("streamEngineStatus %d\n", streamEngineStatus);
    MOD_DEC_USE_COUNT;
}

int alloc_mmap_buffer (void)
{
    int i, npages;

    pdebug ("%s()\n", __FUNCTION__);
    npages = itt_max_width * itt_max_height * 2 / PAGE_SIZE + 1;

    mmapBufferPages = (uint8_t **) kmalloc (npages * sizeof(unsigned long),
                                            GFP_KERNEL);
    if (!mmapBufferPages)
        return -ENOMEM;

    for (i = 0;  i < npages;  i++) {
        mmapBufferPages[i] = (uint8_t *) __get_free_pages (GFP_KERNEL |
                                                           GFP_DMA, 0);
        if (!mmapBufferPages[i])
            break;
        mem_map_reserve (virt_to_page(mmapBufferPages[i]));
        
    }
    if (i-- < npages) {
        printk ("%s(): couldn't allocate mmap pages\n", __FUNCTION__);
        for (;  i > 0;  i--) {
            mem_map_unreserve (virt_to_page(mmapBufferPages[i]));
            free_pages ((unsigned long) mmapBufferPages[i], 0);
        }
        kfree (mmapBufferPages);
        mmapBufferPages = NULL;
        return -ENOBUFS;
    }
    mmapBufferNumPages = npages;

    //FIXME uh oh
    for (i = 0;  i < npages;  i++) {
        int j;
                
        for (j = 0;  j < PAGE_SIZE;  j++) {
            //mmapBufferPages[i][j] = (255 - j) % 255;
            mmapBufferPages[i][j] = i;
            //mmapBufferPages[i][j] = 0x00;
        }
    }

#if 0
    for (i = 0;  i < PAGE_SIZE;  i++)
        mmapBufferPages[0][i] = 0xff;
#endif

    return 0;
}

void free_mmap_buffer (void)
{
    int i;

    if (!mmapBufferPages)
        return;

    for (i = 0;  i < mmapBufferNumPages;  i++) {
        mem_map_unreserve (virt_to_page(mmapBufferPages[i]));
        free_pages ((unsigned long) mmapBufferPages[i], 0);
    }
    kfree (mmapBufferPages);
    mmapBufferPages = NULL;
}

void setup_dma_page_tables (void)
{
    int i;
    uint32_t busAddr;
    uint32_t *p;

    pdebug ("%s()\n", __FUNCTION__);

    if (!mmapBufferPages) {
        printk ("%s():  error mmapBufferPages == NULL\n", __FUNCTION__);
        return;
    }

    p = (uint32_t *) (lgv_5480tvr->fb_base + 
                     lgv_5480tvr->page_table_start_addr);

    for (i = 0;  i < mmapBufferNumPages;  i++) {
        busAddr = (uint32_t) virt_to_bus (mmapBufferPages[i]);
        if (busAddr & 0x0000fff) {
            printk ("%s(): fuck 0x%x\n", __FUNCTION__, busAddr);
        }
        //FIXME endianess
        /*              page present     word swap  */
        *p = busAddr | 0x00000001     | 0x000080;
        //*p = busAddr | 0x00000001;
        p++;
    }
}

int lgv_5480tvr_mmap (struct video_device *dev, const char *adr,
                      unsigned long size)
{
    int i;

    pdebug ("%s(): 0x%lx size %ld\n", __FUNCTION__, (unsigned long) adr, size);
    if (size != itt_max_width * itt_max_height * 2)
        return -EINVAL;
    if (!mmapBufferPages) {
        printk ("%s(): mmap pages haven't been allocated yet\n", __FUNCTION__);
        return -ENOBUFS;
    }
    for (i = 0;  i < mmapBufferNumPages;  i++) {
        if (remap_page_range((unsigned long) adr,
                             virt_to_phys(mmapBufferPages[i]),
                             PAGE_SIZE, PAGE_SHARED))
        {
            return -EAGAIN;
        }
        adr += PAGE_SIZE;
        if (size <= PAGE_SIZE)
            break;
        size -= PAGE_SIZE;
    }

    return 0;
}

int lgv_5480tvr_ioctl (struct video_device *dev,
                       unsigned int         cmd,
                       void                *arg)
{
    switch (cmd) {
    case VIDIOCGTUNER: {  /* get tuner capabilities */
        struct video_tuner v;
        if (copy_from_user(&v, arg, sizeof(v)) != 0)
            return -EFAULT;
        if (v.tuner)
            return -EINVAL;
        v.rangelow = vidtuner.rangelow;
        v.rangehigh = vidtuner.rangehigh;
        v.flags = vidtuner.flags;
        v.mode = vidtuner.mode;
        v.signal = vidtuner.signal;
        strcpy (v.name, vidtuner.name);
        if (copy_to_user(arg, &v, sizeof(v))!=0)
            return -EFAULT;
        return 0;
    }
    case VIDIOCSTUNER: {  /* set the desired tuner */
        struct video_tuner v;
        //pdebug ("VIDIOCSTUNER\n");
        if (copy_from_user(&v, arg, sizeof(v)))
            return -EFAULT;
        if (v.tuner != 0)
            return -EINVAL;
        return 0;
    }
    case VIDIOCGFREQ: {  /* get current frequency */
        //pdebug ("VIDIOCGFREQ\n");
        if (copy_to_user(arg, &current_freq, sizeof(unsigned long)))
            return -EFAULT;
        return 0;
    }
    case VIDIOCSFREQ: {  /* set frequency */
        u32 freq;
        //pdebug ("VIDIOCSFREQ\n");
        if (copy_from_user(&freq, arg, sizeof(unsigned long)) != 0)
            return -EFAULT;
        if (tuner_set_tv_frequency(freq) < 1)
            return -EINVAL;
        return 0;
    }
    case VIDIOCGCAP: {  /* get card capabilities */
        struct video_capability v;
        //pdebug ("VIDIOCGCAP\n");
        //v.type = VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY | VID_TYPE_TUNER |
        //VID_TYPE_CAPTURE;
        v.type = VID_TYPE_CAPTURE | VID_TYPE_TUNER;
        v.channels = 1;  //FIXME
        v.audios = 1;  //FIXME
        v.maxwidth = itt_max_width;
        v.minwidth = 16;
        v.maxheight = itt_max_height;
        v.minheight = 16;
        strcpy (v.name, "lgv-5480tvr");
        if (copy_to_user(arg, &v, sizeof(v)))
            return -EFAULT;
        return 0;
    }
    case VIDIOCGCHAN: {  /* get specified input's capabilities */
        struct video_channel v;
        //pdebug ("VIDIOCGCHAN\n");
        if (copy_from_user(&v, arg, sizeof(v)))
            return -EFAULT;
        if (v.channel != 0) {
            printk ("VIDIOCGCHAN v.channel == %d\n", v.channel);
            return -EINVAL;
        }
        v.flags = VIDEO_VC_TUNER | VIDEO_VC_AUDIO;
        v.tuners = 1;
        v.type = VIDEO_TYPE_TV;
        v.norm = VIDEO_MODE_NTSC;
        strcpy (v.name, "television");
        if(copy_to_user(arg, &v, sizeof(v)))
            return -EFAULT;
        return 0;
    }
    case VIDIOCSCHAN: {  /* set which input to use (ex: s-vhs, TV tuner) */
        struct video_channel v;
        //pdebug ("VIDIOCSCHAN\n");
        if (copy_from_user(&v, arg, sizeof(v)))
            return -EFAULT;
        if (v.channel != 0)
            return -EINVAL;
        if (v.norm != VIDEO_MODE_NTSC) {
            printk ("VIDIOCSCHAN invalid norm %d\n", v.norm);
            return -EINVAL;
        }
        return 0;
    }
    case VIDIOCGPICT: {  /* get color values, depth, palette */
#if 0
        struct video_picture v;
        //pdebug ("VIDIOCGPICT\n");
        v.brightness = 32768;
        v.hue = 32768;
        v.colour = 32768;  /* saturation */
        v.contrast = 32768;  /* brightness */

        v.whiteness = 32768;
        v.depth = 16;  //FIXME crappola
        //v.palette = VIDEO_PALETTE_RGB565;  //FIXME
        v.palette = VIDEO_PALETTE_YUV422;

        if (copy_to_user(arg, &v, sizeof(v)))
            return -EFAULT;
#endif
        if (copy_to_user(arg, &vidpicture, sizeof(vidpicture)))
            return -EFAULT;
        return 0;
    }
    case VIDIOCSPICT: {  /* set color values, depth, palette, etc. */
        struct video_picture v;
        //pdebug ("VIDIOCSPICT\n");
        if (copy_from_user(&v, arg, sizeof(v)))
            return -EFAULT;
        if (v.palette != VIDEO_PALETTE_YUV422)
        {
            pdebug ("VIDIOCSPICT invalid palette %d %s\n", v.palette,
                    Palettes[v.palette]);
            return -EINVAL;
        }
        switch (v.palette) {
        case VIDEO_PALETTE_YUV420P:
            if (v.depth != 12) {
                pdebug ("VIDIOCSPICT YUV420P invalid depth %d\n", v.depth);
                return -EINVAL;
            }
            break;
        case VIDEO_PALETTE_YUV422:
            if (v.depth != 16) {
                pdebug ("VIDIOCSPICT YUV422 invalid depth %d\n", v.depth);
                return -EINVAL;
            }
            break;
        default:
            pdebug ("VIDIOCSPICT invalid depth %d\n", v.depth);
            return -EINVAL;
        }

        /* lgv_5480tvr_set_capture_colorspace (v.palette); */

        /*FIXME set picture parameters here */
        /*FIXME set hardware values here */
        memcpy (&vidpicture, &v, sizeof (struct video_picture));
        return 0;
    }
    /*FIXME does app set frame buffer info ?? */
    case VIDIOCGFBUF: {  /* get framebuffer info */
        //pdebug ("VIDIOCGFBUF\n");
        if (copy_to_user(arg, &vidbuffer, sizeof(vidbuffer)))
            return -EFAULT;
        return 0;
    }
    case VIDIOCSFBUF: {  /* set framebuffer info */
        struct video_buffer v;
        //pdebug ("VIDIOCSFBUF\n");
        if (!capable(CAP_SYS_ADMIN))
            return -EPERM;
#if 1       
        if (copy_from_user(&v, arg, sizeof(v)))
            return -EFAULT;
        if (v.width != vidbuffer.width)
            return -EINVAL;
        if (v.height != vidbuffer.height)
            return -EINVAL;
        if (v.depth != vidbuffer.depth)
            return -EINVAL;
        if (v.bytesperline != vidbuffer.bytesperline)
            return -EINVAL;
        if (v.base != vidbuffer.base)
            return -EINVAL;
#endif

#if 0
        memcpy(&vidbuffer, &v, sizeof(v));
        pdebug ("vidbuffer.base: 0x%lx\n", (unsigned long) vidbuffer.base);
        pdebug ("vidbuffer.height: %d\n", vidbuffer.height);
        pdebug ("vidbuffer.width: %d\n", vidbuffer.width);
        pdebug ("vidbuffer.depth: %d\n", vidbuffer.depth);
        pdebug ("vidbuffer.bytesperline: %d\n", vidbuffer.bytesperline);
        //hardware_set_fb(&v);
#endif
        return 0;
    }
    case VIDIOCGWIN: {  /* get video window dimensions */
        //pdebug ("VIDIOCGWIN\n");
        if(copy_to_user(arg, &vidwin, sizeof(vidwin)))
            return -EFAULT;
        return 0;
    }
    case VIDIOCSWIN: {  /* set video window dimensions */
        struct video_window v;
        pdebug ("VIDIOCSWIN\n");
        if(copy_from_user(&v, arg, sizeof(v)))
            return -EFAULT;
        if (v.width < 16  ||  v.width > itt_max_width)
            return -EINVAL;
        if (v.height < 16  ||  v.height > itt_max_height)
            return -EINVAL;
        gd5480_capture_off ();
        vidwin.width = v.width;
        vidwin.height = v.height;

        /*
        pdebug ("%s(): width %d  height %d\n", __FUNCTION__,
                vidwin.width, vidwin.height);
        */
        capW = vidwin.width;
        capH = vidwin.height;
#if 0
        gd5480_set_video_capture_buffers_1_and_2_offset (vidwin.width * bpp / 8);

#if 0
        gd5480_set_stream_engine_source_buffer_pitch_width (itt_max_width * bpp / 8);
        gd5480_set_stream_engine_dest_buffer_pitch_width (itt_max_width * bpp / 8);
        gd5480_set_stream_engine_target_buffer_width (itt_max_width * bpp / 8);
        gd5480_set_stream_engine_target_buffer_height (itt_max_height);
#endif

        gd5480_set_capture_dimensions (0, 0, vidwin.width, vidwin.height);
#if 0
        if (vidwin.chromakey != v.chromakey) {
            vidwin.chromakey = v.chromakey;
            gd5480_set_chromakey (v.chromakey);
        }
#endif
        itt_set_window (vidwin.width, vidwin.height);
        vidwin.x = v.x;
        vidwin.y = v.y;
#endif
        //gd5480_set_window (1, vidwin.x, vidwin.y, 
        //                   vidwin.width, vidwin.height);
        //gd5480_capture_on ();
        return 0;
    }
#if 0
    case VIDIOCCAPTURE: {  /* turn overlay on/off */
        int v;
        pdebug ("VIDIOCCAPTURE\n");
        if (get_user(v, (int *)arg))
            return -EFAULT;
        if (v == 0) {
            gd5480_capture_off ();
            gd5480_window1_off ();
            itt_off ();
        }
        else {
            itt_on ();
            gd5480_capture_on ();
            gd5480_window1_on ();
        }
        pdebug ("VIDIOCCAPTURE %d\n", v);
        return 0;
    }
#endif
    /* Sets settings for capture.  If capture off, turn on.  Capture a 
     * frame
     */
    case VIDIOCMCAPTURE: {  /* sets settings for capture, and begin cap */
        struct video_mmap v;

        //pdebug ("VIDIOCMCAPTURE\n");
        if (copy_from_user(&v, arg, sizeof(v)))
            return -EFAULT;
        if (v.frame != 0) {
            pdebug ("VIDIOCMCAPTURE invalid frame number %d\n", v.frame);
            return -EINVAL;
        }
        if (v.height > itt_max_height  ||  v.height < 16) {
            pdebug ("VIDIOCMCAPTURE invalid height %d\n", v.height);
            return -EINVAL;
        }
        if (v.width > itt_max_width  ||  v.width < 16) {
            pdebug ("VIDIOCMCAPTURE invalid width %d\n", v.width);
            return -EINVAL;
        }
        //        if (v.format != VIDEO_PALETTE_YUV422  &&  v.format != VIDEO_PALETTE_YUV420P) {
        if (v.format != VIDEO_PALETTE_YUV422) {
            pdebug ("VIDIOCMCAPTURE invalid palette %d %s\n", v.format,
                    Palettes[v.format]);
            return -EINVAL;
        }
        if (streamEngineStatus != TransferOff  &&
            streamEngineStatus != TransferDone)
            return -EBUSY;

        if (lgv_5480tvr->stream_width != v.width  ||
            lgv_5480tvr->stream_height != v.height)
        {
            pdebug ("VIDIOCMCAPTURE setting size %dx%d --> %dx%d\n", 
                    lgv_5480tvr->stream_width, lgv_5480tvr->stream_height,
                    v.width, v.height);
            gd5480_capture_off ();
            itt_off ();
            capW = v.width;
            capH = v.height;
            itt_set_window (v.width, v.height);
            gd5480_set_video_capture_buffers_1_and_2_offset (v.width * bpp / 8);
            gd5480_set_stream_engine_source_buffer_pitch_width (v.width * bpp / 8);
            gd5480_set_stream_engine_dest_buffer_pitch_width (v.width * bpp / 8);
            gd5480_set_stream_engine_target_buffer_width (v.width * bpp / 8);
            gd5480_set_stream_engine_target_buffer_height (v.height);
            gd5480_set_capture_dimensions (0, 0,
                                           v.width,
                                           v.height);
            cr_write (0x52, 0x00);  /* vid cap buf 1 and 2 horizontal shrink */
            cr_write (0x53, 0x00);  /* vid cap buf 1 and 2 vertical shrink */
            lgv_5480tvr->stream_width = v.width;
            lgv_5480tvr->stream_height = v.height;
        }

        vidmmap.frame = v.frame;
        vidmmap.format = v.format;

        if (!Capturing) {
            itt_on ();
            gd5480_capture_on ();
        }
        streamEngineStatus = TransferNeed;
        return 0;
    }
    case VIDIOCGMBUF: {  /* gives app info to use when mmaping() */
        struct video_mbuf vm;
        int i;

        //pdebug ("VIDIOCGMBUF\n");
        vm.size = itt_max_width * itt_max_height * 2;
        vm.frames = 1;
        for (i = 0;  i < VIDEO_MAX_FRAME;  i++) {
            vm.offsets[i] = 0;
        }
        if (copy_to_user(arg, &vm, sizeof(vm)))
            return -EFAULT;
        return 0;
    }
        
    /* app tells us we used a frame and can transfer data again, block */
    case VIDIOCSYNC: {
        DECLARE_WAITQUEUE(wait, current);
        int frameNum;
        
        //pdebug ("VIDIOCSYNC\n");
        if (copy_from_user(&frameNum, arg, sizeof(int)))
            return -EFAULT;
        if (frameNum != 0)
            return -EINVAL;

        add_wait_queue (&capture_wait, &wait);
        current->state = TASK_INTERRUPTIBLE;
        while (streamEngineStatus != TransferDone) {
            schedule ();
            if (signal_pending(current)) {
                remove_wait_queue (&capture_wait, &wait);
                current->state = TASK_RUNNING;
                return -EINTR;
            }
        }
        remove_wait_queue (&capture_wait, &wait);
        current->state = TASK_RUNNING;
        streamEngineStatus = TransferOff;

        return 0;
    }
    case VIDIOCGAUDIO: {
        struct video_audio v;
        if (copy_from_user(&v, arg, sizeof(v)))
            return -EFAULT;
        if (v.audio != 0)
            return -EINVAL;
        strcpy (v.name, "TV");
        //FIXME does it handle stereo??
        v.mode = VIDEO_SOUND_MONO;
        if (muted)
            v.flags = VIDEO_AUDIO_MUTE;
        else
            v.flags = 0;
        if (copy_to_user(arg, &v, sizeof(v)))
            return -EFAULT;
        return 0;
    }
    case VIDIOCSAUDIO: {
        struct video_audio v;
        if (copy_from_user(&v, arg, sizeof(v)))
            return -EFAULT;
        if (v.audio != 0)
            return -EINVAL;
#if 0
        if (v.flags != 0) {
            pdebug ("%s():  invalid flags 0x%x\n", __FUNCTION__, v.flags);
            return -EINVAL;
        }
#endif
#if 0
        if (v.mode != VIDEO_SOUND_MONO) {
            pdebug ("%s():  only VIDEO_SOUND_MONO  0x%x\n",
                    __FUNCTION__, v.mode);
            return -EINVAL;
        }
#endif
        if (v.flags & VIDEO_AUDIO_MUTE) {
            if (!muted) {
                px0002_off ();
                muted = 1;
            }
        }
        else {
            if (muted) {
                px0002_on ();
                muted = 0;
            }
        }
        return 0;
    }
    default:
        pdebug ("ioctl 0x%x not implemented.\n", cmd);
        return -ENOIOCTLCMD;
    }
}

int init_module (void)
{
    struct pci_dev *dev = NULL;
    uint8_t SR0F;
    int videoram;
    int t1, t2;
    int result;

    pdebug ("%s()\n", __FUNCTION__);
    if (video_register_device(&lgv_5480tvr_device, VFL_TYPE_GRABBER, -1) < 0)
        return -EINVAL;

    dev = pci_find_device (PCI_VENDOR_ID_CIRRUS, PCI_DEVICE_ID_CIRRUS_5480,
                           dev);
    if (!dev)
        return -ENODEV;
    else
        pdebug ("found gd5480\n");

    if (pci_dma_supported (dev, 0xffffffff)) {
        pdebug ("%s(): DMA supported\n", __FUNCTION__);
        pci_set_master (dev);
    }
    else {
        pdebug ("%s(): DMA not supported\n", __FUNCTION__);
    }

    pdebug ("%s(): dma_mask 0x%lx\n", 
            __FUNCTION__, (unsigned long) dev->dma_mask);
    lgv_5480tvr = kmalloc (sizeof(struct lgv_5480tvr_data), GFP_KERNEL);
    if (!lgv_5480tvr)
        return -ENOMEM;
    memset (lgv_5480tvr, 0, sizeof(struct lgv_5480tvr_data));

    lgv_5480tvr->dev = dev;

    lgv_5480tvr->mmio_base = ioremap_nocache (dev->resource[1].start &
                                              PCI_BASE_ADDRESS_MEM_MASK,
                                              4096);
    if (!lgv_5480tvr->mmio_base) {
        kfree (lgv_5480tvr);
        return -ENOMEM;
    }
    pdebug ("mmio_base : 0x%lx to 0x%x\n",
            (unsigned int) dev->resource[1].start & PCI_BASE_ADDRESS_MEM_MASK,
            (unsigned int) lgv_5480tvr->mmio_base);

    gd5480_save ();

    /* 0x68  0110 1000 */
    gr_write (0x17, 0x68);  /* enable i2c, disable DCLK Pin driver */
    sr_write (0x08, 0xff);  /* set i2c outputs to high impedence */

    t1 = itt_read_direct (0x00);
    if (t1 < 0) {
        printk ("%s(): failed to find video decoder at i2c addr 0x%x\n",
                __FUNCTION__, ittI2cAddr);
        goto init_error;
    }
    t2 = (itt_read_direct(0x02) << 8) | itt_read_direct(0x01);
    pdebug ("%s(): videodecoder 0x%02x %04x\n", __FUNCTION__, t1, t2);
    decoderId = t2;
    switch (t2) {
    case 0x4680:
    case 0x4260:
    case 0x4280:
        break;
    case VPX3225D_ID:  /* 0x7230 */
        pdebug ("found vpx3225d\n");
        fpReadReg = 0x36;
        fpWriteReg = 0x37;
        fpDataReg = 0x38;
        fpStatusReg = 0x35;
        break;
    default:
        printk ("%s(): unsupported video decoder\n", __FUNCTION__);
        goto init_error;
    }

    //FIXME tuner type, pass in as arg and/or keep table
    /* check i2c address of tuner */
    if (i2c_read(tuner_i2c_addr | 1) >= 0) {
        pdebug ("tuner at i2c addr 0x%x\n", tuner_i2c_addr);
    }
    else {
        printk ("%s(): couldn't find tuner at i2c addr 0x%x\n", __FUNCTION__,
                tuner_i2c_addr);
        goto init_error;
    }
    /* count ram kb */
    videoram = 1024;
    SR0F = sr_read (0x0F);
    if ((SR0F & 0x18) == 0x18) {  /* 2 or 4 mb */
        videoram = 2048;
        if (SR0F & 0x80)  /* second bank enabled */
            videoram = 4096;
    }
    lgv_5480tvr->ram_size = videoram * 1024;
    if (videoram < 4096) {
        printk ("%s(): FIXME video ram less than 4096, not supported yet.\n",
                __FUNCTION__);
        goto init_error;
    }

    lgv_5480tvr->fb_base = ioremap_nocache (dev->resource[0].start &
                                    PCI_BASE_ADDRESS_MEM_MASK,
                                    lgv_5480tvr->ram_size);
    if (!lgv_5480tvr->fb_base) {
        printk ("couldn't ioremap fb_base\n");
        iounmap (lgv_5480tvr->mmio_base);
        kfree (lgv_5480tvr);
        return -ENOMEM;
    }
    pdebug ("%s(): pci fb_base 0x%lx  to  0x%lx\n", __FUNCTION__, 
            dev->resource[0].start & PCI_BASE_ADDRESS_MEM_MASK,
            (unsigned long) lgv_5480tvr->fb_base);


#if 1
    /*FIXME this shit is for v4l-conf ?? */
    vidbuffer.base = (void *) (dev->resource[0].start &
                               PCI_BASE_ADDRESS_MEM_MASK);
#endif

    //FIXME
    displayWidth = 1024;
    displayHeight = 768;
    bpp = 16;
    maxWidth = itt_max_width;
    maxHeight = itt_max_height;

    lgv_5480tvr->page_table_start_addr = 2 * 1024 * 1024;
    lgv_5480tvr->num_page_tables = itt_max_width * itt_max_height * 
                                   2 / PAGE_SIZE + 1;
    lgv_5480tvr->offset = lgv_5480tvr->page_table_start_addr +
        lgv_5480tvr->num_page_tables * 4;
    lgv_5480tvr->offset = (lgv_5480tvr->offset + 7) & -8;
    pdebug ("%s(): num_page_tables %d\n", __FUNCTION__,
            lgv_5480tvr->num_page_tables);
    pdebug ("%s(): offset 0x%04x\n", __FUNCTION__, lgv_5480tvr->offset);
    gd5480_set_bus_master_page_table_start_address (lgv_5480tvr->page_table_start_addr);

    /* note: uses lgv_5480tvr->page_table_start_addr value */
    result = alloc_mmap_buffer ();
    if (result) {
        video_unregister_device (&lgv_5480tvr_device);
        iounmap (lgv_5480tvr->mmio_base);
        kfree (lgv_5480tvr);
        return result;
    }
    setup_dma_page_tables ();  /* uses alloced()  mmap buffer */

    return 0;

 init_error:
    video_unregister_device (&lgv_5480tvr_device);
    iounmap (lgv_5480tvr->mmio_base);
    kfree (lgv_5480tvr);
    return -ENODEV;
}

void cleanup_module (void)
{
    uint8_t command;

    pdebug ("%s()\n", __FUNCTION__);

    if (mmapBufferPages) 
        free_mmap_buffer ();

    gd5480_restore ();

    /* disable PCI bus-mastering */
    pci_read_config_byte (lgv_5480tvr->dev, PCI_COMMAND, &command);
    command &= ~PCI_COMMAND_MASTER;
    pci_write_config_byte (lgv_5480tvr->dev, PCI_COMMAND, command);

    video_unregister_device (&lgv_5480tvr_device);
    iounmap (lgv_5480tvr->fb_base);
    iounmap (lgv_5480tvr->mmio_base);
    kfree (lgv_5480tvr);

    pdebug ("interrupts %ld [vport %ld] [stream %ld] [dmaErr %ld] / %ld\n",
            ourInterrupts, vportInterrupts,
            streamTransferInterrupts, dmaErrorInterrupts, totalInterrupts);
}

EXPORT_NO_SYMBOLS;
