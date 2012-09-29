#ifndef bitops_h_included
#define bitops_h_included

//#include <inttypes.h>

enum { Bit0 = 0, Bit1, Bit2, Bit3, Bit4, Bit5, Bit6, Bit7, Bit8, Bit9, Bit10,
       Bit11, Bit12, Bit13, Bit14, Bit15, Bit16, Bit17, Bit18, Bit19, Bit20,
       Bit21, Bit22, Bit23, Bit24, Bit25, Bit26, Bit27, Bit28, Bit29, Bit30,
       Bit31
     };

#define BIT_0(x)  (  (x) & 0x1)
#define BIT_1(x)  ((  (x) & 0x2) >> 1)
#define BIT_2(x)  (( (x) & 4) >> 2)
#define BIT_3(x)  (( (x) & 8) >> 3)
#define BIT_4(x)  (( (x) & 16) >> 4)
#define BIT_5(x)  (( (x) & 32) >> 5)
#define BIT_6(x)  (( (x) & 64) >> 6)
#define BIT_7(x)  (( (x) & 128) >> 7)
#define BIT_8(x)  (( (x) & 256) >> 8)
#define BIT_9(x)  (( (x) & 512) >> 9)
#define BIT_10(x)  (( (x) & 1024) >> 0xa)
#define BIT_11(x)  (( (x) & 2048) >> 0xb)

#define BIT_0to1(x)  ( (x) & 0x3)
#define BIT_0to2(x)  ( (x) & 0x7)

uint8_t u8ReadBit (uint8_t source, int bitnum);
uint8_t u8WriteBit (uint8_t source, int bitnum, int value);

uint8_t u8ReadBits (uint8_t source, int bitStart, int bitEnd);
uint8_t u8WriteBits (uint8_t source, int bitStart, int bitEnd, uint8_t value);

uint32_t u32ReadBit (uint32_t source, int bitnum);

uint32_t u32ReadBits (uint32_t source, int bitStart, int bitEnd);

#endif /* bitops_h_included */
