#ifndef bitops_c_included
#define bitops_c_included
#include <linux/kernel.h>
#include "bitops.h"

uint8_t u8ReadBits (uint8_t source, int bitStart, int bitEnd)
{
    if (bitStart > bitEnd  ||  bitStart < 0  || bitStart > 7  ||
        bitEnd < 0  ||  bitEnd > 7)
    {
        printk ("%s():  invalid bitStart and/or bitEnd: %d -> %d\n",
                __FUNCTION__, bitStart, bitEnd);
        return source;
    }

    return (source & (0x00ff >> (7 - bitEnd))) >> bitStart;
}

uint8_t u8ReadBit (uint8_t source, int bitnum)
{
    if (bitnum > 7  ||  bitnum < 0) {
        printk ("%s():  invalid bitnum: %d\n", __FUNCTION__, bitnum);
        return source;
    }

    return u8ReadBits(source, bitnum, bitnum);
}

uint8_t u8WriteBits (uint8_t source, int bitStart, int bitEnd,  uint8_t value)
{
    uint8_t top, middle, bottom;

    if (bitStart > bitEnd  ||  bitStart < 0  || bitStart > 7  ||
        bitEnd < 0  ||  bitEnd > 7)
    {
        printk ("%s():  invalid bitStart and/or bitEnd: %d -> %d\n",
                __FUNCTION__, bitStart, bitEnd);
        return source;
    }

    /* power == bitEnd - bitStart,  0x00ff >> (7 - (bitEnd - bitStart)) */
    if (value > (0x00ff >> (7 - (bitEnd - bitStart)))) {
        printk ("%s():  value (0x%x) too big for range (%d -> %d)\n",
                __FUNCTION__, value, bitStart, bitEnd);
        return source;
    }
    /* ex: u8WriteBits (source   = ssss ssss,  
     *                  bitStart = 2,                             
     *                  bitEnd   = 5,                               
     *                  value    = vvvv)                       
     * ssss ssss  == source                              
     *      vvvv  == value                                
     * ssvv vvss  == result                                           
     *                                                  
     * ss00 0000  == top                                     
     * 00vv vv00  == middle                        
     * 0000 00ss  == bottom         
     */

    top = (source & (0x00ff << (bitEnd + 1))) & 0x00ff;
    middle = value << bitStart;
    bottom = source & (0x00ff >> (7 - bitStart + 1));

    return top | middle | bottom;
}

uint8_t u8WriteBit (uint8_t source, int bitnum, int value)
{
    if (bitnum > 7  ||  bitnum < 0  ||  value > 1  ||  value < 0) {
        printk ("%s():  invalid bitnum and/or value: %d %d\n",
                __FUNCTION__, bitnum, value);
        return source;
    }

    return u8WriteBits(source, bitnum, bitnum, value);
}

uint32_t u32ReadBits (uint32_t source, int bitStart, int bitEnd)
{
    if (bitStart > bitEnd  ||  bitStart < 0  || bitStart > 31  ||
        bitEnd < 0  ||  bitEnd > 31)
    {
        printk ("%s():  invalid bitStart and/or bitEnd: %d -> %d\n",
                __FUNCTION__, bitStart, bitEnd);
        return source;
    }

    return (source & (0xffffffff >> (31 - bitEnd))) >> bitStart;
}

uint32_t u32ReadBit (uint32_t source, int bitnum)
{
    if (bitnum > 31  ||  bitnum < 0) {
        printk ("%s():  invalid bitnum %d\n", __FUNCTION__, bitnum);
        return source;
    }
    return u32ReadBits(source, bitnum, bitnum);
}

uint32_t u32WriteBits (uint32_t source, int bitStart, int bitEnd,
                       uint32_t value)
{
    uint32_t top, middle, bottom;

    if (bitStart > bitEnd    ||
        bitStart < 0         ||
        bitStart > 31        ||
        bitEnd   < 0         ||
        bitEnd   > 31)
    {
        printk ("%s():  invalid bitStart and/or bitEnd: %d -> %d\n",
                __FUNCTION__, bitStart, bitEnd);
        return source;
    }

    /* power = bitEnd - bitStart,  0xffffffff >> (31 - (bitEnd - bitStart)) */
    if (value > (0xffffffff >> (31 - (bitEnd - bitStart)))) {
        printk ("%s():  value (0x%x) too big for range (%d -> %d)\n",
                __FUNCTION__, value, bitStart, bitEnd);
        return source;
    }

    /* ex:  u32WriteBits (source   = ssss ssss ssss ssss ssss ssss ssss ssss,
     *                    bitStart = 17,                         
     *                    bitEnd   = 26,                          
     *                    value    = vv vvvv vvvv)               
     *                                                      
     * ssss ssss ssss ssss  ssss ssss ssss ssss                    
     *                             vv vvvv vvvv                     
     * ssss svvv vvvv vvvs  ssss ssss ssss ssss                    
     *                                                               
     * ssss s000 0000 0000  0000 0000 0000 0000   top            
     * 0000 0vvv vvvv vvv0  0000 0000 0000 0000   middle         
     * 0000 0000 0000 000s  ssss ssss ssss ssss             
     */

    top = (source & (0xffffffff << (bitEnd + 1))) & 0xffffffff;
    middle = value << bitStart;
    bottom = source & (0xffffffff >> (31 - bitStart + 1));

    return top | middle | bottom;
}


#endif  /* bitops_c_included */
