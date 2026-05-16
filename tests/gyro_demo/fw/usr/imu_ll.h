#ifndef _IMU_LL_H_
#define _IMU_LL_H_

#include "drivers.h"


class IMULL
{
    public:
        int init(I2C_Interface &i2c);
        int read_status();
        void read();

    private:    
        int16_t read_word(unsigned char dev_adr, unsigned char reg_adr);

    public:
        float g_yaw;
        float g_yaw_ofs;

        
    private:
        I2C_Interface *i2c;
};

#endif