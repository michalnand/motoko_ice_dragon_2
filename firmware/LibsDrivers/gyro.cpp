#include "gyro.h"

#include "gyro_regs.h"


#define F_CPU 216000000UL


int Gyro::init(I2C_Interface &i2c)
{
    this->i2c = &i2c;

    this->g_yaw       = 0; 
    this->g_yaw_ofs   = 0;


    // LSM6DSMTR connection check
    if (read_status() != 1)
    {
        return -1;
    }
    

    // gyroscope init
    // 500 dps range, FS_G = 0 1 (bits 2, 3)
    // 416Hz data rate, ODR_G = 0 1 1 0, (bits 7, 6, 5, 4)
    //this->i2c->write_reg(LSM6DSMTR_ADDR, CTRL2_G, (1<<6)|(1<<5)|(1<<2));

    // 250 dps range, FS_G = 0 1 (bits 2, 3)
    // 416Hz data rate, ODR_G = 0 1 1 0, (bits 7, 6, 5, 4)
    this->i2c->write_reg(LSM6DSMTR_ADDR, CTRL2_G, (1<<6)|(1<<5));

    // disable LFP
    this->i2c->write_reg(LSM6DSMTR_ADDR, CTRL6_C, 0x00);

    // disable HPF
    this->i2c->write_reg(LSM6DSMTR_ADDR, CTRL7_G, 0x00);


    // delay 100ms
    delay_loops(F_CPU*0.2);       

    // dummy read
    for (unsigned int i = 0; i < 32; i++)
    {
        this->read();  
        
        //delay 2ms
        delay_loops(F_CPU*0.002);   
    }         


    // estimate gyro offset 

    uint32_t n_samples = 100;

    float g_yaw_ofs_tmp = 0.0f; 

    for (unsigned int i = 0; i < n_samples; i++)
    {
        this->read();                

        g_yaw_ofs_tmp+= this->g_yaw;

        delay_loops(F_CPU*0.002);  
    }

    // zero values
    this->g_yaw_ofs   = g_yaw_ofs_tmp/n_samples;

    this->read();

    return 0;   
}

// return acc in mg and gyro in mdps
float Gyro::read()      
{
    // read gyro
    int16_t g_yaw = -read_word(LSM6DSMTR_ADDR, OUTZ_L_G);   
    
   
    // convert gyroscope to deg/s, swap sign - gyro mounted on board bottom   
    this->g_yaw = g_yaw*GYRO_RANGE - this->g_yaw_ofs;

    return this->g_yaw;
}   


int Gyro::read_status()
{
    if (i2c->read_reg(LSM6DSMTR_ADDR, WHO_AM_I) != WHO_AM_I_VAL)
    {
        return 0;
    }
    
    return 1;
}


// Helper function to read 16-bit signed word (high + low byte)
int16_t Gyro::read_word(unsigned char dev_adr, unsigned char reg_adr) 
{
    uint8_t low   = i2c->read_reg(dev_adr, reg_adr);    
    uint8_t high  = i2c->read_reg(dev_adr, reg_adr + 1);

    return (int16_t)((high << 8) | low);
}   

