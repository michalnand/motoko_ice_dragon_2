#ifndef IMU_LL_REGS_H
#define IMU_LL_REGS_H

// IMU : LSM6DSMTR
#define LSM6DSMTR_ADDR      ((uint8_t)0xD4) //already 8bit address
#define WHO_AM_I            ((uint8_t)0x0F)

#define WHO_AM_I_VAL        ((uint8_t)0x6A)


#define CTRL1_XL            ((uint8_t)0x10)
#define CTRL2_G             ((uint8_t)0x11)
#define CTRL7_G             ((uint8_t)0x16)
#define CTRL6_C             ((uint8_t)0x15)
#define CTRL8_XL            ((uint8_t)0x17)

// gyroscope X axis
#define OUTX_L_G            ((uint8_t)0x22)
#define OUTX_H_G            ((uint8_t)0x23)

// gyroscope Y axis
#define OUTY_L_G            ((uint8_t)0x24)
#define OUTY_H_G            ((uint8_t)0x25)

// gyroscope Z axis
#define OUTZ_L_G            ((uint8_t)0x26)
#define OUTZ_H_G            ((uint8_t)0x27)


// accelerometer X axis
#define OUTX_L_XL            ((uint8_t)0x28)
#define OUTX_H_XL            ((uint8_t)0x29)

// accelerometer Y axis
#define OUTY_L_XL            ((uint8_t)0x2A)
#define OUTY_H_XL            ((uint8_t)0x2B)

// accelerometer Z axis
#define OUTZ_L_XL            ((uint8_t)0x2C)
#define OUTZ_H_XL            ((uint8_t)0x2D)

// +-2g range, is 0.061mg/LSB
#define ACC_RANGE            ((float)0.061)

// 0.0175dps/LSB (+-500dps range)
//#define GYRO_RANGE           ((float)0.0175)  

// 0.00875dps/LSB (+-250dps range)
#define GYRO_RANGE           ((float)0.00875)


#endif