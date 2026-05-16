#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_

#include <drivers.h>
#include <stdint.h>
#include <array.h>


#define IR_SENSORS_COUNT        ((unsigned int)4)


/*
    IR front left   :  idx 0,  LL_ADC_CHANNEL_9
    IR left         :  idx 1,  LL_ADC_CHANNEL_8
    IR right        :  idx 2,  LL_ADC_CHANNEL_11
    IR front right  :  idx 3,  LL_ADC_CHANNEL_10
*/

class IRSensor
{
    public:
        void init();

        void callback();

        float obstacle_distance();
        int obstacle_detected();
   

    public:
        uint32_t measurement_id;
        Array<float, IR_SENSORS_COUNT> ir_result;

    private: 
        uint32_t state; 

        Gpio<'A', 8, GPIO_MODE_OUT>  ir_led_0;   // IR front left 
        Gpio<'C', 12, GPIO_MODE_OUT> ir_led_1;   // IR left 
        Gpio<'D', 2, GPIO_MODE_OUT>  ir_led_2;   // IR right 
        Gpio<'C', 3, GPIO_MODE_OUT>  ir_led_3;   // IR front right

        Array<int, IR_SENSORS_COUNT> ir_off;
        Array<int, IR_SENSORS_COUNT> ir_on;


        float filter_coeff;


};

#endif