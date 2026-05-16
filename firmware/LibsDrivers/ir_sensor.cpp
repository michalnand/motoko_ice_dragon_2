#include <ir_sensor.h>
#include <tmath.h>
#include <libs_drivers.h>







void IRSensor::init()
{
    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        ir_off[i]   = 0;
        ir_on[i]    = 0;
        ir_result[i] = 0;
    }

    ir_led_0  = 0;
    ir_led_1  = 0;
    ir_led_2  = 0;
    ir_led_3  = 0;

   
    //initial state
    state           = 0;    

    filter_coeff    = 0.2;

    measurement_id  = 0;
}   


void IRSensor::callback()
{
    // all leds off, read baseline
    if (state == 0)
    {
        // read IR led off state
        ir_off[0] = adc.get()[0 + IR_SENSOR_OFFSET];
        ir_off[1] = adc.get()[1 + IR_SENSOR_OFFSET];
        ir_off[2] = adc.get()[2 + IR_SENSOR_OFFSET];
        ir_off[3] = adc.get()[3 + IR_SENSOR_OFFSET];

        // turn on first IR led
        ir_led_0  = 1;
        
        state++;
    }
    else if (state == 1)
    {
        // read IR led on state
        ir_on[0] = adc.get()[0 + IR_SENSOR_OFFSET];

        // turn prev IR led off, and current on
        ir_led_0  = 0;
        ir_led_1  = 1;
        
        state++;
    }
    else if (state == 2)
    {
        // read IR led on state
        ir_on[1] = adc.get()[1 + IR_SENSOR_OFFSET];

        // turn prev IR led off, and current on
        ir_led_1  = 0;
        ir_led_2  = 1;
        
        state++;
    }
    else if (state == 3)
    {
        // read IR led on state
        ir_on[2] = adc.get()[2 + IR_SENSOR_OFFSET];

        // turn prev IR led off, and current on
        ir_led_2  = 0;
        ir_led_3  = 1;
        
        state++;
    }
    else if (state == 4)
    {
        // read IR led on state
        ir_on[3] = adc.get()[3 + IR_SENSOR_OFFSET];

        // turn off all leds
        ir_led_3  = 0;
        ir_led_2  = 0;
        ir_led_1  = 0;
        ir_led_0  = 0;
        
        
        state++;
    }
    else if (state == 5)
    {
        // wait state
        state = 0;
    }
    


    // process readed data
    if (state == 0)
    {
        measurement_id++;

        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            float diff   = (ir_off[i] - ir_on[i])/4096.0f;  

            // higher value, higher distance 
            diff         = 1.0f - diff;
            diff         = clip(diff, 0.0f, 1.0f);
            
            // low pass filter
            ir_result[i] = (1.0f - filter_coeff)*ir_result[i] + filter_coeff*diff;
        }
    }
}