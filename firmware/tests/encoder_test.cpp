#include <libs_drivers.h>
#include "as5600_t.h"

// init magnetic encoders
// print readed raw reading (0 .. 4096) and accumulated angle
void encoder_test()
{
    /*
    AS5600T<10, 11,  50,  'C'> right_encoder;
    AS5600T<12, 13,  50,  'B'> left_encoder;
 
    right_encoder.init();
    left_encoder.init();    

    uint32_t cnt = 0;
    while (1)
    {       
        right_encoder.update();
        left_encoder.update();
        

        timer.delay_ms(1);  

        if ((cnt%100) == 0) 
        {
            terminal << right_encoder.angle << " " << right_encoder.position << " " << left_encoder.angle << " " <<  left_encoder.position << "\n";
        }

        cnt++;
    }
    */


    uint32_t cnt = 0;
    while (1)
    {       
        timer.delay_ms(1);  

        if ((cnt%100) == 0) 
        {
            terminal << motor_control.get_right_position_raw() << " " <<  motor_control.get_left_position_raw() << "\n";
        }

        cnt++;
    }
}