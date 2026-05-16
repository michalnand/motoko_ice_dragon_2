#include <libs_drivers.h>
#include <tmath.h>

// run both BLDC motors forward, and backward, using FOC controll loop
void motor_driver_test()
{
    while (1)   
    {
        for (float v = 0.0f; v < 0.8f; v+=0.01)
        {
            motor_control.set_left_torque(v); 
            motor_control.set_right_torque(v);
            timer.delay_ms(50); 
        }

        timer.delay_ms(1000);

        motor_control.set_left_torque(0.0);
        motor_control.set_right_torque(0.0);
        timer.delay_ms(1000);
    }   
}
