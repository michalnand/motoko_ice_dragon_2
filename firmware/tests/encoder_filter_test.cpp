#include <libs_drivers.h>
#include <tmath.h>



void encoder_filter_test()
{
    // 300 rpm to rad/s
    float required_velocity  = 200*2.0*PI/60.0;

    
    uint32_t cnt = 0;

    float torque[] = {0.0f, 0.05f, 0.07f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f};

    uint32_t t_initial = timer.get_time();
    while (1)   
    {               
        uint32_t idx = (cnt/400)%8;
        float required_torque = torque[idx];

        // left wheel velocity in rad/s
        // velocity already filtered via alpha beta filter

        motor_control.set_right_torque(required_torque);
        motor_control.set_left_torque(required_torque); 

        // rad/s to rpm
        float right_velocity_raw      = motor_control.get_right_velocity_raw()*60.0f/(2.0*PI);
        float right_velocity_observed = motor_control.get_right_velocity()*60.0f/(2.0*PI);
        
        float left_velocity_raw      = motor_control.get_left_velocity_raw()*60.0f/(2.0*PI);
        float left_velocity_observed = motor_control.get_left_velocity()*60.0f/(2.0*PI);
        
            
        uint32_t t = timer.get_time() - t_initial;
        
        terminal << t << " " <<required_torque << " ";  

        terminal << right_velocity_raw << " " << right_velocity_observed << " ";
        terminal << left_velocity_raw << " " << left_velocity_observed << " ";
        terminal << "\n";
        
        
        cnt++;  
        timer.delay_ms(4); 
    }   
}
