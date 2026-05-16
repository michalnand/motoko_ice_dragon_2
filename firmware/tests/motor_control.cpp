#include <libs_drivers.h>
#include <tmath.h>



void motor_control_test()
{
    /*
    uint32_t cnt = 0;   

    float position[] = {0, 90, 0, 180, 0, 360};

    uint32_t t_initial = timer.get_time();  
    while (1)   
    {       
        uint32_t idx = (cnt/50)%2;
        float required_position = position[idx];

       

        motor_control.set_right_position(required_position*PI/180.0f);
        motor_control.set_left_position(required_position*PI/180.0f);
        
        // rads to degrees
        float observed_right_position = motor_control.state.right_position_est*180.0f/PI;
        float observed_left_position  = motor_control.state.left_position_est*180.0f/PI;

        uint32_t t = timer.get_time() - t_initial;
        
        terminal << t << " " << required_position << " " << observed_right_position << " " << observed_left_position << "\n";

        cnt++;
        timer.delay_ms(4); 
    }
    */

    
    uint32_t cnt = 0;   

    float velocity[] = {0, 20, 60, 100, 200, 500};

    uint32_t t_initial = timer.get_time();  

    while (1)    
    {       
        uint32_t idx = (cnt/200)%6; 
        float required_velocity = velocity[idx];

            
        // rpm to rads/     
        motor_control.set_right_velocity(required_velocity*2.0f*PI/60.0f);
        motor_control.set_left_velocity(required_velocity*2.0f*PI/60.0f);   

        // rad/s to rpm
        float right_velocity_raw      = motor_control.get_right_velocity_raw()*60.0f/(2.0*PI);
        float right_velocity_observed = motor_control.get_right_velocity()*60.0f/(2.0*PI);
        
        float left_velocity_raw      = motor_control.get_left_velocity_raw()*60.0f/(2.0*PI);
        float left_velocity_observed = motor_control.get_left_velocity()*60.0f/(2.0*PI);
        

        uint32_t t = timer.get_time() - t_initial;
        
        terminal << t << " " << required_velocity << " ";
        terminal << right_velocity_raw << " " << right_velocity_observed << " ";
        terminal << left_velocity_raw << " " << left_velocity_observed << "\n";

        cnt++;
        timer.delay_ms(4); 
    }  
    
    
}
