#include <libs_drivers.h>
#include <tmath.h>



void position_test()
{
    uint32_t cnt = 0;
    uint32_t t_initial = timer.get_time();
   
    while (1)    
    {       
        float distance          = motor_control.get_distance();
        float angle             = motor_control.get_angle();
        float velocity          = motor_control.get_velocity();
        float angular_velocity  = motor_control.get_angular_velocity();

        float gyro_velocity = gyro.read();

        // distance to mm
        // velocity mm/s
        // angle to degrees
        // angular rate to deg/s
        distance = distance*1000.0f;
        angle    = angle*180.0f/PI;

        velocity            = velocity*1000.0f;
        angular_velocity    = angular_velocity*180.0f/PI;
        //gyro_velocity       = gyro_velocity*180.0f/PI;

            
        
      
        uint32_t t = timer.get_time() - t_initial;
        
        if ((cnt%50) == 0)
        {
            terminal << t << "   ";
            /*
            terminal << motor_control.get_right_position()*180.0f/PI << "  ";
            terminal << motor_control.get_left_position()*180.0f/PI << "  ";
            terminal << motor_control.get_right_velocity()*60.0f/(2.0f*PI) << "  ";
            terminal << motor_control.get_left_velocity()*60.0f/(2.0f*PI) << "  ";
            */
            
            terminal << distance << "  ";
            terminal << angle << "  ";
            terminal << velocity << "  ";
            terminal << angular_velocity << "  ";
            terminal << gyro_velocity << "  ";
            
            
            terminal << "\n";
        }

        cnt++;
        timer.delay_ms(4); 
    }  
    
    
}
