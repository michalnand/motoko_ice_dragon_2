#include <libs_drivers.h>
#include <tmath.h>

#include <control_loop.h>
#include <shaper_filter.h>

void PositionControlTest()
{
    ControlLoop control_loop;
    control_loop.init();


    uint32_t steps_initial = control_loop.steps;
    timer.delay_ms(100);
    uint32_t steps_now = control_loop.steps;
    
    terminal << "control loop frequency = ";
    terminal << (steps_now - steps_initial)*10;
    terminal << "\n\n\n";

    uint32_t cnt = 0;   

    //float distance_req_all[] = {0.0f, 100.0f, 0.0f, 0.0f};
    //float angle_req_all[]    = {0.0f,  0.0f, 0.0f, 90.0f};

    float distance_req_all[] = {0.0f, 0.0f, 0.0f, 0.0f};
    float angle_req_all[]    = {0.0f, 90.0f, 0.0f, 180.0f};


   

    uint32_t t_initial = timer.get_time();  
    while (1)        
    {       
        uint32_t idx = (cnt/100)%4;     

        float distance_req = distance_req_all[idx]*0.001;
        float angle_req    = angle_req_all[idx]*PI/180.0f;

        
        // set control value        
        control_loop.set_state(distance_req, angle_req);

        // obtain current state
        float distance            = motor_control.get_distance();
        float velocity            = motor_control.get_velocity();
        float angle               = motor_control.get_angle();
        float angular_velocity    = motor_control.get_angular_velocity();


        uint32_t t = timer.get_time() - t_initial;

        // print result
        terminal << t << " ";
        terminal << distance_req << " ";
        terminal << angle_req << " ";

        terminal << distance << " ";
        terminal << velocity << " ";
        terminal << angle << " ";
        terminal << angular_velocity << " ";

        terminal << "\n";

        cnt++;
        timer.delay_ms(4); 
    }    
}
