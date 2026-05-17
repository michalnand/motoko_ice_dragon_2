#include <libs_drivers.h>
#include <tmath.h>


#include <control_loop.h>
#include <shaper_filter.h>

void gyro_control_test()
{
   
    
    ControlLoop control_loop;
    control_loop.init();

    ShaperFilter distance_shaper, angle_shaper;

    distance_shaper.init(0.5f);
    angle_shaper.init(0.5f);


    uint32_t cnt = 0;
    uint32_t dt  = 4;

    float angle = 0;
    
    while (1)   
    {   
        led.on(LED::GREEN); 

        uint32_t status = gyro.read_status();
        
        float res = gyro.read()*PI/180.0f;    
        angle+= res*dt*0.001;


        float distance_req = distance_shaper.step(0.0f);
        float angle_req    = angle_shaper.step(-angle);

        
        // set control value        
        control_loop.set_state(distance_req, angle_req);

        // obtain current state
        float distance            = motor_control.get_distance();
        float velocity            = motor_control.get_velocity();
        float angle               = motor_control.get_angle();
        float angular_velocity    = motor_control.get_angular_velocity();
        

        led.off(LED::GREEN);    

        timer.delay_ms(dt);
        cnt++;
    }
}
