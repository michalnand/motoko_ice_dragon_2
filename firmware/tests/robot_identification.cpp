#include <libs_drivers.h>
#include <tmath.h>
#include <shaper_filter.h>


#define NUM_SAMPLES     ((uint32_t)2000)
#define MODEL_DT        ((uint32_t)2)



uint32_t g_random_var = 0;

uint8_t random()
{
    g_random_var = (g_random_var * 1664525) + 1013904223;
    return (int32_t)g_random_var >> 3;
}

void RobotIdentification()
{
    float u_forward_res[NUM_SAMPLES];
    float u_turn_res[NUM_SAMPLES];
    
    float distance_res[NUM_SAMPLES];
    float velocity_res[NUM_SAMPLES];
    float angle_res[NUM_SAMPLES];
    float angular_velocity_res[NUM_SAMPLES];


    for (unsigned int n = 0; n < NUM_SAMPLES; n++)
    {
        u_forward_res[n]        = 0.0f;
        u_turn_res[n]           = 0.0f;
        
        distance_res[n]         = 0.0f;
        velocity_res[n]         = 0.0f;
        angle_res[n]            = 0.0f;
        angular_velocity_res[n] = 0.0f;
    }

    // led blinking, waiting for key
    Gpio<'C', 4, GPIO_MODE_IN_PULLUP> key_0;

    timer.delay_ms(100);

    
    // wait for key press to start experiment
    while (key_0 == 1)
    {
        led.on(LED::GREEN);  
        timer.delay_ms(100);
        led.off(LED::GREEN);  
        timer.delay_ms(800);
    }   

    while (key_0 == 0)
    {
        led.on(LED::GREEN);  
        timer.delay_ms(100);
        led.off(LED::GREEN);  
        timer.delay_ms(100);
    }

    // random seed
    g_random_var = timer.get_time();

    led.on(LED::GREEN);  

    timer.delay_ms(500); 

    float forward_sign = 0.0f;
    float turn_sign = 0.0f;

    ShaperFilter shaper_distance, shaper_angle;
    shaper_distance.init(0.3f);
    shaper_angle.init(0.3f);  


    for (unsigned int n = 0; n < NUM_SAMPLES; n++)
    {
        uint32_t time_stop = timer.get_time() + MODEL_DT;


        // obtain current state
        float distance            = motor_control.get_distance();
        float velocity            = motor_control.get_velocity();
        float angle               = motor_control.get_angle();
        float angular_velocity    = motor_control.get_angular_velocity();


        // pseudo random binary sequence
        if ((n%20) == 0)    
        {
            switch (random()%5)
            {
                case 0:
                    forward_sign = 0.0f;
                    break;
                
                case 1:
                    forward_sign = 0.5f;
                    break;

                case 2:
                    forward_sign = -0.5f;
                    break;  

                case 3:
                    forward_sign = 1.0f;
                    break;

                case 4:
                    forward_sign = -1.0f;
                    break;
            }


            switch (random()%5)
            {
                case 0:
                    turn_sign = 0.0f;
                    break;
                
                case 1:
                    turn_sign = 0.5f;   
                    break;

                case 2:
                    turn_sign = -0.5f;
                    break;

                case 3:
                    turn_sign = 1.0f;
                    break;

                case 4: 
                    turn_sign = -1.0f;
                    break;
            }
        }

        
        // generate two kind of motions, separated for forward only and turn only
        // the both motions are square wave like pattern with sweeping frequency
        // the switch of direction is when given distance travelled, or angle rotated
        // motions have zero mean value - basically it is simple bang bang control with sweeping frequency
        float u_forward = shaper_distance.step(forward_sign * 0.2f); 
        float u_turn    = shaper_angle.step(turn_sign * 0.2f);          


        // saturation, prioritize turning
        u_turn = clip(u_turn, -1.0f, 1.0f); 

        // remaining energy after turn command
        float max_forward = 1.0f - abs(u_turn);

        // clip forward so left/right outputs remain in [-1, 1]
        u_forward = clip(u_forward, -max_forward, max_forward);

        // convert to right and left motor torque
        float u_r = u_forward + u_turn;
        float u_l = u_forward - u_turn;

        // apply torque, in range -1 .. 1
        motor_control.set_right_torque(u_r);
        motor_control.set_left_torque(u_l);

      
        // store to log
        u_forward_res[n] = u_forward;
        u_turn_res[n]    = u_turn;

        distance_res[n] = distance;
        velocity_res[n] = velocity;
        angle_res[n] = angle;
        angular_velocity_res[n] = angular_velocity;
        

        // dummy wait until dt time
        while (timer.get_time() < time_stop)
        {
            __asm("nop");
        }
    }


    motor_control.set_right_torque(0);
    motor_control.set_left_torque(0);

    led.off(LED::GREEN); 

    timer.delay_ms(200);     
    

    // results printing
    while (true)
    {
        while (key_0 == 1)
        {
            led.on(LED::BLUE);
            timer.delay_ms(100);
            led.off(LED::BLUE);
            timer.delay_ms(900);
        }

        terminal << "\n\n\n";
        
        // print results
        for (unsigned int n = 0; n < NUM_SAMPLES; n++)
        {
            led.on(LED::RED);

            terminal << n << " ";

            terminal << u_forward_res[n] << " ";
            terminal << u_turn_res[n] << " ";

            terminal << distance_res[n] << " ";
            terminal << velocity_res[n] << " ";
            terminal << angle_res[n] << " ";
            terminal << angular_velocity_res[n] << " ";

            terminal << "\n";

            led.off(LED::RED);
            timer.delay_ms(5);
        }

        terminal << "\n\n\n";
    }
}