#include <libs_drivers.h>
#include <tmath.h>

#define BATCH_SIZE  ((uint32_t)(32))

void FrictionCalibration()
{
    float angle_range  = 30.0f*PI/180.0f;
    float torque_nom   = 0.15f;
    float torque_max   = 0.2f;

    float lr = 0.01f;      

    float torque_turn = 0.0f;

    float Kv = 0.0f;
    
    float Ktr = 0.0f;
    float Ktl = 0.0f; 

    uint32_t state = 0;

    uint32_t cnt = 0;

    // read initial wheel conditions
    float right_initial = 0.0f;
    float left_initial = 0.0f;

    for (unsigned int i = 0; i < 100; i++)
    {
        right_initial+= motor_control.get_right_position();
        left_initial+= motor_control.get_left_position();
        timer.delay_ms(2);
    }

    right_initial = right_initial/100.0f;
    left_initial  = left_initial/100.0f;


    uint32_t batch_ptr = 0;
    float asymmetry_batch[BATCH_SIZE];
    float right_torque_batch[BATCH_SIZE];
    float left_torque_batch[BATCH_SIZE];

    while (1)
    {
        // current state
        float right_position = motor_control.get_right_position() - right_initial;
        float left_position  = motor_control.get_left_position()  - left_initial;
        float angle          = right_position - left_position;

      

        /*
        // side by side robot moving
        if (state == 0)
        { 
            torque_turn = torque_nom;
            if (angle > angle_range)
            {
                state = 1;
            }
        }
        else
        {
            torque_turn = -torque_nom;
            if (angle < -angle_range)
            {
                state = 0;
            }
        }
        */

        // compute torque
        float right_torque  = torque_turn;
        float left_torque   = -torque_turn;

          
        float asymmetry = (right_position + left_position);    

        // collect batch
        asymmetry_batch[batch_ptr]      = asymmetry;
        right_torque_batch[batch_ptr]   = right_torque;
        left_torque_batch[batch_ptr]    = left_torque;

        batch_ptr++;
        
        // add torque compensation
        float right_torque_corr = (1.0f + Kv)*right_torque + Ktr*sgn(right_torque);
        float left_torque_corr  = (1.0f - Kv)*left_torque  + Ktl*sgn(left_torque);

        right_torque_corr = clip(right_torque_corr, -torque_max, torque_max);
        left_torque_corr  = clip(left_torque_corr, -torque_max, torque_max);

        // send to motors
        motor_control.set_right_torque(right_torque_corr);
        motor_control.set_left_torque(left_torque_corr);

        // time delay
        timer.delay_ms(2);




        // batch full, training
        if (batch_ptr >= BATCH_SIZE)
        {
            float eps = 0.0000001f;
            
            // Kv estimation
            {
                float diff = 0.0f;
                for (unsigned int n = 0; n < BATCH_SIZE; n++)
                {
                    diff+= asymmetry_batch[n]*(right_torque_batch[n] - left_torque_batch[n]);
                }
                Kv+= -lr*diff/((float)BATCH_SIZE);
            }   
            
            Kv = clip(Kv, -0.8f, 0.8f);



            // Ktr estimation
            {
                float diff = 0.0f;
                for (unsigned int n = 0; n < BATCH_SIZE; n++)
                {
                    diff+= asymmetry_batch[n]*sgn(right_torque_batch[n]);
                }
                Ktr+= -0.1f*lr*diff/((float)BATCH_SIZE);
            }   

            Ktr = clip(Ktr, 0.0f, 1.0f);

            // Ktl estimation
            {
                float diff = 0.0f;
                for (unsigned int n = 0; n < BATCH_SIZE; n++)
                {
                    diff+= asymmetry_batch[n]*sgn(left_torque_batch[n]);
                }
                Ktl+= -0.1f*lr*diff/((float)BATCH_SIZE);
            }   

            Ktl = clip(Ktl, 0.0f, 1.0f);
            


            batch_ptr = 0;

            float asymmetry_mean = 0.0f;
            for (unsigned int n = 0; n < BATCH_SIZE; n++)
            {
                asymmetry_mean+= asymmetry_batch[n]*asymmetry_batch[n];
            }   
            asymmetry_mean = asymmetry_mean/BATCH_SIZE;

            // print
            terminal << cnt << " ";
            terminal << asymmetry_mean << " ";
            terminal << Kv << " ";
            terminal << Ktr << " ";
            terminal << Ktl << " ";
            terminal << "\n";

            // stop motors
            motor_control.set_right_torque(0);
            motor_control.set_left_torque(0);

            // time delay
            timer.delay_ms(200);    

            // new initial pos
            right_initial = 0.0f;
            left_initial = 0.0f;
            for (unsigned int i = 0; i < 100; i++)
            {
                right_initial+= motor_control.get_right_position();
                left_initial+= motor_control.get_left_position();
                timer.delay_ms(2);
            }

            right_initial = right_initial/100.0f;
            left_initial  = left_initial/100.0f;



            if (state == 0)
            { 
                torque_turn = torque_nom;
                state = 1;
            }
            else
            {
                torque_turn = -torque_nom;
                state = 0;
            }

        }
        
        cnt++;
    }

}
