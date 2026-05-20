#include <libs_drivers.h>
#include <tmath.h>

#define NUM_SAMPLES  ((uint32_t)(32))


void estimate_start_torque()
{
    float torque_min = 0.0f;
    float torque_max = 0.1f;
    float vel_th     = 30.0f* (2.0f*PI/180.0f);

    float right_torque_start_result[NUM_SAMPLES];
    float left_torque_start_result[NUM_SAMPLES];

    for (unsigned int j = 0; j < NUM_SAMPLES; j++)
    {
        // stop motors
        motor_control.set_right_torque(0);
        motor_control.set_left_torque(0);
        timer.delay_ms(200);

        // estimate initial velocity
        float right_velocity_initial = 0.0f;
        float left_velocity_initial  = 0.0f;

        for (unsigned int n = 0; n < 100; n++)
        {
            right_velocity_initial+= motor_control.get_right_velocity();
            left_velocity_initial+= motor_control.get_left_velocity();
            timer.delay_ms(2);
        }

        right_velocity_initial/= 100;
        left_velocity_initial/= 100;

        
        // start slowly rising torque
        unsigned int n_steps = 50;

        float right_torque_start = torque_max;
        float left_torque_start  = torque_max;

        for (unsigned int n = 0; n < 100; n++)
        {
            float k      = n/(n_steps - 1.0f);
            float torque = (1.0f - k)*torque_min + k*torque_max;

            float right_velocity = motor_control.get_right_velocity();
            float left_velocity  = motor_control.get_left_velocity();

           
            // capture minimal torque for movement start
            if (abs(right_velocity_initial - right_velocity) > vel_th)
                if (abs(torque) < abs(right_torque_start))
                {
                    right_torque_start = torque;
                }

            if (abs(left_velocity_initial - left_velocity) > vel_th)
                if (abs(torque) < abs(left_torque_start))
                {
                    left_torque_start = torque;
                }

            motor_control.set_right_torque(torque);
            motor_control.set_left_torque(torque);
            
            timer.delay_ms(50);
        }    

        terminal << right_torque_start << " " << left_torque_start << "\n";

        right_torque_start_result[j] = right_torque_start;
        left_torque_start_result[j]  = left_torque_start;
    }


    // stop motors
    motor_control.set_right_torque(0);
    motor_control.set_left_torque(0);
    timer.delay_ms(200);

    float Ktr = 0.0f;
    float Ktl = 0.0f;
    for (unsigned int j = 0; j < NUM_SAMPLES; j++)
    {
       Ktr+= right_torque_start_result[j];
       Ktl+= left_torque_start_result[j];
    }

    Ktr = Ktr/(float)NUM_SAMPLES;
    Ktl = Ktl/(float)NUM_SAMPLES;

    terminal << "\n";
    terminal << "torque start " << Ktr << " " << Ktl << "\n";

    terminal << "\n\n\n";
}   


void estimate_asymmetry(float Ktr, float Ktl)
{
    float torque_nom = 0.2f;


    float Ks = 0.0f;    
    float lr = 0.001f;

    uint32_t num_samples = 1000;

    float eps = 0.01f;


    for (unsigned int n = 0; n < num_samples; n++)
    {
        float right_u = (1.0f + Ks)*torque_nom;
        float left_u  = (1.0f - Ks)*torque_nom;

        right_u+= Ktr*sgn(right_u);   
        left_u+= Ktl*sgn(left_u);   

        motor_control.set_right_torque(right_u);
        motor_control.set_left_torque(left_u);
        timer.delay_ms(100);    

        float right_velocity = motor_control.get_right_velocity();
        float left_velocity  = motor_control.get_left_velocity();

        //Ks = (1.0f - lr)*Ks + lr*(left_velocity - right_velocity)/(eps + abs(right_velocity) + abs(left_velocity));

        Ks+= lr*(left_velocity - right_velocity);   
        
        terminal << n << " " << right_velocity << " " << left_velocity << " " << Ks << "\n";
    }   

    motor_control.set_right_torque(0);
    motor_control.set_left_torque(0);
    timer.delay_ms(200);    
}

void FrictionCalibration()
{
    //estimate_start_torque();

    estimate_asymmetry(0.033, 0.034);
    // Ktr = 0.033
    // Ktl = 0.034
    // Ks  = 0.2


    while (1)
    {
        timer.delay_ms(100);
    }

}
