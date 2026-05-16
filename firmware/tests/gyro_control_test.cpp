#include <libs_drivers.h>
#include <tmath.h>

#include <pid_controller.h>

// run both BLDC motors forward, no feedback loop, just simple three phase sine waves
void gyro_control_test()
{
    uint32_t cnt = 0;
    uint32_t dt  = 3;

    float angle = 0;
    
    PIDController pid;

    float kp = 0.8f;
    float ki = 0.0f;
    float kd = 1.0f;
    
    pid.init(kp, ki, kd, MOTOR_CONTROL_MAX_VELOCITY);

    while (1)   
    {   
        led.on(LED::GREEN); 

        uint32_t status = gyro.read_status();
        float res = gyro.read();    

        angle+= res*dt*0.001;

        float u = pid.step(0, angle);

            
        motor_control.set_right_velocity(u);        
        motor_control.set_left_velocity(-u);
  
        led.off(LED::GREEN);    

        timer.delay_ms(dt);
        cnt++;
    }
}
