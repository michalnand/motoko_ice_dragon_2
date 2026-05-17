#include <libs_drivers.h>

// run both BLDC motors forward, no feedback loop, just simple three phase sine waves
void motor_pwm_test()
{
    PWMLeftThreePhase  left_pwm;
    PWMRightThreePhase right_pwm;

    left_pwm.init();
    right_pwm.init();

    Gpio<'B', 9, GPIO_MODE_OUT> re;
    re = 1;

    Gpio<'C', 9, GPIO_MODE_OUT> le;
    re = 1;
    


    uint32_t phase = 0;

    uint32_t cnt = 0;
    uint32_t inc = 1;

    while (1)   
    {
        uint32_t phase_a    = (phase + (0*SINE_TABLE_SIZE)/3)%SINE_TABLE_SIZE;
        uint32_t phase_b    = (phase + (1*SINE_TABLE_SIZE)/3)%SINE_TABLE_SIZE;
        uint32_t phase_c    = (phase + (2*SINE_TABLE_SIZE)/3)%SINE_TABLE_SIZE;

        uint32_t pwm_a      = (0.4*PWM_VALUE_MAX*sine_table[phase_a])/SINE_VALUE_MAX;
        uint32_t pwm_b      = (0.4*PWM_VALUE_MAX*sine_table[phase_b])/SINE_VALUE_MAX;
        uint32_t pwm_c      = (0.4*PWM_VALUE_MAX*sine_table[phase_c])/SINE_VALUE_MAX;

        left_pwm.set(pwm_a, pwm_b, pwm_c);      
        right_pwm.set(pwm_a, pwm_b, pwm_c);     

        phase+= inc;      
        
        
        if (((cnt%200) == 0) && (inc < 20))
        {
            inc++;
        }   
        

        cnt++;  

        timer.delay_ms(1);
    }
}
