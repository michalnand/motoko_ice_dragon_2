#include <libs_drivers.h>
#include "as5600_t.h"
#include "tmath.h"

#define LUT_SIZE                ((uint32_t)64)
#define MOTOR_POLES             ((int32_t)14)    

class MotorFOCControl
{
    public:
        void init();
        void set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, uint32_t motor_id);
    
    private:
        int32_t clip(int32_t value, int32_t min_value, int32_t max_value);

    private:
        // motors PWM init
        PWMLeftThreePhase  left_pwm;
        PWMRightThreePhase right_pwm;
};


void MotorFOCControl::init()
{
    // PWM init
    left_pwm.init();
    right_pwm.init();

    // drivers enable
    Gpio<'B', 9, GPIO_MODE_OUT> re;
    re = 1;

    Gpio<'C', 9, GPIO_MODE_OUT> le;
    re = 1; 

    timer.delay_ms(100);
}


void MotorFOCControl::set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, uint32_t motor_id)
{
    torque = clip(torque, -(int32_t)PWM_VALUE_MAX, (int32_t)PWM_VALUE_MAX);

    // convert mechanical angle to electrical angle and index into sine table
    uint32_t table_angle = (rotor_angle*MOTOR_POLES*SINE_TABLE_SIZE)/(2*ENCODER_RESOLUTION);

    if (torque >= 0)
    {   
        table_angle = (table_angle - (3*SINE_TABLE_SIZE)/4) % SINE_TABLE_SIZE;
    }
    else if (torque < 0)          
    {   
        torque = -torque;
        table_angle = (table_angle + (3*SINE_TABLE_SIZE)/4) % SINE_TABLE_SIZE;
    }    

    
    // produce 3-phase shifted angles
    uint32_t angle_a = table_angle % SINE_TABLE_SIZE;
    uint32_t angle_b = (table_angle + SINE_TABLE_SIZE / 3) % SINE_TABLE_SIZE;
    uint32_t angle_c = (table_angle + 2 * (SINE_TABLE_SIZE / 3)) % SINE_TABLE_SIZE;

    // read sinusoidal values
    int32_t sa = sine_table[angle_a];
    int32_t sb = sine_table[angle_b];   
    int32_t sc = sine_table[angle_c];

    int32_t pwm_a = (sa*torque)/SINE_VALUE_MAX;
    int32_t pwm_b = (sb*torque)/SINE_VALUE_MAX; 
    int32_t pwm_c = (sc*torque)/SINE_VALUE_MAX; 


    //pwm values    
    pwm_a = clip(pwm_a, (int32_t)(0), ((int32_t)PWM_VALUE_MAX)-1);
    pwm_b = clip(pwm_b, (int32_t)(0), ((int32_t)PWM_VALUE_MAX)-1);
    pwm_c = clip(pwm_c, (int32_t)(0), ((int32_t)PWM_VALUE_MAX)-1);

    if (motor_id == 0)      
    {
        left_pwm.set(pwm_a, pwm_b, pwm_c);
    }   
    else         
    {
        right_pwm.set(pwm_a, pwm_b, pwm_c);
    }
}

int32_t MotorFOCControl::clip(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value)
    {
        value = min_value;
    }

    if (value > max_value)
    {
        value = max_value;
    }

    return value;
}


// init magnetic encoders
// print readed raw reading (0 .. 4096) and accumulated angle
void encoder_calibration()
{
    // encoder init
    AS5600T<10, 11,  50,  'C'> right_encoder;
    AS5600T<12, 13,  50,  'B'> left_encoder;
 
    right_encoder.init();   
    left_encoder.init();   

    // init FOC
    MotorFOCControl foc_control;
    foc_control.init();
    
    for (unsigned int i = 0; i < 10; i++)     
    {
        // torque, rotor angle, motor ID (left or right)
        int32_t torque = (PWM_VALUE_MAX*i)/10;
        foc_control.set_torque_from_rotation(torque, (90*(int32_t)ENCODER_RESOLUTION)/360, 0);
        foc_control.set_torque_from_rotation(torque, (90*(int32_t)ENCODER_RESOLUTION)/360, 1);
        timer.delay_ms(4);
    }


    // wait for steady tate
    timer.delay_ms(500);

    right_encoder.update();
    left_encoder.update();  

   

    terminal << "initial offset\n";
    terminal << "right encoder " << right_encoder.angle << "\n";
    terminal << "left encoder  " << left_encoder.angle << "\n";
    terminal << "\n\n";      

    /*
    right_encoder.calibrate(right_encoder.angle);
    left_encoder.calibrate(left_encoder.angle);

    // set left physical offset
    //left_encoder.calibrate(3014);

    for (unsigned int n = 0; n < ENCODER_RESOLUTION; n++)
    {            
        right_encoder.update();
        left_encoder.update();  

        timer.delay_ms(1);  

        uint32_t idx = n/(ENCODER_RESOLUTION/LUT_SIZE);

        if ((n%(ENCODER_RESOLUTION/LUT_SIZE)) == 0)
        {
            terminal << n << " ";   
            terminal << right_encoder.angle << " ";
            terminal << left_encoder.angle << " ";
            terminal << "\n";      
        }
        
    
        foc_control.set_torque_from_rotation(PWM_VALUE_MAX*0.3, n, 0);
        foc_control.set_torque_from_rotation(PWM_VALUE_MAX*0.3, n, 1);
    }   
    */


    foc_control.set_torque_from_rotation(0, 0, 0);
    foc_control.set_torque_from_rotation(0, 0, 1);

    timer.delay_ms(100);

    
    while (1)
    {
     
    }
}