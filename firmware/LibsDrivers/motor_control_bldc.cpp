#include <libs_drivers.h>
#include <motor_control_bldc.h>

#include <tmath.h>
#include <sine_table.h>
    
#define MOTOR_TIMER_FREQ             ((uint32_t)2000)
#define MOTOR_CONTROL_DT             ((float)(1.0f/MOTOR_TIMER_FREQ))
#define MOTOR_CONTROL_MAX_TORQUE     ((float)1.0)
    
#define MOTOR_POLES             ((int32_t)14)       

#define SQRT3       ((int32_t)1773)      // sqrt(3)     = 1773/1024
#define SQRT3INV    ((int32_t)591)       // 1/sqrt(3)   = 591/1024

#define max2(x,y) (((x) >= (y)) ? (x) : (y)) 
#define min2(x,y) (((x) <= (y)) ? (x) : (y))

#define max3(x, y, z) (max2(max2(x, y), z))
#define min3(x, y, z) (min2(min2(x, y), z))



#ifdef __cplusplus
extern "C" {
#endif

MotorControl *g_motor_control_ptr;

// timer 2 interrupt handler, running velocity control
void TIM2_IRQHandler(void)
{ 
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
    {
        LL_TIM_ClearFlag_UPDATE(TIM2);
        g_motor_control_ptr->callback();
    }
} 


#ifdef __cplusplus
}
#endif


//init motor control process
int MotorControl::init()
{
    g_motor_control_ptr = this;

    this->steps = 0;

    this->left_req_torque         = 0;
    this->right_req_torque        = 0;

    this->left_req_velocity     = 0.0f;
    this->right_req_velocity    = 0.0f;


    this->left_cl_mode  = TORQUE;
    this->right_cl_mode = TORQUE;

    ControlMode  left_cl_mode, right_cl_mode;

    // PWM init
    left_pwm.init();    
    right_pwm.init();

    // drivers enable
    Gpio<'B', 9, GPIO_MODE_OUT> re;
    re = 1;

    Gpio<'C', 9, GPIO_MODE_OUT> le;
    re = 1; 


    // init encoders
    int right_init_res = right_encoder.init();   
    int left_init_res  = left_encoder.init(); 

    // calibrate zero offset
    right_encoder.calibrate(3018);
    left_encoder.calibrate(1883);

    // refresh encoders 
    right_encoder.update();      
    left_encoder.update();    
    

    //right_filter.init(4, 32, 0.1f, 0.1f, MOTOR_CONTROL_DT);
    //left_filter.init(4, 32, 0.1f, 0.1f, MOTOR_CONTROL_DT);

    //right_filter.init(1, 20, 0.1f, 0.05f, MOTOR_CONTROL_DT);
    //left_filter.init(1, 20, 0.1f, 0.05f, MOTOR_CONTROL_DT);

    //right_filter.init(1, 20, 0.1f, 0.1f, MOTOR_CONTROL_DT);
    //left_filter.init(1, 20, 0.1f, 0.1f, MOTOR_CONTROL_DT);

    right_filter.init(3, 20, 0.1f, 0.1f, MOTOR_CONTROL_DT);
    left_filter.init(3, 20, 0.1f, 0.1f, MOTOR_CONTROL_DT);


    state_filter.init(WHEEL_RADIUS_MM/1000.0f, WHEEL_BRACE_MM/1000.0f, 1.0f, 0.1f);



    // LQR controller init  
    float Kx  =  0.00246767;    
    float Ki  =  9.83273537e-05; 

    right_controller.init(Kx, Ki, 1.0f);
    left_controller.init(Kx, Ki, 1.0f);

    //init timer
    timer_init();   


    timer.delay_ms(100);
    this->reset_position();
  

    if (left_init_res != 0)
    {
        return -1;
    }

    if (right_init_res != 0)
    {
        return -2;
    }

    return 0;
}


// range : -1, 1, for min and max torque
void  MotorControl::set_right_torque(float right_torque)
{
    this->right_cl_mode = TORQUE;
    this->right_req_torque  = right_torque;
    
}

// range : -1, 1, for min and max torque
void MotorControl::set_left_torque(float left_torque)
{
    this->left_cl_mode = TORQUE;
    this->left_req_torque   = left_torque;
}

// range : -MOTOR_CONTROL_MAX_VELOCITY, MOTOR_CONTROL_MAX_VELOCITY
void  MotorControl::set_right_velocity(float right_velocity)
{
    this->right_cl_mode = VELOCITY; 
    this->right_req_velocity  = right_velocity;
    
}

// range : -MOTOR_CONTROL_MAX_VELOCITY, MOTOR_CONTROL_MAX_VELOCITY
void MotorControl::set_left_velocity(float left_velocity)
{
    this->left_cl_mode = VELOCITY;
    this->left_req_velocity   = left_velocity;
}   


void MotorControl::reset_position()
{
    state_filter.reset_position(this->right_filter.position_est, this->left_filter.position_est);
}
    


// return smooth wheels encoder observed state

float MotorControl::get_right_position()
{
    return this->right_filter.position_est;
}

float MotorControl::get_left_position()
{
    return this->left_filter.position_est;
}

float MotorControl::get_right_velocity()
{
    return this->right_filter.velocity_est;
}

float MotorControl::get_left_velocity()
{
    return this->left_filter.velocity_est;
}

// robot odometry state
float MotorControl::get_distance()
{
    return this->state_filter.distance_est;
}

float MotorControl::get_angle()
{
    return this->state_filter.angle_est;
}

float MotorControl::get_velocity()
{   
    return this->state_filter.velocity_est;
}

float MotorControl::get_angular_velocity()
{
    return this->state_filter.angular_rate_est;
}



// raw encoder state, mostly for debug

float MotorControl::get_right_position_raw()
{
    return this->right_filter.position;
}

float MotorControl::get_left_position_raw()
{
    return this->left_filter.position;
}

float MotorControl::get_right_velocity_raw()
{
    return this->right_filter.velocity;
}

float MotorControl::get_left_velocity_raw()
{
    return this->left_filter.velocity;
}





void MotorControl::callback()
{   
    // refresh encoders
    right_encoder.update();      
    left_encoder.update();     
    
    // encoder noise filtering
    this->right_filter.step(right_encoder.position);
    this->left_filter.step(-left_encoder.position);


    this->state_filter.step(this->right_filter.position_est, this->right_filter.velocity_est, this->left_filter.position_est, this->left_filter.velocity_est);
    

    float ks = 0.0; 

    if (this->right_cl_mode == ControlMode::VELOCITY)
    {
        this->right_controller.x_req = right_req_velocity;
        this->right_controller.x_obs = this->right_filter.velocity_est;
        
        this->right_controller.step();  

        this->right_req_torque = this->right_controller.u + ks*sgn(right_req_velocity);
    }

    if (this->left_cl_mode == ControlMode::VELOCITY)
    {
        this->left_controller.x_req = left_req_velocity;
        this->left_controller.x_obs = this->left_filter.velocity_est;
        
        this->left_controller.step();

        this->left_req_torque = this->left_controller.u + ks*sgn(left_req_velocity);
    }   

    

    // scale -1...1 range into -PWM_VALUE_MAX .. PWM_VALUE_MAX
    // send torques to motors   
    set_torque_from_rotation(this->right_req_torque*PWM_VALUE_MAX, right_encoder.angle, 1);
    set_torque_from_rotation(-this->left_req_torque*PWM_VALUE_MAX, left_encoder.angle, 0);
    

    this->steps++;  
}









void MotorControl::timer_init()
{
    /* Enable TIM2 clock (on APB1) */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    /* Disable TIM2 during configuration */
    LL_TIM_DisableCounter(TIM2);

    /* Timer clock = 216 MHz (APB1 doubled) */
    /* Prescaler = 215 → 216 MHz / 216 = 1 MHz timer tick */
    LL_TIM_SetPrescaler(TIM2, 215);

    /* Auto-reload for 4 kHz: 1 MHz / 250 = 4000 Hz */
    LL_TIM_SetAutoReload(TIM2, 1000000/MOTOR_TIMER_FREQ);


    /* Enable update interrupt (UIE) */
    LL_TIM_EnableIT_UPDATE(TIM2);       

    // TIM2 highest priority, p = 0
    NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(3, 0, 0));
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Clear update flag */
    LL_TIM_ClearFlag_UPDATE(TIM2);

    /* Start counter */
    LL_TIM_EnableCounter(TIM2);
}





void MotorControl::set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, uint32_t motor_id)
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
        left_pwm.set(pwm_c, pwm_b, pwm_a);
    }   
    else         
    {
        right_pwm.set(pwm_a, pwm_b, pwm_c);
    }
}   
