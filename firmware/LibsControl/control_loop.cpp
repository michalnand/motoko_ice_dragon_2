#include "control_loop.h"

#define CONTROLLER_UPDATE_RATE_HZ  ((uint32_t)500)

#ifdef __cplusplus  
extern "C" {    
#endif


ControlLoop *g_control_loop_ptr;

// timer 8 interrupt handler, running controller update
void TIM8_UP_TIM13_IRQHandler(void)
{ 
    if (LL_TIM_IsActiveFlag_UPDATE(TIM8))
    {
        LL_TIM_ClearFlag_UPDATE(TIM8);
        g_control_loop_ptr->callback();
    }
} 


#ifdef __cplusplus
}
#endif

int ControlLoop::init()
{
    // instance init, variables set
    g_control_loop_ptr = this;
    
    this->steps = 0;   
    
    this->distance_req = 0.0f;
    this->angle_req    = 0.0f;

    // LQR controller init
        


    /*
    float lqr_kx[8] = {
        0.777260f, 0.020954f, 0.000000f, 0.000000f,
        0.000000f, 0.000000f, 0.052727f, 0.001355f
    };

    float lqr_ki[8] = {
        0.003149f, 0.000000f, 0.000000f, 0.000000f,
        0.000000f, 0.000000f, 0.000314f, 0.000000f
    };


    this->position_controller.init(lqr_kx, lqr_ki, 1.0f, 1.0f);
    */           

    distance_shaper.init(0.1f);
    angle_shaper.init(0.025f);    
    

    pid_forward.init(3.0f, 0.0002f, 0.5f, 1.0f);   
    pid_turn.init(0.3f, 0.0003f, 0.1f, 1.0f);         



    // init periodic timer, running CONTROLLER_UPDATE_RATE_HZ
    this->timer_init();

    return 0;
}


void ControlLoop::set_state(float distance_req, float angle_req)
{
    this->distance_req  = distance_req;
    this->angle_req     = angle_req;
}

void ControlLoop::callback()
{
    // update gyro reading, convert degrees/s to rads/s
    //float gyro_angular_velocity = gyro.read()*2.0f*PI/180.0f;

    // get actual already filtered state
    float distance          = motor_control.get_distance();
    float velocity          = motor_control.get_velocity();
    float angle             = motor_control.get_angle();
    float angular_velocity  = motor_control.get_angular_velocity();

    // desired state
    float distance_req = this->distance_req;    
    float angle_req    = this->angle_req;   

    float angle_diff = angle_req - angle_shaper.get();
    if ( abs(angle_diff) > (PI/2.0f) )  
    {
        angle_req = angle_shaper.get() + (PI/2.0f)*sgn(angle_req);
    }       

    angle_req = angle_shaper.step(angle_req);

    /*
    // update state for controller
    this->position_controller.set_x(0, distance);
    this->position_controller.set_x(1, velocity);
    this->position_controller.set_x(2, angle);
    this->position_controller.set_x(3, angular_velocity);

    // update desired state
    this->position_controller.set_xr(0, distance_req);
    this->position_controller.set_xr(2, angle_req);   
    
    // compute controller output
    this->position_controller.step();

    // controller output is u_forward and u_turn, split into motors
    // values are already clamped for torque constrains
    float u_forward = this->position_controller.get_u(0);
    float u_turn    = this->position_controller.get_u(1);
    */

    float u_forward = 0; //pid_forward.step(distance_req - distance);
    float u_turn    = pid_turn.step(angle_req - angle);

    float u_right   = u_forward + u_turn;   
    float u_left    = u_forward - u_turn;

    motor_control.set_right_torque(u_right);
    motor_control.set_left_torque(u_left);  

    this->steps++;
}


void ControlLoop::timer_init()
{
    // Enable TIM8 clock (on APB2) 
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);

    // Disable TIM8 during configuration 
    LL_TIM_DisableCounter(TIM8);

    // Timer clock = 216 MHz (APB2) 
    // Prescaler = 215 → 216 MHz / 216 = 1 MHz timer tick 
    LL_TIM_SetPrescaler(TIM8, 215);

    // Auto-reload for desired frequency : 1 MHz / CONTROLLER_UPDATE_RATE_HZ 
    // CONTROLLER_UPDATE_RATE_HZ = 200 → ARR = 5000, 200Hz  
    // CONTROLLER_UPDATE_RATE_HZ = 400 → ARR = 2500, 400Hz 
    LL_TIM_SetAutoReload(TIM8, 1000000/CONTROLLER_UPDATE_RATE_HZ - 1);

    // Count direction up 
    LL_TIM_SetCounterMode(TIM8, LL_TIM_COUNTERMODE_UP);

    // No repetition 
    LL_TIM_SetRepetitionCounter(TIM8, 0);

    // Enable update interrupt (UIE) 
    LL_TIM_EnableIT_UPDATE(TIM8);       

    // TIM8 update interrupt, preemption priority = 2 (lower than motor FOC, higher than SysTick) 
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, NVIC_EncodePriority(0, 2, 0));
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    // Clear update flag 
    LL_TIM_ClearFlag_UPDATE(TIM8);

    // Start counter 
    LL_TIM_EnableCounter(TIM8);  
}
    