#ifndef _MOTOR_CONTROL_BLDC_H_
#define _MOTOR_CONTROL_BLDC_H_

#include <as5600_t.h>
#include <tmath.h>

#include <encoder_filter.h>
#include <state_filter.h>

// 2000rpm max speed, convert to rad/s
#define MOTOR_CONTROL_MAX_VELOCITY  ((float)2000.0f*2.0f*PI/60.0f)

#define WHEEL_RADIUS_MM             (float(28.0f/2.0f))   
#define WHEEL_BRACE_MM              (float(79.0f))        




class MotorControl
{
    
    public:
        enum ControlMode
        {
            TORQUE    = 0,
            VELOCITY  = 1,
            POSITION  = 2,
        };


        int init();

        // desired state setting
    public:
        void set_right_torque(float right_torque);
        void set_left_torque(float left_torque);

        void set_left_velocity(float left_velocity);
        void set_right_velocity(float right_velocity);
        

        void reset_position();

        // encoder state getting
    public:
        // returns wheel traveled distance in rad
        float get_right_position();

        // returns wheel velocity, in rad/s
        float get_left_position();

        // returns wheel traveled distance in rad
        float get_right_velocity();

        // returns wheel velocity, in rad/s
        float get_left_velocity();  


        // robot state getting
    public:
        float get_distance();
        float get_angle();

        float get_velocity();
        float get_angular_velocity();

    public:
        // unfiltered wheel state, mostly for debug, data collection, filters tunning
        float get_right_position_raw();
        float get_left_position_raw();  

        float get_right_velocity_raw();
        float get_left_velocity_raw();  

     

    public:
        void callback();
        void set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, uint32_t motor_id);

    private:
        void timer_init();

    private: 
        AS5600T<10, 11,  50,  'C'> right_encoder;
        AS5600T<12, 13,  50,  'B'> left_encoder;

        PWMRightThreePhase right_pwm;
        PWMLeftThreePhase  left_pwm;

        EncoderFilter right_filter, left_filter;
        StateFilter   state_filter;

        LQRMotor right_controller, left_controller;
    
        

    public:
        uint32_t steps;

    private:
        float left_req_torque, right_req_torque;
        float left_req_velocity, right_req_velocity;

        ControlMode  left_cl_mode, right_cl_mode;
};


#endif

