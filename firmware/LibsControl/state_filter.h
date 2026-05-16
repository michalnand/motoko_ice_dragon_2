#ifndef _STATE_FILTER_H_
#define _STATE_FILTER_H_


#include <tmath.h>

class StateFilter
{
    public: 
        void init(float wheel_radius, float wheel_brace, float alpha_pos, float alpha_vel);
        void reset_position(float right_position, float left_position);
        void step(float right_position, float right_velocity, float left_position, float left_velocity);

    private:
        float get_distance(float right_position, float left_position);
        float get_angle(float right_position, float left_position);

    public:
        float distance_est, angle_est;
        float velocity_est, angular_rate_est;

    private:
        float right_initial, left_initial; 
        
        float wheel_radius, wheel_brace;
        float alpha_pos, alpha_vel;
};


#endif