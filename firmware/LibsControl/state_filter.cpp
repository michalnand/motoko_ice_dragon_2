#include "state_filter.h"

void StateFilter::init(float wheel_radius, float wheel_brace, float alpha_pos, float alpha_vel)
{
    this->wheel_radius     = wheel_radius;
    this->wheel_brace      = wheel_brace;   

    this->alpha_pos   = alpha_pos;
    this->alpha_vel   = alpha_vel;

    this->right_initial     = 0.0f;
    this->left_initial      = 0.0f;

    this->distance_est      = 0.0f;
    this->angle_est         = 0.0f;
    this->velocity_est      = 0.0f;
    this->angular_rate_est  = 0.0f;
}



void StateFilter::reset_position(float right_position, float left_position)
{
    this->right_initial = right_position;
    this->left_initial  = left_position;

    this->distance_est      = 0.0f;
    this->angle_est         = 0.0f;
    this->velocity_est      = 0.0f;
    this->angular_rate_est  = 0.0f;
}

void StateFilter::step(float right_position, float right_velocity, float left_position, float left_velocity)
{
    right_position-= this->right_initial;
    left_position-= this->left_initial; 

    // linear distance estimation and filtering
    float distance = this->get_distance(right_position, left_position);
    this->distance_est = (1.0f - this->alpha_pos)*this->distance_est + this->alpha_pos*distance;

    // angle estimation and filtering
    float angle = this->get_angle(right_position, left_position);
    this->angle_est = (1.0f - this->alpha_pos)*this->angle_est + this->alpha_pos*angle;


    // linear velocity estimation   
    float velocity = this->get_distance(right_velocity, left_velocity);
    this->velocity_est = (1.0f - this->alpha_vel)*this->velocity_est + this->alpha_vel*velocity;

    // linear velocity estimation
    float angular_rate = this->get_angle(right_velocity, left_velocity);
    this->angular_rate_est = (1.0f - this->alpha_vel)*this->angular_rate_est + this->alpha_vel*angular_rate;
}


float StateFilter::get_distance(float right_position, float left_position)
{
    return (right_position + left_position)*0.5*this->wheel_radius;
}

float StateFilter::get_angle(float right_position, float left_position)
{
    return (right_position - left_position)*this->wheel_radius/wheel_brace;
}

