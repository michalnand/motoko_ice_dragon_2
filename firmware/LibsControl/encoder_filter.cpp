#include "encoder_filter.h"

#define ENCODER_RESOLUTION  (int32_t)(4096)


//dx : number of minimal steps change to refresh velocity b
//dt : sampling time interval in seconds
void EncoderFilter::init(int32_t d_min, uint32_t steps_max, float alpha_pos, float alpha_vel, float dt)
{
    this->d_min     = d_min;
    this->steps_max = steps_max;
    this->alpha_pos = alpha_pos;
    this->alpha_vel = alpha_vel;
    this->dt        = dt;

    this->position_prev = 0;

    this->position      = 0.0f;
    this->position_est  = 0.0f;

    this->velocity      = 0.0f;
    this->velocity_est  = 0.0f;

    this->cnt = 0;
}
        

//update velocity estimator
//position_now : raw encoder integer steps
void EncoderFilter::step(int32_t position_now)
{
    // change triggered velocity estimation
    int32_t diff = position_now - this->position_prev;

    if ( ((abs(diff) >= this->d_min) && (this->cnt >= 1)) || (this->cnt >= this->steps_max) )
    {
        // convert raw velocity into rad/s 
        this->velocity  = (diff/(cnt*dt))*(2.0f*PI/ENCODER_RESOLUTION);
        this->position_prev = position_now;
        this->cnt = 0;
    }

    // convert position to radians
    this->position = position_now*(2.0f*PI/ENCODER_RESOLUTION);
    
    // EMA for position filtering
    this->position_est = (1.0f - this->alpha_pos)*this->position_est + this->alpha_pos*this->position;

    // EMA filtered velocity
    this->velocity_est = (1.0f - this->alpha_vel)*this->velocity_est + this->alpha_vel*this->velocity;

    this->cnt++;
}

       