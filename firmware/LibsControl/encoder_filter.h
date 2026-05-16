#ifndef _ENCODER_FILTER_H_
#define _ENCODER_FILTER_H_


#include <tmath.h>

class EncoderFilter
{
    public: 
        //dx : number of minimal steps change to refresh velocity b
        //dt : sampling time interval in seconds
        
        void init(int32_t d_min, uint32_t steps_max, float alpha_pos, float alpha_vel, float dt);
        
        
        //update velocity estimator
        //position_now : raw encoder integer steps
        void step(int32_t position_now);
       
       
        private:
            int32_t     d_min;
            uint32_t    steps_max;
            float       alpha_pos, alpha_vel;
            float       dt;

            int32_t     position_prev;
            uint32_t    cnt;

        public:
            float position, position_est;
            float velocity, velocity_est;
};


#endif