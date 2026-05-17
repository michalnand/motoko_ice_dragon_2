#ifndef _CONTROL_LOOP_H_
#define _CONTROL_LOOP_H_

#include <libs_drivers.h>
#include <tmath.h>

#include <pid.h>
#include <shaper_filter.h>

class ControlLoop
{
    public:
        int init();
        void set_state(float distance_req, float angle_req);

        void callback();
    
    private:
        void timer_init();

    public:
        uint32_t steps;


    private:    
        float distance_req, angle_req;
        
        //LQRIRobot<4> position_controller;

        ShaperFilter distance_shaper, angle_shaper;

        PID pid_turn, pid_forward;

};

#endif
