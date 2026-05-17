#ifndef _LQRI_ROBOT_H_
#define _LQRI_ROBOT_H_

#include "matrix.h"

/*
    LQR - linear quadratic regulator for differential drive robot

    system_order    : system order, num of states
    system_inputs   : num of controllable inputs into plat

    u[0] : forward motion control
    u[1] : turn motion control
*/
template<uint32_t system_order, uint32_t system_inputs = 2>
class LQRIRobot
{

    public:
        void init(float *Kx, float *Ki, float u_forward_max, float u_turn_max)
        {   
            this->x.init();
            this->xr.init();
            
            this->u.init();
            this->z.init();

            this->Kx.from_array(Kx);
            this->Ki.from_array(Ki);    

            this->u_forward_max = u_forward_max;
            this->u_turn_max    = u_turn_max;
        }

        void step() 
        {   
            // integral action  
            auto error = this->xr - this->x; 

            // integrate in u-space for simpler antiwindup
            auto z_new = z + this->Ki*error;

            // LQR control law  
            auto u_new = this->Kx*error + z_new;

            float u_forward = u_new[0];
            float u_turn    = u_new[1];

          
            // prioritize turning, remaining energy for forward motion
            u_turn = clip(u_turn, -this->u_turn_max, this->u_turn_max); 

            // remaining energy after turn command
            float max_forward = this->u_forward_max - abs(u_turn);

            // clip forward so left/right outputs remain in [-1, 1]
            u_forward = clip(u_forward, -max_forward, max_forward);

            this->u[0] = u_forward;
            this->u[1] = u_turn;


            // antiwindup with back calculating integration
            // second term is active only if u_new is outside of already clipped u range
            this->z = z_new - (u_new - this->u);
        }

        void set_xr(uint32_t idx, float value)
        {
            this->xr[idx] = value;
        }

        void set_x(uint32_t idx, float value)
        {
            this->x[idx] = value;
        }

        float get_u(uint32_t idx)
        {
            return this->u[idx];
        }
        

    private:
        //inputs and outputs
        Matrix<float, system_order, 1>      x;
        Matrix<float, system_order, 1>      xr;
        Matrix<float, system_inputs, 1>     u; 

        // integral term
        Matrix<float, system_inputs, 1>  z; 

    private:
        float u_forward_max, u_turn_max;

        //controller gain
        Matrix<float, system_inputs, system_order> Kx;
        Matrix<float, system_inputs, system_order> Ki;
};

#endif