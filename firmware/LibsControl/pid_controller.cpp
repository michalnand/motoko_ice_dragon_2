#include "pid_controller.h"



PIDController::PIDController()
{
    this->k0 = 0;
    this->k1 = 0;
    this->k2 = 0;

    this->e0 = 0;
    this->e1 = 0;
    this->e2 = 0;

    this->antiwindup = 0;
    this->u = 0;
}


PIDController::~PIDController()
{

} 

void PIDController::init(float  kp, float ki, float kd, float antiwindup)
{
    this->k0 = kp + ki + kd;
    this->k1 = -kp - 2*kd;
    this->k2 = kd;

    this->antiwindup = antiwindup;
    this->u = 0;

    this->e0 = 0;
    this->e1 = 0;
    this->e2 = 0;
}

float PIDController::step(float xr, float x)
{
    this->e2 = this->e1;
    this->e1 = this->e0;
    this->e0 = xr - x;

    float du = this->k0*this->e0 + this->k1*this->e1 + this->k2*this->e2;

    this->u = _clip(this->u + du, -antiwindup, antiwindup);
    

    return this->u;
}

void PIDController::reset(float initial_u)
{
    this->e0 = 0;
    this->e1 = 0;
    this->e2 = 0;

    this->u = initial_u;
}



float PIDController::_clip(float v, float min_v, float max_v)
{
    if (v < min_v)
    {
        v = min_v;
    }
    else if (v > max_v)
    {
        v = max_v;
    }

    return v;
}
