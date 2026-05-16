#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_



class PIDController
{   
    public:
        PIDController();
        
        virtual ~PIDController();
        
        //k  : gain
        //ki : integral action gain
        void init(float  kp, float ki, float kd, float antiwindup);
        
        //xr required value
        //x  actual value
        float step(float xr, float x);
        
        void reset(float initial_u = 0);
    private:
        float _clip(float v, float min_v, float max_v);

       
    private:
        float k0, k1, k2;
        float e0, e1, e2;
        
        float antiwindup;

    public:
        float u;
};

#endif
