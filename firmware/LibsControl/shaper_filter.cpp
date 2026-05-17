#include "shaper_filter.h"

ShaperFilter::ShaperFilter()
{
    init(0.0f);
}

ShaperFilter::~ShaperFilter()
{
    
}

void ShaperFilter::init(float k)
{
    this->k = k;
    this->xa = 0.0f;
    this->xb = 0.0f;
}

// x_target : desired position
// returns  : shaped (filtered) position
float ShaperFilter::step(float x)   
{
    this->xa = (1.0f - this->k)*this->xa + this->k*x;
    this->xb = (1.0f - this->k)*this->xb + this->k*this->xa;
    return this->xb;
}   

float ShaperFilter::get()
{
    return this->xb;
}