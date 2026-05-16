#pragma once

#include "drivers.h"
#include "gpio.h"

class LED
{
    public:
        enum LedId
        {
            RED   = 0,
            GREEN = 1,
            BLUE  = 2,
        };

        void init();
        void on(LedId id);
        void off(LedId id);

        void all_off();

        
    private:
        void set(LedId id, int value);

        Gpio<'B', 2, GPIO_MODE_OUT> led_0; // red
        Gpio<'B', 10, GPIO_MODE_OUT> led_1; // green
        Gpio<'B', 11, GPIO_MODE_OUT> led_2; // blue
    
};