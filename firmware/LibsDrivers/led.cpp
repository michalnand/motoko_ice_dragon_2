#include "led.h"

void LED::init()
{
    // 1 = off, 0 = on (NPN transistor circuit)
    led_0 = 1;
    led_1 = 1;
    led_2 = 1;
}

void LED::on(LedId id)
{
   set(id, 0); // active low
}   

void LED::off(LedId id)
{
   set(id, 1); // active low
}


void LED::all_off()
{
    set(RED, 1);
    set(GREEN, 1);
    set(BLUE, 1);
}

void LED::set(LedId id, int value)
{
    switch (id)
    {
        case RED:    led_0 = value; break;
        case GREEN:  led_1 = value; break;
        case BLUE:   led_2 = value; break;
        default: break;
    }
}

