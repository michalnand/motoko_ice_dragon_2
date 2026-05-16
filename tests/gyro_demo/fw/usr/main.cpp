#include "drivers.h"
#include "common.h"
#include "tmath.h"

#include "imu_ll.h"

Terminal terminal;
Timer timer;
TI2C<'B', 7, 8, 5> i2c;

#include "tmath.h"

// my device address
#define DEVICE_ADR   ((uint8_t)100)

int main() 
{       
    drivers_init();   
    uart_init(); 

        
    Gpio<'A', 5> led_1;
    Gpio<'A', 6> led_2;


    //Terminal terminal;
    terminal << "\n\nuart init done\n";

    timer.init();
    i2c.init();

    IMULL imu_ll;
    
    // turn on led if reading done
    if (imu_ll.init(i2c) == 0)
    {
        led_2 = 1;
    }

    float yaw_sum = 0;

    uint32_t time_now  = timer.get_time();
    uint32_t time_prev = time_now;

    while (1)
    {
        time_prev   = time_now;
        time_now    = timer.get_time();
        float dt    = (time_now - time_prev)*0.001; // convert to seconds

        led_2 = 0;
        led_1 = 1;

        int status = imu_ll.read_status();
        imu_ll.read();  

        yaw_sum += imu_ll.g_yaw*dt;
        
        //terminal <<  time_now << " " << imu_ll.g_yaw << " "<<  yaw_sum << " " <<  imu_ll.g_yaw_ofs << "\n";
        

        terminal << "\n\n>";
        terminal << "{\n";  
        terminal << "\"status\"  : " << status << ",\n";
        terminal << "\"time\"  : " << time_now << ",\n";
        terminal << "\"gyro\" : [" << imu_ll.g_yaw << ", " << yaw_sum << "]\n";
        terminal << "}\n";
        terminal << "<";

        
        delay_loops(10000);
        
        if (status == 1)
        {
            led_2 = 1;
        }

        led_1 = 0;
        delay_loops(100000);
    }



   
    return 0;
}
