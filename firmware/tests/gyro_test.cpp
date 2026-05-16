#include <libs_drivers.h>



// run both BLDC motors forward, no feedback loop, just simple three phase sine waves
void gyro_test()
{
    if (gyro.read_status() == 1)
    {
        terminal << "gyro init DONE\n";
    }
    else
    {
        terminal << "gyro init ERROR\n";
    }

    uint32_t cnt = 0;
    uint32_t dt  = 5;

    float angle = 0;
    
    while (1)
    {   
        uint32_t status = gyro.read_status();
        float res = gyro.read();    

        angle+= res*dt*0.001;

        timer.delay_ms(dt);
        
        if ((cnt%50) == 0)
        {
            led.on(LED::GREEN);
            terminal << status << " "  << gyro.g_yaw_ofs << " " << res << " " << angle << "\n";
            led.off(LED::GREEN);
        }

        cnt++;
    }
}
