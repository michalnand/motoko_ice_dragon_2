#include "libs_drivers.h"


// global drivers are instantianed here

TI2C<'B', 15, 14, 20> gyro_i2c;

LED         led;
Terminal    terminal;
Timer       timer;    

Gyro        gyro;

ADC_driver  adc;

LineSensor   line_sensor;
IRSensor     ir_sensor;

MotorControl motor_control;


void blink_error(uint32_t error)
{
    while (1)
    {
        for (unsigned int i = 0; i < error; i++)
        {
            led.on(LED::RED);
            timer.delay_ms(100);
            led.off(LED::RED);
            timer.delay_ms(400);
        }

        timer.delay_ms(1000);
    }
}


void LibsDriversInit()
{
    // low level init, cache, clock
    drivers_init();  

    // uart initialisation
    uart_init();

    led.init();

    // terminal init
    terminal.init();

    terminal << "\n\n\n";
    terminal << "terminal init done\n";


    // timer
    timer.init();

    timer.delay_ms(200);

    
    // IMU init
    gyro_i2c.init();
    int gyro_init_res = gyro.init(gyro_i2c);

    if (gyro_init_res == 0)
    {
        terminal << "gyro init done\n";
        terminal << "gyro offset " << gyro.g_yaw_ofs << "\n";
        terminal << "gyro result " << gyro.g_yaw << "\n";
    }   
    else
    {
        terminal << "gyro init FAILED\n";
        blink_error(1);
    }   
    
    /*
    // ADC init
    adc.init(); 
    terminal << "ADC init done\n";

    // line sensor init
    line_sensor.init();
    terminal << "line sensor init done\n";

    // distance IR sensor init
    ir_sensor.init();
    terminal << "IR sensor init done\n";
    */    
    
    
    // motor control init
    int motor_init_res = motor_control.init();

    if (motor_init_res == 0)
    {
        terminal << "motor init done\n";
    }
    else    
    {
        terminal << "motor init FAILED\n";
        blink_error(2);
    }
    

    led.on(LED::BLUE);
}