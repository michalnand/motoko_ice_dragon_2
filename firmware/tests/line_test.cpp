#include <libs_drivers.h>
#include <tmath.h>

// raw read adc channels
// print raw readings and how many measurements per second
void line_test()
{
    while (1)
    {
        uint32_t counter_prev = adc.measurement_id;
        timer.delay_ms(100);
        uint32_t counter_now = adc.measurement_id;

        uint32_t fps = (counter_now - counter_prev)*10;



        led.on(LED::GREEN);

        terminal << "adc_result : ";
        for (unsigned int i = 0; i < line_sensor.adc_result.size(); i++)
            terminal << line_sensor.adc_result[i] << " ";
        terminal << "\n";

        for (unsigned int i = 0; i < line_sensor.adc_calibration_q.size(); i++)
            terminal << line_sensor.adc_calibration_q[i] << " ";
        terminal << "\n";

        for (unsigned int i = 0; i < line_sensor.adc_calibration_k.size(); i++)
            terminal << line_sensor.adc_calibration_k[i] << " ";
        terminal << "\n";

        terminal << "\n\n";
            
        terminal << "\n";   

        terminal << "line_lost_type =   " << line_sensor.line_lost_type << "\n";
        terminal << "on_line_count  =   " << line_sensor.on_line_count << "\n";
        terminal << "left_position  =   " << line_sensor.left_position << "\n";
        terminal << "right_position =   " << line_sensor.right_position << "\n";
        terminal << "fps            =   " << fps << "\n";
        
        terminal << "\n\n\n";

        led.off(LED::GREEN);
    }
}
