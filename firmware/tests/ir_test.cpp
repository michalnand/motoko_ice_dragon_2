#include <libs_drivers.h>
#include <tmath.h>

// raw read adc channels
// print raw readings and how many measurements per second
void ir_test()
{
    while (1)
    {
        uint32_t counter_prev = ir_sensor.measurement_id;
        timer.delay_ms(100);
        uint32_t counter_now = ir_sensor.measurement_id;

        uint32_t fps = (counter_now - counter_prev)*10;

        led.on(LED::GREEN);

        terminal << "fps = " << fps << "\n";

        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            terminal << ir_sensor.ir_result[i] << " ";
        }
        terminal << "\n\n\n";   

        led.off(LED::GREEN);
    }
}
