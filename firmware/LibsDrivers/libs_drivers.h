#ifndef _LIBS_DRIVERS_H_
#define _LIBS_DRIVERS_H_

#include <drivers.h>
#include <common.h>

#include "button.h"
#include "led.h"

#include "adc_driver.h"

#include "gyro.h"

#include "line_sensor.h"
#include "ir_sensor.h"

#include "motor_pwm.h"  
#include "motor_control_bldc.h"


// availible from all files
extern LED          led;
extern Terminal     terminal;
extern Timer        timer;

extern Gyro         gyro;

extern ADC_driver   adc;

extern LineSensor   line_sensor;
extern IRSensor     ir_sensor;


extern MotorControl motor_control;

void LibsDriversInit();

#endif

