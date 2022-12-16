#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define PIN_D0 18           //Ausgangspin für PWM
#define PIN_BUTTON 15       //Sensorpin für den Button
//#define INTERRUPT_TIME 15   //Anzahl der us nachdem der Timer erneut aufgerufen werden soll
#define PWM_PIN 18
#define DUTY_10_PCT 6553
// Maximum "top" is set at 65534 to be able to achieve 100% duty with 65535.
#define TOP_MAX 65534



//volatile uint16_t TimerReset = 0;
volatile uint16_t TimerCount = 0;
volatile uint16_t PotiRead = 0;
volatile bool ButtonPressed = false;

int set_pwm_freq (uint slice, int freq, uint32_t *div, uint32_t *top);
int set_pwm_duty (uint slice, uint channel, uint32_t top, uint32_t duty);
long map(long x, long in_min, long in_max, long out_min, long out_max);


int main(void)
{  
    //inizialiserung der Pins
    stdio_init_all(); 
    //gpio_init(PIN_D0);
    gpio_init(PIN_BUTTON);
    //gpio_set_dir(PIN_D0, GPIO_OUT);
    gpio_set_dir(PIN_BUTTON, GPIO_IN);

    //erstellen von Variablen für PWM Signal
    uint32_t div = 0, top = 0;
    uint slice_num, channel;
 
    //generiert sclie_num und channel aus der Nummer die PWM_PIN zugeordet ist
    slice_num = pwm_gpio_to_slice_num (PWM_PIN);
    channel = pwm_gpio_to_channel (PWM_PIN);
    gpio_set_function (PWM_PIN, GPIO_FUNC_PWM);


    // Set up a 100 hz freq PWM
    set_pwm_freq (slice_num, (int)100, &div, &top);

    // Set the PWM counter wrap value to reset on
    pwm_set_wrap (slice_num, top);
    
    //inizialisierung des Timers
    /* struct repeating_timer timer;
    add_repeating_timer_us(INTERRUPT_TIME, repeating_timer_callback, NULL, &timer); */
    
    for(;;)
    {
        
        // alle Funktionen die auserhalb des Timer gemacht werden sollen
        PotiRead = adc_read();                                      //Potentiometer wird ausgelesen 
        PotiRead = map(PotiRead, 0, 4096, 0, 1000);              //Potentiometerdaten werden auf zwischen 1000 und 2000 gelegt
        if(ButtonPressed == true && gpio_get(PIN_BUTTON) == 0)      
        {
            ButtonPressed = false;
        }

        set_pwm_duty (slice_num, channel, top, (uint32_t) DUTY_10_PCT + DUTY_10_PCT*PotiRead/1000);
    }

    
}

// Code used from https://github.com/raspberrypi/micropython/blob/pico/ports/rp2/machine_pwm.c
// Function - machine_pwm_freq
// Shaped for my needs (Scott Beasley) No Copyright.
// MIT License (MIT) Damien P. George Copyright (c) 2020
int set_pwm_freq (uint slice, int freq, uint32_t *div, uint32_t *top) 
{
    // Set the frequency, making "top" as large as possible for maximum resolution.
    *div = (uint32_t)(16 * clock_get_hz (clk_sys) / (uint32_t)freq);
    *top = 1;
    for (;;) {
        // Try a few small prime factors to get close to the desired frequency.
        if (*div >= 16 * 5 && *div % 5 == 0 && *top * 5 <= TOP_MAX) {
            *div /= 5;
            *top *= 5;
        } else if (*div >= 16 * 3 && *div % 3 == 0 && *top * 3 <= TOP_MAX) {
            *div /= 3;
            *top *= 3;
        } else if (*div >= 16 * 2 && *top * 2 <= TOP_MAX) {
            *div /= 2;
            *top *= 2;
        } else {
            break;
        }
    }
 
    if (*div < 16) {
        *div = 0;
        *top = 0;
        return -1; // freq too large
    } else if (*div >= 256 * 16) {
        *div = 0;
        *top = 0;
        return -2; // freq too small
    }
 
    return 0;
}
 
// Code used from https://github.com/raspberrypi/micropython/blob/pico/ports/rp2/machine_pwm.c
// Function - machine_pwm_duty_u16
// Shaped for my needs (Scott Beasley) No Copyright.
// MIT License (MIT) Damien P. George Copyright (c) 2020
int set_pwm_duty (uint slice, uint channel, uint32_t top, uint32_t duty)
{
    // Set duty cycle.
    uint32_t cc = duty * (top + 1) / 65535;
    pwm_set_chan_level (slice, channel, cc);
    pwm_set_enabled (slice, true);
 
    return 0;
}



long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min)/ (in_max - in_min) + out_min;
}

/* bool repeating_timer_callback(struct repeating_timer *t)
{
    //wenn der Timer 0 ist soll PWM 1 sein
    if(TimerCount == 0) 
    {
        gpio_put(PIN_D0, 1);
    }

    //wenn der Timer gleich 1 ms + die PotiReadZeit erreicht hat soll PWM 0 sein
    if(TimerCount == PotiRead*(1000/INTERRUPT_TIME)) 
    {
        gpio_put(PIN_D0, 0);
    }

    //erhöht den Timer Count um 1
    TimerCount++; //erhöht den Timer Count um 1

    //Wenn 10 ms erreicht sind soll PWM wieder von vorne anfangen --> TimerCount 0
    if(TimerCount >= 10000/INTERRUPT_TIME) 
    {
        TimerCount = 0;
    }

    //wenn Knopf gedrückt und in der Mitte des PWM Intervalls 
    if(gpio_get(PIN_BUTTON) == 1 && TimerCount == (int) 5000/INTERRUPT_TIME && ButtonPressed == false)
    {
        gpio_put(PIN_D0,1);
        ButtonPressed = true;
    
    }

    if(TimerCount == 2 + (int)5000/INTERRUPT_TIME)
    {
        gpio_put(PIN_D0, 0);
    }
   

    return true;
} */


