#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"


#define PIN_BUTTON 15       //Sensorpin für den Button
#define PWM_PIN 18          //PWM Pin



//volatile uint16_t TimerReset = 0;
volatile uint16_t TimerCount = 0;
volatile uint16_t PotiRead = 0;
volatile bool ButtonPressed = false;

/* int set_pwm_freq (uint slice, int freq, uint32_t *div, uint32_t *top);
int set_pwm_duty (uint slice, uint channel, uint32_t top, uint32_t duty); */
void set_pwm_pin(uint pin, uint freq, uint duty_c);

//Map nimmt einen Wert in einer Range und findet den dazu passenden Wert in einer neuen Range
long map(long x, long in_min, long in_max, long out_min, long out_max);


int main(void)
{  
    //inizialiserung der Pins
    stdio_init_all(); 
    gpio_init(PIN_BUTTON);
    gpio_set_dir(PIN_BUTTON, GPIO_IN);

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    set_pwm_pin(PWM_PIN,100,1000);

    //inizialisierung des Timers
    /* struct repeating_timer timer;
    add_repeating_timer_us(INTERRUPT_TIME, repeating_timer_callback, NULL, &timer); */
    
    while(1)
    {
        
        // alle Funktionen die auserhalb des Timer gemacht werden sollen
        PotiRead = adc_read();                                      //Potentiometer wird ausgelesen 
        PotiRead = map(PotiRead, 0, 4096, 1000, 2000);              //Potentiometerdaten werden auf zwischen 1000 und 2000 gelegt
        if(ButtonPressed == true && gpio_get(PIN_BUTTON) == 0)      
        {
            ButtonPressed = false;
        }
        pwm_set_gpio_level(PWM_PIN, PotiRead);

    }
}

void set_pwm_pin(uint pin, uint freq, uint duty_c) // duty_c between 0..10000
{ 
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (freq * 10000);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 10000); 
    pwm_init(slice_num, &config, true); // start the pwm running according to the config
    pwm_set_gpio_level(pin, duty_c); //connect the pin to the pwm engine and set the on/off level. 
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


