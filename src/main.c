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
#define PEAK_PIN 19
#define READ_PIN 20
#define INTERRUPT_TIME 10 
#define ALARM_TIMER 3




volatile uint16_t PotiRead = 0;
//volatile bool ButtonPressed = false;

void set_pwm_pin(uint pin, uint freq, uint duty_c);
void repeating_timer_callback(struct repeating_timer *t);
uint64_t alarm_callback(alarm_id_t id, void *user_data);
void sendPeak();

//Map nimmt einen Wert in einer Range und findet den dazu passenden Wert in einer neuen Range
long map(long x, long in_min, long in_max, long out_min, long out_max);


int main(void)
{  
    //inizialiserung der Pins
    stdio_init_all(); 
    gpio_init(PIN_BUTTON);
    gpio_set_dir(PIN_BUTTON, GPIO_IN);
    gpio_init(PEAK_PIN);
    gpio_set_dir(PEAK_PIN, GPIO_IN);

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    set_pwm_pin(PWM_PIN,100,1000);

    //inizialisierung des Timers
    struct repeating_timer timer;
    add_repeating_timer_ms(INTERRUPT_TIME, repeating_timer_callback, NULL, &timer); 
    
    
    while(1)
    {
        
        // alle Funktionen die auserhalb des Timer gemacht werden sollen
        PotiRead = adc_read();                                      //Potentiometer wird ausgelesen 
        PotiRead = map(PotiRead, 0, 4096, 1000, 2000);              //Potentiometerdaten werden auf zwischen 1000 und 2000 gelegt
        
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

void repeating_timer_callback(struct repeating_timer *t)
{

    if(gpio_get(READ_PIN) != 0){
    
       //add_alarm_in_ms(ALARM_TIMER, alarm_callback, NULL,false);
       printf("mist");
    }
    else{

        sendPeak();
    }
  
}

uint64_t alarm_callback(alarm_id_t id, void *user_data){
    sendPeak();
    return 0;
}

void sendPeak(){
    
    
    gpio_put(PEAK_PIN, 1);
    sleep_us(100);
    gpio_put(PEAK_PIN, 0);
    printf("jawoll");
    

}

