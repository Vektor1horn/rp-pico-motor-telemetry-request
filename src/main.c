#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "hardware/uart.h"


#define PIN_BUTTON 15       //Sensorpin für den Button
#define PWM_PIN 18          //PWM Pin
#define PEAK_PIN 16         //test Pin um Peak asz tz lesen
#define INTERRUPT_TIME 10  //in ms

#define BLOCK_TIME 3000     //in us
#define PEAK_TIME 30       //in us

#define BAUDRATE 115200
#define READ_LENGTH 10

#define UART_ID uart0

#define FLAG_VALUE 123





//forward definition
long map(long x, long in_min, long in_max, long out_min, long out_max);
void set_pwm_pin(uint pin, uint freq, uint duty_c);
bool repeating_timer_callback(struct repeating_timer *t);
uint64_t alarm_callback(alarm_id_t id, void *user_data);
void sendPeak();

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{ 
    uint8_t crc_u, i; crc_u = crc; crc_u ^= crc_seed; 
    for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 ); return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{ 
    uint8_t crc = 0, i; 
    for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc); 
    return (crc);
}

void calculate_values(uint8_t *buff, uint16_t *values)
{ 
    //Temperatur wird als erster Wert direkt gespeichert, es wird keine Umrechnung benötigt
    values[0] = buff[0];
    //In der For Schleife werden immer zwei Byte zu einer 16 bit Zahl kombiniert und in Values gespeichert
    for (int i = 1; i < 5; i++) {
        values[i] = (buff[(i * 2)-1] << 8 | buff[(i * 2)]); 
        }
    //CRC8 wird als letzter Wert gespeichert: = heißt erfolgreich
    values[5] = get_crc8(buff, READ_LENGTH);    
}


void core1_entry() {
 
    multicore_fifo_push_blocking(FLAG_VALUE);
 
    uint32_t g = multicore_fifo_pop_blocking();
 
    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 1!\n");
    else
        printf("Its all gone well on core 1!");
 
    
    uint8_t buff[READ_LENGTH];
    uint16_t values[6];
    uint8_t i = 0;

    while (uart_is_enabled(UART_ID) == false){
        
    }

    uart_set_fifo_enabled(UART_ID, true);  
    
    while (1)
    {
        if(uart_is_readable(UART_ID))
        {   
        
            uart_read_blocking(UART_ID, buff, READ_LENGTH);
            uart_set_fifo_enabled(UART_ID, false);
             

            
            calculate_values(&buff, &values);
           
            printf("%d °C, %.2f V, %.2f A, %d mAh, %.2f Rpm, CRC8: %d", values[0],(float)values[1]/100, (float)values[2]/100, values[3], (float)values[4]*100/12, values[5]);
             if(values[5] != 0){
                printf(" | Data corrupted: ");
                for(int i = 0; i < READ_LENGTH; ++i)
                    printf("%d, ", buff[i]); 
            }
            printf("\n");
            uart_set_fifo_enabled(UART_ID, true);      
        }
    }
        
}

volatile uint16_t PotiRead = 0;

int main(void)
{  
    //inizialiserung der Pins
    stdio_init_all(); 
    gpio_init(PIN_BUTTON);
    gpio_set_dir(PIN_BUTTON, GPIO_IN);
    gpio_init(PEAK_PIN);
    gpio_set_dir(PEAK_PIN, GPIO_OUT);

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    set_pwm_pin(PWM_PIN,100,1000);

    uart_init(UART_ID, BAUDRATE);
    gpio_set_function(0,GPIO_FUNC_UART);
    gpio_set_function(1,GPIO_FUNC_UART); 
    multicore_launch_core1(core1_entry);

    // Wait for it to start up
 
    uint32_t g = multicore_fifo_pop_blocking();
 
    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 0!\n");
    else {
        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("It's all gone well on core 0!");
    }

    //inizialisierung des Timers
    struct repeating_timer timer;
    add_repeating_timer_ms(INTERRUPT_TIME, repeating_timer_callback, NULL, &timer); 
    
    
    while (1)
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

//Map nimmt einen Wert in einer Range und findet den dazu passenden Wert in einer neuen Range
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min)/ (in_max - in_min) + out_min;
}

bool repeating_timer_callback(struct repeating_timer *t)
{       
    if(gpio_get(PWM_PIN) == 1)
    {
        printf("blocked\n");
        int time = micros();
        while (micros() <= time + BLOCK_TIME)
        {
            /* pass */
        }
    }

    sendPeak();
    return true;
  
}

uint64_t alarm_callback(alarm_id_t id, void *user_data){
    sendPeak();
    return 0;
}

//
void sendPeak(){
    
    
    gpio_put(PEAK_PIN, 1);
    int time = micros();
    
    while(micros() <= time + PEAK_TIME )
    {
        /* pass */
    }
    
    gpio_put(PEAK_PIN, 0);
    //printf("jawoll\n");
    

}
