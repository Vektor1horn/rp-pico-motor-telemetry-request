/* 
    Authoren: Moritz Kallenbach, Frederik Zimmermann
    (moritz_fynn.kallenbach@mailbox.tu-dresden.de, frederik_luca.zimmermann@mailbox.tu-dresden.de)

    Das macht der Code:
    Steuerung eines KISS ESC32 motor controllers und auslesen der Telemtriedaten.

 */

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#define PWM_PIN 18          // PWM pin
#define PEAK_PIN 16         // Pin to send kea
#define INTERRUPT_TIME 10   // in ms
#define BLOCK_TIME 3000     // in us
#define PEAK_TIME 30        // in us
#define BAUDRATE 115200     // communicationspeed via USB
#define READ_LENGTH 10      // number of Bytes returned by the ESC 
#define UART_ID uart0       // used UART interface


// forward definition to crate PWM and peak
long map(long x, long in_min, long in_max, long out_min, long out_max);
void set_pwm_pin(uint pin, uint freq, uint duty_c);
bool repeating_timer_callback(struct repeating_timer *t);
void sendPeak();

// forward definition to read data
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);
void calculate_values(uint8_t *buff, uint16_t *values);
void core1_entry();


volatile uint16_t PotiRead = 0; // variables to save poti inread

// start of the programm
// inizialize
// programm running on core0
// creating/manupulating PWM and peak (mit hardware_timer)
int main(void)
{  
    
    stdio_init_all();                       // inizialize communitaion with PC
    gpio_init(PEAK_PIN);                    // inizialize peak PIN
    gpio_set_dir(PEAK_PIN, GPIO_OUT);       // set peak PIN as output

    adc_init();                             // inizialize analaog read in
    adc_gpio_init(26);                      // inizialize PIN 26 (ADC0 Pin)
    adc_select_input(0);                    // select ADC0 as input
    set_pwm_pin(PWM_PIN,100,1000);          // inizialize PWM PIN with 100 Hz and 1ms high time

    uart_init(UART_ID, BAUDRATE);           // inizialize UART with uart0 interface
    gpio_set_function(0,GPIO_FUNC_UART);    // setting UART communications PINs
    gpio_set_function(1,GPIO_FUNC_UART); 
    multicore_launch_core1(core1_entry);    // start of core1 running function core1_entry()

    // inizialize timer
    struct repeating_timer timer;
    add_repeating_timer_ms(INTERRUPT_TIME, repeating_timer_callback, NULL, &timer); 
    
    while (1)
    {
        
        // all functions done outside the timer interrupt
        PotiRead = adc_read();                              // read poti
        PotiRead = map(PotiRead, 0, 4096, 1000, 2000);      // map poit data between 1000 an 2000 
        
        pwm_set_gpio_level(PWM_PIN, PotiRead);              // regulating the high time --> corresponds to dutycycle between 10-20%

    }
}

// progrmam running on core1
// reading data from ESC and returning it to the PC
void core1_entry() {
    
    uint8_t buff[READ_LENGTH];      // inizialize of variables to store data read
    uint16_t values[6];

    // watinig until UART is available
    while (uart_is_enabled(UART_ID) == false)
    {
        /* pass */
    }

    // activate UART reading FIFO
    uart_set_fifo_enabled(UART_ID, true);  
    
    //Programmschleife die die wiederholt wird bis Programmabruch
    while (1)
    {
        //  if FIFO readabel read an return it to the PC
        if(uart_is_readable(UART_ID))
        {   
        
            // reading the FIFO (10 bytes)
            uart_read_blocking(UART_ID, buff, READ_LENGTH);
            // deactivate UART reading FIFO to clear it
            uart_set_fifo_enabled(UART_ID, false);
             

            // calculating the pyhsical data
            calculate_values(&buff, &values);
           
           // return of the pyhical data to the PC (via printf)
            printf("%d °C, %.2f V, %.2f A, %d mAh, %.2f Rpm, CRC8: %d", values[0],(float)values[1]/100, (float)values[2]/100, values[3], (float)values[4]*100/12, values[5]);
            if(values[5] != 0)
            {
                printf(" | Data corrupted: ");              // if raw data received wrong --> return raw data 
                for(uint8_t i = 0; i < READ_LENGTH; ++i)
                    printf("%d, ", buff[i]); 
            }
            printf("\n");
            
            // activate UART reading FIFO
            uart_set_fifo_enabled(UART_ID, true);      
        }
    }
        
}

// creating PWM signal
void set_pwm_pin(uint pin, uint freq, uint duty_c) // duty_c between 0..10000us
{ 
    gpio_set_function(pin, GPIO_FUNC_PWM);                          // set PWM PIN --> look up SDK dokumentation
    uint slice_num = pwm_gpio_to_slice_num(pin);                     
    pwm_config config = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (freq * 10000);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 10000); 
    pwm_init(slice_num, &config, true);                             // start the pwm running according to the config
    pwm_set_gpio_level(pin, duty_c);                                // connect the pin to the pwm engine and set the on/off level. 
}

// return read in number between max and min
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min)/ (in_max - in_min) + out_min;
}


// sending peak when timer called
bool repeating_timer_callback(struct repeating_timer *t)
{       

    if(gpio_get(PWM_PIN) == 1)                      // check if PWM signal high
    {
        printf("blocked\n");
        int time = micros();
        while (micros() <= time + BLOCK_TIME)      // if true peak gets delayed
        {
            /* pass */
        }
    }
    sendPeak();                                     // sending out peak
    return true; 
}

// sending high (1) on peak PIN for PEAK_TIME
void sendPeak(){   
    gpio_put(PEAK_PIN, 1);
    int time = micros();
    while(micros() <= time + PEAK_TIME )
    {/* pass */}
    gpio_put(PEAK_PIN, 0);
}

// calculating a part of CRC8 check systems
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{ 
    uint8_t crc_u, i; crc_u = crc; crc_u ^= crc_seed; 
    for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 ); return (crc_u);
}

// retunrs 0, if no error in read in data
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{ 
    uint8_t crc = 0, i; 
    for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc); 
    return (crc);
}

// converts 10 found Integers to 6 values, still needed to be scaled 
void calculate_values(uint8_t *buff, uint16_t *values)
{ 
    // temperature saved as first value, no scaling needed
    values[0] = buff[0];
    //in the loop combining two Byte to one 16 bit nimber
    for (int i = 1; i < 5; i++) {
        values[i] = (buff[(i * 2)-1] << 8 | buff[(i * 2)]); 
        }
    //CRC8 return gets saved
    values[5] = get_crc8(buff, READ_LENGTH);    
}



