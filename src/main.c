#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
//#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
//#include "hardware/adc.h"
#include "pico/multicore.h"
 

#define BAUDRATE 115200
#define READ_LENGTH 10
#define UART_ID uart0

#define FLAG_VALUE 123


//forward definition
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

int main()
{
    stdio_init_all();
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
    
    while (1)
    {
        tight_loop_contents();
    }
}