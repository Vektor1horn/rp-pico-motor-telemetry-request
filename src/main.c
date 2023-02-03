#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
//#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
//#include "hardware/adc.h"

#define BAUDRATE 115200
#define READ_LENGTH 10
#define UART_ID uart0
#define BAUDRATE 115200
#define READ_LENGTH 10
#define UART_ID uart0


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


void calculate_values(uint8_t *buff, uint8_t *data)
{ 
    data[0] = buff[0];

    for (int i = 1; i < 5; i++) {
        data[i] = (buff[i * 2] << 8 | buff[(i * 2) + 1]); 
        }

    data[5] = get_crc8(buff, READ_LENGTH);
}



int main()
{
    stdio_init_all();
    uart_init(UART_ID, BAUDRATE);
    gpio_set_function(0,GPIO_FUNC_UART);
    gpio_set_function(1,GPIO_FUNC_UART);

    uint8_t buff[READ_LENGTH];
    uint8_t values[6];
    uint8_t crc_output;
    uint8_t i = 0;

    while (uart_is_enabled(UART_ID) == false){
        //printf("Uart is enabled: %d\n", uart_is_enabled(UART_ID));
    }

    while (1)
    {

        //printf("Uart is readable: %d\n", uart_is_readable(UART_ID));
        if(uart_is_readable(UART_ID))
        {
            //uart_read_blocking(UART_ID, buff, READ_LENGTH);

            /* for(int i = 0; i < READ_LENGTH -1; ++i)
            {
                buff[i] = buff[i+1];
            }

            buff[READ_LENGTH] = uart_getc(UART_ID); */

            buff[i] = uart_getc(UART_ID);
            ++i;
            if(i>9)
            {
                i = 0;
                calculate_values(buff, values);
            printf("%d °C, %d V, %d A, %d mAh, %d Rpm, CRC8: %d\n", values[0],values[1]/100,values[2]/100,values[3],values[4]*100/12,values[5]);
            }

            /* for(int i = 0; i < READ_LENGTH; ++i)
            {
                printf("%d, ", buff[i]);
            } */

            /* calculate_values(buff, values);
            printf("%d °C, %d V, %d A, %d mAh, %d Rpm, CRC8: %d\n", values[0],values[1]/100,values[2]/100,values[3],values[4]*100/12,values[5]);
             */
           
        }
    }
   
    

    

}