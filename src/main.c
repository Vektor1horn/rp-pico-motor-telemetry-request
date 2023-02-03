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


int main()
{
    stdio_init_all();
    uart_init(UART_ID, BAUDRATE);
    gpio_set_function(0,GPIO_FUNC_UART);
    gpio_set_function(1,GPIO_FUNC_UART);

    uint8_t buff[READ_LENGTH];
    uint8_t crc_output;

    while (uart_is_enabled(UART_ID) == false){

    }

    while (1)
    {
        if(uart_is_readable(UART_ID) == READ_LENGTH)
        {
            uart_read_blocking(UART_ID, buff, READ_LENGTH);
            crc_output = get_crc8(buff, READ_LENGTH);

            for(int i = 0; i < READ_LENGTH; ++i)
            {
                pico_printf("%d, ", i);
            }

            pico_printf(" CRC8 Auswertung ist %d\n", crc_output);
        }
    }
    

}