#include <stdio.h>
#include "pico/stdlib.h"
#define LED_PIN 0

int main(void)
{  
    stdio_init_all();
    printf("\n\nHello World\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
}
