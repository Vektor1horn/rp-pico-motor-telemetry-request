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

#define PWM_PIN 18          //PWM Pin
#define PEAK_PIN 16         //Pin um Peak zu senden
#define INTERRUPT_TIME 10   //in ms
#define BLOCK_TIME 3000     //in us
#define PEAK_TIME 30        //in us
#define BAUDRATE 115200     //Kommunikationsgeschwindigkeit über USB
#define READ_LENGTH 10      //Anzahl der Bytes die vom ESC ausgegeben werden
#define UART_ID uart0       //genutzte UART Schnittstelle
#define FLAG_VALUE 123      //Ausgabe wenn 2ter Code richtig gestartet ist


//forward Definition für setzen des PWM Signals und des Peaks
long map(long x, long in_min, long in_max, long out_min, long out_max);
void set_pwm_pin(uint pin, uint freq, uint duty_c);
bool repeating_timer_callback(struct repeating_timer *t);
void sendPeak();

//forward Definition um Daten auszulesen
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);
void calculate_values(uint8_t *buff, uint16_t *values);
void core1_entry();


volatile uint16_t PotiRead = 0; //Variable um den eingelesenen Poti Wert zu verwenden


//Start des Porgramms
//Porgramm das auf 1ten Core läuft
//Inizialister alles
//Setzt das PWM Signals, Setzt den Peak (mit hardware_timer)
int main(void)
{  
    
    stdio_init_all();                       //Inizialiserung aller Kommunikation mit dem PC
    gpio_init(PEAK_PIN);                    //Inizialiserung des Pins auf dem der Peak ausgesendet wird
    gpio_set_dir(PEAK_PIN, GPIO_OUT);       //setzten des Peak Pins als Output

    adc_init();                             //Inizialiserung des analogen Einlesens
    adc_gpio_init(26);                      //Inizialiserung auf Pin 26 (geht nur auf PIn 26, 27 und 28)
    adc_select_input(0);                    //Wählt des ADC_Pin 0 --> entspircht GPIO_Pin 26
    set_pwm_pin(PWM_PIN,100,1000);          //Inizialiserung des PWM Pins mit 100 Hz und 1ms High Time

    uart_init(UART_ID, BAUDRATE);           //Inizialiserung von UART mit der uart0 Schnittstelle des PI
    gpio_set_function(0,GPIO_FUNC_UART);    //setzen der UART Kommunikations Pins für UART
    gpio_set_function(1,GPIO_FUNC_UART); 
    multicore_launch_core1(core1_entry);    //Starten des 2ten Cores mit dem Programm core1_entry

    //inizialisierung des Timers
    struct repeating_timer timer;
    add_repeating_timer_ms(INTERRUPT_TIME, repeating_timer_callback, NULL, &timer); 
    
    while (1)
    {
        
        // alle Funktionen die auserhalb des Timer gemacht werden sollen
        PotiRead = adc_read();                              //Potentiometer wird ausgelesen 
        PotiRead = map(PotiRead, 0, 4096, 1000, 2000);      //Potentiometerdaten werden auf zwischen 1000 und 2000 gelegt
        
        pwm_set_gpio_level(PWM_PIN, PotiRead);              //setzen der Hightime

    }
}

//Programm das auf 2ten Core läuft
//liest die gesendeten Daten des ESC's und gibt diese an den PC per USB aus
void core1_entry() {
    
    uint8_t buff[READ_LENGTH];      //Inizialisierung von Variablen zum Speichern der ausgelesenen Werte
    uint16_t values[6];

    //warten und check bis UART gestartet ist
    while (uart_is_enabled(UART_ID) == false)
    {
        /* pass */
    }

    //aktiviert den FIFO
    uart_set_fifo_enabled(UART_ID, true);  
    
    //Programmschleife die die wiederholt wird bis Programmabruch
    while (1)
    {
        //Wenn FIFO des UART auslesbar ist wird schleife gestartet
        if(uart_is_readable(UART_ID))
        {   
        
            //liest den FIFO aus (10 Bytes)
            uart_read_blocking(UART_ID, buff, READ_LENGTH);
            //deaktiviert FIFO um ihn zu lehren
            uart_set_fifo_enabled(UART_ID, false);
             

            //berechnet aus eingelesenen Bytes die pyhsikalischen Werte
            calculate_values(&buff, &values);
           
           //Ausgabe der physikalischen Daten
            printf("%d °C, %.2f V, %.2f A, %d mAh, %.2f Rpm, CRC8: %d", values[0],(float)values[1]/100, (float)values[2]/100, values[3], (float)values[4]*100/12, values[5]);
            if(values[5] != 0)
            {
                printf(" | Data corrupted: ");              //wenn pyhsikalische Daten unplausibel --> Rohdaten mit ausgegben
                for(uint8_t i = 0; i < READ_LENGTH; ++i)
                    printf("%d, ", buff[i]); 
            }
            printf("\n");
            
            //aktiviert den FIFO
            uart_set_fifo_enabled(UART_ID, true);      
        }
    }
        
}

//erstellen eines PWM Signals
void set_pwm_pin(uint pin, uint freq, uint duty_c) // duty_c between 0..10000
{ 
    gpio_set_function(pin, GPIO_FUNC_PWM);                          //setzen des PWM_Pins
    uint slice_num = pwm_gpio_to_slice_num(pin);                    //siehe SDK Dokumentatins PWM Signal
    pwm_config config = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (freq * 10000);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 10000); 
    pwm_init(slice_num, &config, true);                             // start the pwm running according to the config
    pwm_set_gpio_level(pin, duty_c);                                //connect the pin to the pwm engine and set the on/off level. 
}

//Map nimmt einen Wert in einer Range und findet den dazu passenden Wert in einer neuen Range
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min)/ (in_max - in_min) + out_min;
}


//Wird über Timer aufgerufen und sendet den Peak Pin an den ESC
bool repeating_timer_callback(struct repeating_timer *t)
{       

    if(gpio_get(PWM_PIN) == 1)                      //checkt ob PWM Signal gerade in der Hightime ist
    {
        printf("blocked\n");
        int time = micros();
        while (micros() <= time + BLOCK_TIME)      //wenn ja wird Peak um die Blockzeit verzögert
        {
            /* pass */
        }
    }
    sendPeak();                                     //senden des Peaks
    return true; 
}

//Sendet eine 1 auf dem PEAK_PIN für PEAK_TIME
void sendPeak(){   
    gpio_put(PEAK_PIN, 1);
    int time = micros();
    while(micros() <= time + PEAK_TIME )
    {/* pass */}
    gpio_put(PEAK_PIN, 0);
}

//berechnet ein Teil des CRC8 Check Systems
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{ 
    uint8_t crc_u, i; crc_u = crc; crc_u ^= crc_seed; 
    for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 ); return (crc_u);
}

//Gibt eine 0 zurück, falls keine Fehler in dem Datensatz gefunden wurde
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{ 
    uint8_t crc = 0, i; 
    for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc); 
    return (crc);
}

//Wandelt die 10 gefundenen Integer in 6 Werte um, die noch skalliert werden müssen
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



