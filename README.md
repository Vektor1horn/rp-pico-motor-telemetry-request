English:
## Raspberry Pi Pico --> KISS ESC-32bit control/readout

This is a project where the Raspberry Pi Pico is used to read the telemetry data of the KISS ESC-32bit and output it to a PC.

* The VS code extensions Plaftomr IO and the [**framework**](https://github.com/Wiz-IO/wizio-pico) written for it by Github user : [**Wiz-IO**](https://github.com/Wiz-IO) are used.

* The program generates a PWM singal (100Hz) on PIN 24 which controls the motor speed via the KISS ESC-32bit.

* The duty cycle of the program can be set between 10-20%. For this purpose a potentiometer is read in on PIN 31.

* To request the telemetry data from the KISS ESC-32bit a 30us peak is generated on PIN 21. This can be superimposed with diodes with the PWM signal.

* The KISS ESC-32bit will output 10 bytes (via UART protocol) of data at its output.

* This output must be pulled up to pin 2 on the RP Pico. The use of a pull-up resistor is necessary to get unaltered data.

* The calculated physical values are then sent to the PC.

* The data can be read out via a serial monitor with a baud rate of 115200.

Translated with www.DeepL.com/Translator (free version)
 <p>
German:
## Raspberry Pi Pico --> KISS ESC-32bit Ansteuerung/Auslesen

Dies ist ein Projekt bei welchem der Raspberry Pi Pico genutzt wird um die Telemtetrie daten des KISS ESC-32bit aus zu lesen unf auf einem PC aus zu geben.

* Gentutz werden die VS-Code Extentions Plaftomr IO sowie das dazu geschriebene [**Framework**](https://github.com/Wiz-IO/wizio-pico) von Github User : [**Wiz-IO**](https://github.com/Wiz-IO)

* Das Programm erzeugt ein PWM Singal (100Hz) auf PIN 24 welches die Motorgeschwindikeit über den KISS ESC-32bit steuert.
* Der Duty Cycle des Programmes kann zwischen 10-20% eingestellt werden. Dazu wird ein Potentionmeter auf PIN 31 eingelesen.

* Um die Telemetriedaten vom KISS ESC-32bit anzufordern wir ein 30us Peak auf PIN 21 erzeugt. Diesr mit mit Dioden mit dem PWM Singal überlagert werden.

* Der KISS ESC-32bit wird 10 Bytes (über das UART Protokoll) an Daten an seinem Ausgang ausgegeben.
* Dieser Ausgng muss am RP Pico auf Pin 2 gezogen werden. Dabei ist die Verwendung eines Pull-Up Wiederstandes notwendig um unverauschte Daten zu bekommen.
* Die errechneten Physikalischen Werte werden anschließend an den PC ausgegben.
* Die Daten könne über einen Serial-Monitor mit einer Baudrate von 115200 ausgelsen werden.
