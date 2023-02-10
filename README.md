Dies ist ein Projekt bei welchem der Raspberry Pi Pico genutzt wird um die Telemtetrie daten des KISS ESC-32bit aus zu lesen unf auf einem PC aus zu geben.

* Gentutz werden die VS-Code Extentions Plaftomr IO sowie das dazu geschriebene [**Framework**] (https://github.com/Wiz-IO/wizio-pico) von Github User : Wiz-IO

Das Programm erzeugt ein PWM Singal (100Hz) auf PIN 24 welches die Motorgeschwindikeit über den KISS ESC-32bit steuert.
Der Duty Cycle des Programmes kann zwischen 10-20% eingestellt werden. Dazu wird ein Potentionmeter auf PIN 31 eingelesen.

Um die Telemetriedaten vom KISS ESC-32bit anzufordern wir ein 30us Peak auf PIN 21 erzeugt.
Diesr mit mit Dioden mit dem PWM Singal überlagert werden.

Der KISS ESC-32bit wird 10 Bytes (über das UART Protokoll) an Daten an seinem Ausgang ausgegeben.
Dieser Ausgng muss am RP Pico auf Pin 2 gezogen werden. Dabei ist die Verwendung eines Pull-Up Wiederstandes notwendig um unverauschte Daten zu bekommen.
Die errechneten Physikalischen Werte werden anschließend an den PC ausgegben.
Die Daten könne über einen Serial-Monitor mit einer Baudrate von 115200 ausgelsen werden.
