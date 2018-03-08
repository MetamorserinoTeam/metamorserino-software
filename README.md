# In diesem Verzeichnis findet man die Unterlagen für den Arduino-basierten CW Keyer "metMorserino".

Unterverzeichnisse:
- Arduino_Libraries: diese Bibliotheken müsen in der Arduino Entwicklungsumgebung
	  vorhanden sein, damit fehlerfrei kompiliert werden kann. Es wurde derzeit die Arduino
	  Entwicklungsumgebung Version 1.8.1 von arduino.cc verwendet.
- Driver_USB_Mac_PC: Damit der PC (oder Mac) mit dem Arduino Nano kommunizieren kann,
	  muss ein passender Treiber installiert werden.
- Hardware: Schaltung und Aufbauanleitung für den Bausatz.
- Version_2: Arduino Software für Version 2 (CW Keyer + CW Trainer)
- Version_3: Arduino Software für Version 3 (CW Keyer + CW Trainer + CW Dekoder)
- Version_4: Arduino Software für Version 4 (Unterstützung externer Paddles, Erweiterungen)

## Hardware – Versionshistorie:

- Version 1: erste Auflage des Bausatzes
- Version 1a: zweite Auflage des Bausatzes. Fehlerbehebung auf Platine, anderer Lautsprecher (der alte ist 
	    nicht mehr lieferbar), anderer Drehgeber.

## Software - Versionshistorie:

- Version 1 (nicht veröffentlicht): 
	Automatischer CW Keyer mit Touch-Sensoren
- Version 2: zusätzliche Funktion: CW Übungsgenerator m. 5er-Gruppen und Callsigns
- Version 3: zusätzliche Funktion: CW Dekoder (Input entweder Tonsignal, oder Handtaste). Tonsignal-Input 
	   erfordert Hardware-Erweiterung (NF Verstärker)
- Version 4: 
	- zusätzliche Funktion: Möglichkeit, externes mechanisches Paddle anzuschließen anstelle der 
	                         Touch-Sensoren
        - erweiterte Funktion: CW Dekoder nicht nur für „Straight Key“ oder Tonsignal, sondern auch mit 
			        Touchsensor (als Cootie / Sideswiper) verwendbar
        - erweiterte Funktion: Dekoder kann zum Tasten eines Senders verwendet werden (außer bei Tonsignal 
				als Input)
        - erweiterte Funktion: CW Übungsgenerator kann auch Q-Gruppen und übliche CW-Abkürzungen ausgeben

Um die Funktion des CW Dekoders zu verwenden, muss hardwareseitig ergänzt werden!

- um eine Handtaste verwenden zu können, ist die Handtaste an den Pin D12 des Arduino anzuschließen.
- um Tonsignale zu dekodieren, ist eine Verstärkerschaltung erforderlich, deren Ausgang über
  einen Kondesator an Pin A6 des Arduino anzuschließen - dieser Pin ist außerdem über jeweils
  einen 10k Widerstand an +5V und GND zu legen, so dass im Ruhezustand (kein Eingangssignal vom
  Verstärker) 2,5 V am Pin A6 liegen!
  Ein Schaltungsvorschlag für den Verstärker wird in Kürze ebenfalls im Verzeichnis Hardware
  hinterlegt werden.

ab V. 4.2 erweiterte Funktionen: 
	a) Möglichkeit, auch im Betrieb als Übungsgenerator den Ausgang zu tasten
	b) Option, ACS (Automatic Character Spacing) beim CW Keyer zu verwenden

V. 4.3: Bugfix: im CW Trainer Mode wurden bei den 5er Gruppen gewisse Zeichen (zB "9" bei den Ziffern) nicht 
		ausgegeben.

V. 4.4: Beim Start wird der I2C Bus gescannt, und festgestellt, auf welcher Adresse das Display liegt. Damit können
		Displays mit unterschiedlichen I2C Busadressen ohne Neukompilieren verwendet werden 
		(üblich sind welche mit Adresse 0x27 und 0x3f).

V. 4.5: Es wurde ein Fehler behoben, der im Keyer-Modus falsches Timing in den Modi Iambic B und Iambic B+ zur 
		Folge hatte. Vor allem B+ sollte jetzt deutlich besser funktionieren und jenen Zwisten, die auf
		den Iambic B Modus eingeschossen sind, weniger Probleme bereiten (der Modus B sollte eher nicht 
		verwendet werden).

## I M P O R T A N T ! !   W I C H T I G ! !  2. Serie Bausätze und Version 4!

  * There are 2 (two) versions of hardware, currently, the only difference affecting the code is the
    rotary encoder
  * older versions: uses the KY-040 encoder (recognazible as it is on a separate PCB
  * newer versions: uses a more "standard" encoder
  * You NEED to #define which encoder is being used - if you fail to do so, the Arduino IDE (compiler) will 
    throw an error ar you!
  * For this purpose, eliminate the '//' in front of one of the following lines in the source code! (at the 
    type that is being used)
    
  * Es gibt derzeit 2 (zwei) unterschiedliche Hardware Versionen, die sich vor allem im verwendeten 
    Roray Encoder unterscheiden
  * die ältere version: benutzt einen KY-040 Encoder - man erkennt ihn leicht daran, dass er auf einer 
    separaten kleinen Platine sitzt
  * die neuere Version: benutzt einen Encoder, der eher "Standard" ist
  * Du MUSST mit #define festlegen, welcher Encoder benutzt wird, sonst gibt es einen Fehler beim Kompilieren!
  * Dazu musst du die '//' bei einer der folgenden Zeilen im Source Code entfernen! (Bei dem Typ, der 
    verwendet wird)

Ca. l. 33

	//#define KY_040_ENCODER
	//#define STANDARD_ENCODER
  
