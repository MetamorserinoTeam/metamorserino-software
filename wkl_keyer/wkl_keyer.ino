
 #define metaVERSION "4.5"
 const byte MorserSignature = '*';   // 4.4. was the same; 4.2 and 4.3 had '?';  3.2 was '$', 4.0 was '@', 4.1 was '+'

 
/**********************************************************************************************************************************
 * Copyright 2017, Willi Kraml (OE1WKL) 
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, 
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *********************************************************************************************************************************/

 /**********************  I M P O R T A N T ! !   W I C H T I G ! !  *************************************************************
  *  There are 2 (two) versions of hardware, currently, the only difference affecting the code is the rotary encoder
  *  older versions: uses the KY-040 encoder (recognazible as it is on a separate PCB
  *  newer versions: uses a more "standard" encoder
  *  You NEED to #define which encoder is being used - if you fail to do so, the Arduino IDE (compiler) will throw an error ar you!
  *  For this purpose, eliminate the '//' (starts a comment line) in front of one of the following lines! (at the type that is being used)
  *  
  *  Es gibt derzeit 2 (zwei) unterschiedliche Hardware Versionen, die sich vor allem im verwendeten Rotary Encoder unterscheiden
  *  die ältere Version: benutzt einen KY-040 Encoder - man erkennt ihn leicht daran, dass er auf einer separaten kleinen Platine sitzt
  *  die neuere Version: benutzt einen Encoder, der eher "Standard" ist
  *  Du MUSST mit #define festlegen, welcher Encoder benutzt wird, sonst gibt es einen Fehler beim Kompilieren!
  *  Dazu musst du die '//' (Beginn einer Kommentarzeile) bei einer der folgenden Zeilen entfernen! (Bei dem Typ, der verwendet wird)
  */

#define KY_040_ENCODER
//#define STANDARD_ENCODER

/* the next line is a factor determining the sensitivity of the paddles */
/* lower value: higher sensitivity; range should be within 0.90 and 0.99 */

 #define SENS_FACTOR 0.96

 /// we define auto-character spacing in the next line...
 /// ACS_LENGTH normally should be 3, if less strict then 2 
 
 #define ACS_LENGTH 3

////// code by others used in this sketch:
/*
 *  NewliquidCrystal for LCD/I2C/TWI display - see https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
 *                                             and http://arduino-info.wikispaces.com/LCD-Blue-I2C
 *                                             written by F. Malpartida (https://bitbucket.org/fmalpartida)
 *  Touch sensor interface by Martin Pittermann, see -> https://github.com/martin2250/ADCTouch and http://playground.arduino.cc/Code/ADCTouch
 *  ClickButton library -> https://code.google.com/p/clickbutton/ by Ragnar Aronsen
 *  Rotary Encoder code based on arduinoalemans code, with some changes,  see https://forum.arduino.cc/index.php?topic=242356.0
 *  For persistent storage (EEPROM) I am using the extended EEPROM library by Thijs Elenbaas 
 *                                            -> http://thijs.elenbaas.net/2012/07/extended-eeprom-library-for-arduino/
 *  For volume control of NF output: Volume3 by Connor Nishijima, see https://github.com/connornishijima/arduino-volume3 and
 *                                             https://hackaday.io/project/11957-10-bit-component-less-volume-control-for-arduino
 *  Routines for morse decoder - to a certain degree based on code by Hjalmar Skovholm Hansen OZ1JHM - see http://skovholm.com/cwdecoder
 */

// include the library code:
#include <Wire.h>           // TWI interface
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include "ClickButton.h"    // button control library
#include <EEPROMex.h>       // extended EEPROM library
#include <Volume3.h>        // volume library
#include <ADCTouch.h>       // touch sensor library


// set the LCD address for a 16 chars 2 line display
// We use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
//LiquidCrystal_I2C lcd(0x5f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
LiquidCrystal_I2C lcd(0x5f);  // Set the LCD I2C address

// this is a placeholder - the real address is determined now in setup() !!! can be 0x27 or 0x3f

/// globald constants and variables
///
/// Arduino pins used:

const int leftPin =  A2;          // connect left paddle to pin A2
const int rightPin = A0;         // and right one to Pin A0
const int sidetonePin = 9;      // to generate an ugly keyer side tone
const int modeButtonPin = 4;    // input pin for mode button
const int keyerPin = 13;        // this keys the transmitter /through a 2N2222 NPN transistor - at the same time lights up the LED on the Arduino
const int straightPin = 12;     // input for straight key - for Morse decoder
const int audioInPin = A6;      // audio in for Morse decoder

// set up the button
ClickButton modeButton(modeButtonPin, LOW, CLICKBTN_PULLUP);  // initialize mode button

// things for reading the encoder
volatile boolean TurnDetected;
volatile boolean up;
volatile unsigned long rotating;

const int PinCLK=2;                   // Used for generating interrupts using CLK signal
const int PinDT=3;                    // Used for reading DT signal

volatile short encoderPos = 0; //this variable stores our current value of encoder position.

/// other constants and variables
unsigned long charCounter = 25;            // we use this to count characters after changing speed - after n characters we decide to write the config into EEPROM
int refLeft, refRight;       //reference values to remove offset
boolean leftKey, rightKey;
int value0, value1;


//// DISPLAY LAYOUT of top line - 16 characters wide - keyer mode
////            |0...+....9....+.|
////            |99wpm A+ .- 550∎|
//// DISPLAY LAYOUT of top line - 16 characters wide - trainer mode
////            |0...+....9....+.|
////            |99w/1 äa1.<>550∎|
////  new: Farnsworth factor as /1 instead of pm
///   instead of mode & polarity: symbols for target characters (max 6 long)
const int lcdFarnsworth = 2;
const int lcdSymbols = 5;
const int lcdSpeed = 0;
const int lcdMode = 6;
const int lcdPolarity = 9;
const int lcdPitch = 12;
const int lcdVolume = 15;


char CWspeed_buffer[3];

// define modes for state machine of the various modi the encoder can be in
 
enum encoderMode {speedSettingMode, curtisSettingMode, polaritySettingMode, pitchSettingMode, volumeSettingMode }; 

encoderMode encoderState = speedSettingMode;    // we start with adjusting the speed


/// the states the morserino can be in - top level menu
enum morserinoMode {morseKeyer, morseTrainer, morseDecoder, invalid };
morserinoMode morseState = morseKeyer;


// display buffers for bottom display line
char scroll_line[17] = "                "; // a string of 16 blanks, automatically terminates with a zero byte in 17th position
char scroll_memory[17];



// defines for keyer modi
//

#define    IAMBICA      0          // Curtis Mode A
#define    IAMBICB      1          // Curtis Mode B
#define    IAMBICBplus  2          // improved Curtis B mode
#define    ULTIMATIC    3          // Ultimatic mode


// variables to hold EEPROM addresses and things we need for reading / storing values in EEPROM
int addressSignature ;   
int addressCWsettings  ;

// the following is only used to initially see if we have something stored in EEPROM
byte eSignature;

///// GROUPOF5
// 0 = A, 1 = 9, 2 = A9, 3 = A9?, 4 = ?<>, 5 = A9?<>, 6 = äA, 7= äA9, 8 = äA9.<>, 9 = calls, 10=q-groups & abbreviations
// Grouping:   a    |  9   |  .   |  <>   |  a9  | .<>   |   äa    | äa9   | a9.   | a9.<> | äa9.<>
//            4-29  | 30-39| 40-48| 48-52 | 4-39 | 40-52 | 0-29    | 0-39  | 4-48  | 4-52  | 0-52 

const byte bounds[][2] = {  {4,29}, {30,39}, {4, 39}, {4,48}, {40, 52}, {4, 52}, {0,29}, {0, 39}, {0, 52}, {0,0}, {0,0}};
const char groups[][7] = { " a    ", "  9   ", " a9   ", " a9?  ", "   ?<>", " a9?<>", "\xE1" "a    ", "\xE1" "a9   ", "\xE1" "a9?<>", "CALLs ", "ABBRV " };

// the funny spelling of German ä: reason is that these are "local" characters within the LCD display, not Unicode or whatever, so I need to use hex escapes
// in order to clearly indicate where the hex escape ends, I need to break the string into substrings....

// structure to contain persistent CW settings, to be stored in EEPROM
struct CWs {
  int wpm;                      // keyer speed in wörtern pro minute
  unsigned sidetoneFreq;        // side tone frequency
  int sidetoneVolume;           // side tone volume
  boolean didah;                // paddle polarity
  byte keyermode;               // Iambic keyer mode: see the #defines above
  byte farnsworthMode;           // trainer: extend pauses by 1, 2, 3 to 9
  byte generatorMode;            // trainer: what symbol (groups) are we going to send?
  int tLeft;                    // threshold for left paddle
  int tRight;                   // threshold for right paddle
  boolean useExtPaddle;         // true if we use an external (mechanical) paddle instead of the touch sensors
  boolean ACS;                  // true if we use Automatic Character Spacing
  byte i2cAddress;              // result of i2c scanner
};

struct CWs CWsettings = {
  15, 650, 5, false, 0, 4, 2, 110, 110, false, false, 0x3F };

boolean settingsDirty = false;    // we use this to see if anything in CW settings has changed, so we need to write settings into EEPROM

int realVolume[] = {0, 64, 160, 270, 400, 650, 800, 1023};


//  keyerControl bit definitions

#define     DIT_L      0x01     // Dit latch
#define     DAH_L      0x02     // Dah latch
#define     DIT_LAST   0x04     // Dit was last processed element

//  Global Keyer Variables
//
unsigned char keyerControl = 0; // this holds the latches for the paddles and the DIT_LAST latch, see above


boolean DIT_FIRST = false; // first latched was dit?
unsigned int ditLength ;        // dit length in milliseconds - 100 = 60bpm = 12 wpm
unsigned int dahLength ;        // dahs are 3 dits long
unsigned char keyerState;


///////////////////////////////////////////////////////////////////////////////
//
//  Iambic Keyer State Machine Defines
 
enum KSTYPE {IDLE_STATE, DIT, DAH, KEY_START, KEYED, INTER_ELEMENT };

// morse code decoder

struct linklist {
     const unsigned char symb;
     const byte dit;
     const byte dah;
};


const struct linklist CWtree[64]  = {
       {0,1,2},      // 0
      {'e', 3,4},     // 1
      {'t',5,6},      // 2
//
      {'i', 7, 8},      // 3
  {'a', 9,10},      // 4
  {'n', 11,12},     // 5
  {'m', 13,14},     // 6
//
  {'s', 15,16},     // 7
  {'u', 17,18},     // 8
  {'r', 19,20},     // 9
  {'w', 21,22},     //10
  {'d', 23,24},       //11
  {'k', 25, 26},      //12
  {'g', 27, 28},      //13
  {'o', 29,30},     //14
//---------------------------------------------
  {'h', 31,32},     // 15
  {'v', 33, 34},      // 16
  {'f', 63, 63},      // 17
  {245, 35, 36},      // 18 german ue
  {'l', 37, 38},      // 19
  {225, 39, 63},      // 20 german ae
  {'p', 63, 40},      // 21
  {'j', 63, 41},      // 22
  {'b', 42, 43},      // 23
  {'x', 44, 63},      // 24
  {'c', 63, 45},      // 25
  {'y', 46, 63},      // 26
  {'z', 47, 48},      // 27
  {'q', 63, 63},      // 28
  {239, 49, 63},      // 29 german oe
  {1, 50, 51},      // 30 !!! german 'ch' 
//---------------------------------------------
  {'5', 63, 63},      // 31
  {'4', 63, 63},      // 32
  {'*', 63, 52},      // 33  
  {'3', 63,63},     // 34
  {165, 53,63,},      // 35 a square, centered dot - used for all unidentifiable characters
  {'2', 63, 63},      // 36 
  {2, 63,63},     // 37 !! <as>
  {165, 54, 63},      // 38
  {'+', 63, 55},      // 39
  {165, 56, 63},      // 40
  {'1', 57, 63},      // 41
  {'6', 63, 58},      // 42
  {'=', 63, 63},      // 43
  {'/', 63, 63},      // 44
  {3, 59, 60},      // 45 !! <ka>
  {4, 63, 63},      // 46   !! <kn>
  {'7', 63, 63},      // 47
  {165, 63, 61},      // 48
  {'8', 62, 63},      // 49
  {'9', 63, 63},      // 50
  {'0', 63, 63},      // 51
//
  {5, 63, 63},      // 52 !! <sk>
  {'?', 63, 63},      // 53
  {'"', 63, 63},      // 54
  {'.', 63, 63},      // 55
  {'@', 63, 63},      // 56
  {'\'',63, 63},      // 57
  {'-', 63, 63},      // 58
  {';', 63, 63},      // 59
  {'!', 63, 63},      // 60
  {',', 63, 63},      // 61
  {':', 63, 63},      // 62
//
  {165, 63, 63}     // 63 Default for all unidentified characters
};

byte treeptr = 0;                          // pointer used to navigate within the linked list representing the dichotomic tree
char morseCharacter[5];                // result of decoding

unsigned long interWordTimer = 0;      // timer to detect interword spaces
unsigned long acsTimer = 0;            // timer to use for automatic character spacing (ACS)

boolean trainerMode = false;
long int seed;

/////// variables for morse code trainer mode

byte NoE = 0;             // Number of Elements
byte nextElement[8];      // the list of elements; 0 = dit, 1 = dah
unsigned char generatorState, generatorMode;
unsigned int interCharacterSpace, interWordSpace;   // need to be properly initialised!
unsigned long timer;
unsigned short pointer = 0;
unsigned short startPool, endPool;
unsigned long characterCounter = 1; 
boolean startAgain = false;       // to indicate that we are starting a new sequence in trainer mode
byte atStart[] = {25, 25, 25, 50, 53, 53};
byte k = 0;        /// pointer for atStart
boolean active = false;                          // flag for trainer mode
boolean keyTrainerMode = false;


//// special characters are: ch, <as>, <ka>, <kn>, <sk>
//                           01   02    03    04    05 
//                           53   54    55    56    57
// Alpha        a
// Numerisch    9
// Interpunkt   . , : - / = ? @ +    010101 110011 111000 100001 10010 10001 001100 011010 01010
// Betriebsabk  <>  <as> <ka> <kn> <sk>
// German       ä 
// Grouping:   a    |  9   |  .   |  <>   |  a9  | .<>   |   äa    | äa9   | a9.   | a9.<> | äa9.<>
//            4-29  | 30-39| 40-48| 48-52 | 4-39 | 40-52 | 0-29    | 0-39  | 4-48  | 4-52  | 0-52 

// for each character:
// byte length// byte morse encoding as binary value, beginning with most significant bit

byte poolPair[2];           // storage in RAM fpr one morse code character

const byte pool[][2] PROGMEM = {
               {B01010000, 4},  // ä    0   
               {B11100000, 4},  // ö
               {B00110000, 4},  // ü
               {B11110000, 4},  // ch   3
               {B01000000, 2},  // a    4     ASCII +93
               {B10000000, 4},  // b
               {B10100000, 4},  // c
               {B10000000, 3},  // d
               {B00000000, 1},  // e
               {B00100000, 4},  // f
               {B11000000, 3},  // g
               {B00000000, 4},  // h
               {B00000000, 2},  // i
               {B01110000, 4},  // j 
               {B10100000, 3},  // k
               {B01000000, 4},  // l
               {B11000000, 2},  // m  16
               {B10000000, 2},  // n
               {B11100000, 3},  // o
               {B01100000, 4},  // p  19
               {B11010000, 4},  // q
               {B01000000, 3},  // r
               {B00000000, 3},  // s
               {B10000000, 1},  // t
               {B00100000, 3},  // u
               {B00010000, 4},  // v
               {B01100000, 3},  // w
               {B10010000, 4},  // x
               {B10110000, 4},  // y
               {B11000000, 4},  // z  29
               {B11111000, 5},  // 0  30    ASCII +18
               {B01111000, 5},  // 1
               {B00111000, 5},  // 2
               {B00011000, 5},  // 3
               {B00001000, 5},  // 4
               {B00000000, 5},  // 5
               {B10000000, 5},  // 6
               {B11000000, 5},  // 7
               {B11100000, 5},  // 8
               {B11110000, 5},  // 9  39
// Interpunkt   . , : - / = ? @ +    010101 110011 111000 100001 10010 10001 001100 011010 01010
               {B01010100, 6},  // .  40    ASCII +6
               {B11001100, 6},  // ,  41    ASCII +3
               {B11100000, 6},  // :  42    ASCII +16
               {B10000100, 6},  // -  43    ASCII +2
               {B10010000, 5},  // /  44    ASCII +3
               {B10001000, 5},  // =  45    ASCII +16
               {B00110000, 6},  // ?  46    ASCII +17
               {B01101000, 6},  // @  47    ASCII +17
               {B01010000, 5},  // +  48    (at the same time <ar> !) ASCII -5
// Betriebsabk  <>  <as> <ka> <kn> <sk>
               {B01000000, 5},  // <as> 49
               {B10101000, 5},  // <ka> 
               {B10110000, 5},  // <kn>
               {B00010100, 6}   // <sk> 52         
            };


const int NUMBER_OF_ELEMENTS = 241;
const int MAX_SIZE = 9;

/// strings for q-groups and abbreviations to be stored in PROGMEM
const char abbreviations [NUMBER_OF_ELEMENTS] [MAX_SIZE] PROGMEM = { 
{ "33" },
{ "44" },
{ "55" },
{ "72" },
{ "73" },
{ "88" },
{ "99" },
{ "aa" },
{ "ab" },
{ "abt" },
{ "ac" },
{ "adr" },
{ "af" },
{ "agc" },
{ "agn" },
{ "alc" },
{ "am" },
{ "am" },
{ "ans" },
{ "ant" },
{ "atv" },
{ "avc" },
{ "award" },
{ "awdh" },
{ "awds" },
{ "bc" },
{ "bci" },
{ "bcnu" },
{ "bd" },
{ "bfo" },
{ "bk" },
{ "bpm" },
{ "btr" },
{ "btw" },
{ "bug" },
{ "buro" },
{ "b4" },
{ "call" },
{ "cfm" },
{ "cl" },
{ "conds" },
{ "condx" },
{ "congrats" },
{ "cq" },
{ "cu" },
{ "cuagn" },
{ "cul" },
{ "cw" },
{ "db" },
{ "dc" },
{ "de" },
{ "diff" },
{ "dr" },
{ "dwn" },
{ "dx" },
{ "ee" },
{ "el" },
{ "elbug" },
{ "es" },
{ "excus" },
{ "fb" },
{ "fer" },
{ "fm" },
{ "fone" },
{ "fr" },
{ "frd" },
{ "freq" },
{ "fwd" },
{ "ga" },
{ "gb" },
{ "gd" },
{ "gd" },
{ "ge" },
{ "gl" },
{ "gm" },
{ "gn" },
{ "gnd" },
{ "gp" },
{ "gs" },
{ "gt" },
{ "gud" },
{ "ham" },
{ "hf" },
{ "hi" },
{ "hpe" },
{ "hr" },
{ "hrd" },
{ "hrs" },
{ "hv" },
{ "hvy" },
{ "hw" },
{ "i" },
{ "iaru" },
{ "if" },
{ "ii" },
{ "info" },
{ "inpt" },
{ "irc" },
{ "itu" },
{ "k" },
{ "khz" },
{ "km" },
{ "kw" },
{ "ky" },
{ "lbr" },
{ "lf" },
{ "lid" },
{ "lis" },
{ "lng" },
{ "log" },
{ "lp" },
{ "lsb" },
{ "luf" },
{ "lw" },
{ "ma" },
{ "mesz" },
{ "mez" },
{ "mgr" },
{ "mhz" },
{ "min" },
{ "mins" },
{ "mni" },
{ "mod" },
{ "msg" },
{ "mtr" },
{ "muf" },
{ "my" },
{ "n" },
{ "net" },
{ "nf" },
{ "nil" },
{ "no" },
{ "nr" },
{ "nr" },
{ "nw" },
{ "ok" },
{ "om" },
{ "op" },
{ "osc" },
{ "oscar" },
{ "output" },
{ "ow" },
{ "pa" },
{ "pep" },
{ "pm" },
{ "pse" },
{ "psed" },
{ "pwr" },
{ "px" },
{ "qra" },
{ "qrb" },
{ "qrg" },
{ "qrl" },
{ "qrm" },
{ "qrn" },
{ "qro" },
{ "qrp" },
{ "qrq" },
{ "qrs" },
{ "qrt" },
{ "qru" },
{ "qrv" },
{ "qrx" },
{ "qrz" },
{ "qsb" },
{ "qsk" },
{ "qsl" },
{ "qso" },
{ "qsp" },
{ "qst" },
{ "qsy" },
{ "qtc" },
{ "qth" },
{ "qtr" },
{ "r" },
{ "rcvd" },
{ "re" },
{ "ref" },
{ "rf" },
{ "rfi" },
{ "rig" },
{ "rpt" },
{ "rprt" },
{ "rst" },
{ "rtty" },
{ "rx" },
{ "sase" },
{ "shf" },
{ "sigs" },
{ "sked" },
{ "sn" },
{ "sp" },
{ "sri" },
{ "ssb" },
{ "sstv" },
{ "stn" },
{ "sure" },
{ "swl" },
{ "swr" },
{ "t" },
{ "temp" },
{ "test" },
{ "tia" },
{ "tks" },
{ "tnx" },
{ "trx" },
{ "tu" },
{ "tvi" },
{ "tx" },
{ "u" },
{ "ufb" },
{ "uhf" },
{ "ukw" },
{ "unlis" },
{ "up" },
{ "ur" },
{ "usb" },
{ "utc" },
{ "v" },
{ "vert" },
{ "vfo" },
{ "vhf" },
{ "vl" },
{ "vln" },
{ "vy" },
{ "w" },
{ "watts" },
{ "wid" },
{ "wkd" },
{ "wkg" },
{ "wl" },
{ "wpm" },
{ "wtts" },
{ "wx" },
{ "xcus" },
{ "xcvr" },
{ "xmas" },
{ "xtal" },
{ "xyl" },
{ "yl" },
{ "z" }
 };

//////////////////////////////////////////////////////////////////////////////
//
//   State Machine Defines
 
enum MORSE_TYPE {KEY_DOWN, KEY_UP };
enum GEN_TYPE { GROUPOF5, CALLSIGNS,  Q_ABBREV, TESTALL };

///--------------------------
/// LCD glyphs for volume control

byte volGlyph[][8] = {
 {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11000,
  0b11000
},  {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11100,
  0b11000,
  0b11000
}, {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11100,
  0b11100,
  0b11000,
  0b11000
},  {
  0b00000,
  0b00000,
  0b00000,
  0b11110,
  0b11100,
  0b11100,
  0b11000,
  0b11000
},  {
  0b00000,
  0b00000,
  0b11110,
  0b11110,
  0b11100,
  0b11100,
  0b11000,
  0b11000
}, 
{
  0b00000,
  0b11111,
  0b11110,
  0b11110,
  0b11100,
  0b11100,
  0b11000,
  0b11000
}, {
  0b11111,
  0b11111,
  0b11110,
  0b11110,
  0b11100,
  0b11100,
  0b11000,
  0b11000
}};

/// variables for morse decoder
///////////////////////////////

/// const int audioInPin = A6; already defined above

boolean keyTx = false;             // when state is set by manual key or touch paddle, then true!
                                   // we use this to decide if Tx should be keyed or not

float magnitude ;
int magnitudelimit = 100;
int magnitudelimit_low = 100;

//boolean filteredState = false;
//boolean filteredStateBefore = false;

/// state machine for decoding CW
enum DECODER_STATES  {LOW_, HIGH_, INTERELEMENT_, INTERCHAR_};
DECODER_STATES decoderState = LOW_;


///////////////////////////////////////////////////////////
// The sampling frq will be 8928 on a 16 mhz             //
// without any prescaler etc                             //
// because we need the tone in the center of the bins    //
// you can set the tone to 496, 558, 744 or 992          //
// then n the number of samples which give the bandwidth //
// can be (8928 / tone) * 1 or 2 or 3 or 4 etc           //
// init is 8928/558 = 16 *4 = 64 samples                 //
// try to take n = 96 or 128 ;o)                         //
// 48 will give you a bandwidth around 186 hz            //
// 64 will give you a bandwidth around 140 hz            //
// 96 will give you a bandwidth around 94 hz             //
// 128 will give you a bandwidth around 70 hz            //
// BUT remember that high n take a lot of time           //
// so you have to find the compromice - i use 48         //
///////////////////////////////////////////////////////////

float coeff;
float Q1 = 0;
float Q2 = 0;
float sine;
float cosine;
const float sampling_freq = 8928.0;
const float target_freq = 744.0; /// adjust for your needs see above
const int n = 62; //// if you change  her please change next line also  ---- 64: BW = 140 Hz
int testData[128];
float bw;

//////////////////////////////
// Noise Blanker time which //
// shall be computed?? so     //
// this is initial          //
//////////////////////////////
const int nbtime = 6;  /// ms noise blanker

unsigned long startTimeHigh;
unsigned long highDuration;
//long lasthighduration;
//long hightimesavg;
//long lowtimesavg;
long startTimeLow;
long lowDuration;
boolean stop = false;
boolean speedChanged = true;
unsigned long ditAvg, dahAvg;     /// average values of dit and dah lengths to decode as dit or dah and to adapt to speed change
boolean filteredState = false;
boolean filteredStateBefore = false;


///////////////////////////////////////////////////////////////////////////
///////// SETUP
///////////////////////////////////////////////////////////////////////////

void setup() {

byte i2cAddress;

Serial.begin(9600);          // only for debugging purposes! comment out for production code!

// read what is stored in EEPROM:
//   signature  byte (to see if we actually have values we can meaningfully read - otherwise we use the defaults
//   CWsettings struct CWs CWsettings (speed, polarity,  mode etc) - 

// calculate addresses
  addressSignature    = EEPROM.getAddress(sizeof(eSignature));
  addressCWsettings   = EEPROM.getAddress(sizeof(CWsettings));

  // check signature
  eSignature = EEPROM.readByte(addressSignature);
  if (eSignature == MorserSignature) {                    // OK, we read in values from EEPROM
    EEPROM.readBlock(addressCWsettings, CWsettings);      // otherwise we use the defaults defined in this program
  }
    // first use of device with this version: use defaults, run i2c scanner, write results into EEPROM
  Wire.begin();
  i2cAddress = i2cScan();
  if (CWsettings.i2cAddress != i2cAddress) {
    CWsettings.i2cAddress = i2cAddress;
    saveConfig();
  }
 

// we reset the LCD address for a 16 chars 2 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
  lcd = LiquidCrystal_I2C( CWsettings.i2cAddress, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  
  // set up the LCD's number of rows and columns:   
  lcd.begin(16, 2);           // initialize display (no of cols and rows)
  
  
  // create 7 glyphs for volume display
  lcd.createChar(1, volGlyph[0]);
  lcd.createChar(2, volGlyph[1]);
  lcd.createChar(3, volGlyph[2]);
  lcd.createChar(4, volGlyph[3]);
  lcd.createChar(5, volGlyph[4]);
  lcd.createChar(6, volGlyph[5]);
  lcd.createChar(7, volGlyph[6]);
 
  
  pinMode(keyerPin, OUTPUT);        // we can use the built-in LED to show when the transmitter is being keyed

  digitalWrite(keyerPin, LOW);
  pinMode(straightPin, INPUT_PULLUP);
  pinMode(leftPin, INPUT_PULLUP);   // the touch library does this itself - we just do it here in case we use the eternal paddles
  pinMode(rightPin, INPUT_PULLUP);
  
  // set up the encoder
  pinMode(PinCLK,INPUT_PULLUP);
  pinMode(PinDT,INPUT_PULLUP);  

#if defined (KY_040_ENCODER)
  attachInterrupt (1,isr,CHANGE);   // interrupt 1 is always connected to pin 2 on Arduino UNO /// needs to be LOW insted aof CHANGE for the new encoders
#elif defined (STANDARD_ENCODER)
  attachInterrupt (1,isr,LOW);   // interrupt 1 is always connected to pin 2 on Arduino UNO /// needs to be LOW insted aof CHANGE for the new encoders
#else
  #error Encoder undefined ! Encoder-Typ nicht definiert ! Read  souce code! Sourcecode lesen ! Line/Zeilen 19 ff !!!
#endif

  // Setup button timers (all in milliseconds / ms)
  // (These are default if not set, but changeable for convenience)
  modeButton.debounceTime   = 12;   // Debounce timer in ms
  modeButton.multiclickTime = 275;  // Time limit for multi clicks
  modeButton.longClickTime  = 400; // time until "held-down clicks" register


  
  // calibrate the paddles

  refLeft = ADCTouch.read(leftPin, 1000);    //create reference values to
  refRight = ADCTouch.read(rightPin, 1000);      //account for the capacitance of the pad

  if (CWsettings.useExtPaddle)   {                   // ext paddle configured, check if it is there
    if (refLeft < 630) {
      CWsettings.useExtPaddle = false;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Use Touch Paddle"));
      delay(1000);
      saveConfig();
    }
  } else {                                              // touch configured, check for calibration and for external paddle
                                                        // check for external paddles connected; 
                                                        // you need to close at least one of the contacts for that purpose at start-up
    pinMode(leftPin, INPUT_PULLUP);pinMode(rightPin, INPUT_PULLUP);
    if ((analogRead(leftPin) < 50) || (analogRead(rightPin) < 50)) {    // a mechanical paddle is pressed, so switch to use them
      CWsettings.useExtPaddle = true;
      saveConfig();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Use Ext. Paddle"));
      delay(1000);
    } // end check for external paddle
    
      else if (refLeft > 630 && refRight > 630) {             // we use touch sensors and detect touch sensor squeeze at start-up: calibration
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Calibration"));
      CWsettings.tLeft = refLeft; CWsettings.tRight = refRight;
      delay(1000);
        lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Release paddles!"));
        delay(1250);
      refLeft = ADCTouch.read(leftPin, 1000);    //create reference values to
      refRight = ADCTouch.read(rightPin, 1000);      //account for the capacitance of the pad
      CWsettings.tLeft =  (int)((float)(CWsettings.tLeft - refLeft) * SENS_FACTOR);
      CWsettings.tRight = (int)((float)(CWsettings.tRight - refRight) * SENS_FACTOR);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("off: ")); lcd.print(refLeft); lcd.print(" "); lcd.print(refRight);
       lcd.setCursor(0,1);
      lcd.print(F("on+: ")); lcd.print(CWsettings.tLeft); lcd.print(" "); lcd.print(CWsettings.tRight);
      saveConfig();
      delay(3000);
    } // end calibration mode


  }
  
    
  // display startup screen 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("MetaMorserino"));
  lcd.setCursor(0,1);
  lcd.print(F("V " metaVERSION "  (oe1wkl)"));
  delay(1200);
  lcd.clear();

  topMenu();
}


void setupTrainerMode() {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Start CW Trainer"));
      lcd.setCursor(0,1);
      lcd.print(F("on/off: Squeeze"));
      seed = (analogRead(A1) % 17) * (analogRead(A1) % 23) * (millis() % 997);    /// prime the PRG
      randomSeed(seed);
      trainerMode = true;
      active = false;
      k=0; 
      startAgain = true;

      updateGeneratorMode();

      ditLength = 1200 / CWsettings.wpm;                    // set new value for length of dits and dahs
      dahLength = 3 * ditLength;
      interCharacterSpace = 3 * ditLength  *CWsettings.farnsworthMode;
      interWordSpace = 7 * ditLength * CWsettings.farnsworthMode;
      delay(1000);
      TurnDetected = false;
      lcd.clear();
      displayTopLine();
      timer = millis();   
}

void setupKeyerMode() {
  trainerMode = false;
  // set up dit and dah length, as well as keyer initial state
  ditLength = 1200 / CWsettings.wpm;                    // set new value for length of dits and dahs
  dahLength = 3 * ditLength;
  keyerState = IDLE_STATE;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Start CW Keyer"));
  clear_PaddleLatches();
  delay(1000);
  TurnDetected = false;
  lcd.clear();
  displayTopLine();
}


void setupDecoderMode() {
  /// here we will do the init for decoder mode
  trainerMode = false;
  encoderState = volumeSettingMode;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Start CW Decoder"));
  ///lcd.setCursor(0,1);
  ///lcd.print(F("not yet done!"));
 delay(1000);
  TurnDetected = false;
  speedChanged = true;
  lcd.clear();
  displayCWspeed (CWsettings.wpm);
  displayVolume();
    
  /// set up variables for Goertzel Morse Decoder
  setupGoertzel();
  filteredState = filteredStateBefore = false;
  decoderState = LOW_;
  ditAvg = 60;
  dahAvg = 180;
}




void loop() {

   checkPaddles();
   switch (morseState) {
      case morseKeyer:    if (doPaddleIambic(leftKey, rightKey))
                            return;                                                        // we are busy keying and so need a very tight loop !
                          break;
      case morseTrainer:  if (leftKey  && rightKey)   {                                    // touching both paddles starts and stops the generation of code
                          // for debouncing:
                          while (checkPaddles() )
                              ;                                                           // wait until paddles are released
                          active = !active;
                          delay(100);
                    
                          if (!active)
                             vol.noTone();                     // stop side tone 
                          else {
                             characterCounter = 1;
                             generatorState = KEY_UP; 
                             startAgain = true;
                             fetchNextChar();
                            }
                          }
                          if (active)
                            generateCW();
                          break;

      case morseDecoder: doDecode();
                         if (speedChanged) {
                            speedChanged = false;
                            displayCWspeed (CWsettings.wpm);
                          }
                          break;
                         
  } // end switch and code depending on state of metaMorserino

/// if we have time check for button presses

    modeButton.Update();
    if (morseState == morseDecoder && modeButton.clicks) {      // action during decoder mode!
        topMenu();
        return;
    }
    switch (modeButton.clicks) {                               // actions based on enocder button
      case 1: if (encoderState == volumeSettingMode)
              encoderState = speedSettingMode;
            else
              encoderState = (encoderMode((int(encoderState) + 1)));
            displayEncoderMode();
            break;
      case 2: if (encoderState == speedSettingMode)
              encoderState = volumeSettingMode;
            else
              encoderState = (encoderMode((int(encoderState) - 1)));
            displayEncoderMode();
            break;
      case -1: if (encoderState != speedSettingMode) {
                  encoderState = speedSettingMode;
                  displayEncoderMode();
            } else {
                  topMenu();
                  return;
               }
            break;
    }
    
/// and we have time to check the encoder
     if (TurnDetected) {
             vol.tone(sidetonePin, 250, realVolume[CWsettings.sidetoneVolume]);
             delay(8);
             vol.noTone();
             encoderPos = (up ? 1 : -1);
             TurnDetected = false;
         }
    
    if(encoderPos) {
    // Serial.println(encoderPos);
    switch (encoderState) {
      case speedSettingMode:  CWsettings.wpm += encoderPos;
                              encoderPos = 0;                                     // reset the encoder
                              CWsettings.wpm = constrain(CWsettings.wpm, 5, 40);
                              ditLength = 1200 / CWsettings.wpm;             // set new value for length of dits and dahs
                              dahLength = 3 * ditLength;
                              interCharacterSpace = 3*ditLength*CWsettings.farnsworthMode;
                              interWordSpace = 7*ditLength*CWsettings.farnsworthMode;
 
                              displayCWspeed(CWsettings.wpm);                     // update display of CW speed
                              charCounter = 0;                                    // reset character counter
                              break;
      case curtisSettingMode: if (!trainerMode)
                                setCurtisMode();
                              else
                                setFarnworthMode();
                              break;
      case polaritySettingMode:
                              if (!trainerMode)
                                setPolarityMode();
                              else
                                setGeneratorMode();
                              break;
      case pitchSettingMode:  CWsettings.sidetoneFreq += (encoderPos *50);
                              encoderPos = 0;                                     // reset the encoder
                              CWsettings.sidetoneFreq = constrain(CWsettings.sidetoneFreq, 250, 950);
                              displayPitch();
                              settingsDirty = true;
                              displayEncoderMode();
                              break;
      case volumeSettingMode: CWsettings.sidetoneVolume += encoderPos;
                              encoderPos = 0;                                     // reset the encoder
                              CWsettings.sidetoneVolume = constrain(CWsettings.sidetoneVolume, 0, 7);
                              displayVolume();
                              settingsDirty = true;
                              displayEncoderMode();
                              break;
      }
    
    } // encoder 
}


///// break out of some mode functions here:

void setCurtisMode() {
  CWsettings.keyermode += (encoderPos+4);
  encoderPos = 0;                                     // reset the encoder
  CWsettings.keyermode = CWsettings.keyermode % 4; 
  displayCurtisMode(CWsettings.keyermode);              // and display
  settingsDirty = true;
  displayEncoderMode();
}

void setFarnworthMode() {
  CWsettings.farnsworthMode += (encoderPos+8);
  encoderPos = 0;                                     // reset the encoder
  CWsettings.farnsworthMode = 1 + (CWsettings.farnsworthMode % 9); 
  displayFarnsworthMode();                           // and display
  interCharacterSpace = 3*ditLength*CWsettings.farnsworthMode;
  interWordSpace = 7*ditLength*CWsettings.farnsworthMode;
 
  settingsDirty = true;
  displayEncoderMode();
}

void setPolarityMode() {
  CWsettings.didah = !(CWsettings.didah);
  
   encoderPos = 0;                                     // reset the encoder
   displayPolarity();
   settingsDirty = true;
   displayEncoderMode();
}

void setGeneratorMode() {
  CWsettings.generatorMode += (encoderPos+11);
  encoderPos = 0;                                     // reset the encoder
  CWsettings.generatorMode = CWsettings.generatorMode % 11;
  displayGeneratorMode();
  updateGeneratorMode();
  settingsDirty = true;
 
  displayEncoderMode();
}

void updateGeneratorMode() {
  /// set the boundaries, the mode (GROUPOF5 or CALLSIGNS or Q_ABBREV)
  if (CWsettings.generatorMode < 9) {
    generatorMode = GROUPOF5;
    startPool = bounds[CWsettings.generatorMode][0];
    endPool = bounds[CWsettings.generatorMode][1] +1;
    }
  else if (CWsettings.generatorMode == 9)
    generatorMode = CALLSIGNS;
  else generatorMode = Q_ABBREV;
}

// functions related to paddle actions
// paddle reads a capacitive sensor, connected to pin, 
// and returns a logical value true or false
// but if mdoe is set to use external apddles, we just check if the input has been down close to 0

boolean checkPaddles() {
  long t = millis();
  if (CWsettings.useExtPaddle) {           // see if paddles shortened to GROUND
    pinMode(leftPin, INPUT_PULLUP);
    if (analogRead(leftPin) < 50) {
      // Serial.print("left: "); Serial.println(analogRead(leftPin));
      leftKey = true;
    }
    else
      leftKey = false;
  pinMode(rightPin, INPUT_PULLUP);
  if (analogRead(rightPin) < 50) {
    // Serial.print("right: "); Serial.println(analogRead(rightPin));
      rightKey = true;
    }
    else
      rightKey = false;
  } else {                                  // evaluate sensors
    value0 = ADCTouch.read(leftPin, 12);   //no second parameter
    value1 = ADCTouch.read(rightPin, 12);     //   --> 100 samples
    
    value0 -= refLeft;       //remove offset
    value1 -= refRight;
  
    if (value0>40) {                                                                  // adaptive calibration
        CWsettings.tLeft = (7*CWsettings.tLeft + (int)((float)value0 * SENS_FACTOR)  ) / 8;
    //Serial.print("left t,v : "); Serial.print(CWsettings.tLeft); Serial.print(" "); Serial.println(value0);// Serial.println(millis()-t);
    }
    if (value0 > CWsettings.tLeft)
        leftKey = true;
      else
        leftKey = false;
    
    if (value1 > 40) {                                                               // adaptive calibration
      CWsettings.tRight = (7*CWsettings.tRight + (int)((float)value1 * SENS_FACTOR) ) / 8;
    //Serial.print("right t,v : " ); Serial.print(CWsettings.tRight);  Serial.print(" "); Serial.println(value1);//Serial.println(millis()-t);
    }
    if (value1 > CWsettings.tRight)
        rightKey = true;
      else
        rightKey = false;
  }
  return (leftKey || rightKey);
}


///////////////////
// we use the paddle for iambic keying
/////

boolean doPaddleIambic (boolean dit, boolean dah) {
  boolean paddleSwap;                      // temp storage if we need to swap left and right
  static long ktimer;                      // timer for current element (dit or dah)
  static long curtistimer;                 // timer for early paddle latch in Curtis mode B+
 
  if (!CWsettings.didah)   {              // swap left and right values if necessary!
      paddleSwap = dit; dit = dah; dah = paddleSwap; 
  }
      

  switch (keyerState) {                     // this is the keyer state machine
     case IDLE_STATE:
         // display the interword space, if necessary
         if (millis() > interWordTimer) {
             to_scroll(' ');
             interWordTimer = 4294967000;  // almost the biggest possible unsigned long number :-) - do not output extra spaces!
         }
       // Was there a paddle press?
        if (dit || dah ) {
            update_PaddleLatch(dit, dah);  // trigger the paddle latches
            treeptr = 0;

            if (dit) {
                setDITstate();          // set next state
                DIT_FIRST = true;          // first paddle pressed after IDLE was a DIT
            }
            else {
                setDAHstate();  
                DIT_FIRST = false;         // first paddle was a DAH
            } 
        }
        else return false;                // we return false if there was no paddle press in IDLE STATE - Arduino can do other tasks for a bit
        break;

    case DIT:
    /// first we check that we have waited as defined by ACS settings
            if (CWsettings.ACS && (millis() <= acsTimer))  // if we do automatic character spacing, and haven't waited for 3 dits...
              break;
            clear_PaddleLatches();                           // always clear the paddle latches at beginning of new element
            keyerControl |= DIT_LAST;                        // remember that we process a DIT

            ktimer = ditLength;                              // prime timer for dit
            switch (CWsettings.keyermode) {
              case IAMBICB:  curtistimer = max(12, ditLength /6) ;                // Curtis mode B registers paddle presses almost immediately when element begins
                             break;
              case IAMBICBplus:
                             curtistimer = ditLength * .4;    // enhanced Curtis mode B starts checking after 40% of a dit
                             break;
              case IAMBICA:
              case ULTIMATIC:
                             curtistimer = ditLength;        // no early paddle checking in Curtis mode A or Ultimatic mode
                             break;
            }
            keyerState = KEY_START;                          // set next state of state machine
            break;
            
    case DAH:
    /// first we check that we have waited as defined by ACS settings
            if (CWsettings.ACS && (millis() <= acsTimer))  // if we do automatic character spacing, and haven't waited for 3 dits...
                  break;
            clear_PaddleLatches();        // clear the paddle latches
            keyerControl &= ~(DIT_LAST);                      // clear dit latch  - we are not processing a DIT
            
            ktimer = dahLength;
            switch (CWsettings.keyermode) {
              case IAMBICB:  curtistimer = max(12, ditLength /6) ;               // Curtis mode B registers paddle presses almost immediately when element begins
                             break;
              case IAMBICBplus:
                             curtistimer = dahLength * .4;    // enhanced Curtis mode B starts checking after 40% of dah length
                             break;
              case IAMBICA:
              case ULTIMATIC:
                             curtistimer = dahLength;        // no early paddle checking in Curtis mode A or Ultimatic mode
                             break;
            }
            keyerState = KEY_START;                          // set next state of state machine
            break;
     

      
    case KEY_START:
        // Assert key down, start timing, state shared for dit or dah
        // digitalWrite(ledPin, HIGH);             // turn the LED on, key transmitter or whatever
           vol.tone( sidetonePin, CWsettings.sidetoneFreq, realVolume[CWsettings.sidetoneVolume] );    // start generating side tone
           digitalWrite(keyerPin, HIGH);           // turn the LED on, key transmitter, or whatever

           ktimer += millis();                     // set ktimer to interval end time
           //Serial.print("Curtistimer: ");
           //Serial.println(curtistimer);
           
           curtistimer += millis();                // set curtistimer to curtis end time
           keyerState = KEYED;                     // next state
           break;
 
    case KEYED:
                                                   // Wait for timers to expire
           if (millis() > ktimer) {                // are we at end of key down ?
            digitalWrite(keyerPin, LOW);           // turn the LED off, unkey transmitter, or whatever
                 vol.noTone();                     // stop side tone  
                 ktimer = millis() + ditLength;    // inter-element time
                 keyerState = INTER_ELEMENT;       // next state
            }
            else if (millis() > curtistimer ) {    // in Curtis mode we check paddle as soon as Curtis time is off
                 update_PaddleLatch(dit, dah);     // but we remain in the same state until element time is off! 
            }
            break;
 
    case INTER_ELEMENT:
            update_PaddleLatch(dit, dah);          // latch paddle state while between elements       
            if (millis() > ktimer) {               // at end of INTER-ELEMENT
                switch(keyerControl) {
                      case 3:                                         // both paddles are latched
                      case 7: 
                              if (CWsettings.keyermode <= 2)          // we are in a Iambic mode
                                  if (keyerControl & DIT_LAST)        // last element was a dit - this is case 7, really
                                      setDAHstate();               // next element will be a DAH
                                  else                                // and this is case 3 - last element was a DAH
                                      setDITstate();               // the next element is a DIT 
                              else                                    // in Ultimatic mode
                                  if (DIT_FIRST)                      // when first element was a DIT
                                         setDAHstate();            // next element is a DAH
                                  else                                // but when first element was a DAH
                                         setDITstate();            // the next element is a DIT!
                      break;
                                                                      // dit only is latched, regardless what was last element  
                      case 1:
                      case 5:  
                               setDITstate();
                               break;
                                                                      // dah only latched, regardless what was last element
                      case 2:
                      case 6:  
                               setDAHstate();
                               break;
                                                                      // none latched, regardless what was last element
                      case 0:
                      case 4:  
                               keyerState = IDLE_STATE;               // we are at the end of the character and go back into IDLE STATE
                               displayMorse();                        // display the decoded morse character(s)
                               ++charCounter;                         // count this character
                               // if we have seen 12 chars since changing speed, we write the config to EEPROM
                               if (charCounter == 12) {
                                  saveConfig();
                               }
                               acsTimer = millis() + (ACS_LENGTH * ditLength); // prime the ACS timer
                               interWordTimer = millis() + 6*ditLength;  // prime the timer to detect a space between characters
                                                                         // nominally 7 dit-lengths, but we are not quite so strict here
                               keyerControl = 0;                      // clear all latches completely before we go to IDLE
                      break;
                } // switch keyerControl : evaluation of flags
            } // end of INTER_ELEMENT
  } // end switch keyerState - end of state machine
  if (keyerControl & 3)                                               // any paddle latch?                            
    return true;                                                      // we return true - we processed a paddle press
  else
    return false;                                                     // when paddle latches are cleared, we return false
} // end function


///
/// Keyer subroutines
///

// update the paddle latches in keyerControl
void update_PaddleLatch(boolean dit, boolean dah)
{
    if (dit)
      keyerControl |= DIT_L;
    if (dah)
      keyerControl |= DAH_L;
}

// clear the paddle latches in keyer control
void clear_PaddleLatches ()
{
    keyerControl &= ~(DIT_L + DAH_L);   // clear both paddle latch bits
}

// functions to set DIT and DAH keyer states
void setDITstate() {
  keyerState = DIT;
  treeptr = CWtree[treeptr].dit;
}

void setDAHstate() {
  keyerState = DAH;
  treeptr = CWtree[treeptr].dah;
}


// toggle polarity of paddles
void togglePolarity () {
      CWsettings.didah = !CWsettings.didah; 
     displayPolarity();
}
  

/// displaying decoded morse code
void displayMorse() {
  if (treeptr == 0)
    return;
  if (CWtree[treeptr].symb > (unsigned char) 32) {
  morseCharacter[0] = CWtree[treeptr].symb;
  morseCharacter[1] = (byte) 0;
  }
  else
    switch (CWtree[treeptr].symb) {
  case 1: strcpy (morseCharacter, "ch");
        break;
  case 2: strcpy (morseCharacter, "<as>");
        break;
  case 3: strcpy (morseCharacter, "<ka>");
        break;
  case 4: strcpy (morseCharacter, "<kn>");
        break;
  case 5: strcpy (morseCharacter, "<sk>");
        break;
  default: strcpy (morseCharacter, "*");
    }
  // now display the string morseCharacter ....  
  //Serial.println(treeptr);
  //Serial.println(morseCharacter);
  scroll_display(morseCharacter);
  treeptr = 0;                                    // reset tree pointer
}



/// display functions for a scrolling display in 2nd line
void display_line (){
    lcd.setCursor(0,1);
    lcd.print(scroll_line);
}


void to_scroll (char symb) {            /// push one character to the scroll line from the right
  // first move all chars one to the left
  for (byte i = 1; i<16; ++i)
    scroll_line[i-1] = scroll_line[i];
  scroll_line[15] = symb;          // now push character into last position
  display_line();                    // and display the line
  //delay(100);
}
    
void scroll_display ( char string[]) {
   for (unsigned i = 0; string[i]; ++i)
     to_scroll(string[i]);
}


////////// Display functions


void clearTopLine () {                                 // for config settings, clear line and display littel arrows, unless in decoder mode
    if (morseState != morseDecoder) {
      lcd.setCursor(0,0);
      lcd.print((char)127); lcd.print((char)126);
      lcd.print(F("              "));                   // two arrows + 14 blanks in top line
    }
}

void displayTopLine() {
  displayCWspeed(CWsettings.wpm);                       // update display of CW speed
  if (!trainerMode) {
    displayCurtisMode(CWsettings.keyermode);              // and of Curtis mode
    displayPolarity();                                    // and paddle polarity
  } else {
    displayFarnsworthMode();
    displayGeneratorMode();
  }
  
  displayPitch();                                       // sidetone frequency
  displayVolume();                                      // sidetone volume
}

void displayCWspeed (int wpm) {
  // displayCurtisMode(CWsettings.keyermode);            // display keyer mode
  lcd.setCursor(lcdSpeed, 0);      // and current speed
  sprintf(CWspeed_buffer, "%2i", wpm);
  lcd.print(CWspeed_buffer);
  if(!trainerMode)
    lcd.print("wpm");
  //else
    //lcd.print("*");
  //displayPolarity();                                  // and polarity of paddle
}


void displayCurtisMode( byte mode) {                   // this is what we display in Curtis setting Mode
  lcd.setCursor(lcdMode, 0);

  switch (mode) {
    case 0: lcd.print("A "); break;                     // 0 means we are Iambic A (no paddle eval during dah)
    case 1: lcd.print("B "); break;                     // 1: orig Curtis mode: paddle eval during element
    case 2: lcd.print("B+"); break;                     // 2: B+ enhanced mode B
    case 3: lcd.print("Ul");                            // 3: Ultimatic Mode
  }
}

void displayPolarity () {
  lcd.setCursor(lcdPolarity, 0);
  lcd.print( (!CWsettings.didah) ? "_." : "._");
}

void displayPitch () {
  lcd.setCursor(lcdPitch, 0);
  lcd.print (CWsettings.sidetoneFreq);
}

void displayVolume () {
  lcd.setCursor(lcdVolume, 0);
  //lcd.print (CWsettings.sidetoneVolume);  /// change for glyphs!
  if (CWsettings.sidetoneVolume)
    lcd.write(byte(CWsettings.sidetoneVolume));
  else
    lcd.print(" ");
}

void displayFarnsworthMode() {
    lcd.setCursor(lcdFarnsworth, 0);
    lcd.print("*");
    lcd.print(CWsettings.farnsworthMode);
}

void displayGeneratorMode() {
    lcd.setCursor(lcdSymbols, 0);
    lcd.print(groups[CWsettings.generatorMode]);
}


void displayEncoderMode() {       /// this is used to show the parameter that can be adjusted by rotating the encoder
  int pos;
  clearTopLine();
  switch (encoderState) {
    case speedSettingMode:    displayTopLine(); 
                              if (settingsDirty) {          // if settings have been changed when we return to speedsetting, save the config in EEPROM
                                  saveConfig();
                                  settingsDirty = false;
                              }
                              return;
    case curtisSettingMode:   clearTopLine();
                              if (!trainerMode)
                                displayCurtisMode(CWsettings.keyermode);
                              else
                                displayFarnsworthMode();
          break;
    case polaritySettingMode: clearTopLine();
                              if (!trainerMode)
                                displayPolarity();
                              else
                                displayGeneratorMode();
                              break;
    case pitchSettingMode:    clearTopLine();
                              displayPitch();
                              break;
    case volumeSettingMode:   clearTopLine();
                              displayVolume();
                              break;
  }
}


/// functions for training mode

void generateCW () {          // we use a list of elements for that character, and the number of elements

  switch (generatorState) {                        // CW generator state machine
    case KEY_DOWN:
            if (millis() > timer) {                // are we at end of key down ?
                 vol.noTone();                     // stop side tone
                 digitalWrite(keyerPin, LOW);      // if we key in trainer mode, or not....
                 ++pointer; 
                 if (pointer < NoE)               // we need to output another element for this character
                    timer = millis() + ditLength;  // pause is inter element (= 1 dit = ditLength)
                 else {
                    scroll_display(morseCharacter);// display the old character before we begin with a new one
                    //Serial.println(morseCharacter);
                    fetchNextChar();               // function to retrieve the next character we should output
                    pointer = 0;                   // reset pointer to start of list of elements
                    timer = millis() + (NoE == 0 ? interWordSpace : interCharacterSpace); // next space is between characters or between words?
                    if (NoE == 0) {
                      fetchNextChar();
                      scroll_display((char *)" ");
                      }
                    }
                 generatorState = KEY_UP;          // next state
                 }
            break;
    case KEY_UP:
            if (millis() > timer) {                // at end of the pause
                vol.tone( sidetonePin, CWsettings.sidetoneFreq, realVolume[CWsettings.sidetoneVolume] );    // start generating side tone  
                if (keyTrainerMode) 
                  digitalWrite(keyerPin, HIGH);           // turn the LED on, key transmitter, or whatever
                timer = millis() + (nextElement[pointer] ? dahLength : ditLength);                          // start a dit or a dah and set timer accordingly
                generatorState = KEY_DOWN;         // next state
                }
             break;            
  }
}


void fetchNextChar() {      // retrieve the next character to be output, and set up NoE (number of elements) and nextElement[] (list of elements)
  byte character;
  byte i, bitmask;
  static byte j = 0;        // counter for TEST

  while (startAgain && (k<6)) {       /// this outputs vvv<ka> at the begin of each training run
    byte next = atStart[k];
    ++k;
    if (k==6) {
        k=0; startAgain = false;
    }
    character = next;
    break;
  }

  if (!startAgain) {
    switch (generatorMode) {
      case GROUPOF5:
              if ((characterCounter % 6) == 0)       /// every 6th character is a space, so to say
                  character = 53;           //// we use this as a place holder for a space
              else
                  character = random(startPool, endPool);
              //Serial.print(characterCounter);
              //Serial.print(" ");
              //Serial.println(character);
  
              break;
      case CALLSIGNS:                               /// 
              character = generateCallsign( ( characterCounter == 1 ? true : false) );
              break;
      case Q_ABBREV:
              character = generateAbbrev( (characterCounter == 1 ? true : false ) );
              break;
      case TESTALL:                                 /// TESTALL is for debugging puposes only
              if (j > 52)
                character = j = 0;
              else {
                character = j++;
                //Serial.println(character);
              }
              break;
  
      // insert other cases here, e.g. call signs, qso texts, clear texts or whatever
    }         
    ++ characterCounter;
    //Serial.print("CWsettings.generatorMode: ");
    //Serial.println(CWsettings.generatorMode);
    //Serial.print("generatorMode: ");
    //Serial.println(generatorMode);
    //Serial.print("characterCounter: ");
    //Serial.println(characterCounter);
    //Serial.println(character);
  }
            
    // now initialise everything
    if (character > 52 )          /// space
      NoE = 0;
    else {                        //// look up in our table - read it from Progmem into buffer first
      NoE = pgm_read_byte( &pool[character][1]);
      bitmask = pgm_read_byte( &pool[character][0]);
      //Serial.print("Bitmask: ");
      //Serial.println(bitmask, BIN);
      for (i=0; i<NoE; ++i) {
        nextElement[i] = (bitmask & B10000000 ? 1 : 0 );    // get MSB and store it in array
        bitmask = bitmask << 1;                 // shift bitmask 1 bit to the left 
              //Serial.print("Bitmask: ");
      //Serial.println(bitmask, BIN);

      }
    }
    
    /// now look up the character or string we need to output on the display
    morseCharacter[1] = 0;         // this terminates the string for all single characters

    if ((character > 3) && (character < 30))
      morseCharacter[0] = character + 93;
    else if ((character > 29) && (character < 40)) 
      morseCharacter[0] = character + 18;
    else switch(character) {
          case 0:  morseCharacter[0] = 225; // ä
                break;
          case 1:  morseCharacter[0] = 239; // ö
                break;
          case 2:  morseCharacter[0] = 245; // ü
                break;
          case 40:  morseCharacter[0] = character +6;
                break;
          case 44: 
          case 41:  morseCharacter[0] = character +3;
                break;
          case 45:
          case 42:  morseCharacter[0] = character +16;
                break;
          case 43:  morseCharacter[0] = character +2;
                break;
          case 46:
          case 47:  morseCharacter[0] = character +17;
                break;
          case 48:  morseCharacter[0] = character -5;
                break;
          case 53:  morseCharacter[0] = ' ';
        }
    if (character == 3)
        // insert ch
        strcpy(morseCharacter, "ch");
    if (character == 49)
        // insert <as>
        strcpy(morseCharacter, "<as>");
    if (character == 50)
        // insert <ka>
        strcpy(morseCharacter, "<ka>");
    if (character == 51)
        // insert <kn>
         strcpy(morseCharacter, "<kn>");
    if (character == 52)
        // insert <sk>
        strcpy(morseCharacter, "<sk>");  
}

char generateCallsign(boolean next) {             // generate a new random string that looks like a callsign
  static char call[9];
  const byte prefixType[] = {1,0,1,2,3,1};      // o= a, 1 = aa, 2 = a9, 3 = 9a
  static byte i, l;                             // i = index, l = length of callsign (including terminating 53 / space)
  
  byte prefix;

  if (next) {
    i = l;
  }
  if (i == l) {                                 // we need to generate a new call sign
    i = l = 0;
    prefix = prefixType[random(0,6)];           // what type of prefix?
    switch (prefix) {
      case 1: call[l] = random(4,30);
              ++l;
      case 0: call[l] = random(4,30);
              ++l;
              break;
      case 2: call[l] = random(4,30);
              ++l;
              call[l] = random(30,40);
              ++l;
              break;
      case 3:call[l] = random(30,40);
              ++l;
              call[l] = random(4,30);
              ++l;
              break;
    } // we have a prefix by now
    // generate a number
    call[l] = random(30,40);
    ++l;
    // generate a suffix, 1 2 or 3 chars long - we re-use prefix for the suffix length
    prefix = random(1,4);
    prefix = (prefix == 2 ? prefix :  random(1,4)); // increase the likelihood for suffixes of length 2
    while (prefix--) {
      call[l] = random(4,30);
      ++l;
    } // now we have the suffix
    // are we /p or /m? - we do this only in rare cases - 1 out of 10
    if (! random(0,10)) {
      call[l] = 44;
      ++l;
      call[l] = ( !random(0,2) ? 16 : 19 );
      ++l;
    }
    // we have a complete call sign! finish by adding a placeholder for space
    call[l] = 53;
    ++l;
  }
  // return next chr of callsign
  return call[i++];
}


char generateAbbrev(boolean next) {     // randomly fetch q-groups and abbreviations from PROGMEM and return the characters byte by byte
  static char abbrev[MAX_SIZE];
  char c;
  static byte i=1, l=1;                 // i = index, l = length of abbrev (including terminating 53 / space)
  int j;

 if (next) {
    i = l;
  }
  if (i == l) {                                 // we need to fetch a new abbrev
    //Serial.print("Next:" );
    i = 0;
    j = random(0,NUMBER_OF_ELEMENTS);
    //Serial.println(j);
    memcpy_P (&abbrev, &abbreviations[j], sizeof abbrev);
    //Serial.println(abbrev);
    l = strlen(abbrev);
    //Serial.println(l);
    abbrev[l] = 53 + 93;                             // we need this as the final char
    ++l;
  }
  // return next char of abbreviation
 c = abbrev[i++];
 return (c < 58 ? c-18 : c-93 );
 // we do NOT use ASCII encoding for generating morse code!!
}

/////////////////////////
//// TOP MENU
////////////////////////

void topMenu() {                            // display top-menu and wait for selection
  const byte topLevels = 3;          
  int newMode = (int)morseState;

  vol.noTone();                     // stop side tone  - just in case
  digitalWrite(keyerPin, LOW);           // turn the LED off, unkey transmitter, or whatever; just in case....

 displayTopMenu(newMode);
  while (true) {
        modeButton.Update();
        if (modeButton.clicks) {
          if (modeButton.clicks == 2) {            // a double click in top menu means we toggle keyer output on/off
            keyTrainerMode = !keyTrainerMode;
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(F("Key CW Trainer:"));
            lcd.setCursor(0,1);
            lcd.print( keyTrainerMode ? "On" : "Off" );
            delay(1500);
            displayTopMenu(newMode);
            modeButton.Update();
            }
          else if (modeButton.clicks == 3) {            // a triple click in top menu means we toggle ACS on/off
            CWsettings.ACS = !CWsettings.ACS;
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(F("Auto Char Space:"));
            lcd.setCursor(0,1);
            lcd.print( CWsettings.ACS ? "On" : "Off" );
            saveConfig();
            delay(1500);
            displayTopMenu(newMode);
            modeButton.Update();
            }
          else {
          if (newMode != morseState)
              morseState = (morserinoMode)newMode;

          switch (morseState) {
            case morseKeyer:    setupKeyerMode();
                                break;
            case morseTrainer:  setupTrainerMode();
                                break;
            case morseDecoder:  setupDecoderMode();
                                break;   
          }
          return;       // we got a click, and do the rest only otherwise
        }
        } 
          
        if (TurnDetected) {
             vol.tone(sidetonePin, 250, realVolume[CWsettings.sidetoneVolume]);
             delay(8);
             vol.noTone();
             encoderPos = (up ? 1 : -1);
             TurnDetected = false;
         }
         if (encoderPos) {
            newMode = (newMode + topLevels + encoderPos) % topLevels;
            encoderPos = 0;
            lcd.setCursor(0,1);
            switch ((morserinoMode)newMode) {
                  case morseKeyer: lcd.print(F("1 - CW Keyer  "));
                                   break;
                  case morseTrainer:
                                   lcd.print(F("2 - CW Trainer"));
                                   break;
                  case morseDecoder:
                                   lcd.print(F("3 - CW Decoder"));
                                   break;
           }
         }
  
  } // end while - we leave as soon as the button has been pressed
} // end function topMenu

void displayTopMenu( int mode) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Select Mode:"));
  lcd.setCursor(0,1);
            switch (mode) {
                  //case invalid:    newMode = 0;
                  case 0: lcd.print(F("1 - CW Keyer  "));
                                   break;
                  case 1:
                                   lcd.print(F("2 - CW Trainer"));
                                   break;
                  case 2:
                                   lcd.print(F("3 - CW Decoder"));
                                   break;
           }
}


// encoder subroutines
/// interrupt service routine

void isr ()  {                    // Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
 if ( micros() < (rotating + 11500) ) return;  
 if (digitalRead(PinCLK))
   up = digitalRead(PinDT);
 else
   up = !digitalRead(PinDT);
 rotating = micros();
 TurnDetected = true;
}

///
// Save configuration to EEPROM 
///
void saveConfig () {
      EEPROM.updateByte(addressSignature, MorserSignature);
      EEPROM.updateBlock(addressCWsettings, CWsettings);  
      return;
}

////////////////////////////
///// Routines for morse decoder - to a certain degree based on code by Hjalmar Skovholm Hansen OZ1JHM - copyleft licence
///////////////////////////

void setupGoertzel () {                 /// pre-compute some values that are compute-imntensive and won't change anyway
  bw = (sampling_freq / n);

  int  k;
  float omega;
  k = (int) (0.5 + ((n * target_freq) / sampling_freq));
  omega = (2.0 * PI * k) / n;
  sine = sin(omega);
  cosine = cos(omega);
  coeff = 2.0 * cosine;
}


boolean checkTone() {                 /// check if we have a tone signal at A6 with Gortzel's algorithm, and apply some noise blanking as well
                                   /// the result will be in globale variable filteredState
                                   /// we return true when we detected a change in state, false otherwise!
  static boolean realstate = false;
  static boolean realstatebefore = false;
  static unsigned long lastStartTime = 0;

if (((!digitalRead(straightPin))) || checkPaddles() ) {
    realstate = true;
    keyTx = true;
    }
else {
    realstate = false;
    keyTx = false;
    for (char index = 0; index < n; index++)
        testData[index] = analogRead(audioInPin);
    for (char index = 0; index < n; index++) {
      float Q0;
      Q0 = coeff * Q1 - Q2 + (float) testData[index];
      Q2 = Q1;
      Q1 = Q0;
    }
    float magnitudeSquared = (Q1 * Q1) + (Q2 * Q2) - Q1 * Q2 * coeff; // we do only need the real part //
    magnitude = sqrt(magnitudeSquared);
    Q2 = 0;
    Q1 = 0;
  
    //Serial.print(magnitude); Serial.println();  //// here you can measure magnitude for setup..
  
    ///////////////////////////////////////////////////////////
    // here we will try to set the magnitude limit automatic //
    ///////////////////////////////////////////////////////////
  
    if (magnitude > magnitudelimit_low) {
      magnitudelimit = (magnitudelimit + ((magnitude - magnitudelimit) / 6)); /// moving average filter
    }
  
    if (magnitudelimit < magnitudelimit_low)
      magnitudelimit = magnitudelimit_low;
  
    ////////////////////////////////////
    // now we check for the magnitude //
    ////////////////////////////////////
  
    if (magnitude > magnitudelimit * 0.6) // just to have some space up
      realstate = true;
    else
      realstate = false;
  }



  /////////////////////////////////////////////////////
  // here we clean up the state with a noise blanker //
  // (debouncing)                                    //
  /////////////////////////////////////////////////////

  if (realstate != realstatebefore)
    lastStartTime = millis();
  if ((millis() - lastStartTime) > nbtime) {
    if (realstate != filteredState) {
      filteredState = realstate;
    }
  }
  realstatebefore = realstate;
 if (realstate) {
  //Serial.print("Realstate; ");
  //Serial.println(realstate);
  //Serial.print("Filteredstate; ");
  //Serial.println(filteredState);
 }
 if (filteredState == filteredStateBefore)
  return false;                                 // no change detected in filteredState
 else {
    filteredStateBefore = filteredState;
    return true;                                // change detected in filteredState
 }
}   /// end checkTone()


void doDecode() {
  float lacktime;
  int wpm;

    switch(decoderState) {
      case INTERELEMENT_: if (checkTone()) {
                              ON_();
                              decoderState = HIGH_;
                          } else {
                              lowDuration = millis() - startTimeLow;             // we record the length of the pause
                              lacktime = 3.0;                 ///  when high speeds we have to have a little more pause before new letter 
                              if (CWsettings.wpm > 35) lacktime = 3.2;
                                else if (CWsettings.wpm > 30) lacktime = 3.4;
                              if (lowDuration > (lacktime * ditAvg)) {
                                displayMorse();                   /// decode the morse character and display it
                                wpm = (CWsettings.wpm + (int) (7200 / (dahAvg + 3*ditAvg))) / 2; //// recalculate speed in wpm
                                if (CWsettings.wpm != wpm) {
                                  CWsettings.wpm = wpm;
                                  speedChanged = true;
                                }
                                decoderState = INTERCHAR_;
                              }
                          }
                          break;
      case INTERCHAR_:    if (checkTone()) {
                              ON_();
                              decoderState = HIGH_;
                          } else {
                              lowDuration = millis() - startTimeLow;             // we record the length of the pause
                              lacktime = 7.0;                 ///  when high speeds we have to have a little more pause before new word
                              if (CWsettings.wpm > 35) lacktime = 8.0;
                                else if (CWsettings.wpm > 30) lacktime = 9.0;
                              if (lowDuration > (lacktime * ditAvg)) {
                                to_scroll(' ');
                                decoderState = LOW_;
                              }
                          }
                          break;
      case LOW_:          if (checkTone()) {
                              ON_();
                              decoderState = HIGH_;
                          }
                          break;
      case HIGH_:         if (checkTone()) {
                              OFF_();
                              decoderState = INTERELEMENT_;
                          }
                          break;
    }
}

void ON_() {                                  /// what we do when we just detected a rising flank, from low to high
   unsigned long timeNow = millis();
   lowDuration = timeNow - startTimeLow;             // we record the length of the pause
   startTimeHigh = timeNow;                          // prime the timer for the high state
   vol.tone( sidetonePin, 744, realVolume[CWsettings.sidetoneVolume] );    // start generating side tone
   if (keyTx)
      digitalWrite(keyerPin, HIGH);                 // turn the LED on, key transmitter, or whatever

   lcd.setCursor(14,0);
   lcd.write(255);
   if (lowDuration < ditAvg * 2.4)                    // if we had an inter-element pause,
      recalculateDit(lowDuration);                    // use it to adjust speed
}

void OFF_() {                                 /// what we do when we just detected a falling flank, from high to low
  unsigned long timeNow = millis();
  unsigned int threshold = (int) ( ditAvg * sqrt( dahAvg / ditAvg));

  Serial.print("threshold: ");
  Serial.println(threshold);
  highDuration = timeNow - startTimeHigh;
  startTimeLow = timeNow;

  if (highDuration > (ditAvg * 0.5) && highDuration < (dahAvg * 2.5)) {    /// filter out VERY short and VERY long highs
      if (highDuration < threshold) { /// we got a dit -
            treeptr = CWtree[treeptr].dit;
            //Serial.print(".");
            recalculateDit(highDuration);
      }
      else  {        /// we got a dah
            treeptr = CWtree[treeptr].dah;   
            //Serial.print("-");   
            recalculateDah(highDuration);                 
      }
  }
  vol.noTone();                     // stop side tone
  digitalWrite(keyerPin, LOW);      // stop keying Tx
  lcd.setCursor(14,0);
  lcd.write(' ');

}

void recalculateDit(unsigned long duration) {       /// recalculate the average dit length
  static int rot = 0;
  static unsigned long collector;
  
  if (rot<4) {
    collector += duration;
    ++rot;
  } else {
    ditAvg = (4* ditAvg + collector) >> 3; //// equivalen to divide by eight - 4 old avg values plus the new 4 samples
    rot = 0;
    collector = 0;
  }
  //Serial.print("ditAvg: ");
  //Serial.println(ditAvg);
}

void recalculateDah(unsigned long duration) {       /// recalculate the average dah length
  static int rot = 0;
  static unsigned long collector;

  if (duration > 2* dahAvg)   {                       /// very rapid decrease in speed!
      dahAvg = (dahAvg + 2* duration) / 3;            /// we adjust faster, ditAvg as well!
      ditAvg = ditAvg/2 + dahAvg/6;
  }
  else {
      if (rot<2) {
        collector += duration;
        ++rot;
      } else {
        dahAvg = (3* ditAvg + dahAvg + collector) >> 2; //// equivalen to divide by four - 1 old avg dah values, plus one computed from avg dit,  plus the new 2 samples
        rot = 0;
        collector = 0;
      }
  }
    //Serial.print("dahAvg: ");
    //Serial.println(dahAvg);
}

byte i2cScan() {                                     // run a scanner to find out I2C address of LCD display - no other i2c devices should be active during scan!
  byte address, error;                               // we just check for 0x27 - if this is the wrong address, we assume it has to be 0x3f,
                                                     // as these are the two addresses, depending on the chip being used (either 8574T or 8574AT)


  //delay(2000);
  for (address = 20; address < 96; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0) {
      //Serial.print("Device found on address (decimal): ");
      //Serial.println(address);
      return address;
    }
   
  }
  return 39;
}

