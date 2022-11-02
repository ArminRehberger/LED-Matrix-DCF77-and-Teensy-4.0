/*
""" ############################################################################################ """
""" ############################################################################################ """
""" LED matrix (8*X) WS2812B control with Teensy4.0 """
""" DCF77 clock signal
""" V1_00 2022-11-02, Armin Rehberger """
""" ############################################################################################ """
""" ############################################################################################ """
*/

/*
// ###############################################################################################################
// ###############################################################################################################
##### Libary for LEDs driver: OctoWS2811
https://github.com/PaulStoffregen/OctoWS2811
#include <OctoWS2811.h>

##### Libary for color and shift the LEDs
LEDMatrix8.h
Integrated with "", so copy the files LEDMatrix8.cpp and LEDMatrix8.h in the same directory than the file CNC_LED_Class.ino
#include "LEDMatrix8.h"
*/

/*
// ###############################################################################################################
// ###############################################################################################################
Frequency calculation, example RGB LED strip with 300LEDs
800Khz = 1.25us
Per bit (high or low): 1.25us
Per LED: 3 Byte = 24 Bit = 30us
Reset: 50us
Per 300 LED: 30us * 300LED = 9000us + 50us = 9500us = 9.5ms
Frequency: F=1/t = 1/0.0095s = 105.26Hz

Frequency calculation, example RGB LED strip with 256LEDs
800Khz = 1.25us
Per bit (high or low): 1.25us
Per LED: 3 Byte = 24 Bit = 30us
Reset: 50us
Per 256 LED: 30us * 256LED = 7680us + 50us = 7730us = 7.73ms
Frequency: F=1/t = 1/0.00773s = 129.36Hz

Frequency calculation, example RGB LED strip with 125LEDs
800Khz = 1.25us
Per bit (high or low): 1.25us
Per LED: 3 Byte = 24 Bit = 30us
Reset: 50us
Per 125 LED: 30us * 125LED = 3750us + 50us = 3800us = 3.8ms
Frequency: F=1/t = 1/0.0038s = 263.16Hz
*/

/*
// ###############################################################################################################
// ###############################################################################################################
Current calculation
Used voltage: 5V
Per LED 60mA, when all three LEDs are on (color white)
300LEDs  = 18A
2400LEDs = 144A (300LED * 8 stripes)
256LEDs  = 15,36A
1024LEDs = 61.4A (256LED * 4 stripes)
2048LEDs = 122.8A (256LED * 8 stripes)
*/

// ###############################################################################################################
// ###############################################################################################################
// ##### Include
#include <OctoWS2811.h>
#include "LEDMatrix8.h"
#include <DCF77.h>       //https://github.com/thijse/Arduino-DCF77
#include <TimeLib.h>

// ###############################################################################################################
// ###############################################################################################################
// ##### Common definitions
#define DEBUGMODE true // Print diagnostic values to serial monitor

// ##### Definitions LED matrix
#define NONE 0
#define LEFT 1
#define RIGHT 2
#define UP 3
#define DOWN 4

#define COLOR0 0
#define COLOR1 1
#define COLOR2 2

// ##### Definitions DCF77
#define DCF_PIN 23         // Connection pin to DCF 77 device --> pinMode(DCF_PIN, INPUT);
#define DCF_INTERRUPT 23	 // Interrupt associated with pin. Must be the same then DCF_PIN. --> attachInterrupt(DCF_INTERRUPT, int0handler, CHANGE);

// ###############################################################################################################
// ###############################################################################################################
// ##### Global strings to write to the LED matrix
// All together max. 1773 characters

// Already defined letters in function "InitializeLetters"
// "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890!?.:+-=# {}[]"
// { Haert, part 1
// } Haert, part 2
// [ Smily, part 1
// ] Smily, part 2

char StringToDraw[3][100] = {
  " DD:MM:YYYY HH:MM:SS                       ",
  "MERRY CHRISTMAS AND A HAPPY NEW YEAR {}{} ",
  "FROHE WEIHNACHTEN UND EIN GUTES NEUES JAHR [][] ",
  };

// ###############################################################################################################
// ###############################################################################################################
// ##### Global struct to draw the scenes
typedef struct
{
  bool ArrayIndexUsed;
  int StringNo;
  unsigned long DelaytimeMillis;
  int ColorSceneNo;
  int Color;
  int MaxIntense;
  int AnimationNo;
  int AmountNo;
} ParameterDrawScene;
ParameterDrawScene ParaDraw[100] = {};

// ###############################################################################################################
// ###############################################################################################################
// ##### Forward declaration functions
void DrawScene(time_t, bool);
void DrawPixels(int, const byte[], unsigned long);
void digitalClockDisplay(time_t);
void printDigits(int);

// ###############################################################################################################
// ###############################################################################################################
// ##### Initialize scenes
// LED-Matrix will start with scene 0, 1, ... After last scene, it starts again with scene 0

// ArrayIndexUsed: Must be true, when the scene should be drawen
// StringNo: String no. in the StringToDraw array to be drawen
// DelaytimeMillis: Delaytime in ms to shift the pixel array e.g. 10, 20, 25, 50, 100, 2000, 0=fastest possible speed
// ColorSceneNo: COLOR0, whole matrix one times BLUE, then one times GREEN, then one times RED, ...YELLOW PINK ORANGE WHITE
// ColorSceneNo: COLOR1, whole matrix blue MinIntense..MaxIntense..MinIntense, green MinIntense..MaxIntense..MinIntense, red MinIntense..MaxIntense..MinIntense
// ColorSceneNo: COLOR2, whole matrix in value Color
// Color: Used for COLOR2. RED, GREEN, BLUE, YELLOW, PINK, ORANGE, WHITE, BACKGROUNDCOLUR
// MaxIntense: Used for Color1, MaxIntense 0..255. MinIntense = 1
// AnimationNo: NONE, LEFT, RIGHT, UP, DOWN
// AmountNo: Show string x times, e.g. LEFT 7 times

void InitValuesForScenes()
{
  // ##### Local variables
  int Index = 0;

  // ##### Scene 0, String no. 0, COLOR0
  ParaDraw[Index].ArrayIndexUsed = true;
  ParaDraw[Index].StringNo = 0;
  ParaDraw[Index].DelaytimeMillis = 0;
  ParaDraw[Index].ColorSceneNo = COLOR2;
  ParaDraw[Index].Color = GREEN; // Just for COLOR2
  ParaDraw[Index].MaxIntense = 96; // Just for COLOR1
  ParaDraw[Index].AnimationNo = NONE;
  ParaDraw[Index].AmountNo = 0;
  Index++;
}

// ###############################################################################################################
// ###############################################################################################################
// ##### Create and configure an OctoWS2811 object
// The total number of pixels is "ledsPerStrip * numPins"
const int ledsPerStrip = 256; // LED-matrix 8x32
const int numPins = 4; // Amount of conected output pins, 1, 2, 3, 4, ... 8
byte pinList[numPins] = {7, 6, 5, 4}; // Connected output pin numbers. With Teensy 4.x, you can use any group of pins.
const int bytesPerLED = 3;  // RGB 3 bytes needed, RGBW 4 bytes needed
DMAMEM int displayMemory[ledsPerStrip * numPins * bytesPerLED / 4]; // Allocate DMA memory Teensy4 DMAChannel.h. Divide by 4 int -> byte
int drawingMemory[ledsPerStrip * numPins * bytesPerLED / 4];

const int config = WS2811_GRB | WS2811_800kHz; // LED Matrix WS2811_GRB, LED stripe WS2811_RGB
OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config, numPins, pinList);

// ###############################################################################################################
// ###############################################################################################################
// ##### Create and configure an LED matrix object
int AmountOfStrings;    // Amount of strings in the StringToDraw char-array
int AmountOfIndex;      // Amount of index in the ParaDraw array
int ActiveIndex = 0;
const int AmountOfPixelsDrawingArray = ledsPerStrip * numPins;
LEDMatrix8 matrix(StringToDraw, AmountOfPixelsDrawingArray, ActiveIndex); // String array, amount of pixels, active string no.

// ###############################################################################################################
// ###############################################################################################################
// ##### Create a DCF77 object
DCF77 DCF = DCF77(DCF_PIN, DCF_INTERRUPT, true); // DCF pin, DCF interrupt pin, OnRisingFlank

// ###############################################################################################################
// ###############################################################################################################
// ##### Initialization setup()
void setup()
{

  // ##### Define LED on board
  pinMode(LED_BUILTIN, OUTPUT); // LED on board

  // ##### Serieller Monitor
  if(DEBUGMODE == true)
  {
    Serial.begin(9600);
    Serial.println("Startup");
  }

  // ##### Start OctoWS2811
  leds.begin();
  leds.show();

  // ##### Initialize values for scenes
  InitValuesForScenes();

  // ##### Get amount of strings in the StringToDraw char-array
  AmountOfStrings = sizeof StringToDraw / sizeof *StringToDraw;
  // ##### Get amount of index in the ParaDraw array
  AmountOfIndex = sizeof ParaDraw / sizeof *ParaDraw;

  // ##### Check if in first scene string no. != 0, because matrix is initialized with string no. 0
  // If so, initialize scene again
  if(ParaDraw[ActiveIndex].StringNo != 0)
  {
      if(ParaDraw[ActiveIndex].StringNo >= AmountOfStrings)
      {
        ParaDraw[ActiveIndex].StringNo = 0;
      }
      matrix.InitSzene(REDRAW, ParaDraw[ActiveIndex].StringNo);
  }

  // ##### Start DCF77
  DCF.Start();

} // void setup()

// ###############################################################################################################
// ###############################################################################################################
// ##### Main program loop()
void loop()
{
  // ##### Local variables
  bool ledon = false;
  static unsigned long previousMillis = 0;
  static unsigned long currentMillis = 0;
  time_t DCFtime;

  // ##### DCF77
  // Get DCF time and update LED matrix every second
  currentMillis = millis();
  if(currentMillis >= previousMillis)
  {
    previousMillis = currentMillis + 1000;

    // Get DCF time
    DCFtime = DCF.getTime();

    if(DCFtime!=0)
    {
      // ##### Set time
      setTime(DCFtime);

      // ##### Draw scene
      DrawScene(DCFtime, true);

      // ##### Print on serial monitor
      if(DEBUGMODE == true)
      {
        Serial.print(" DCF: ");    
        digitalClockDisplay(DCFtime);
      }
    }	
    else
    {
      // ##### Draw scene
      DrawScene(now(), false);

      // ##### Print on serial monitor
      if(DEBUGMODE == true)
      {
        Serial.print(" Running: ");
        digitalClockDisplay(now());
      }
    }
  } // if(currentMillis >= previousMillis)

  // ##### DFC input pin to LED on board
  ledon = digitalRead(DCF_PIN);
  digitalWrite(LED_BUILTIN, ledon);

} // void loop()

// ###############################################################################################################
// ###############################################################################################################
// ##### Draw scene
void DrawScene(time_t _time, bool Sync)
{
  // ##### Local variables
  tmElements_t tm;
  static bool sync = false;
  char strActTime[40] ={};

  breakTime(_time, tm);

  if(tm.Second == 1 && Sync == false)
  {
    sync = false;
  }
  if(tm.Second == 1 && Sync == true)
  {
    sync = true;
  }

  if(sync == false)
    sprintf(strActTime, " %02i:%02i:%04i %02i:%02i:%02i", tm.Day, tm.Month, tm.Year+1970, tm.Hour, tm.Minute, tm.Second);
  else
    sprintf(strActTime, " %02i:%02i:%04i-%02i:%02i:%02i", tm.Day, tm.Month, tm.Year+1970, tm.Hour, tm.Minute, tm.Second);
  
  strcpy(matrix.StringToDraw[0], strActTime);
  matrix.InitSzene(REDRAW, ParaDraw[ActiveIndex].StringNo);
  matrix.ColorScene2(ParaDraw[ActiveIndex].Color); // ColorScene 2, whole matrix in one color
  DrawPixels(AmountOfPixelsDrawingArray, matrix.DrawPixelArray, 0); //ParaDraw[ActiveIndex].DelaytimeMillis);
}

// ###############################################################################################################
// ###############################################################################################################
// ##### Draw the pixels
void DrawPixels(int inAmountOfPixelsDrawingArray, const byte inDrawPixelArray[], unsigned long inDelaytimeMillis)
{
  // ##### Local variables
  static unsigned long previousMillis = 0;
  static unsigned long currentMillis = 0;
  bool LEDbusy;
  int ColorPixel;
  int i;

  // ##### Set the pixels color
  for (i=0; i < inAmountOfPixelsDrawingArray; i++)
  {
    if(inDrawPixelArray[i] == 0) // 0 = pixel for background
    {
      ColorPixel = BACKGROUNDCOLUR;
    }
    else if(inDrawPixelArray[i] == 1) // 1 = pixel for letter / number
    {
      ColorPixel = matrix.ColorPixel[i];
    }
    else if(inDrawPixelArray[i] == 2) // 2 = pixel for underline
    {
      //ColorPixel = RED;
      ColorPixel = BACKGROUNDCOLUR;
      //ColorPixel = matrix.ColorPixel[i];
    }
    else
    {
      ColorPixel = BACKGROUNDCOLUR;
    }
    leds.setPixel(i, ColorPixel); // Set the pixels color
  }

  // ##### Draw the pixels
  leds.show();

  // ##### Delaytime in ms
  if(inDelaytimeMillis != 0)
  {
    currentMillis = millis();
    previousMillis = currentMillis;
    do
    {
      currentMillis = millis();
    } while (currentMillis - previousMillis < inDelaytimeMillis);    
  }
  // ##### Fastest possible speed
  else
  {
    do
    {
      LEDbusy = leds.busy();
    }while (LEDbusy == true);    
  }
}// void DrawPixels

// ###############################################################################################################
// ###############################################################################################################
// ##### Serial print DCF77 time
void digitalClockDisplay(time_t _time)
{
  tmElements_t tm;   
  breakTime(_time, tm);

  Serial.print("Time: ");
  Serial.print(tm.Hour);
  Serial.print(":");
  printDigits(tm.Minute);
  Serial.print(":");
  printDigits(tm.Second);
  Serial.print(" Date: ");
  Serial.print(tm.Day);
  Serial.print(".");
  Serial.print(tm.Month);
  Serial.print(".");
  Serial.println(tm.Year+1970);
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

