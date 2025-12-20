// Copyright 2025 Rich Heslip
//
// Author: Rich Heslip 
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Modal voice for 2HPico Eurorack module 
// uses Arduino DaisySP library https://github.com/rheslip/DaisySP_Teensy
// also uses Adafruit NeoPixel library
// V1.0 RH Nov 2025

// top jack - trigger input
// middle jack - pitch CV input
// bottom jack - audio out

// page 1 parameters - Red LED
// Pot 1 - structure
// pot 2 - damping
// pot 3 - brightness
// pot 4 - frequency

// page 2 parameters - Green LED
// Pot 1 - accent
// pot 2 - sustain - rotate past 12 o'clock for sustained output
// pot 3 - unused
// pot 4 - unused

#include <2HPico_io.h>
#include <I2S.h>
#include <Adafruit_NeoPixel.h>
#include "pico/multicore.h"

#define MONITOR_CPU1  // define to enable 2nd core monitoring

//#define SAMPLERATE 11025 
#define SAMPLERATE 22050 // about the best a Pico 2 can do at 250 Mhz
//#define SAMPLERATE 44100

#define NUMPIXELS 1 // 
#define RED 0x1f0000  // only using 5 bits to keep LEDs from getting too bright
#define GREEN 0x001f00
#define BLUE 0x00001f
#define ORANGE (RED|GREEN)
#define VIOLET (RED|BLUE)
#define AQUA (GREEN|BLUE)

Adafruit_NeoPixel LEDS(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

I2S DAC(OUTPUT);  // using once channel of a PT8211 stereo DAC

#define DEBUG   // comment out to remove debug code

// constants for integer to float and float to integer conversion
#define MULT_16 2147483647
#define DIV_16 4.6566129e-10


#include "daisysp.h"

// including the source files is a pain but that way you compile in only the modules you need
// DaisySP statically allocates memory and some modules e.g. reverb use a lot of ram
#include "physicalmodeling/resonator.cpp"
#include "physicalmodeling/modalvoice.cpp"

float samplerate=SAMPLERATE;  // for daisySP


// create daisySP processing objects

#define VOICES 1   // RP2350 can't quite do 2 voices at 250mHz. my boards won't run 300mHz

daisysp::ModalVoice voice;

// a/d values from pots
// the pots are "locked" when the parameter page changes
// this prevents an immediate change when we switch pages
// we unlock each pot and allow parameter values to change when there is a significant movement of the pot

#define AD_BITS 12
#define AD_RANGE 4096  // RP2350 has 12 bit A/D so lets use it
#define POT1_2  AIN3  // Pots 1 & 2 together on mux
#define POT3_4  AIN2  // Pots 3 & 4 ""
#define MIN_POT_CHANGE 100 // pot reading must change by this in order to register
#define POT_AVERAGING 5  // A/D average over this many readings
#define NUMPOTS 4
uint16_t pot[NUMPOTS]; // pot A/D readings
bool potlock[NUMPOTS]; // when pots are locked it means they must change by MIN_POT_CHANGE to register

#define CVIN_VOLT 580.6  // a/d count per volt - **** adjust this value for more accurate V/octave input
#define CV1IN AIN0   // CV1 input - top jack
#define CV2IN AIN1   // CV2 input - middle jack
#define CV_AVERAGING 10  // A/D average over this many readings
#define CVOUT_VOLT 6554 // D/A count per volt - nominally +-5v range for -+32767 DAC values- ***** adjust this value for accurate V/octave out
#define CVOUTMIN -2*CVOUT_VOLT  // lowest output CV ie MIDI note 0

float minfreq=10;

bool trigger=0;
bool button=0;
#define NUMUISTATES 2
enum UIstates {SET1,SET2} ;
uint8_t UIstate=SET1;
uint32_t buttontimer,trigtimer,parameterupdate;
#define DEBOUNCE 10
#define TRIG_DEBOUNCE 1  // some modules output a very short trigger
#define PARAMETERUPDATE 100  // modal doesn't like values that jump around a lot so limit the changes

// sample the pots. potlock means apply hysteresis so we only change when the pot is moved significantly
void samplepots(void) {
  for (int i=0; i<NUMPOTS;++i) {
    uint val=0;
    digitalWrite(MUXCTL,!(i&1)); // set analog mux to correct pot
    for (int j=0; j<POT_AVERAGING;++j) val+=analogRead(POT1_2-((i&2)>>1)); // read the A/D a few times and average for a more stable value
    val=val/POT_AVERAGING;
    if (potlock[i]) {
      int delta=pot[i]-val;  // this needs to be done outside of the abs() function - see arduino abs() docs
      if (abs(delta) > MIN_POT_CHANGE) {
        potlock[i]=0;   // flag pot no longer locked
        pot[i]=val; // save the new reading
      }
    }
    else pot[i]=val; // pot is unlocked so save the reading
  }
}

// lock all pots. when locked they have to move significantly to change value. 
// prevents immediately setting new values when parameter set is changed with front panel button
void lockpots(void) {
  for (int i=0; i<NUMPOTS;++i) potlock[i]=1;
}

// sample the CV1 input. 
uint16_t sampleCV1(void) {
  uint16_t val=0;
  for (int16_t j=0; j<CV_AVERAGING;++j) val+=analogRead(CV1IN); // read the A/D a few times and average for a more stable value
  val=val/CV_AVERAGING;
  return val;
}

// sample the CV2 input. 
uint16_t sampleCV2(void) {
  uint16_t val=0;
  for (int16_t j=0; j<CV_AVERAGING;++j) val+=analogRead(CV2IN); // read the A/D a few times and average for a more stable value
  val=val/CV_AVERAGING;
  return val;
}

// like the map() function but maps to float values
float mapf(long x, long in_min, long in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() { 
  Serial.begin(115200);

#ifdef DEBUG

  Serial.println("starting setup");  
#endif

// set up I/O pins
  pinMode(TRIGGER,INPUT_PULLUP); // gate/trigger in
  pinMode(BUTTON1,INPUT_PULLUP); // button in
  pinMode(MUXCTL,OUTPUT);  // analog switch mux

#ifdef MONITOR_CPU1 // for monitoring 2nd core CPU usage
  pinMode(CPU_USE,OUTPUT); // hi = CPU busy
#endif 

  voice.Init(samplerate);       // initialize the voice object
  voice.SetFreq(150);

  analogReadResolution(AD_BITS); // set up for max resolution
// initialize the pot readings
  for (int16_t i=0; i<NUMPOTS;++i) {
    pot[i]=0;
    potlock[i]=0;
  }

  LEDS.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  LEDS.setPixelColor(0, RED); 
  LEDS.show();
    
// set up Pico I2S for PT8211 stereo DAC
	DAC.setBCLK(BCLK);
	DAC.setDATA(I2S_DATA);
	DAC.setBitsPerSample(16);
	DAC.setBuffers(1, 128, 0); // DMA buffer - 32 bit L/R words
	DAC.setLSBJFormat();  // needed for PT8211 which has funny timing
	DAC.begin(SAMPLERATE);

#ifdef DEBUG  
  Serial.println("finished setup");  
#endif
}



void loop() {

  if (!digitalRead(BUTTON1)) {
    if (((millis()-buttontimer) > DEBOUNCE) && !button) {  // if button pressed advance to next parameter set
      button=1;  
      ++UIstate;
      if (UIstate >= NUMUISTATES) UIstate=SET1;
      lockpots();
    }
  }
  else {
    buttontimer=millis();
    button=0;
  }

  if ((millis() -parameterupdate) > PARAMETERUPDATE) {  // don't update the parameters too often -it messes up the daisySP model
    parameterupdate=millis();
    samplepots();

  // set parameters from panel pots
  // 
    switch (UIstate) {
      case SET1:
        LEDS.setPixelColor(0, RED); 
        if (!potlock[0]) voice.SetStructure(mapf(pot[0],0,AD_RANGE-1,0,1));// top pot on the panel
        if (!potlock[1]) voice.SetBrightness(mapf(pot[1],0,AD_RANGE-1,0,1));  //
        if (!potlock[2]) voice.SetDamping(mapf(pot[2],0,AD_RANGE-1,0,1));
        if (!potlock[3]) minfreq=(mapf(pot[3],0,AD_RANGE-1,10,100)); // 
        break;
      case SET2:
        LEDS.setPixelColor(0, GREEN);       
        if (!potlock[0]) voice.SetAccent(mapf(pot[0],0,AD_RANGE-1,0,1));
        if (!potlock[1]) voice.SetSustain((bool)map(pot[1],0,AD_RANGE-1,0,2)); // sustain = pot over half way
        break;
      default:
        break;
    }
  }

  float cv=(AD_RANGE-sampleCV2()); // CV in is inverted

  voice.SetFreq(pow(2,(cv/CVIN_VOLT))*minfreq); // ~ 7 octave range

  if (!digitalRead(TRIGGER)) {
    if (((millis()-trigtimer) > TRIG_DEBOUNCE) && !trigger) {  // if button pressed advance to next parameter set
      trigger=1;  
      voice.Trig();
    }
  }
  else {
    trigtimer=millis();
    trigger=0;
  }
  LEDS.show();  // update LED

}

// second core setup
// second core is dedicated to sample processing
void setup1() {
delay (1000); // wait for main core to start up peripherals
}

// process audio samples
void loop1(){

static  float sig;
static  int32_t outsample;


  sig=voice.Process();
  outsample = (int32_t)(sig*MULT_16)>>16; // scale output 

#ifdef MONITOR_CPU1  
  digitalWrite(CPU_USE,0); // low - core1 not busy
#endif
 // write samples to DMA buffer - this is a blocking call so it stalls when buffer is full
	DAC.write(int16_t(outsample)); // left
	DAC.write(int16_t(outsample)); // right

#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE,1); // hi = core1 busy
#endif
}





