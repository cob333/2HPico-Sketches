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
/*
R Heslip
// simple bass drum voice synth 
// RH for new 2HP hardware Nov 2025

Top Jack - trigger input 

Middle jack - FM input

Bottom Jack - output

Top pot - Accent

Second pot - Tone

Third pot - Decay

Fourth pot - Frequency

*/



#include <2HPico.h>
#include <I2S.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

#include "pico/multicore.h"

#define DEBUG   // comment out to remove debug code
#define MONITOR_CPU1  // define to enable 2nd core monitoring

//#define SAMPLERATE 11025 
#define SAMPLERATE 22050  // saves CPU cycles
//#define SAMPLERATE 44100

Adafruit_NeoPixel LEDS(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

I2S DAC(OUTPUT);  // 

#include "daisysp.h"
using namespace daisysp;

// including the source files is a pain but that way you compile in only the modules you need
// DaisySP statically allocates memory and some modules e.g. reverb use a lot of ram
#include "drums/analogbassdrum.cpp"
#include "filters/svf.cpp"

float samplerate=SAMPLERATE;  // for DaisySP

// create daisySP processing objects
AnalogBassDrum   bassdrum;  


// a/d values from pots
// pots are used for two or more parameters so we don't change the values till
// there is a significant movement of the pots when the pots are "locked"
// this prevents a waveform or level change ("shift" parameters) from changing the ramp times when the shift button is released

#define CV_VOLT 580.6  // a/d counts per volt - calibrate this value for accurate V/octave

float minfreq=10;

bool trigger=0;
bool button=0;
#define NUMUISTATES 1
enum UIstates {SET1} ;
uint8_t UIstate=SET1;
uint32_t buttontimer,trigtimer,parameterupdate;

void setup() { 
  Serial.begin(115200);

#ifdef DEBUG
  Serial.println("starting setup");  
#endif

// set up I/O pins
 
#ifdef MONITOR_CPU1 // for monitoring 2nd core CPU usage
  pinMode(CPU_USE,OUTPUT); // hi = CPU busy
#endif 

  pinMode(TRIGGER,INPUT_PULLUP); // gate/trigger in
  pinMode(BUTTON1,INPUT_PULLUP); // button in
  pinMode(MUXCTL,OUTPUT);  // analog switch mux

  LEDS.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  LEDS.setPixelColor(0, RED); 
  LEDS.show();

  analogReadResolution(AD_BITS); // set up for max resolution

  bassdrum.Init(samplerate);

  // Enable the AudioShield
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
      Serial.printf("state %d %d\n",UIstate, sizeof(UIstates)); 
    }
  }
  else {
    buttontimer=millis();
    button=0;
  }

  if ((millis() -parameterupdate) > PARAMETERUPDATE) {  // don't update the parameters too often -sometimes it messes up the daisySP models
    parameterupdate=millis();
    samplepots();

// set parameters from panel pots
// 
    switch (UIstate) {
      case SET1:
        LEDS.setPixelColor(0, RED);  
        if (!potlock[0]) bassdrum.SetAccent(mapf(pot[0],0,AD_RANGE,0,1));  // very subtle  - 
        // if (!potlock[0]) bassdrum.SetDecay(mapf(pot[0],0,AD_RANGE,0,2000));  // the SetDecay() function in the drum model doesn't do anything 
        if (!potlock[1]) bassdrum.SetTone(mapf(pot[1],0,AD_RANGE,0,1));
        if (!potlock[2]) bassdrum.SetDecay(mapf(pot[2],0,AD_RANGE,0,1));
        if (!potlock[3]) minfreq=(mapf(pot[3],0,AD_RANGE-1,10,120)); // 
        break;
      default:
        break;
    }
  }

  float cv=(AD_RANGE-sampleCV2()); // CV in is inverted

  bassdrum.SetFreq(pow(2,(cv/AD_RANGE))*minfreq); // ~ 3.5 octave FM range

  if (!digitalRead(TRIGGER)) {
    if (((millis()-trigtimer) > TRIG_DEBOUNCE) && !trigger) {  // 
      trigger=1;  
      bassdrum.Trig();
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


  sig=bassdrum.Process();
  outsample = (int32_t)(sig*MULT_16)>>14; // scale output up 4x - seems very low

#ifdef MONITOR_CPU1  
  digitalWrite(CPU_USE,0); // low - CPU not busy
#endif
 // write samples to DMA buffer - this is a blocking call so it stalls when buffer is full
	DAC.write(int16_t(outsample)); // left
	DAC.write(int16_t(outsample)); // right

#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE,1); // hi = CPU busy
#endif
}





