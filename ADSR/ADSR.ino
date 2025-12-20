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
// ADSR for 2HPico module
// uses DaisySP ADSR 
// RH Nov 2025

#include <2HPico.h>
#include <I2S.h>
#include <Adafruit_NeoPixel.h>
#include "pico/multicore.h"

#define DEBUG   // comment out to remove debug code

#define MONITOR_CPU1  // define to enable 2nd core monitoring on CPU_USE pin

//#define SAMPLERATE 11025 
#define SAMPLERATE 22050 // 
//#define SAMPLERATE 44100


Adafruit_NeoPixel LEDS(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

I2S DAC(OUTPUT);  // using once channel of a PT8211 stereo DAC

#include "daisysp.h"

// including the DaisySP source files is a pain but that way you compile in only the modules you need
// DaisySP statically allocates memory and some modules e.g. reverb use a lot of ram so don't compile them unless you need them
#include "control/adsr.cpp"

float samplerate=SAMPLERATE; // foar DaisySP

#define GATE TRIGGER    // semantics - ADSR is generally used with a gate signal
// create daisySP processing objects
Adsr env;

#define CVIN_VOLT 580.6  // a/d count per volt - **** adjust this value to calibrate V/octave input
#define CVOUT_VOLT 6554 // D/A count per volt - nominally +-5v range for -+32767 DAC values- ***** adjust this value to calibrate V/octave out
#define CVOUTMIN -2*CVOUT_VOLT  // lowest output CV ie MIDI note 0

bool gate=0;
bool button=0;
#define NUMUISTATES 1  // only 1 parameter page in this app
enum UIstates {SET1} ;
uint8_t UIstate=SET1;
uint32_t buttontimer,gatetimer,parameterupdate;

void setup() { 
  Serial.begin(115200);

#ifdef DEBUG

  Serial.println("starting setup");  
#endif

  pinMode(TRIGGER,INPUT_PULLUP); // gate/trigger in
  pinMode(BUTTON1,INPUT_PULLUP); // button in
  pinMode(MUXCTL,OUTPUT);  // analog switch mux

#ifdef MONITOR_CPU1 // for monitoring 2nd core CPU usage
  pinMode(CPU_USE,OUTPUT); // hi = CPU busy
#endif 

  env.Init(samplerate);       // initialize the envelope generator

  analogReadResolution(AD_BITS); // set up for max resolution

  LEDS.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  LEDS.setPixelColor(0, RED); 
  LEDS.show();

//  env.SetCurve(-15.0f); // only for AR env
    
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

  if ((millis() -parameterupdate) > PARAMETERUPDATE) {  // don't update the parameters too often -sometimes it messes up the daisySP models
    parameterupdate=millis();
    samplepots();

  // set parameters from panel pots
  // 
    switch (UIstate) {
      case SET1:
        LEDS.setPixelColor(0, RED); 
        if (!potlock[0]) env.SetTime(ADSR_SEG_ATTACK, mapf(pot[0],0,AD_RANGE-1,0,2)); // up to 2 seconds per segment
        if (!potlock[1]) env.SetTime(ADSR_SEG_DECAY, mapf(pot[1],0,AD_RANGE-1,0,2));
        if (!potlock[2]) env.SetSustainLevel(mapf(pot[2],0,AD_RANGE-1,0,1));
        if (!potlock[3]) env.SetTime(ADSR_SEG_RELEASE, mapf(pot[3],0,AD_RANGE-1,0,2));      
        break;
      default:
        break;
    }
  }

  if (!digitalRead(BUTTON1)) gate=1;  // **** kind of hacky - only 1 page of parameters so we can use the button as a manual gate 
  else {
    if (!digitalRead(GATE)) {  // if gate input is active, tell core 1 to process ADSR
      if (((millis()-gatetimer) > GATE_DEBOUNCE) && !gate) {  
        gate=1;  
      }
    }
    else {
      gatetimer=millis();
      gate=0;
    }
  }
  LEDS.show();  // update LED
//  delay(100);
}

// second core setup
// second core is dedicated to sample processing
void setup1() {
delay (1000); // wait for main core to start up peripherals
}

// process audio samples
void loop1(){

static  float envelope;
static  int32_t outsample;

envelope=-env.Process(gate);  // hardware DAC output is inverted so invert it here for +ve output voltage
outsample = (int32_t)(envelope*MULT_16)>>16; // scale output

#ifdef MONITOR_CPU1  
  digitalWrite(CPU_USE,0); // low = core1 stalled because I2S buffer is full
#endif
 // write samples to DMA buffer - this is a blocking call so it stalls when buffer is full
	DAC.write(int16_t(outsample)); // left
	DAC.write(int16_t(outsample)); // right

#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE,1); // hi = core1 busy
#endif
}





