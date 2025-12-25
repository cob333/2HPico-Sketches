
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
Oscillaor example for new 2HP hardware 
R Heslip  Dec 2025

3 oscillators - all have same waveform to keep it simple

Top Jack - FM input 

Middle jack - Volt/Octave input

Bottom Jack - output

First Parameter Page - RED

Top pot - Waveform Sine, Triangle, Saw, Ramp (should be square wave but there is something broken in DaisySP)

Second pot - 1st Oscillator tuning

Third pot - 2nd Oscillator tuning

Fourth pot - 3rd Oscillator tuning

*/

#include "2HPico.h"
#include <I2S.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

#include "pico/multicore.h"

#define DEBUG   // comment out to remove debug code
#define MONITOR_CPU1  // define to enable 2nd core monitoring

#define GATE TRIGGER    // semantics - ADSR is generally used with a gate signal

//#define SAMPLERATE 11025 
//#define SAMPLERATE 22050  // 2HP board antialiasing filters are set up for 22khz
#define SAMPLERATE 44100  // not much DSP needed here so run at higher sample rate

Adafruit_NeoPixel LEDS(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

I2S DAC(OUTPUT);  // using PCM1103 stereo DAC

#include "daisysp.h"

// including the source files is a pain but that way you compile in only the modules you need
// DaisySP statically allocates memory and some modules e.g. reverb use a lot of ram
#include "synthesis/oscillator.cpp"

// we will use only the anitaliased versions of the waveforms
// *** something is wrong here - daisySP generates a ramp instead of a square wave
// my code and DaisySP oscillator code looks correct to me - I can't figure it out
#define NWAVES 4
uint8_t waves[NWAVES]={Oscillator::WAVE_SIN,Oscillator::WAVE_POLYBLEP_TRI,Oscillator::WAVE_POLYBLEP_SAW,Oscillator::WAVE_POLYBLEP_SQUARE};

float samplerate=SAMPLERATE;  // for DaisySP

#define VOICES 1
#define OSCSPERVOICE 3   
#define OSC_MIN_FREQ 20

int waveform=0;
float minfreq[OSCSPERVOICE] ={50,100,200};

// create daisySP processing objects
Oscillator osc[VOICES * OSCSPERVOICE];

#define CV_VOLT 580.6  // a/d counts per volt - trim for V/octave

#define NUMUISTATES 1 // only 1 page in this UI
enum UIstates {OSCS};
uint8_t UIstate=OSCS;

bool button=0;

#define DEBOUNCE 10
uint32_t buttontimer,parameterupdate;

void setup() { 
  Serial.begin(115200);

#ifdef DEBUG

  Serial.println("starting setup");  
#endif

// set up I/O pins
 
#ifdef MONITOR_CPU1 // for monitoring 2nd core CPU usage
  pinMode(CPU_USE,OUTPUT); // hi = CPU busy
#endif 

  pinMode(CV1IN,INPUT); // this app uses top jack as a second CV input
  pinMode(BUTTON1,INPUT_PULLUP); // button in
  pinMode(MUXCTL,OUTPUT);  // analog switch mux

  LEDS.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  LEDS.setPixelColor(0, RED); 
  LEDS.show();

  for (int j=0; j< OSCSPERVOICE; ++j) {
    osc[j].Init(samplerate);       // initialize the voice objects
    osc[j].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);      
    osc[j].SetFreq(minfreq[j]);
  }

  analogReadResolution(AD_BITS); // set up for max resolution

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

// only one page in this app but the code is here in case somebody wants to add pages
  if (!digitalRead(BUTTON1)) {
    if (((millis()-buttontimer) > DEBOUNCE) && !button) {  // if button pressed advance to next parameter set
      button=1;  
      ++UIstate;
      if (UIstate >= NUMUISTATES) UIstate=OSCS;
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

// assign parameters from panel pots
    switch (UIstate) {
        case OSCS:
          LEDS.setPixelColor(0, RED); 
          if (!potlock[0]) {
            uint8_t waveform=(map(pot[0],0,AD_RANGE,0,NWAVES));
            for (int16_t i=0;i< OSCSPERVOICE;++i) osc[i].SetWaveform(waveform);
          }
          if (!potlock[1]) minfreq[0]=(mapf(pot[1],0,AD_RANGE-1,20,160)); // 4 octave tuning range
          if (!potlock[2]) minfreq[1]=(mapf(pot[2],0,AD_RANGE-1,20,160)); // 4 octave range
          if (!potlock[3]) minfreq[2]=(mapf(pot[3],0,AD_RANGE-1,20,160)); // 4 octave range
          break;
        default:
          break;
    }
  }

  float cv=(float)(AD_RANGE-sampleCV2()); // CV inputs are inverted
  float FM=(float)(AD_RANGE-sampleCV1())/AD_RANGE-0.5; // FM range +-0.5 octave

  osc[0].SetFreq(pow(2,(cv/CV_VOLT)+FM)*minfreq[0]); // ~ 7 octave CV range
  osc[1].SetFreq(pow(2,(cv/CV_VOLT)+FM)*minfreq[1]);
  osc[2].SetFreq(pow(2,(cv/CV_VOLT)+FM)*minfreq[2]);
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

  sig=0;
  for (int j=0; j < OSCSPERVOICE; ++j) {
    sig+=osc[j].Process(); // sum oscillators in each voice
  }
  sig=sig*0.68; // scale down to avoid overflow
  outsample = (int32_t)(sig*MULT_16)>>16; // scaling 

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
