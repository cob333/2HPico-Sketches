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
// 8 step sequencer for 2HP hardware Dec 2025
// module must be jumpered for CV out on jack 2 which is used as Gate output
// uses Adafruit NeoPixel library
// V1.0 RH Dec 2025

// top jack - trigger input
// middle jack - gate output
// bottom jack - pitch CV output

// page 1 parameters - Red LED
// Pot 1 - step 1 pitch
// pot 2 - step 2 pitch
// pot 3 - step 3 pitch
// pot 4 - step 4 pitch

// page 2 parameters - Orange LED
// Pot 1 - step 5 pitch
// pot 2 - step 6 pitch
// pot 3 - step 7 pitch
// pot 4 - step 8 pitch

// page 3 parameters GREEN LED
// Pot 1 - scale
// Pot 2 - clock divider
// Pot 3 - number of steps
// Pot 4 - overall pitch

//#include "MIDI.h"
#include <2HPico.h>
#include <I2S.h>
#include <Adafruit_NeoPixel.h>
//#include "RPi_Pico_TimerInterrupt.h"
#include <math.h>

#include "pico/multicore.h"

#define DEBUG   // comment out to remove debug code
#define MONITOR_CPU1  // define to enable 2nd core monitoring

//#define SAMPLERATE 11025 
#define SAMPLERATE 22050  // saves CPU cycles
//#define SAMPLERATE 44100

Adafruit_NeoPixel LEDS(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

I2S DAC(OUTPUT);  // 


// sequencer stuff
#define MAX_STEPS 8
#define MAX_SCALES 4   // there are more scales than this but works best if you only use a few
#define NOTERANGE 36  // 3 octave range seems reasonable
#define CLOCKIN TRIGGER  // top jack is clock


#define CVIN_VOLT 580.6  // a/d count per volt - **** adjust this value to calibrate V/octave input
#define CVOUT_VOLT 6554 // D/A count per volt - nominally +-5v range for -+32767 DAC values- ***** adjust this value to calibrate V/octave out
#define CVOUTMIN -2*CVOUT_VOLT  // lowest output CV ie MIDI note 0

int8_t notes[MAX_STEPS]={0,0,0,0,0,0,0,0};
int16_t gateout=GATELOW;      // sent to right dac channel
int16_t cvout=CVOUTMIN;  // sent to left DAC channel
int16_t cvoffset=12000; // base pitch set by UI
int8_t stepindex=0;
int8_t laststep=7;
int16_t scale=0;

int8_t clockdivideby=1;  // divide input clock by this
int8_t clockdivider=1;   // counts down clocks

bool clocked=0;  // keeps track of clock state
bool button=0;  // keeps track of button state

#define NUMUISTATES 3
enum UIstates {SET1,SET2,SET3} ;
uint8_t UIstate=SET1;
uint32_t buttontimer,clocktimer,clockperiod,clockdebouncetimer,ledtimer, gatetimer, gatelength;

#define LEDOFF 100 // LED trigger flash time 



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
  clocktimer=millis(); // initial clock measurement
}


void loop() {
// ******* still need to implement reset sequence on hold
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

  samplepots();

// set parameters from panel pots
// 
  switch (UIstate) {
    case SET1:
      LEDS.setPixelColor(0, RED);  // set sequencer step values from pot settings
      if (!potlock[0]) notes[0]=map(pot[0],0,AD_RANGE-1,-1,NOTERANGE); // top pot on the panel
      if (!potlock[1]) notes[1]=map(pot[1],0,AD_RANGE-1,-1,NOTERANGE);  // -1 is note off
      if (!potlock[2]) notes[2]=map(pot[2],0,AD_RANGE-1,-1,NOTERANGE); 
      if (!potlock[3]) notes[3]=map(pot[3],0,AD_RANGE-1,-1,NOTERANGE); 
      break;
    case SET2:
      LEDS.setPixelColor(0, ORANGE);
      if (!potlock[0]) notes[4]=map(pot[0],0,AD_RANGE-1,-1,NOTERANGE); // top pot on the panel
      if (!potlock[1]) notes[5]=map(pot[1],0,AD_RANGE-1,-1,NOTERANGE);  // 
      if (!potlock[2]) notes[6]=map(pot[2],0,AD_RANGE-1,-1,NOTERANGE);
      if (!potlock[3]) notes[7]=map(pot[3],0,AD_RANGE-1,-1,NOTERANGE);   
      break;
    case SET3:
      LEDS.setPixelColor(0, GREEN);
      if (!potlock[0]) scale=map(pot[0],0,AD_RANGE,0,MAX_SCALES);  // set scale - too many scales gets hard to discern by ear 
 //     if (!potlock[1]) clockdivideby=map(pot[1],0,AD_RANGE-1,1,5); // set clock divider *** timing is off
      if (!potlock[2]) laststep=map(pot[2],0,AD_RANGE-1,1,MAX_STEPS); // set steps
      if (!potlock[3]) cvoffset=map(pot[3],0,AD_RANGE-1,CVOUTMIN,32767); // sets overall pitch
      break;
    default:
      break;
  }

  if (!digitalRead(CLOCKIN)) {  // look for rising edge of clock input which is inverted
    if (((millis()-clockdebouncetimer) > CLOCK_DEBOUNCE) && !clocked) {  // true if we have a debounced clock rising edge
      --clockdivider;
      clocked=1;
      if (clockdivider <=0) {       
        clockdivider=clockdivideby;
        clockperiod=millis()-clocktimer; // measure clock so we can set gate time relative to clock period
        clocktimer=millis();

        if (notes[stepindex]>=0) {  // negative note value is silent so don't change CV
          cvout=-(quantize(notes[stepindex],scales[scale],0)*(CVOUT_VOLT/12)+CVOUTMIN+cvoffset); // 1v per octave. note numbers are MIDI style 0-127. DAC out is inverted. 
          gateout=GATEHIGH;
          gatelength=clockperiod/2; // could be made adjustable
          gatetimer=millis(); // start gate timer
          LEDS.setPixelColor(0,0 ); // show gate as a LED off flash
          LEDS.show();  // update LED
          ledtimer=millis(); // start led off timer
 //         Serial.printf("scale %s note %s \n",scalenames[scale],notenames[notes[stepindex]%12]);
        }
        else gateout=GATELOW;
        ++stepindex; // advance sequencer  could add sequencer modes - pingpong, reverse etc
        if (stepindex > laststep) stepindex=0;
      }
    }
  }
  else {   
      clocked=0;
      clockdebouncetimer=millis();
  }
//Serial.printf("clkdiv %d len %d \n", clockdivideby,laststep);

  if ((millis()-gatetimer) > gatelength) gateout=GATELOW;  // turn off gate after gate length

  if ((millis()-ledtimer) > LEDOFF ) LEDS.show();  // update LEDs only if not doing off flash
}

// second core setup
// second core is dedicated to sample processing
void setup1() {
delay (1000); // wait for main core to start up peripherals
}

// process audio samples
void loop1(){

#ifdef MONITOR_CPU1  
  digitalWrite(CPU_USE,0); // low - CPU not busy
#endif
 // write values to DMA buffer - this is a blocking call so it stalls when buffer is full
	DAC.write(int16_t(cvout)); // left
	DAC.write(int16_t(gateout)); // right

#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE,1); // hi = CPU busy
#endif
}





