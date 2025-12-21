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
Dec 2025 LFO for 2HPico based on my Quad LFO from Oct 2018

Implemented like a cycling envelope generator - change the rise and fall times vs setting frequency directly
This allows for a wide variety of waveshapes and duty cycles
Rise and Fall controls are exponential which gives a very wide frequency range approx 50hz to around 30 minutes per cycle
LED color changes to indicate the waveform. 
LED brightness changes with the waveform level. Not affected by level setting - otherwise you wouldn't see anything at low output levels

Top Jack - sync input - resets to rise phase on +v clock edge. Sine and Ramp reset to 0v which seems more correct to me

Middle jack - bipolar LFO output -5v to +5v range (provided you have the board strapped for DAC out on jack 2)

Bottom Jack - bipolar LFO output -5v to +5v range

Top pot - Rise time

Second pot - Fall Time

Third pot - Waveform - full ccw triangle/ramp (Red), sine (Orange), exponential (Green), reverse exponential (Blue), random amplitude pulse (Aqua), pulse (Violet)

Fourth pot - Output level

*/
#include <2HPico.h>
#include "lut.h"
#include <I2S.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG   // comment out to remove debug code

#define MONITOR_CPU1  // define to enable 2nd core monitoring on CPU_USE pin

//#define SAMPLERATE 22050
#define SAMPLERATE 44100  // note very compute intensive so we can do 44khz with lots of core 1 cycles to spare

// *** communication between cores ****
// if core 0 writes a variable and core 1 reads it (or vice versa) that will work OK because RPxxxx writes are atomic
// if BOTH cores are writing the same variable you must use some form of synchronization between the cores
// the RPXXXX processors have a FIFO for this purpose so thats what we use here
// Core 1 in this app writes to lfo.acc constantly but to sync the LFO we have to set lfo.acc and lfo.phase to 0
// this is done by sending a sync command from core 0 to core 1 via the FIFO

#define SYNC TRIGGER // semantics - trigger input is used for LFO sync
#define SYNC_COMMAND 0   // command to tell core 1 to reset waveform

I2S DAC(OUTPUT);  // using stereo DAC

#define NUM_LFOS 1  // you can easily extend this sketch to do more LFOs but managing the LED colors gets tricky for more than 1
#define NWAVES 6  // number of different waveforms

enum {RAMP, SINE, EXPO, QUARTIC, RANDOM1, PULSE}; // wave shape indexes
int32_t wavelut[NWAVES]={RED,ORANGE,GREEN,BLUE,AQUA,VIOLET}; // maps waveforms to different colors

Adafruit_NeoPixel LEDS(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

// maximum/minimum increments for the DDS which set the fastest/slowest LFO rates
#define RANGE 6  // exponential map 10***0 to 10**5. max ~50hz at 44khz sampling
#define MIN_DELTA 300 // minimum value for DDS adder - avoids rediculously slow ramps - this gives a few minutes at 44.1khz
#define MAX_AMPLITUDE AD_RANGE  // for scaling the output

struct lfodata {
  byte wave; // waveform
  int32_t rate1; // rate for first section of waveform
  int32_t rate2; // rate for second section of waveform
  uint16_t amplitude; // output level 0-32k
  int32_t acc; // bottom 16 are accumulator for DDS algorithm, top 16 used for waveform generation
  bool phase; // flags first or second section of waveform
  int16_t dacout; // current DAC value
  int16_t scaledout; // scaled DAC output
  }lfo[NUM_LFOS];

bool button=0;
bool sync=0;
#define NUMUISTATES 1  // only 1 parameter page in this app
enum UIstates {SET1} ;
uint8_t UIstate=SET1;
uint32_t buttontimer,parameterupdate,synctimer;


void setup() { 
  Serial.begin(115200);

#ifdef DEBUG

  Serial.println("starting setup");  
#endif

  pinMode(SYNC,INPUT_PULLUP); // sync in
  pinMode(BUTTON1,INPUT_PULLUP); // button in
  pinMode(MUXCTL,OUTPUT);  // analog switch mux

#ifdef MONITOR_CPU1 // for monitoring 2nd core CPU usage
  pinMode(CPU_USE,OUTPUT); // hi = CPU busy
#endif 

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


// in the main loop we sample the pot values, check the button, update the LFO parameters and update the LEDs

void loop() {

// button actually not used in this sketch. could easily be extended to 2 LFOs with a 2nd set of parameters tho
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
    samplepots();  // continuous update of parameters is OK for this app

  // set parameters from panel pots
  // 
    switch (UIstate) {
      case SET1:
        LEDS.setPixelColor(0, RED); 
        if (!potlock[0]) lfo[0].rate1=(long)(pow(10,mapf(pot[0],0,AD_RANGE-1,RANGE,0)))+MIN_DELTA; // exponential ramp rates for a very large time range
        if (!potlock[1]) lfo[0].rate2=(long)(pow(10,mapf(pot[1],0,AD_RANGE-1,RANGE,0)))+MIN_DELTA; //
        if (!potlock[2]) lfo[0].wave=map(pot[2],0,AD_RANGE,0,NWAVES);
        if (!potlock[3]) lfo[0].amplitude=map(pot[3],0,AD_RANGE,0,MAX_AMPLITUDE);       
        break;
      default:
        break;
    }
  }

  if (!digitalRead(SYNC)) {  // if sync input is active tell core 1 to reset phase
    if (((millis()-synctimer) > CLOCK_DEBOUNCE) && !sync) {  
      sync=1;  
      rp2040.fifo.push(SYNC_COMMAND); // send sync command to 2nd core
    }
  }
  else {
    synctimer=millis();
    sync=0;
  }

 // there is a glitch someplace - LEDs turn off at peak of SINE and RAMP waveform - interpolation code bug at phase change?
 // for SK6812 RGB leds we can only use 5 bits or its just too bright

  int32_t level;
  int32_t color1;
  level=-(lfo[0].dacout+32768); // convert to 0-64k - needs sign swap for LEDs because output buffer is inverting
  level=level>>11 & 0x1f; // scale to 5 bits otherwise LED is too bright
  color1=wavelut[lfo[0].wave]/31;  // set color for this wave scaled down by 5 bits
  color1=color1*level; // scale back up again by the output level
  LEDS.setPixelColor(0, color1); // scale 16 bit DAC value to 5 bits for LEDS

  LEDS.show();   // Send the updated pixel colors to the hardware.

}

// second core setup
// second core dedicated to sample processing
void setup1() {
delay (1000); // wait for main core to start up perhipherals
}

// second core calculates samples and sends to DAC
// I suspect the interpolation code is not quite right but it works

void loop1() {
  bool phasechange; // indicates ramp has changed direction
  unsigned a,x,y,delta,out; 
  unsigned char i;

  while (rp2040.fifo.available()) { // only command in this sketch is sync
    rp2040.fifo.pop(); //
    if ((lfo[i].wave == SINE) || (lfo[i].wave == RAMP)) { // these two need a phase offset so they reset to 0v
      lfo[0].acc=0x1fffffff; 
      lfo[0].phase=1;
    }
    else {   // when reset via sync the expo and quartic waveforms go to +5 and the pulse goes to -5V. not sure what is best
      lfo[0].acc=0;
      lfo[0].phase=0;
    }
  }

  for (i=0; i<NUM_LFOS; ++i) {
    phasechange=0;  
    
    if (lfo[i].phase == 0) lfo[i].acc+=lfo[i].rate1; // ramp up for first part
        else lfo[i].acc-=lfo[i].rate2; // ramp down for second part

    if (lfo[i].acc >= 0x3fffffff) {  // test for accumulator overflow
      lfo[i].phase = 1 ;// ramp down once we hit the top
      lfo[i].acc=0x3fffffff; // set to max for next DAC output
      phasechange=1;
    }
    if (lfo[i].acc <=0) { // test for accumulator underflow
      lfo[i].phase = 0; //ramp up when we hit bottom
      lfo[i].acc=0; // set to 0 min for start of up ramp
      phasechange=1;
    }

    switch (lfo[i].wave) {
      case RAMP:
        lfo[i].dacout=(lfo[i].acc >> 14)-32768; // get top 16 bits of accumulator. ramp is 0-65535 but DAC needs signed
        break;    
      case PULSE: // variable duty cycle pulse
        if (lfo[i].phase) lfo[i].dacout=32767;
        else lfo[i].dacout=-32767;
        break;
      case RANDOM1:  // random value every time we hit max or min ramp
        if (phasechange) lfo[i].dacout=random(65535);
        break;
      // table lookup for exponential and quartic
      case EXPO:
        a=lfo[i].acc >> 14; // our 16 bit ramp value 
        if (lfo[i].phase) a=65535-a; // reverse the lookup for 2nd half
        x=(a>>8)&0xff; // 256 word lookup table - it has 16 bit values
          // note that x+1 will index off the end of the table so I made the table 257 long
        delta=(lut_env_expo[x+1]-lut_env_expo[x])>>4; // interpolate between the table values - 16 steps
        y=lut_env_expo[x]; // lookup base value from table
        out=(y+(a&0xf)*delta);
        if (lfo[i].phase) out=65535-out; // invert the value for 2nd half
        lfo[i].dacout=out -32768; // DAC uses signed values
        break; 
      case QUARTIC:
        a=lfo[i].acc >> 14; // our 16 bit ramp value 
        if (lfo[i].phase) a=65535-a; // reverse the lookup for 2nd half
        x=(a>>8)&0xff; // 256 word lookup table - it has 16 bit values
          // note that x+1 will index off the end of the table so I made the table 257 long
        delta=(lut_env_quartic[x+1]-lut_env_quartic[x])>>4; // interpolate between the table values - 16 steps
        y=lut_env_quartic[x]; // lookup base value from table
        out=(y+(a&0xf)*delta); // interpolated output = base value + steps*delta. scale the 16 bit table value to 12 bits
        if (lfo[i].phase) out=65535-out; // invert the value for 2nd half
        lfo[i].dacout=out-32768; 
        break;         
      default:  // in case there are bogus values in EEPROM

      case SINE:
        a=lfo[i].acc >> 14; // our 16 bit ramp value range 0-65535
        if (lfo[i].phase) a=65535-a; // reverse the lookup for 2nd half
        x=(a>>8)&0xff; // 256 word lookup table - it has 16 bit values
          // note that x+1 will index off the end of the table so I made the table 257 long
        delta=(lut_raised_cosine[x+1]-lut_raised_cosine[x])>>4; // interpolate between the table values - 16 steps
        y=lut_raised_cosine[x]; // lookup base value from table
        out=(y+(a&0xf)*delta); // interpolated output = base value + steps*delta. 
        if (lfo[i].phase) out=65535-out; // invert the value for 2nd half
        lfo[i].dacout=out-32768;      // DAC uses signed values
       // the trig version - pot phase is wrong with this code 
       // double rads=PI/2+PI*(double(lfo[i].acc)/double(0xfffffff)); // convert accumulator value to radians
       // PI/2 shift above gives us a tilted sinewave when rate1 and rate2 are different
      //  lfo[i].dacout=2048+2047*(sin(rads)); // scale and offset the DAC output
        break;
    }

  // scale the output
    int32_t scaledout=((int32_t)lfo[i].dacout*(int32_t)lfo[i].amplitude)/AD_RANGE; // use long int math here or it will overflow
    lfo[i].scaledout=scaledout; //  

  }

#ifdef MONITOR_CPU1  
  digitalWrite(CPU_USE,0); // low = core1 stalled because I2S buffer is full
#endif
   // send the output value to the appropriate DAC - will stall on buffer full
  DAC.write(lfo[0].scaledout); // an interesting option would be to have one channel uipolar and one bipolar, or opposite phases, quadrature phases
  DAC.write(lfo[0].scaledout); // 
#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE,1); // hi = core1 busy
#endif
}






