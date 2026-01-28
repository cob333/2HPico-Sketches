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
Reverb for 2HPico module using DaisySP ReverbSc
RH Dec 2025

Jack 1 - Audio in
Jack 2 - Mix CV in
Jack 3 - Audio out

Pot 1 - Decay
Pot 2 - Damp
Pot 3 - Predelay
Pot 4 - Mix
*/

#include <2HPico.h>
#include <I2S.h>
#include <Adafruit_NeoPixel.h>
#include "pico/multicore.h"

#define DEBUG   // comment out to remove debug code
#define MONITOR_CPU1  // define to enable 2nd core monitoring on CPU_USE pin

// Reverb is CPU heavy, use a conservative sample rate for RP2350.
#define SAMPLERATE 22050

Adafruit_NeoPixel LEDS(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

I2S DAC(OUTPUT);

#include "reverbsc.h"
using namespace daisysp;

float samplerate = SAMPLERATE;

ReverbSc reverb;

#define PREDELAY_MAX_MS 250
#define PREDELAY_MAX_SAMPLES ((SAMPLERATE * PREDELAY_MAX_MS) / 1000 + 1)

static int16_t predelay_buf[PREDELAY_MAX_SAMPLES];
static uint32_t predelay_write = 0;

static volatile float mix_amt = 0.3f;
static volatile float fb_amt = 0.85f;
static volatile float lp_freq = 8000.0f;
static volatile uint32_t predelay_samples = 0;

bool button = 0;
#define NUMUISTATES 1
enum UIstates {SET1};
uint8_t UIstate = SET1;
uint32_t buttontimer, parameterupdate;

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static const float kAdcToFloat = 2.0f / (float)AD_RANGE;
static const float kS16ToFloat = 1.0f / 32768.0f;
static const float kFloatToS16 = 32767.0f;

void setup() {
  Serial.begin(115200);

#ifdef DEBUG
  Serial.println("starting setup");
#endif

#ifdef MONITOR_CPU1
  pinMode(CPU_USE, OUTPUT);
#endif

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(MUXCTL, OUTPUT);
  pinMode(CV1IN, INPUT);
  pinMode(CV2IN, INPUT);

  analogReadResolution(AD_BITS);

  LEDS.begin();
  LEDS.setPixelColor(0, BLUE);
  LEDS.show();

  reverb.Init(samplerate);

  DAC.setBCLK(BCLK);
  DAC.setDATA(I2S_DATA);
  DAC.setBitsPerSample(16);
  DAC.setBuffers(1, 128, 0);
  DAC.setLSBJFormat();
  DAC.begin(SAMPLERATE);

#ifdef DEBUG
  Serial.println("finished setup");
#endif
}

void loop() {
  if (!digitalRead(BUTTON1)) {
    if (((millis() - buttontimer) > DEBOUNCE) && !button) {
      button = 1;
      ++UIstate;
      if (UIstate >= NUMUISTATES) UIstate = SET1;
      lockpots();
    }
  } else {
    buttontimer = millis();
    button = 0;
  }

  if ((millis() - parameterupdate) > PARAMETERUPDATE) {
    parameterupdate = millis();
    samplepots();

    switch (UIstate) {
      case SET1:
        LEDS.setPixelColor(0, BLUE);
        if (!potlock[0]) {
          fb_amt = mapf(pot[0], 0, AD_RANGE - 1, 0.2f, 0.98f);
          float fb = fb_amt;
          reverb.SetFeedback(fb);
        }
        if (!potlock[1]) {
          lp_freq = mapf(pot[1], 0, AD_RANGE - 1, 200.0f, samplerate * 0.45f);
          float lp = lp_freq;
          reverb.SetLpFreq(lp);
        }
        if (!potlock[2]) {
          float pd_ms = mapf(pot[2], 0, AD_RANGE - 1, 0.0f, (float)PREDELAY_MAX_MS);
          uint32_t pd_samps = (uint32_t)(pd_ms * samplerate / 1000.0f);
          if (pd_samps >= PREDELAY_MAX_SAMPLES) pd_samps = PREDELAY_MAX_SAMPLES - 1;
          predelay_samples = pd_samps;
        }
        {
          float mix_pot = mapf(pot[3], 0, AD_RANGE - 1, 0.0f, 1.0f);
          float mix_cv = (float)(AD_RANGE - sampleCV2()) / (float)AD_RANGE;
          mix_amt = clampf(mix_pot + mix_cv, 0.0f, 1.0f);
        }
        break;
      default:
        break;
    }
    LEDS.show();
  }
}

// second core setup
void setup1() {
  delay(1000);
}

// process audio samples
void loop1() {
  static float wet_l, wet_r;
  static float in, wet, out;
  static int32_t outsample;

  int adc = analogRead(CV1IN);
  adc = AD_RANGE - adc;
  in = ((float)adc * kAdcToFloat) - 1.0f;
  float in_clamped = clampf(in, -1.0f, 1.0f);
  int16_t in_s16 = (int16_t)(in_clamped * kFloatToS16);

  float predelay_in = in;
  uint32_t delay = predelay_samples;
  if (delay > 0) {
    uint32_t read = predelay_write + PREDELAY_MAX_SAMPLES - delay;
    if (read >= PREDELAY_MAX_SAMPLES) read -= PREDELAY_MAX_SAMPLES;
    predelay_in = (float)predelay_buf[read] * kS16ToFloat;
  }
  predelay_buf[predelay_write] = in_s16;
  if (++predelay_write >= PREDELAY_MAX_SAMPLES) predelay_write = 0;

  reverb.Process(predelay_in, predelay_in, &wet_l, &wet_r);
  wet = 0.5f * (wet_l + wet_r);

  float mix = mix_amt;
  out = (1.0f - mix) * in + mix * wet;
  outsample = (int32_t)(out * MULT_16) >> 16;

#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE, 0);
#endif
  DAC.write(int16_t(outsample));
  DAC.write(int16_t(outsample));
#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE, 1);
#endif
}
