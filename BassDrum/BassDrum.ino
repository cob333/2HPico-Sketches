// Copyright 2025 Rich Heslip
//
// Author: Rich Heslip
// Contributor: Wenhao Yang
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
Combined Bass Drum

Top Jack - trigger input
Middle Jack - Accent input
Bottom Jack - output

Top pot - Accent
Second pot - Tone
Third pot - Decay
Fourth pot - Frequency

Button: switch model
RED: 808
ORANGE: 909
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

I2S DAC(OUTPUT);

#include "daisysp.h"
// DaisySP statically allocates memory; include only modules needed.
#include "drums/analogbassdrum.cpp"
#include "drums/synthbassdrum.cpp"
#include "filters/svf.cpp"

#include "BassDrumModels.h"

float samplerate = SAMPLERATE;
static inline float clamp01(float x) { return (x < 0.0f) ? 0.0f : (x > 1.0f ? 1.0f : x); }

// Accent input processing
#define ACCENT_POT_GAIN 2.0f
#define ACCENT_CV_GAIN 2.0f
#define ACCENT_CV_INVERT 1
#define ACCENT_CV_DEADZONE 20
#define ACCENT_FM_MIN 0.0f
#define ACCENT_FM_MAX 1.0f
#define ACCENT_DIRT_MIN 0.0f
#define ACCENT_DIRT_MAX 1.0f

// UI mapping (shared across models)
#define DECAY_MIN_SEC 0.02f
#define DECAY_MAX_SEC 2.0f
#define FREQ_MIN_HZ 10.0f
#define FREQ_MAX_HZ 120.0f

// create daisySP processing objects
BassDrum808 drum808;
BassDrum909 drum909;

enum Model { MODEL_808, MODEL_909 };
volatile Model current_model = MODEL_808;

float baseaccent = 0.15f;
float accent_level = 0.15f;
float param_tone = 0.5f;
float ui_decay = 0.5f;
float ui_freq = 0.5f;
uint16_t cv2_zero = 0;

bool trigger = 0;
bool button = 0;
uint32_t buttontimer, trigtimer, parameterupdate;

static inline float knob_to_decay_time(float knob) {
  knob = clamp01(knob);
  return DECAY_MIN_SEC + knob * (DECAY_MAX_SEC - DECAY_MIN_SEC);
}

static inline float knob_to_freq_hz(float knob) {
  knob = clamp01(knob);
  const float ratio = FREQ_MAX_HZ / FREQ_MIN_HZ;
  return FREQ_MIN_HZ * powf(ratio, knob);
}

static inline float decay_time_to_909_param(float t, float sr) {
  if(t <= 0.0f || sr <= 0.0f) return 0.0f;
  const float exp_term = expf(-1.0f / (t * sr));
  const float a = (0.02f * sr) * (1.0f - exp_term);
  if(a <= 0.0f) return 0.0f;
  float decay_internal = -0.2f * (logf(a) / logf(2.0f));
  decay_internal = clamp01(decay_internal);
  return sqrtf(decay_internal);
}

void setup() {
  Serial.begin(115200);

#ifdef DEBUG
  Serial.println("starting setup");
#endif

#ifdef MONITOR_CPU1
  pinMode(CPU_USE, OUTPUT); // hi = CPU busy
#endif

  pinMode(TRIGGER, INPUT_PULLUP); // gate/trigger in
  pinMode(BUTTON1, INPUT_PULLUP); // button in
  pinMode(MUXCTL, OUTPUT);  // analog switch mux

  LEDS.begin();
  LEDS.setPixelColor(0, RED);
  LEDS.show();

  analogReadResolution(AD_BITS);
  cv2_zero = sampleCV2(); // capture baseline so unpatched CV doesn't dominate

  drum808.Init(samplerate);
  drum909.Init(samplerate);

  const float init_freq = knob_to_freq_hz(ui_freq);
  const float init_decay_time = knob_to_decay_time(ui_decay);
  const float init_decay_909 = decay_time_to_909_param(init_decay_time, samplerate);

  drum808.SetTone(param_tone);
  drum909.SetTone(param_tone);
  drum808.SetDecay(ui_decay);
  drum909.SetDecay(init_decay_909);
  drum808.SetFreq(init_freq);
  drum909.SetFreq(init_freq);
  drum808.SetAccent(accent_level);

  drum909.SetAccent(0.15f);     // keep fixed accent; pot/CV modulates FM + dirtiness
  drum909.SetDirtiness(0.2f);
  drum909.SetFmEnvelopeAmount(0.5f);
  drum909.SetFmEnvelopeDecay(0.3f);

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
    if (((millis() - buttontimer) > DEBOUNCE) && !button) {
      button = 1;
      current_model = (current_model == MODEL_808) ? MODEL_909 : MODEL_808;
      lockpots();
    }
  } else {
    buttontimer = millis();
    button = 0;
  }

  if ((millis() - parameterupdate) > PARAMETERUPDATE) {
    parameterupdate = millis();
    samplepots();

    if (!potlock[0]) baseaccent = clamp01(mapf(pot[0], 0, AD_RANGE, 0, 1) * ACCENT_POT_GAIN);
    if (!potlock[1]) param_tone = mapf(pot[1], 0, AD_RANGE, 0, 1);
    if (!potlock[2]) ui_decay = mapf(pot[2], 0, AD_RANGE, 0, 1);
    if (!potlock[3]) ui_freq = mapf(pot[3], 0, AD_RANGE - 1, 0, 1);

    const float decay_time = knob_to_decay_time(ui_decay);
    const float decay_909 = decay_time_to_909_param(decay_time, samplerate);
    const float freq_hz = knob_to_freq_hz(ui_freq);

    drum808.SetTone(param_tone);
    drum909.SetTone(param_tone);
    drum808.SetDecay(ui_decay);
    drum909.SetDecay(decay_909);
    drum808.SetFreq(freq_hz);
    drum909.SetFreq(freq_hz);
  }

  uint16_t cv_raw = sampleCV2();
  int32_t cv_delta = ACCENT_CV_INVERT
    ? (int32_t)cv2_zero - (int32_t)cv_raw
    : (int32_t)cv_raw - (int32_t)cv2_zero;
  if (cv_delta < 0) cv_delta = 0;
  if (cv_delta < ACCENT_CV_DEADZONE) cv_delta = 0;
  if (cv_delta > AD_RANGE) cv_delta = AD_RANGE;
  float cvaccent = clamp01(mapf(cv_delta, 0, AD_RANGE, 0, 1) * ACCENT_CV_GAIN);

  accent_level = clamp01(baseaccent + (cvaccent * (1.0f - baseaccent)));
  drum808.SetAccent(accent_level);

  float fm_amt = ACCENT_FM_MIN + (ACCENT_FM_MAX - ACCENT_FM_MIN) * accent_level;
  float dirt = ACCENT_DIRT_MIN + (ACCENT_DIRT_MAX - ACCENT_DIRT_MIN) * accent_level;
  drum909.SetFmEnvelopeAmount(fm_amt);
  drum909.SetDirtiness(dirt);

  if (!digitalRead(TRIGGER)) {
    if (((millis() - trigtimer) > TRIG_DEBOUNCE) && !trigger) {
      trigger = 1;
      if (current_model == MODEL_808) {
        drum808.Trig();
      } else {
        drum909.Trig();
      }
    }
  } else {
    trigtimer = millis();
    trigger = 0;
  }

  LEDS.setPixelColor(0, (current_model == MODEL_808) ? RED : ORANGE);
  LEDS.show();
}

// second core setup
void setup1() {
  delay(1000); // wait for main core to start up peripherals
}

// process audio samples
void loop1() {
  static float sig;
  static int32_t outsample;

  Model model = current_model;

  if (model == MODEL_808) {
    sig = drum808.Process();
    outsample = (int32_t)(sig * MULT_16) >> 14; // scale output up 4x - seems very low
  } else {
    sig = drum909.Process();
    sig = std::fmax(-1.0f, std::fmin(1.0f, sig * 0.5f)); // 50% gain, prevent clipping
    outsample = (int32_t)(sig * 32767.0f);
  }

#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE, 0); // low - CPU not busy
#endif
  // write samples to DMA buffer - this is a blocking call so it stalls when buffer is full
  DAC.write(int16_t(outsample)); // left
  DAC.write(int16_t(outsample)); // right

#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE, 1); // hi = CPU busy
#endif
}
