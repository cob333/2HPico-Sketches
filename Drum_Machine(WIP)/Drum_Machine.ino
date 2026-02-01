// Copyright 2026 Rich Heslip
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
// 909-style drum machine for 2HP hardware Feb 2026
//
// Jack 1: Clock input (uses internal clock when absent)
// Jack 2: Clock output (internal or external raw clock)
// Jack 3: Audio output
//
// Mode 1 (RED): Edit
// Mode 2 (GREEN): Realtime

#include <2HPico.h>
#include <I2S.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

#include "pico/multicore.h"

#define DEBUG   // comment out to remove debug code
#define MONITOR_CPU1  // define to enable 2nd core monitoring

//#define SAMPLERATE 11025
#define SAMPLERATE 22050  // saves CPU cycles on RP2350
//#define SAMPLERATE 44100

Adafruit_NeoPixel LEDS(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

I2S DAC(OUTPUT);  //

#include "Drums.h"
using namespace daisysp;

#define MAX_STEPS 16
#define NUM_INSTR 8
#define CLOCKIN TRIGGER

#define CLOCK_TIMEOUT_MS 1000
#define CLOCK_PULSE_MS 5
#define LED_FLASH_MS 50

#define BPM_MIN 20.0f
#define BPM_MAX 200.0f

#define ACCENT_ON 0.95f
#define ACCENT_OFF 0.55f

#define MODE_SWITCH_THRESHOLD (AD_RANGE / 2)

// Instrument indices
enum Instrument
{
  INST_BD = 0,
  INST_SD,
  INST_CLAP,
  INST_RIM,
  INST_TOM,
  INST_CHH,
  INST_OHH,
  INST_CYM,
};

static const uint32_t kInstrumentColors[NUM_INSTR] = {
  RED,     // BassDrum
  ORANGE,  // SnareDrum
  YELLOW,  // HandClap
  GREEN,   // Rimshot
  TIFFANY, // Tom
  AQUA,    // CloseHihat
  BLUE,    // OpenHihat
  VIOLET,  // Cymbal
};

// Drum voices
BassDrum   voice_bd;
SnareDrum  voice_sd;
HandClap   voice_clap;
Rimshot    voice_rim;
Tom        voice_tom;
CloseHihat voice_chh;
OpenHihat  voice_ohh;
Cymbal     voice_cym;

// Sequencer state
uint16_t step_on[NUM_INSTR]  = {0};
uint16_t step_acc[NUM_INSTR] = {0};
volatile bool mute[NUM_INSTR]     = {0};

uint8_t  step_count = 16;
uint8_t  seq_step   = 0;
uint8_t  edit_step  = 0;
uint8_t  edit_inst  = INST_BD;

// UI state
enum UIMode { MODE_EDIT = 0, MODE_REALTIME };
UIMode ui_mode = MODE_EDIT;

bool button = false;
uint32_t buttontimer = 0;
uint32_t buttonpress = 0;
bool swing_editing = false;
bool length_editing = false;

uint32_t parameterupdate = 0;

// Clock state
bool external_clock_present = false;
bool clocked = false;
uint32_t clockdebouncetimer = 0;
uint32_t last_external_ms = 0;
uint32_t last_external_pulse_us = 0;
uint32_t external_period_us = 0;
bool last_external_present = false;

float bpm = 120.0f;
uint32_t step_interval_us = 0;
uint32_t next_internal_pulse_us = 0;

uint8_t swing_percent = 0;  // 0 - 50

// Clock output gate
volatile int16_t gateout = GATELOW;
uint32_t gateoff_ms = 0;

// Playback state
bool playing = false;

// Pending swing trigger
bool pending_step = false;
uint32_t pending_step_time_us = 0;
uint8_t pending_step_index = 0;

// Trigger masks for audio thread
volatile uint16_t trig_mask = 0;
volatile uint16_t acc_mask = 0;

// LED state
uint32_t current_led = 0;
uint32_t led_override_color = 0;
uint32_t led_override_until = 0;
uint32_t led_flash_until = 0;

static inline uint8_t PotToIndex(uint16_t val, uint8_t count)
{
  if (count <= 1) return 0;
  uint32_t idx = (uint32_t)val * count / AD_RANGE;
  if (idx >= count) idx = count - 1;
  return (uint8_t)idx;
}

static inline uint32_t ComputeStepIntervalUs(float bpm_in)
{
  if (bpm_in < BPM_MIN) bpm_in = BPM_MIN;
  if (bpm_in > BPM_MAX) bpm_in = BPM_MAX;
  // 16th note clock: 4 steps per beat
  return (uint32_t)(60000000.0f / (bpm_in * 4.0f));
}

static inline uint32_t ModeColor(UIMode mode)
{
  return (mode == MODE_EDIT) ? RED : GREEN;
}

static inline uint32_t InstrumentColor(uint8_t inst)
{
  if (inst >= NUM_INSTR) return RED;
  return kInstrumentColors[inst];
}

static inline bool StepEnabled(uint8_t inst, uint8_t step)
{
  return (step_on[inst] & (1u << step)) != 0;
}

static inline bool StepAccent(uint8_t inst, uint8_t step)
{
  return (step_acc[inst] & (1u << step)) != 0;
}

static inline void SetStepEnabled(uint8_t inst, uint8_t step, bool on)
{
  if (on)
    step_on[inst] |= (1u << step);
  else
    step_on[inst] &= ~(1u << step);
}

static inline void SetStepAccent(uint8_t inst, uint8_t step, bool on)
{
  if (on)
    step_acc[inst] |= (1u << step);
  else
    step_acc[inst] &= ~(1u << step);
}

static inline void ShowTempColor(uint32_t color, uint32_t duration_ms)
{
  led_override_color = color;
  led_override_until = millis() + duration_ms;
}

static inline void SetLedColor(uint32_t color)
{
  if (color != current_led)
  {
    LEDS.setPixelColor(0, color);
    LEDS.show();
    current_led = color;
  }
}

static void UpdateLed()
{
  uint32_t now = millis();
  if (led_override_until && (int32_t)(now - led_override_until) < 0)
  {
    SetLedColor(led_override_color);
    return;
  }

  if (ui_mode == MODE_EDIT)
  {
    uint32_t color = GREY;
    if (edit_step == 0)
      color = WHITE;
    else if (StepEnabled(edit_inst, edit_step))
      color = InstrumentColor(edit_inst);
    SetLedColor(color);
  }
  else
  {
    if ((int32_t)(now - led_flash_until) < 0)
      SetLedColor(ModeColor(ui_mode));
    else
      SetLedColor(0);
  }
}

static inline void FlashClockLed()
{
  led_flash_until = millis() + LED_FLASH_MS;
}

static inline void TriggerStep(uint8_t step)
{
  if (!playing || ui_mode != MODE_REALTIME)
    return;

  uint16_t mask = 0;
  uint16_t acc = 0;

  for (uint8_t i = 0; i < NUM_INSTR; ++i)
  {
    if (mute[i])
      continue;
    if (StepEnabled(i, step))
    {
      uint16_t bit = (1u << i);
      mask |= bit;
      if (StepAccent(i, step))
        acc |= bit;
    }
  }

  if (mask)
  {
    __atomic_fetch_or(&acc_mask, acc, __ATOMIC_RELEASE);
    __atomic_fetch_or(&trig_mask, mask, __ATOMIC_RELEASE);
  }
}

static inline void ScheduleStep(uint32_t base_time_us, uint32_t period_us)
{
  uint32_t delay_us = 0;
  if (swing_percent > 0 && (seq_step & 1))
  {
    delay_us = (period_us * (uint32_t)swing_percent) / 100;
  }

  if (delay_us == 0)
  {
    TriggerStep(seq_step);
  }
  else
  {
    pending_step = true;
    pending_step_time_us = base_time_us + delay_us;
    pending_step_index = seq_step;
  }

  ++seq_step;
  if (seq_step >= step_count)
    seq_step = 0;
}

static inline void StartClockPulse()
{
  gateout = GATEHIGH;
  gateoff_ms = millis() + CLOCK_PULSE_MS;
}

static inline void UpdateClockPulse()
{
  if (gateout == GATEHIGH && (int32_t)(millis() - gateoff_ms) >= 0)
    gateout = GATELOW;
}

static inline void HandleExternalClock()
{
  if (!digitalRead(CLOCKIN))
  {
    if (((millis() - clockdebouncetimer) > CLOCK_DEBOUNCE) && !clocked)
    {
      clocked = true;
      uint32_t now_ms = millis();
      uint32_t now_us = micros();

      external_clock_present = true;
      last_external_ms = now_ms;

      if (last_external_pulse_us != 0)
      external_period_us = now_us - last_external_pulse_us;
      last_external_pulse_us = now_us;

      StartClockPulse();
      if (ui_mode == MODE_REALTIME)
        FlashClockLed();

      if (playing && ui_mode == MODE_REALTIME)
      {
        uint32_t period_us = external_period_us ? external_period_us : step_interval_us;
        ScheduleStep(now_us, period_us);
      }
    }
  }
  else
  {
    clocked = false;
    clockdebouncetimer = millis();
  }

  if (external_clock_present && (millis() - last_external_ms) > CLOCK_TIMEOUT_MS)
  {
    external_clock_present = false;
    external_period_us = 0;
  }
}

static inline void HandleInternalClock()
{
  if (external_clock_present)
    return;

  uint32_t now_us = micros();
  if ((int32_t)(now_us - next_internal_pulse_us) >= 0)
  {
    StartClockPulse();
    if (ui_mode == MODE_REALTIME)
      FlashClockLed();
    if (playing && ui_mode == MODE_REALTIME)
      ScheduleStep(now_us, step_interval_us);

    // avoid drift by stepping forward in whole intervals
    do
    {
      next_internal_pulse_us += step_interval_us;
    } while ((int32_t)(now_us - next_internal_pulse_us) >= 0);
  }
}

static inline void HandlePendingStep()
{
  if (!pending_step)
    return;

  uint32_t now_us = micros();
  if ((int32_t)(now_us - pending_step_time_us) >= 0)
  {
    TriggerStep(pending_step_index);
    pending_step = false;
  }
}

void setup()
{
  Serial.begin(115200);

#ifdef DEBUG
  Serial.println("starting setup");
#endif

#ifdef MONITOR_CPU1
  pinMode(CPU_USE, OUTPUT); // hi = CPU busy
#endif

  pinMode(TRIGGER, INPUT_PULLUP); // clock input
  pinMode(BUTTON1, INPUT_PULLUP); // button
  pinMode(MUXCTL, OUTPUT);        // analog mux

  LEDS.begin();
  LEDS.setPixelColor(0, RED);
  LEDS.show();

  analogReadResolution(AD_BITS);

  // Init drum voices
  voice_bd.Init(SAMPLERATE);
  voice_sd.Init(SAMPLERATE);
  voice_clap.Init(SAMPLERATE);
  voice_rim.Init(SAMPLERATE);
  voice_tom.Init(SAMPLERATE);
  voice_chh.Init(SAMPLERATE);
  voice_ohh.Init(SAMPLERATE);
  voice_cym.Init(SAMPLERATE);

  // Init clock
  step_interval_us = ComputeStepIntervalUs(bpm);
  next_internal_pulse_us = micros() + step_interval_us;

  // Set up I2S for PT8211 stereo DAC
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

void loop()
{
  // button handling
  bool buttonraw = !digitalRead(BUTTON1);
  if (buttonraw)
  {
    if (((millis() - buttontimer) > DEBOUNCE) && !button)
    {
      button = true;
      buttonpress = millis();
      swing_editing = false;
      length_editing = false;
    }
  }
  else
  {
    if (button)
    {
      uint32_t held_ms = millis() - buttonpress;
      bool short_press = (held_ms < 400) && !swing_editing && !length_editing;

      if (short_press)
      {
        if (ui_mode == MODE_EDIT)
        {
          bool on = StepEnabled(edit_inst, edit_step);
          SetStepEnabled(edit_inst, edit_step, !on);
          if (!on)
          {
            bool acc = (pot[2] >= MODE_SWITCH_THRESHOLD);
            SetStepAccent(edit_inst, edit_step, acc);
          }
          else
          {
            SetStepAccent(edit_inst, edit_step, false);
          }
        }
        else
        {
          playing = !playing;
          if (playing)
          {
            seq_step = 0;
            pending_step = false;
            next_internal_pulse_us = micros() + step_interval_us;
          }
        }
      }
    }
    button = false;
    buttontimer = millis();
  }

  if (!playing || ui_mode != MODE_REALTIME)
    pending_step = false;

  if ((millis() - parameterupdate) > PARAMETERUPDATE)
  {
    parameterupdate = millis();
    samplepots();

    // Mode selection (Pot4)
    if (!potlock[3])
    {
      UIMode new_mode = (pot[3] >= MODE_SWITCH_THRESHOLD) ? MODE_REALTIME : MODE_EDIT;
      if (new_mode != ui_mode)
      {
        ui_mode = new_mode;
        ShowTempColor(ModeColor(ui_mode), 300);
        lockpots();
        if (ui_mode == MODE_EDIT)
          playing = false;
      }
    }

    // Instrument selection (Pot1)
    if (!potlock[0])
    {
      uint8_t new_inst = PotToIndex(pot[0], NUM_INSTR);
      if (new_inst != edit_inst)
      {
        edit_inst = new_inst;
        ShowTempColor(InstrumentColor(edit_inst), 300);
      }
    }

    if (ui_mode == MODE_EDIT)
    {
      // Step select / length edit (Pot2)
      if (!potlock[1])
      {
        if (button)
        {
          length_editing = true;
          uint8_t steps = (pot[1] * MAX_STEPS + (AD_RANGE / 2)) / AD_RANGE;
          if (steps < 1) steps = 1;
          if (steps > MAX_STEPS) steps = MAX_STEPS;
          if (steps != step_count)
          {
            step_count = steps;
            if (edit_step >= step_count) edit_step = step_count - 1;
          }
        }
        else
        {
          uint8_t step = PotToIndex(pot[1], step_count);
          if (step != edit_step)
            edit_step = step;
        }
      }

      // Accent edit (Pot3)
      if (!potlock[2])
      {
        bool acc = (pot[2] >= MODE_SWITCH_THRESHOLD);
        SetStepAccent(edit_inst, edit_step, acc);
      }
    }
    else
    {
      // Realtime mode
      // Mute (Pot2)
      if (!potlock[1])
      {
        bool is_muted = (pot[1] < MODE_SWITCH_THRESHOLD);
        mute[edit_inst] = is_muted;
      }

      // BPM / Swing (Pot3)
      if (!potlock[2])
      {
        if (button)
        {
          swing_editing = true;
          uint8_t new_swing = map(pot[2], 0, AD_RANGE - 1, 0, 50);
          if (new_swing != swing_percent)
          {
            swing_percent = new_swing;
          }
        }
        else if (!external_clock_present)
        {
          float new_bpm = mapf(pot[2], 0, AD_RANGE - 1, BPM_MIN, BPM_MAX);
          if (fabsf(new_bpm - bpm) > 0.5f)
          {
            bpm = new_bpm;
            step_interval_us = ComputeStepIntervalUs(bpm);
            next_internal_pulse_us = micros() + step_interval_us;
          }
        }
      }
    }
  }

  HandleExternalClock();
  if (last_external_present && !external_clock_present)
    next_internal_pulse_us = micros() + step_interval_us;
  last_external_present = external_clock_present;
  HandleInternalClock();
  HandlePendingStep();
  UpdateClockPulse();
  UpdateLed();
}

// second core setup
// second core is dedicated to sample processing
void setup1()
{
  delay(1000); // wait for main core to start up peripherals
}

// process audio samples
void loop1()
{
#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE, 0); // low - CPU not busy
#endif

  uint16_t mask = __atomic_exchange_n(&trig_mask, 0, __ATOMIC_ACQ_REL);
  uint16_t acc  = __atomic_exchange_n(&acc_mask, 0, __ATOMIC_ACQ_REL);

  if (mask)
  {
    if (mask & (1u << INST_BD)) { if (acc & (1u << INST_BD)) voice_bd.SetAccent(ACCENT_ON); else voice_bd.SetAccent(ACCENT_OFF); voice_bd.Trig(); }
    if (mask & (1u << INST_SD)) { if (acc & (1u << INST_SD)) voice_sd.SetAccent(ACCENT_ON); else voice_sd.SetAccent(ACCENT_OFF); voice_sd.Trig(); }
    if (mask & (1u << INST_CLAP)) { if (acc & (1u << INST_CLAP)) voice_clap.SetAccent(ACCENT_ON); else voice_clap.SetAccent(ACCENT_OFF); voice_clap.Trig(); }
    if (mask & (1u << INST_RIM)) { if (acc & (1u << INST_RIM)) voice_rim.SetAccent(ACCENT_ON); else voice_rim.SetAccent(ACCENT_OFF); voice_rim.Trig(); }
    if (mask & (1u << INST_TOM)) { if (acc & (1u << INST_TOM)) voice_tom.SetAccent(ACCENT_ON); else voice_tom.SetAccent(ACCENT_OFF); voice_tom.Trig(); }
    if (mask & (1u << INST_CHH)) { if (acc & (1u << INST_CHH)) voice_chh.SetAccent(ACCENT_ON); else voice_chh.SetAccent(ACCENT_OFF); voice_chh.Trig(); }
    if (mask & (1u << INST_OHH)) { if (acc & (1u << INST_OHH)) voice_ohh.SetAccent(ACCENT_ON); else voice_ohh.SetAccent(ACCENT_OFF); voice_ohh.Trig(); }
    if (mask & (1u << INST_CYM)) { if (acc & (1u << INST_CYM)) voice_cym.SetAccent(ACCENT_ON); else voice_cym.SetAccent(ACCENT_OFF); voice_cym.Trig(); }
  }

  float mix = 0.0f;
  float v;

  v = voice_bd.Process();   if (!mute[INST_BD]) mix += v;
  v = voice_sd.Process();   if (!mute[INST_SD]) mix += v;
  v = voice_clap.Process(); if (!mute[INST_CLAP]) mix += v;
  v = voice_rim.Process();  if (!mute[INST_RIM]) mix += v;
  v = voice_tom.Process();  if (!mute[INST_TOM]) mix += v;
  v = voice_chh.Process();  if (!mute[INST_CHH]) mix += v;
  v = voice_ohh.Process();  if (!mute[INST_OHH]) mix += v;
  v = voice_cym.Process();  if (!mute[INST_CYM]) mix += v;

  // mix gain + soft clip to avoid overall distortion
  mix *= 0.25f;
  mix = SoftClip(mix);

  int32_t outsample = (int32_t)(mix * MULT_16) >> 16;

  DAC.write((int16_t)outsample); // left: audio
  DAC.write((int16_t)gateout);   // right: clock out

#ifdef MONITOR_CPU1
  digitalWrite(CPU_USE, 1); // hi = CPU busy
#endif
}
