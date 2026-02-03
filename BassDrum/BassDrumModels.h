#pragma once

#include <math.h>
#include "daisysp.h"

class BassDrum808 {
 public:
  void Init(float sample_rate) {
    sample_rate_ = sample_rate;
    drum_.Init(sample_rate);
    decay_coeff_ = decay_knob_to_coeff(0.5f, sample_rate_);
    amp_env_ = 0.0f;
  }

  void SetTone(float tone) { drum_.SetTone(tone); }

  void SetDecay(float decay) {
    drum_.SetDecay(decay);
    decay_coeff_ = decay_knob_to_coeff(decay, sample_rate_);
  }

  void SetFreq(float freq) { drum_.SetFreq(freq); }

  void SetAccent(float accent) { drum_.SetAccent(accent); }

  void Trig() {
    drum_.Trig();
    trig_pending_ = true;
  }

  float Process() {
    if(trig_pending_) {
      amp_env_ = 1.0f;
      trig_pending_ = false;
    }

    float local_decay = decay_coeff_;
    amp_env_ *= local_decay;

    float sig = drum_.Process();
    return sig * amp_env_;
  }

 private:
  static inline float decay_knob_to_coeff(float knob, float sr) {
    if(knob < 0.0f) knob = 0.0f;
    if(knob > 1.0f) knob = 1.0f;
    const float min_t = 0.02f;
    const float max_t = 2.0f;
    float t = min_t + knob * (max_t - min_t);
    return expf(-1.0f / (t * sr));
  }

  daisysp::AnalogBassDrum drum_;
  volatile bool trig_pending_ = false;
  volatile float decay_coeff_ = 0.999f;
  float amp_env_ = 0.0f;
  float sample_rate_ = 44100.0f;
};

class BassDrum909 {
 public:
  void Init(float sample_rate) { drum_.Init(sample_rate); }

  void SetTone(float tone) { drum_.SetTone(tone); }
  void SetDecay(float decay) { drum_.SetDecay(decay); }
  void SetFreq(float freq) { drum_.SetFreq(freq); }
  void SetAccent(float accent) { drum_.SetAccent(accent); }
  void SetDirtiness(float dirtiness) { drum_.SetDirtiness(dirtiness); }
  void SetFmEnvelopeAmount(float amount) { drum_.SetFmEnvelopeAmount(amount); }
  void SetFmEnvelopeDecay(float decay) { drum_.SetFmEnvelopeDecay(decay); }

  void Trig() { drum_.Trig(); }

  float Process() { return drum_.Process(); }

 private:
  daisysp::SyntheticBassDrum drum_;
};
