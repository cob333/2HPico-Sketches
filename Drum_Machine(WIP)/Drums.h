#pragma once
#ifndef DRUM_MACHINE_DRUMS_H
#define DRUM_MACHINE_DRUMS_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#ifdef __cplusplus

namespace daisysp
{
// Minimal DSP helpers (local to avoid pulling full DaisySP Utility lib).
static constexpr float TWOPI_F = 6.2831853071795864769f;
static constexpr float kRandFrac = 1.f / (float)RAND_MAX;

inline float fclamp(float in, float min, float max)
{
    return fminf(fmaxf(in, min), max);
}

inline float SoftLimit(float x)
{
    return x * (27.f + x * x) / (27.f + 9.f * x * x);
}

inline float SoftClip(float x)
{
    if(x < -3.0f)
        return -1.0f;
    else if(x > 3.0f)
        return 1.0f;
    else
        return SoftLimit(x);
}

// Small helper utilities (header-only).
inline float DrumClamp01(float x) { return fclamp(x, 0.0f, 1.0f); }

inline float DrumMap01(float x, float out_min, float out_max)
{
    x = DrumClamp01(x);
    return out_min + (out_max - out_min) * x;
}

inline float DrumDecayCoeff(float decay_s, float sample_rate)
{
    decay_s = fmaxf(decay_s, 0.0005f);
    return expf(-1.0f / (decay_s * sample_rate));
}

inline float DrumOnePoleCoeff(float cutoff_hz, float sample_rate)
{
    cutoff_hz = fmaxf(cutoff_hz, 5.0f);
    return 1.0f - expf(-TWOPI_F * cutoff_hz / sample_rate);
}

inline float DrumWhiteNoise()
{
    return (rand() * kRandFrac) * 2.0f - 1.0f;
}

class DrumOnePole
{
  public:
    DrumOnePole() : state_(0.0f) {}

    void Init() { state_ = 0.0f; }

    float ProcessLP(float in, float coeff)
    {
        state_ += coeff * (in - state_);
        return state_;
    }

    float ProcessHP(float in, float coeff)
    {
        state_ += coeff * (in - state_);
        return in - state_;
    }

  private:
    float state_;
};

class DrumDecayEnv
{
  public:
    DrumDecayEnv() : value_(0.0f), coeff_(0.999f), sustain_(false) {}

    void Init(float sample_rate)
    {
        sample_rate_ = sample_rate;
        value_       = 0.0f;
        coeff_       = 0.999f;
        sustain_     = false;
    }

    void SetDecay(float decay_s)
    {
        coeff_ = DrumDecayCoeff(decay_s, sample_rate_);
    }

    void Trig(float amp) { value_ = amp; }

    void SetSustain(bool sustain) { sustain_ = sustain; }

    float Process()
    {
        float out = value_;
        if(!sustain_)
        {
            value_ *= coeff_;
        }
        return out;
    }

    float Value() const { return value_; }

  private:
    float sample_rate_;
    float value_;
    float coeff_;
    bool  sustain_;
};

class DrumMetallicNoise
{
  public:
    DrumMetallicNoise() : sample_rate_(48000.0f) {}

    void Init(float sample_rate)
    {
        sample_rate_ = sample_rate;
        for(int i = 0; i < 6; ++i)
        {
            phase_[i] = 0;
        }
    }

    float Process(float freq_hz)
    {
        const float ratios[6] = {
            1.0f,
            1.304f,
            1.466f,
            1.787f,
            1.932f,
            2.536f,
        };

        float f0 = freq_hz / sample_rate_;
        if(f0 < 0.0f)
        {
            f0 = 0.0f;
        }

        uint32_t noise = 0;
        for(int i = 0; i < 6; ++i)
        {
            float f = f0 * ratios[i];
            if(f >= 0.499f)
            {
                f = 0.499f;
            }
            const uint32_t inc = static_cast<uint32_t>(f * 4294967296.0f);
            phase_[i] += inc;
            noise += (phase_[i] >> 31);
        }

        return 0.33f * static_cast<float>(noise) - 1.0f;
    }

  private:
    float    sample_rate_;
    uint32_t phase_[6];
};

/**
    @brief 909-ish bass drum.
*/
class BassDrum
{
  public:
    BassDrum() {}
    ~BassDrum() {}

    void Init(float sample_rate)
    {
        sample_rate_ = sample_rate;
        phase_       = 0.0f;
        trig_        = false;
        sustain_     = false;

        amp_env_.Init(sample_rate_);
        pitch_env_.Init(sample_rate_);
        click_env_.Init(sample_rate_);

        click_lp_.Init();

        SetFreq(55.0f);
        SetTone(0.5f);
        SetDecay(0.5f);
        SetAccent(0.8f);
        SetSweep(0.6f);
        SetSweepDecay(0.35f);
        SetDrive(0.2f);

        click_env_.SetDecay(0.004f);
    }

    float Process(bool trigger = false)
    {
        if(trigger || trig_)
        {
            trig_ = false;
            amp_env_.Trig(0.3f + 0.7f * accent_);
            pitch_env_.Trig(1.0f);
            click_env_.Trig(1.0f);
        }

        float pitch_env = pitch_env_.Process();
        float amp_env   = amp_env_.Process();
        float click_env = click_env_.Process();

        float freq = freq_ * (1.0f + sweep_ * pitch_env);
        phase_ += freq / sample_rate_;
        if(phase_ >= 1.0f)
        {
            phase_ -= 1.0f;
        }

        float body = sinf(TWOPI_F * phase_) * amp_env;

        float noise   = DrumWhiteNoise();
        float clicklp = click_lp_.ProcessLP(noise, click_coeff_);
        float click   = (noise - clicklp) * click_env * click_amount_;

        float out = body + click;
        if(drive_ > 0.0f)
        {
            out = SoftClip(out * (1.0f + drive_ * 4.0f));
        }

        return out;
    }

    void Trig() { trig_ = true; }

    void SetSustain(bool sustain)
    {
        sustain_ = sustain;
        amp_env_.SetSustain(sustain_);
        pitch_env_.SetSustain(sustain_);
    }

    void SetAccent(float accent) { accent_ = DrumClamp01(accent); }

    void SetFreq(float freq_hz) { freq_ = fmaxf(freq_hz, 5.0f); }

    void SetTone(float tone)
    {
        tone_        = DrumClamp01(tone);
        float cutoff = DrumMap01(tone_, 800.0f, 8000.0f);
        click_coeff_ = DrumOnePoleCoeff(cutoff, sample_rate_);
        click_amount_ = DrumMap01(tone_, 0.1f, 0.4f);
    }

    void SetDecay(float decay)
    {
        decay_ = DrumClamp01(decay);
        float decay_s = DrumMap01(decay_, 0.05f, 1.2f);
        amp_env_.SetDecay(decay_s);
    }

    void SetSweep(float sweep) { sweep_ = DrumClamp01(sweep) * 1.6f; }

    void SetSweepDecay(float sweep_decay)
    {
        sweep_decay_ = DrumClamp01(sweep_decay);
        float decay_s = DrumMap01(sweep_decay_, 0.01f, 0.12f);
        pitch_env_.SetDecay(decay_s);
    }

    void SetDrive(float drive) { drive_ = DrumClamp01(drive); }

  private:
    float sample_rate_;
    float phase_;
    float freq_;

    float accent_;
    float tone_;
    float decay_;
    float sweep_;
    float sweep_decay_;
    float drive_;

    bool trig_;
    bool sustain_;

    DrumDecayEnv amp_env_;
    DrumDecayEnv pitch_env_;
    DrumDecayEnv click_env_;

    DrumOnePole click_lp_;
    float       click_coeff_;
    float       click_amount_;
};

/**
    @brief 909-ish snare drum (two modes + noise).
*/
class SnareDrum
{
  public:
    SnareDrum() {}
    ~SnareDrum() {}

    void Init(float sample_rate)
    {
        sample_rate_ = sample_rate;
        phase1_      = 0.0f;
        phase2_      = 0.0f;
        trig_        = false;
        sustain_     = false;

        amp_env_.Init(sample_rate_);
        noise_env_.Init(sample_rate_);

        noise_hp_.Init();

        SetFreq(180.0f);
        SetDecay(0.5f);
        SetTone(0.6f);
        SetSnappy(0.7f);
        SetAccent(0.8f);
    }

    float Process(bool trigger = false)
    {
        if(trigger || trig_)
        {
            trig_ = false;
            amp_env_.Trig(0.3f + 0.7f * accent_);
            noise_env_.Trig(0.3f + 0.7f * accent_);
        }

        float body_env  = amp_env_.Process();
        float noise_env = noise_env_.Process();

        float f1 = freq_;
        float f2 = freq_ * 1.47f;
        phase1_ += f1 / sample_rate_;
        phase2_ += f2 / sample_rate_;
        if(phase1_ >= 1.0f)
            phase1_ -= 1.0f;
        if(phase2_ >= 1.0f)
            phase2_ -= 1.0f;

        float body = (sinf(TWOPI_F * phase1_) + 0.6f * sinf(TWOPI_F * phase2_))
                     * 0.5f;
        body *= body_env * (1.0f - snappy_);

        float noise   = DrumWhiteNoise();
        float hpnoise = noise_hp_.ProcessHP(noise, noise_hp_coeff_);
        float snare   = hpnoise * noise_env * snappy_;

        return SoftClip(body + snare);
    }

    void Trig() { trig_ = true; }

    void SetSustain(bool sustain)
    {
        sustain_ = sustain;
        amp_env_.SetSustain(sustain_);
        noise_env_.SetSustain(sustain_);
    }

    void SetAccent(float accent) { accent_ = DrumClamp01(accent); }

    void SetFreq(float freq_hz) { freq_ = fmaxf(freq_hz, 10.0f); }

    void SetTone(float tone)
    {
        tone_ = DrumClamp01(tone);
        float cutoff   = DrumMap01(tone_, 1200.0f, 9000.0f);
        noise_hp_coeff_ = DrumOnePoleCoeff(cutoff, sample_rate_);
    }

    void SetDecay(float decay)
    {
        decay_ = DrumClamp01(decay);
        UpdateDecay();
    }

    void SetSnappy(float snappy)
    {
        snappy_ = DrumClamp01(snappy);
        UpdateDecay();
    }

  private:
    void UpdateDecay()
    {
        float body_s = DrumMap01(decay_, 0.06f, 0.6f);
        float noise_s = body_s * (0.6f + 0.9f * snappy_);
        amp_env_.SetDecay(body_s);
        noise_env_.SetDecay(noise_s);
    }

    float sample_rate_;
    float phase1_;
    float phase2_;
    float freq_;

    float accent_;
    float tone_;
    float decay_;
    float snappy_;

    bool trig_;
    bool sustain_;

    DrumDecayEnv amp_env_;
    DrumDecayEnv noise_env_;

    DrumOnePole noise_hp_;
    float       noise_hp_coeff_;
};

/**
    @brief 909 style handclap (multi-burst noise).
*/
class HandClap
{
  public:
    HandClap() {}
    ~HandClap() {}

    void Init(float sample_rate)
    {
        sample_rate_ = sample_rate;
        trig_        = false;
        stage_       = kIdle;
        counter_     = 0;

        noise_hp_.Init();

        burst_coeff_ = DrumDecayCoeff(0.01f, sample_rate_);

        SetDecay(0.5f);
        SetTone(0.6f);
        SetAccent(0.8f);

        burst_samples_ = static_cast<int>(sample_rate_ * 0.008f);
        gap_samples_   = static_cast<int>(sample_rate_ * 0.007f);
    }

    float Process(bool trigger = false)
    {
        if(trigger || trig_)
        {
            trig_ = false;
            Start();
        }

        float out = 0.0f;

        if(stage_ == kIdle)
        {
            return 0.0f;
        }

        float noise = DrumWhiteNoise();
        float hp    = noise_hp_.ProcessHP(noise, noise_hp_coeff_);

        switch(stage_)
        {
            case kBurst1:
            case kBurst2:
            case kBurst3:
            {
                out = hp * burst_env_ * accent_;
                burst_env_ *= burst_coeff_;
                if(++counter_ >= burst_samples_)
                {
                    AdvanceStage();
                }
            }
            break;
            case kGap1:
            case kGap2:
            case kGap3:
            {
                if(++counter_ >= gap_samples_)
                {
                    AdvanceStage();
                }
            }
            break;
            case kTail:
            {
                out = hp * tail_env_ * accent_;
                tail_env_ *= tail_coeff_;
                if(tail_env_ < 0.0005f)
                {
                    stage_ = kIdle;
                }
            }
            break;
            default: break;
        }

        return out;
    }

    void Trig() { trig_ = true; }

    void SetAccent(float accent) { accent_ = DrumClamp01(accent); }

    void SetTone(float tone)
    {
        tone_ = DrumClamp01(tone);
        float cutoff   = DrumMap01(tone_, 600.0f, 6000.0f);
        noise_hp_coeff_ = DrumOnePoleCoeff(cutoff, sample_rate_);
    }

    void SetDecay(float decay)
    {
        decay_ = DrumClamp01(decay);
        float tail_s = DrumMap01(decay_, 0.05f, 0.8f);
        tail_coeff_ = DrumDecayCoeff(tail_s, sample_rate_);
    }

  private:
    enum Stage
    {
        kIdle = 0,
        kBurst1,
        kGap1,
        kBurst2,
        kGap2,
        kBurst3,
        kGap3,
        kTail,
    };

    void Start()
    {
        stage_     = kBurst1;
        counter_   = 0;
        burst_env_ = 1.0f;
    }

    void AdvanceStage()
    {
        counter_ = 0;
        switch(stage_)
        {
            case kBurst1: stage_ = kGap1; break;
            case kGap1:
                stage_     = kBurst2;
                burst_env_ = 1.0f;
                break;
            case kBurst2: stage_ = kGap2; break;
            case kGap2:
                stage_     = kBurst3;
                burst_env_ = 1.0f;
                break;
            case kBurst3: stage_ = kGap3; break;
            case kGap3:
                stage_   = kTail;
                tail_env_ = 1.0f;
                break;
            default: stage_ = kIdle; break;
        }
    }

    float sample_rate_;
    bool  trig_;

    Stage stage_;
    int   counter_;
    int   burst_samples_;
    int   gap_samples_;

    float accent_;
    float tone_;
    float decay_;

    float burst_env_;
    float burst_coeff_;
    float tail_env_;
    float tail_coeff_;

    DrumOnePole noise_hp_;
    float       noise_hp_coeff_;
};

/**
    @brief 909-ish rimshot.
*/
class Rimshot
{
  public:
    Rimshot() {}
    ~Rimshot() {}

    void Init(float sample_rate)
    {
        sample_rate_ = sample_rate;
        phase1_      = 0.0f;
        phase2_      = 0.0f;
        trig_        = false;

        env_.Init(sample_rate_);
        click_env_.Init(sample_rate_);
        click_env_.SetDecay(0.008f);

        noise_hp_.Init();

        SetFreq(900.0f);
        SetDecay(0.4f);
        SetTone(0.6f);
        SetAccent(0.8f);
    }

    float Process(bool trigger = false)
    {
        if(trigger || trig_)
        {
            trig_ = false;
            env_.Trig(0.3f + 0.7f * accent_);
            click_env_.Trig(1.0f);
        }

        float env = env_.Process();
        float click_env = click_env_.Process();

        float f1 = freq_;
        float f2 = freq_ * 1.62f;
        phase1_ += f1 / sample_rate_;
        phase2_ += f2 / sample_rate_;
        if(phase1_ >= 1.0f)
            phase1_ -= 1.0f;
        if(phase2_ >= 1.0f)
            phase2_ -= 1.0f;

        float body = (sinf(TWOPI_F * phase1_) + 0.5f * sinf(TWOPI_F * phase2_))
                     * 0.6f;
        body *= env * (1.0f - tone_);

        float noise   = DrumWhiteNoise();
        float hpnoise = noise_hp_.ProcessHP(noise, noise_hp_coeff_);
        float click   = hpnoise * click_env * tone_ * 0.8f;

        return SoftClip(body + click);
    }

    void Trig() { trig_ = true; }

    void SetAccent(float accent) { accent_ = DrumClamp01(accent); }

    void SetFreq(float freq_hz) { freq_ = fmaxf(freq_hz, 50.0f); }

    void SetTone(float tone)
    {
        tone_ = DrumClamp01(tone);
        float cutoff   = DrumMap01(tone_, 1500.0f, 9000.0f);
        noise_hp_coeff_ = DrumOnePoleCoeff(cutoff, sample_rate_);
    }

    void SetDecay(float decay)
    {
        decay_ = DrumClamp01(decay);
        float decay_s = DrumMap01(decay_, 0.03f, 0.25f);
        env_.SetDecay(decay_s);
    }

  private:
    float sample_rate_;
    float phase1_;
    float phase2_;
    float freq_;

    float accent_;
    float tone_;
    float decay_;

    bool trig_;

    DrumDecayEnv env_;
    DrumDecayEnv click_env_;

    DrumOnePole noise_hp_;
    float       noise_hp_coeff_;
};

/**
    @brief 909-ish tom.
*/
class Tom
{
  public:
    Tom() {}
    ~Tom() {}

    void Init(float sample_rate)
    {
        sample_rate_ = sample_rate;
        phase_       = 0.0f;
        trig_        = false;
        sustain_     = false;

        amp_env_.Init(sample_rate_);
        pitch_env_.Init(sample_rate_);

        SetFreq(140.0f);
        SetDecay(0.5f);
        SetTone(0.5f);
        SetPitchDecay(0.3f);
        SetAccent(0.8f);
    }

    float Process(bool trigger = false)
    {
        if(trigger || trig_)
        {
            trig_ = false;
            amp_env_.Trig(0.3f + 0.7f * accent_);
            pitch_env_.Trig(1.0f);
        }

        float amp = amp_env_.Process();
        float pitch_env = pitch_env_.Process();

        float freq = freq_ * (1.0f + pitch_amount_ * pitch_env);
        phase_ += freq / sample_rate_;
        if(phase_ >= 1.0f)
        {
            phase_ -= 1.0f;
        }

        float out = sinf(TWOPI_F * phase_) * amp;
        return out;
    }

    void Trig() { trig_ = true; }

    void SetSustain(bool sustain)
    {
        sustain_ = sustain;
        amp_env_.SetSustain(sustain_);
        pitch_env_.SetSustain(sustain_);
    }

    void SetAccent(float accent) { accent_ = DrumClamp01(accent); }

    void SetFreq(float freq_hz) { freq_ = fmaxf(freq_hz, 20.0f); }

    // Tone controls pitch sweep amount for the tom.
    void SetTone(float tone) { pitch_amount_ = DrumClamp01(tone) * 1.3f; }

    void SetDecay(float decay)
    {
        decay_ = DrumClamp01(decay);
        float decay_s = DrumMap01(decay_, 0.08f, 1.4f);
        amp_env_.SetDecay(decay_s);
    }

    void SetPitchDecay(float pitch_decay)
    {
        pitch_decay_ = DrumClamp01(pitch_decay);
        float decay_s = DrumMap01(pitch_decay_, 0.01f, 0.12f);
        pitch_env_.SetDecay(decay_s);
    }

  private:
    float sample_rate_;
    float phase_;
    float freq_;

    float accent_;
    float decay_;
    float pitch_amount_;
    float pitch_decay_;

    bool trig_;
    bool sustain_;

    DrumDecayEnv amp_env_;
    DrumDecayEnv pitch_env_;
};

class HiHatCore
{
  public:
    HiHatCore() {}
    ~HiHatCore() {}

    void Init(float sample_rate, float min_decay_s, float max_decay_s, float base_freq)
    {
        sample_rate_ = sample_rate;
        min_decay_s_ = min_decay_s;
        max_decay_s_ = max_decay_s;
        freq_        = base_freq;

        trig_    = false;
        sustain_ = false;

        env_.Init(sample_rate_);
        hp_.Init();
        lp_.Init();
        noise_.Init(sample_rate_);

        SetTone(0.7f);
        SetDecay(0.4f);
        SetAccent(0.8f);
    }

    float Process(bool trigger = false)
    {
        if(trigger || trig_)
        {
            trig_ = false;
            env_.Trig(0.3f + 0.7f * accent_);
        }

        float env = env_.Process();
        float metal = noise_.Process(freq_);

        float hp = hp_.ProcessHP(metal, hp_coeff_);
        float lp = lp_.ProcessLP(hp, lp_coeff_);

        return lp * env;
    }

    void Trig() { trig_ = true; }

    void SetSustain(bool sustain)
    {
        sustain_ = sustain;
        env_.SetSustain(sustain_);
    }

    void SetAccent(float accent) { accent_ = DrumClamp01(accent); }

    void SetFreq(float freq_hz) { freq_ = fmaxf(freq_hz, 200.0f); }

    void SetTone(float tone)
    {
        tone_ = DrumClamp01(tone);
        float hp_cut = 500.0f;
        float lp_cut = DrumMap01(tone_, 3500.0f, 12000.0f);
        hp_coeff_ = DrumOnePoleCoeff(hp_cut, sample_rate_);
        lp_coeff_ = DrumOnePoleCoeff(lp_cut, sample_rate_);
    }

    void SetDecay(float decay)
    {
        decay_ = DrumClamp01(decay);
        float decay_s = DrumMap01(decay_, min_decay_s_, max_decay_s_);
        env_.SetDecay(decay_s);
    }

  private:
    float sample_rate_;
    float min_decay_s_;
    float max_decay_s_;
    float freq_;

    float accent_;
    float tone_;
    float decay_;

    bool trig_;
    bool sustain_;

    DrumDecayEnv env_;

    DrumOnePole hp_;
    DrumOnePole lp_;
    float       hp_coeff_;
    float       lp_coeff_;

    DrumMetallicNoise noise_;
};

/**
    @brief Closed hi-hat.
*/
class CloseHihat
{
  public:
    CloseHihat() {}
    ~CloseHihat() {}

    void Init(float sample_rate)
    {
        core_.Init(sample_rate, 0.02f, 0.18f, 3800.0f);
    }

    float Process(bool trigger = false) { return core_.Process(trigger); }

    void Trig() { core_.Trig(); }

    void SetSustain(bool sustain) { core_.SetSustain(sustain); }

    void SetAccent(float accent) { core_.SetAccent(accent); }

    void SetFreq(float freq_hz) { core_.SetFreq(freq_hz); }

    void SetTone(float tone) { core_.SetTone(tone); }

    void SetDecay(float decay) { core_.SetDecay(decay); }

  private:
    HiHatCore core_;
};

/**
    @brief Open hi-hat.
*/
class OpenHihat
{
  public:
    OpenHihat() {}
    ~OpenHihat() {}

    void Init(float sample_rate)
    {
        core_.Init(sample_rate, 0.12f, 1.2f, 3600.0f);
    }

    float Process(bool trigger = false) { return core_.Process(trigger); }

    void Trig() { core_.Trig(); }

    void SetSustain(bool sustain) { core_.SetSustain(sustain); }

    void SetAccent(float accent) { core_.SetAccent(accent); }

    void SetFreq(float freq_hz) { core_.SetFreq(freq_hz); }

    void SetTone(float tone) { core_.SetTone(tone); }

    void SetDecay(float decay) { core_.SetDecay(decay); }

  private:
    HiHatCore core_;
};

/**
    @brief 909-ish cymbal (metallic noise with long decay).
*/
class Cymbal
{
  public:
    Cymbal() {}
    ~Cymbal() {}

    void Init(float sample_rate)
    {
        sample_rate_ = sample_rate;
        trig_        = false;
        sustain_     = false;

        env_.Init(sample_rate_);
        hp_.Init();
        lp_.Init();
        noise1_.Init(sample_rate_);
        noise2_.Init(sample_rate_);

        SetFreq(2500.0f);
        SetTone(0.7f);
        SetDecay(0.7f);
        SetAccent(0.8f);
    }

    float Process(bool trigger = false)
    {
        if(trigger || trig_)
        {
            trig_ = false;
            env_.Trig(0.3f + 0.7f * accent_);
        }

        float env = env_.Process();

        float m1 = noise1_.Process(freq_);
        float m2 = noise2_.Process(freq_ * 1.23f);
        float metal = (m1 * 0.6f + m2 * 0.4f);

        float hp = hp_.ProcessHP(metal, hp_coeff_);
        float lp = lp_.ProcessLP(hp, lp_coeff_);

        return lp * env;
    }

    void Trig() { trig_ = true; }

    void SetSustain(bool sustain)
    {
        sustain_ = sustain;
        env_.SetSustain(sustain_);
    }

    void SetAccent(float accent) { accent_ = DrumClamp01(accent); }

    void SetFreq(float freq_hz) { freq_ = fmaxf(freq_hz, 200.0f); }

    void SetTone(float tone)
    {
        tone_ = DrumClamp01(tone);
        float hp_cut = 400.0f;
        float lp_cut = DrumMap01(tone_, 3000.0f, 14000.0f);
        hp_coeff_ = DrumOnePoleCoeff(hp_cut, sample_rate_);
        lp_coeff_ = DrumOnePoleCoeff(lp_cut, sample_rate_);
    }

    void SetDecay(float decay)
    {
        decay_ = DrumClamp01(decay);
        float decay_s = DrumMap01(decay_, 0.25f, 2.5f);
        env_.SetDecay(decay_s);
    }

  private:
    float sample_rate_;
    float freq_;

    float accent_;
    float tone_;
    float decay_;

    bool trig_;
    bool sustain_;

    DrumDecayEnv env_;

    DrumOnePole hp_;
    DrumOnePole lp_;
    float       hp_coeff_;
    float       lp_coeff_;

    DrumMetallicNoise noise1_;
    DrumMetallicNoise noise2_;
};

} // namespace daisysp

#endif

#endif
