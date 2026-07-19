#ifndef FILTERS_H
#define FILTERS_H

// =============================================================================
//  filters.h  --  DSP primitives + vital-sign extractors
//  Multi-vital monitor (ESP32 + MAX30102 + MLX90614 + AD8232 + MQ3)
//
//  DESIGN RULE FOR THIS FILE:
//    Every extractor reports a VALIDITY state alongside its value.
//    Nothing here ever invents, holds, or smooths a number into existence.
//    If the signal isn't there, the answer is "not valid" -- not a plausible
//    looking figure. Callers must check valid() before displaying anything.
// =============================================================================

#include <Arduino.h>
#include <math.h>

#ifndef F_PI
#define F_PI 3.14159265358979f
#endif

// =============================================================================
//  SECTION 1 -- Core filters
// =============================================================================

// One-pole IIR low-pass.  y[n] += a*(x[n] - y[n])
class LowPassFilter {
  float alpha_, y_;
  bool  init_;
public:
  LowPassFilter(float cutoff_hz, float fs_hz) : y_(0.0f), init_(false) {
    configure(cutoff_hz, fs_hz);
  }
  void configure(float cutoff_hz, float fs_hz) {
    const float dt = 1.0f / fs_hz;
    const float rc = 1.0f / (2.0f * F_PI * cutoff_hz);
    alpha_ = dt / (rc + dt);
  }
  float process(float x) {
    if (!init_) { y_ = x; init_ = true; return y_; }
    y_ += alpha_ * (x - y_);
    return y_;
  }
  float value() const { return init_ ? y_ : NAN; }
  bool  ready() const { return init_; }
  void  reset() { init_ = false; y_ = 0.0f; }
};

// One-pole IIR high-pass (DC blocker).
class HighPassFilter {
  float alpha_, xPrev_, y_;
  bool  init_;
public:
  HighPassFilter(float cutoff_hz, float fs_hz) : xPrev_(0.0f), y_(0.0f), init_(false) {
    configure(cutoff_hz, fs_hz);
  }
  void configure(float cutoff_hz, float fs_hz) {
    const float dt = 1.0f / fs_hz;
    const float rc = 1.0f / (2.0f * F_PI * cutoff_hz);
    alpha_ = rc / (rc + dt);
  }
  float process(float x) {
    if (!init_) { xPrev_ = x; y_ = 0.0f; init_ = true; return y_; }
    y_ = alpha_ * (y_ + x - xPrev_);
    xPrev_ = x;
    return y_;
  }
  void reset() { init_ = false; xPrev_ = 0.0f; y_ = 0.0f; }
};

// High-pass then low-pass cascade.
class BandPassFilter {
  HighPassFilter hp_;
  LowPassFilter  lp_;
public:
  BandPassFilter(float lo_hz, float hi_hz, float fs_hz)
    : hp_(lo_hz, fs_hz), lp_(hi_hz, fs_hz) {}
  float process(float x) { return lp_.process(hp_.process(x)); }
  void  reset() { hp_.reset(); lp_.reset(); }
};

// Direct-form-I biquad.
class Biquad {
  float b0_, b1_, b2_, a1_, a2_;
  float x1_, x2_, y1_, y2_;
public:
  Biquad() : b0_(1), b1_(0), b2_(0), a1_(0), a2_(0),
             x1_(0), x2_(0), y1_(0), y2_(0) {}
  void setCoefficients(float b0, float b1, float b2, float a0, float a1, float a2) {
    const float inv = 1.0f / a0;
    b0_ = b0 * inv; b1_ = b1 * inv; b2_ = b2 * inv;
    a1_ = a1 * inv; a2_ = a2 * inv;
  }
  float process(float x) {
    const float y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    x2_ = x1_; x1_ = x;
    y2_ = y1_; y1_ = y;
    return y;
  }
  void reset() { x1_ = x2_ = y1_ = y2_ = 0.0f; }
};

// Mains-hum notch. Without this, 50/60 Hz survives the one-pole low-pass and is
// then AMPLIFIED by the derivative stage of the R-peak detector (differentiation
// gain rises with frequency), which walks the detected peak position by several
// samples per beat and puts a floor of ~25 ms under RMSSD. Set 50 for India/EU,
// 60 for the Americas.
class NotchFilter {
  Biquad bq_;
  bool   enabled_;
public:
  NotchFilter(float f0_hz, float fs_hz, float q = 20.0f) : enabled_(false) {
    configure(f0_hz, fs_hz, q);
  }
  void configure(float f0_hz, float fs_hz, float q = 20.0f) {
    if (f0_hz <= 0.0f || f0_hz >= fs_hz * 0.5f) { enabled_ = false; return; }
    const float w0    = 2.0f * F_PI * f0_hz / fs_hz;
    const float cw    = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * q);
    bq_.setCoefficients(1.0f, -2.0f * cw, 1.0f,
                        1.0f + alpha, -2.0f * cw, 1.0f - alpha);
    enabled_ = true;
  }
  float process(float x) { return enabled_ ? bq_.process(x) : x; }
  bool  enabled() const  { return enabled_; }
  void  reset() { bq_.reset(); }
};

// Fixed-length moving average.
template <int N>
class MovingAverageFilter {
  float buf_[N];
  int   idx_, count_;
  float sum_;
public:
  MovingAverageFilter() { reset(); }
  void reset() {
    for (int i = 0; i < N; i++) buf_[i] = 0.0f;
    idx_ = 0; count_ = 0; sum_ = 0.0f;
  }
  int  count() const { return count_; }
  bool full()  const { return count_ >= N; }
  float process(float x) {
    sum_ -= buf_[idx_];
    buf_[idx_] = x;
    sum_ += x;
    idx_ = (idx_ + 1) % N;
    if (count_ < N) count_++;
    return sum_ / (float)count_;
  }
  float value() const { return count_ ? sum_ / (float)count_ : NAN; }
};

// Envelope tracker: instant attack, exponential release.
// Used for amplitude gating and for display auto-scaling.
class DecayingMinMax {
  float min_, max_, release_;
  bool  init_;
public:
  explicit DecayingMinMax(float release = 0.0005f) : release_(release) { reset(); }
  void reset() { min_ = 0.0f; max_ = 0.0f; init_ = false; }
  void process(float x) {
    if (!init_) { min_ = max_ = x; init_ = true; return; }
    if (x > max_) max_ = x; else max_ -= (max_ - x) * release_;
    if (x < min_) min_ = x; else min_ += (x - min_) * release_;
  }
  float minimum() const { return min_; }
  float maximum() const { return max_; }
  float range()   const { return max_ - min_; }
  bool  ready()   const { return init_; }
};

// Accumulates min/max/mean over an arbitrary window (one cardiac cycle).
class WindowStatistic {
  float    min_, max_, sum_;
  uint32_t n_;
  bool     init_;
public:
  WindowStatistic() { reset(); }
  void reset() { min_ = max_ = sum_ = 0.0f; n_ = 0; init_ = false; }
  void process(float x) {
    if (!init_) { min_ = max_ = x; init_ = true; }
    if (x < min_) min_ = x;
    if (x > max_) max_ = x;
    sum_ += x; n_++;
  }
  uint32_t samples()    const { return n_; }
  float    minimum()    const { return init_ ? min_ : NAN; }
  float    maximum()    const { return init_ ? max_ : NAN; }
  float    mean()       const { return n_ ? sum_ / (float)n_ : NAN; }
  float    peakToPeak() const { return init_ ? (max_ - min_) : NAN; }
};


// Two-pole Butterworth low-pass. A single pole at 40 Hz barely touches the
// ESP32's ADC noise, which is what makes the rendered trace look fuzzy.
class LowPass2 {
  Biquad bq_;
public:
  LowPass2(float fc_hz, float fs_hz) { configure(fc_hz, fs_hz); }
  void configure(float fc_hz, float fs_hz) {
    const float w0 = 2.0f * F_PI * fc_hz / fs_hz;
    const float cw = cosf(w0), alpha = sinf(w0) / 1.41421356f;
    bq_.setCoefficients((1 - cw) * 0.5f, 1 - cw, (1 - cw) * 0.5f,
                        1 + alpha, -2.0f * cw, 1 - alpha);
  }
  float process(float x) { return bq_.process(x); }
  void  reset() { bq_.reset(); }
};

// Two-pole Butterworth high-pass. Removes baseline wander properly -- a one-pole
// at 0.5 Hz leaves enough breathing drift to walk the trace off screen.
class HighPass2 {
  Biquad bq_;
public:
  HighPass2(float fc_hz, float fs_hz) { configure(fc_hz, fs_hz); }
  void configure(float fc_hz, float fs_hz) {
    const float w0 = 2.0f * F_PI * fc_hz / fs_hz;
    const float cw = cosf(w0), alpha = sinf(w0) / 1.41421356f;
    bq_.setCoefficients((1 + cw) * 0.5f, -(1 + cw), (1 + cw) * 0.5f,
                        1 + alpha, -2.0f * cw, 1 - alpha);
  }
  float process(float x) { return bq_.process(x); }
  void  reset() { bq_.reset(); }
};

// Display auto-scaling for a zero-mean trace.
//
// Scaling on peak-to-peak is fragile: the R peak is most of the peak-to-peak
// range, so it always lands at the edge and clips, AND one motion artefact
// inflates the span for seconds afterwards, squashing the real signal flat.
// RMS is barely moved by a single spike, and the R-peak-to-RMS ratio of an ECG
// is stable enough to pick a multiplier that leaves headroom.
// Fixed display gain with a one-shot auto-fit.
//
// Continuously auto-scaling a trace is a trap: the gain moves while you are
// looking at it, one transient sets it wrong for seconds, and you can never
// tell whether the signal changed or the scale did. A hospital monitor uses
// FIXED gain for exactly that reason.
//
// This measures the signal once over a few seconds, then HOLDS. It re-fits only
// on lead reconnection or an explicit command.
class EcgGain {
  float    gain_, peak_, targetPx_;
  uint32_t n_, settleN_, fitN_;
  bool     fitted_, manual_;
public:
  EcgGain(float fs_hz, float target_px, float settle_s = 1.0f, float fit_s = 5.0f)
    : gain_(40.0f), peak_(0.0f), targetPx_(target_px), n_(0),
      settleN_((uint32_t)(fs_hz * settle_s)),
      fitN_((uint32_t)(fs_hz * (settle_s + fit_s))),
      fitted_(false), manual_(false) {}

  bool  ready()   const { return n_ >= settleN_; }   // baseline tracker settled
  bool  fitted()  const { return fitted_; }
  bool  manual()  const { return manual_; }
  float gain()    const { return gain_; }            // ADC counts per pixel
  float peak()    const { return peak_; }
  float fitProgress() const {
    if (fitted_ || n_ >= fitN_) return 1.0f;
    if (n_ <= settleN_) return 0.0f;
    return (float)(n_ - settleN_) / (float)(fitN_ - settleN_);
  }

  void setManual(float counts_per_px) {
    if (counts_per_px < 1.0f) counts_per_px = 1.0f;
    gain_ = counts_per_px; manual_ = true; fitted_ = true;
  }
  // Re-arm the measurement. Called on lead reconnect and by the serial console.
  void refit() { fitted_ = false; manual_ = false; peak_ = 0.0f; n_ = 0; }

  void process(float x) {
    n_++;
    if (fitted_ || n_ < settleN_) return;
    const float a = fabsf(x);
    if (a > peak_) peak_ = a;
    if (n_ >= fitN_) {
      if (peak_ > 1.0f) gain_ = peak_ / targetPx_;
      if (gain_ < 1.0f) gain_ = 1.0f;
      fitted_ = true;
    }
  }
};

// =============================================================================
//  SECTION 2 -- PPG peak detection
// =============================================================================

// Amplitude-gated peak detector for a ZERO-MEAN (band-passed) signal.
//
// The threshold is a fraction of the tracked signal ENVELOPE, plus a hard
// absolute floor. The old detector multiplied the signal MEAN, which sign-flips
// on the negative half-cycle and therefore gates nothing at all.
class AmplitudePeakDetector {
  DecayingMinMax env_;
  float    frac_, minAmp_;
  float    prev_, prevPrev_;
  bool     armed_;
  uint32_t lastPeakMs_, refractoryMs_;
  float    lastAmp_;
public:
  AmplitudePeakDetector(float fraction, float min_amplitude,
                        uint32_t refractory_ms, float release = 0.0006f)
    : env_(release), frac_(fraction), minAmp_(min_amplitude),
      prev_(0.0f), prevPrev_(0.0f), armed_(true),
      lastPeakMs_(0), refractoryMs_(refractory_ms), lastAmp_(0.0f) {}

  void reset() {
    env_.reset(); prev_ = prevPrev_ = 0.0f; armed_ = true;
    lastPeakMs_ = 0; lastAmp_ = 0.0f;
  }
  float amplitude()         const { return env_.range(); }
  float lastPeakAmplitude() const { return lastAmp_; }

  // Peak is confirmed on the PREVIOUS sample (needs one sample of lookahead).
  bool process(float x, uint32_t now_ms) {
    env_.process(x);
    if (env_.range() < minAmp_) {          // signal too weak to be a pulse
      prevPrev_ = prev_; prev_ = x;
      return false;
    }
    const float thr = env_.maximum() * frac_;
    bool peak = false;
    const bool localMax = (prev_ >= x) && (prev_ > prevPrev_);

    if (armed_ && localMax && prev_ > thr &&
        (lastPeakMs_ == 0 || (now_ms - lastPeakMs_) >= refractoryMs_)) {
      peak        = true;
      armed_      = false;
      lastPeakMs_ = now_ms;
      lastAmp_    = prev_;
    }
    if (x < thr * 0.5f) armed_ = true;     // hysteresis re-arm

    prevPrev_ = prev_;
    prev_     = x;
    return peak;
  }
};

// =============================================================================
//  SECTION 3 -- ECG R-peak detection (Pan-Tompkins style)
// =============================================================================
//
// Chain: band-pass 5-15 Hz -> 5-point derivative -> square
//        -> 150 ms moving-window integrate -> adaptive dual threshold
//
// Peak time is taken from the INTEGRATOR output, which lags the true R-peak by
// roughly half the integration window. That lag is constant across beats of
// similar morphology, so it cancels in R-R INTERVALS -- which is all that HRV
// needs. Sub-sample position is refined by parabolic interpolation.
class RPeakDetector {
  static const int MWI_MAX = 96;

  NotchFilter    notch_;
  BandPassFilter bp_;
  DecayingMinMax qrsEnv_;            // amplitude of the band-passed ECG
  float    minQrsAmp_;               // absolute floor: below this, nothing is a beat
  float    hist_[5];
  int      histN_;
  float    mwi_[MWI_MAX];
  int      mwiN_, mwiIdx_;
  float    mwiSum_;
  float    y1_;                      // integrator output at n-1
  float    spki_, npki_;             // running signal / noise peak estimates
  bool     learned_;
  uint32_t learnSamples_;
  float    fs_;
  uint32_t sampleIdx_;
  int64_t  lastPeakQ_;               // last committed peak, 1/256-sample units
  uint32_t refractorySamples_, maxSearchSamples_;
  float    lastRRms_;
  bool     rrValid_;

  // Peak-search state. Triggering on the FIRST local maximum above threshold
  // lets a noise bump on the rising edge win the race, which jitters R-R by
  // several samples. Instead: cross the threshold, track the maximum until the
  // signal falls back, then commit that maximum. Adds a constant latency that
  // cancels in intervals.
  bool     searching_;
  float    sMax_, sLeft_, sRight_;
  uint32_t sIdx_, sStart_;
  bool     needRight_;

public:
  RPeakDetector(float fs_hz, float mains_hz = 50.0f, float min_qrs_amplitude = 25.0f)
    : notch_(mains_hz, fs_hz), bp_(5.0f, 15.0f, fs_hz),
      qrsEnv_(0.0008f), minQrsAmp_(min_qrs_amplitude),
      histN_(0), mwiIdx_(0), mwiSum_(0.0f), y1_(0.0f),
      spki_(0.0f), npki_(0.0f), learned_(false), learnSamples_(0),
      fs_(fs_hz), sampleIdx_(0), lastPeakQ_(-1),
      lastRRms_(NAN), rrValid_(false),
      searching_(false), sMax_(0.0f), sLeft_(0.0f), sRight_(0.0f),
      sIdx_(0), sStart_(0), needRight_(false) {
    mwiN_ = (int)(0.100f * fs_hz);   // shorter window keeps 180+ bpm beats separate
    if (mwiN_ < 4)       mwiN_ = 4;
    if (mwiN_ > MWI_MAX) mwiN_ = MWI_MAX;
    refractorySamples_ = (uint32_t)(0.200f * fs_hz);   // physiological floor
    maxSearchSamples_  = (uint32_t)(0.250f * fs_hz);
    for (int i = 0; i < 5; i++)       hist_[i] = 0.0f;
    for (int i = 0; i < MWI_MAX; i++) mwi_[i]  = 0.0f;
  }

  void setMinQrsAmplitude(float a) { minQrsAmp_ = a; }
  void setMains(float f0_hz, float fs_hz) { notch_.configure(f0_hz, fs_hz); }

  void reset() {
    notch_.reset(); bp_.reset(); qrsEnv_.reset(); histN_ = 0; mwiIdx_ = 0; mwiSum_ = 0.0f;
    y1_ = 0.0f; spki_ = npki_ = 0.0f;
    learned_ = false; learnSamples_ = 0;
    sampleIdx_ = 0; lastPeakQ_ = -1;
    lastRRms_ = NAN; rrValid_ = false;
    searching_ = false; needRight_ = false;
    for (int i = 0; i < 5; i++)       hist_[i] = 0.0f;
    for (int i = 0; i < MWI_MAX; i++) mwi_[i]  = 0.0f;
  }

  bool  learned()      const { return learned_; }
  float qrsAmplitude() const { return qrsEnv_.range(); }
  float threshold()    const { return npki_ + 0.25f * (spki_ - npki_); }
  // Crude SNR proxy. Below ~2.5 the adaptive threshold has collapsed onto the
  // noise floor and any "beats" it reports are manufactured.
  float snr() const { return (npki_ > 1e-9f) ? (spki_ / npki_) : 0.0f; }
  bool  signalPresent() const { return (qrsEnv_.range() >= minQrsAmp_) && (snr() >= 2.5f); }

  bool process(float ecg_raw) {
    sampleIdx_++;

    const float b = bp_.process(notch_.process(ecg_raw));
    qrsEnv_.process(b);

    for (int i = 4; i > 0; i--) hist_[i] = hist_[i - 1];
    hist_[0] = b;
    if (histN_ < 5) { histN_++; pushMwi(0.0f); y1_ = 0.0f; return false; }

    const float d = (2.0f * hist_[0] + hist_[1] - hist_[3] - 2.0f * hist_[4]) * 0.125f;
    pushMwi(d * d);
    const float y0 = mwiSum_ / (float)mwiN_;

    if (!learned_) {
      if (y0 > spki_) spki_ = y0;
      npki_ = 0.98f * npki_ + 0.02f * y0;
      if (++learnSamples_ >= (uint32_t)(2.0f * fs_)) learned_ = true;
      y1_ = y0;
      return false;
    }

    if (needRight_) { sRight_ = y0; needRight_ = false; }

    const float thr = threshold();
    bool detected = false;

    if (!searching_) {
      if (y0 > thr && signalPresent()) {
        searching_ = true; sStart_ = sampleIdx_;
        sMax_ = y0; sIdx_ = sampleIdx_; sLeft_ = y1_; sRight_ = y0; needRight_ = true;
      } else if (y1_ > y0 && y1_ <= thr && y1_ > 0.0f) {
        npki_ = 0.125f * y1_ + 0.875f * npki_;      // local max below threshold
      }
    } else {
      if (y0 > sMax_) {
        sMax_ = y0; sIdx_ = sampleIdx_; sLeft_ = y1_; sRight_ = y0; needRight_ = true;
      }
      // Terminate on the adaptive threshold OR on a clear descent from this
      // beat's own maximum. At 180+ bpm the threshold alone can stay elevated
      // between beats, letting one search span two QRS complexes.
      const bool fell    = (y0 < thr * 0.5f) || (y0 < sMax_ * 0.40f);
      const bool timeout = (sampleIdx_ - sStart_) > maxSearchSamples_;
      if (fell || timeout) {
        searching_ = false;
        const int64_t peakQ = quantisedPeak();
        if (lastPeakQ_ < 0) {
          lastPeakQ_ = peakQ;
          spki_ = 0.125f * sMax_ + 0.875f * spki_;
        } else {
          const int64_t dq = peakQ - lastPeakQ_;
          if (dq >= (int64_t)refractorySamples_ * 256) {
            lastRRms_  = (float)dq / 256.0f * 1000.0f / fs_;
            rrValid_   = true;
            lastPeakQ_ = peakQ;
            spki_      = 0.125f * sMax_ + 0.875f * spki_;
            detected   = true;
          }
          // inside refractory -> discard, leave estimates untouched
        }
      }
    }

    y1_ = y0;
    return detected;
  }

  bool  rrValid() const { return rrValid_; }
  float lastRR()  const { return lastRRms_; }    // milliseconds

private:
  void pushMwi(float v) {
    mwiSum_ -= mwi_[mwiIdx_];
    mwi_[mwiIdx_] = v;
    mwiSum_ += v;
    mwiIdx_ = (mwiIdx_ + 1) % mwiN_;
  }
  // Parabolic vertex through the tracked maximum and its two neighbours.
  int64_t quantisedPeak() const {
    const float denom = sLeft_ - 2.0f * sMax_ + sRight_;
    float off = 0.0f;
    if (fabsf(denom) > 1e-9f) {
      off = 0.5f * (sLeft_ - sRight_) / denom;
      if (off >  0.5f) off =  0.5f;
      if (off < -0.5f) off = -0.5f;
    }
    return (int64_t)sIdx_ * 256 + (int64_t)(off * 256.0f);
  }
};

// =============================================================================
//  SECTION 4 -- Heart rate variability
// =============================================================================
//
// Standard time-domain HRV over a rolling window of R-R intervals.
// Ectopic / artefact intervals are rejected against the running median before
// they can pollute RMSSD -- a single missed beat would otherwise double it.
template <int N>
class HRVCalculator {
  float    rr_[N];
  int      idx_, count_;
  uint32_t rejected_;
public:
  HRVCalculator() { reset(); }
  void reset() {
    for (int i = 0; i < N; i++) rr_[i] = 0.0f;
    idx_ = 0; count_ = 0; rejected_ = 0;
  }

  int      count()    const { return count_; }
  uint32_t rejected() const { return rejected_; }
  bool     valid()    const { return count_ >= 30; }   // ~30 s at 60 bpm

  // Returns false if the interval was rejected as an artefact.
  bool addInterval(float rr_ms) {
    if (rr_ms < 300.0f || rr_ms > 2000.0f) { rejected_++; return false; }
    if (count_ >= 5) {
      const float med = median();
      if (fabsf(rr_ms - med) > 0.20f * med) { rejected_++; return false; }
    }
    rr_[idx_] = rr_ms;
    idx_ = (idx_ + 1) % N;
    if (count_ < N) count_++;
    return true;
  }

  float meanRR() const {
    if (!count_) return NAN;
    float s = 0.0f;
    for (int i = 0; i < count_; i++) s += rr_[i];
    return s / (float)count_;
  }
  float meanHR() const {
    const float m = meanRR();
    return isnan(m) ? NAN : 60000.0f / m;
  }
  float sdnn() const {
    if (count_ < 2) return NAN;
    const float m = meanRR();
    float s = 0.0f;
    for (int i = 0; i < count_; i++) { const float d = rr_[i] - m; s += d * d; }
    return sqrtf(s / (float)(count_ - 1));
  }
  // RMSSD needs SUCCESSIVE differences, so it walks the ring in insertion order.
  float rmssd() const {
    if (count_ < 2) return NAN;
    float s = 0.0f; int n = 0;
    for (int k = 1; k < count_; k++) {
      const int a = ringPos(k - 1), b = ringPos(k);
      const float d = rr_[b] - rr_[a];
      s += d * d; n++;
    }
    return n ? sqrtf(s / (float)n) : NAN;
  }
  float pnn50() const {
    if (count_ < 2) return NAN;
    int hits = 0, n = 0;
    for (int k = 1; k < count_; k++) {
      const int a = ringPos(k - 1), b = ringPos(k);
      if (fabsf(rr_[b] - rr_[a]) > 50.0f) hits++;
      n++;
    }
    return n ? (100.0f * hits / (float)n) : NAN;
  }

private:
  // k = 0 is the oldest retained interval.
  int ringPos(int k) const {
    if (count_ < N) return k;
    return (idx_ + k) % N;
  }
  float median() const {
    const int n = count_ < 9 ? count_ : 9;
    float tmp[9];
    for (int k = 0; k < n; k++) tmp[k] = rr_[ringPos(count_ - n + k)];
    for (int i = 1; i < n; i++) {           // insertion sort, n <= 9
      const float v = tmp[i];
      int j = i - 1;
      while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
      tmp[j + 1] = v;
    }
    return tmp[n / 2];
  }
};

// =============================================================================
//  SECTION 5 -- Respiration rate (RIIV from the PPG baseline)
// =============================================================================
//
// Breathing modulates venous return, which shifts the PPG DC baseline by a few
// tenths of a percent at 0.1-0.5 Hz. This is the least robust output in the
// system: it needs a still subject and roughly a minute of clean signal.
// It reports valid() only after several consistent breath intervals.
class RespirationEstimator {
  BandPassFilter bp_;
  float    prev_, prevPrev_;
  uint32_t lastPeakMs_;
  float    intervals_[6];
  int      idx_, count_;
  DecayingMinMax env_;
  float    fs_;
public:
  explicit RespirationEstimator(float fs_hz)
    : bp_(0.10f, 0.50f, fs_hz), prev_(0.0f), prevPrev_(0.0f),
      lastPeakMs_(0), idx_(0), count_(0), env_(0.001f), fs_(fs_hz) {
    for (int i = 0; i < 6; i++) intervals_[i] = 0.0f;
  }
  void reset() {
    bp_.reset(); env_.reset();
    prev_ = prevPrev_ = 0.0f; lastPeakMs_ = 0; idx_ = 0; count_ = 0;
  }

  // Feed the PPG DC baseline (not the AC component) at fs_hz.
  void process(float baseline, uint32_t now_ms) {
    const float x = bp_.process(baseline);
    env_.process(x);
    if (env_.range() < 1e-4f) { prevPrev_ = prev_; prev_ = x; return; }

    const float thr      = env_.maximum() * 0.35f;
    const bool  localMax = (prev_ >= x) && (prev_ > prevPrev_);

    if (localMax && prev_ > thr &&
        (lastPeakMs_ == 0 || (now_ms - lastPeakMs_) >= 1800)) {   // >= 1.8 s
      if (lastPeakMs_ != 0) {
        const float dt = (float)(now_ms - lastPeakMs_);
        if (dt <= 12000.0f) {                                     // >= 5 br/min
          intervals_[idx_] = dt;
          idx_ = (idx_ + 1) % 6;
          if (count_ < 6) count_++;
        } else {
          count_ = 0;                                             // gap: restart
        }
      }
      lastPeakMs_ = now_ms;
    }
    prevPrev_ = prev_;
    prev_     = x;
  }

  // Valid only with 4+ breaths AND low dispersion between them.
  bool valid() const {
    if (count_ < 4) return false;
    float mn = intervals_[0], mx = intervals_[0];
    for (int i = 1; i < count_; i++) {
      if (intervals_[i] < mn) mn = intervals_[i];
      if (intervals_[i] > mx) mx = intervals_[i];
    }
    return (mx - mn) < 0.40f * mx;
  }
  float breathsPerMinute() const {
    if (count_ < 1) return NAN;
    float s = 0.0f;
    for (int i = 0; i < count_; i++) s += intervals_[i];
    return 60000.0f / (s / (float)count_);
  }
  int breathCount() const { return count_; }
};

// =============================================================================
//  SECTION 6 -- SpO2 ratio, with an explicit uncalibrated state
// =============================================================================
//
// R = (AC_red / DC_red) / (AC_ir / DC_ir), then SpO2 = a - b*R.
//
// There is no honest way to pick a and b without measuring this specific sensor
// against a reference oximeter. Defaults below are LITERATURE PLACEHOLDERS.
// Until calibrate() is called with real paired samples, calibrated() is false
// and the UI must mark the reading as an estimate.
class SpO2Estimator {
  float    a_, b_;
  bool     calibrated_;
  MovingAverageFilter<8> rFilter_;
  float    lastR_;
  uint32_t beats_;
public:
  SpO2Estimator() : a_(110.0f), b_(25.0f), calibrated_(false),
                    lastR_(NAN), beats_(0) {}

  void setCoefficients(float a, float b, bool is_calibrated) {
    a_ = a; b_ = b; calibrated_ = is_calibrated;
  }
  bool  calibrated() const { return calibrated_; }
  float coefA()      const { return a_; }
  float coefB()      const { return b_; }
  float lastRatio()  const { return lastR_; }
  uint32_t beats()   const { return beats_; }
  void reset() { rFilter_.reset(); lastR_ = NAN; beats_ = 0; }

  // Feed one cardiac cycle's worth of statistics. Returns NAN if unusable.
  float processBeat(float ac_red, float dc_red, float ac_ir, float dc_ir) {
    if (!(dc_red > 0.0f) || !(dc_ir > 0.0f)) return NAN;
    if (!(ac_red > 0.0f) || !(ac_ir > 0.0f)) return NAN;

    const float num = ac_red / dc_red;
    const float den = ac_ir  / dc_ir;
    if (!(den > 1e-9f)) return NAN;

    const float r = num / den;
    if (r < 0.30f || r > 3.00f) return NAN;      // outside any plausible range

    lastR_ = rFilter_.process(r);
    beats_++;
    if (rFilter_.count() < 4) return NAN;        // need a few beats to settle

    const float spo2 = a_ - b_ * lastR_;
    if (spo2 < 60.0f || spo2 > 100.0f) return NAN;
    return spo2;
  }
};

// =============================================================================
//  SECTION 7 -- MQ3 relative alcohol response
// =============================================================================
//
// This sensor CANNOT report blood alcohol content. BAC requires calibration
// against a certified breathalyser plus a breath-delivery protocol, neither of
// which exists here. What it can honestly report is Rs/R0 -- the sensor
// resistance relative to its own clean-air baseline. Values below ~1.0 mean
// "something reducing is present"; the magnitude is relative, not absolute.
//
// Also enforces a warm-up gate. The heater needs minutes from cold, and a
// sensor stored unpowered for months needs far longer to settle.
class MQ3Monitor {
  uint32_t warmupMs_;
  uint32_t bootMs_;
  bool     baselineSet_;
  float    r0Adc_;
  MovingAverageFilter<16> smooth_;
  MovingAverageFilter<20> baselineTracker_;
  float    lastAdc_;
public:
  explicit MQ3Monitor(uint32_t warmup_ms = 90000UL)
    : warmupMs_(warmup_ms), bootMs_(0), baselineSet_(false),
      r0Adc_(NAN), lastAdc_(NAN) {}

  void begin(uint32_t now_ms) { bootMs_ = now_ms; }

  bool warmedUp(uint32_t now_ms) const { return (now_ms - bootMs_) >= warmupMs_; }
  uint32_t warmupRemainingS(uint32_t now_ms) const {
    const uint32_t e = now_ms - bootMs_;
    return (e >= warmupMs_) ? 0 : (warmupMs_ - e + 999) / 1000;
  }
  bool  baselineSet() const { return baselineSet_; }
  int   baselineProgress() const { return baselineTracker_.count(); }
  static int baselineNeeded() { return 20; }
  float baselineAdc() const { return r0Adc_; }
  float lastAdc()     const { return lastAdc_; }

  void process(int adc, uint32_t now_ms) {
    lastAdc_ = smooth_.process((float)adc);
    baselineTracker_.process((float)adc);
    if (!baselineSet_ && warmedUp(now_ms) && baselineTracker_.full()) {
      r0Adc_ = baselineTracker_.value();          // auto-zero in clean air
      baselineSet_ = true;
    }
  }
  void rezero() { baselineSet_ = false; baselineTracker_.reset(); r0Adc_ = NAN; }
  void forceBaseline(float adc) { r0Adc_ = adc; baselineSet_ = true; }

  // Rs/R0 assuming a fixed load resistor: Rs proportional to (FS - adc)/adc.
  // The load value cancels in the ratio, so no board-specific constant needed.
  float ratio() const {
    if (!baselineSet_ || isnan(lastAdc_)) return NAN;
    if (lastAdc_ < 1.0f || r0Adc_ < 1.0f)  return NAN;
    const float rsNow  = (4095.0f - lastAdc_) / lastAdc_;
    const float rsBase = (4095.0f - r0Adc_)   / r0Adc_;
    if (!(rsBase > 1e-6f)) return NAN;
    return rsNow / rsBase;
  }
};

// =============================================================================
//  SECTION 8 -- MLX90614 surface temperature correction
// =============================================================================
//
// The standard MLX90614 has a ~90 degree field of view, so at any realistic
// distance it averages the target with whatever is behind it:
//
//     T_measured = f * T_target + (1 - f) * T_ambient
//  => T_target   = (T_measured - (1 - f) * T_ambient) / f
//
// f is the fraction of the field filled by skin -- distance dependent. This is
// why a FIXED offset appears to work at one distance and drifts everywhere
// else: a +2.8 C offset is what this model produces at f ~ 0.76 in a typical
// room, and nowhere else.
//
// This reports SURFACE temperature. Skin is not core body temperature, and no
// amount of correction here changes that.
class SkinTempEstimator {
  float fillFraction_;
  MovingAverageFilter<8> smooth_;
  DecayingMinMax stability_;
  int  settle_;
public:
  explicit SkinTempEstimator(float fill_fraction = 0.78f)
    : fillFraction_(fill_fraction), stability_(0.02f), settle_(0) {}

  void  setFillFraction(float f) { if (f > 0.05f && f <= 1.0f) fillFraction_ = f; }
  float fillFraction() const { return fillFraction_; }
  void  reset() { smooth_.reset(); stability_.reset(); settle_ = 0; }

  // Solve for f given a known reference surface temperature.
  static float solveFillFraction(float t_measured, float t_ambient, float t_reference) {
    const float denom = t_reference - t_ambient;
    if (fabsf(denom) < 0.5f) return NAN;         // too little contrast to fit
    const float f = (t_measured - t_ambient) / denom;
    return (f > 0.05f && f <= 1.0f) ? f : NAN;
  }

  // Returns NAN until the reading is both plausible and stable.
  float process(float t_object, float t_ambient) {
    if (isnan(t_object) || isnan(t_ambient)) { settle_ = 0; return NAN; }
    if (t_object < 10.0f || t_object > 50.0f) { settle_ = 0; return NAN; }

    const float corrected = (t_object - (1.0f - fillFraction_) * t_ambient) / fillFraction_;
    if (corrected < 20.0f || corrected > 45.0f) { settle_ = 0; return NAN; }

    const float s = smooth_.process(corrected);
    stability_.process(corrected);
    if (settle_ < 6) settle_++;
    if (settle_ < 6) return NAN;
    if (stability_.range() > 1.5f) return NAN;   // still drifting / moving target
    return s;
  }
};

// =============================================================================
//  SECTION 9 -- Supply rail proxy via the AD8232 reference level
// =============================================================================
//
// The AD8232 output idles at VS/2, and the ESP32 ADC references an internal
// bandgap rather than the rail. So the ECG DC baseline tracks the supply:
//
//     V_rail = 2 * (baseline_counts / 4095) * ADC_full_scale
//
// ADC_full_scale is nominally 3.3 V at 11 dB attenuation but varies part to
// part, and the ESP32 ADC is noticeably non-linear above ~3.1 V. Calibrate the
// scale factor once against a multimeter; until then treat this as a TREND
// indicator, not a voltmeter.
class RailProxy {
  LowPassFilter dc_;
  float    scale_;
  bool     valid_;
  uint32_t lastValidMs_;
public:
  RailProxy(float fs_hz, float volts_full_scale = 3.30f)
    : dc_(0.25f, fs_hz), scale_(volts_full_scale), valid_(false), lastValidMs_(0) {}

  void  setScale(float v) { if (v > 1.0f && v < 6.0f) scale_ = v; }
  float scale() const { return scale_; }

  // Feed raw ECG counts, but ONLY while the leads are attached and settled --
  // with leads off the AD8232 output is not a reliable reference level.
  void process(int adc_counts, bool leads_on, uint32_t now_ms) {
    if (!leads_on) return;
    const float b = dc_.process((float)adc_counts);
    if (b > 500.0f && b < 3500.0f) { valid_ = true; lastValidMs_ = now_ms; }
  }
  bool  valid()    const { return valid_; }
  float baseline() const { return dc_.value(); }
  float volts()    const {
    if (!valid_ || !dc_.ready()) return NAN;
    return 2.0f * (dc_.value() / 4095.0f) * scale_;
  }
  uint32_t ageMs(uint32_t now_ms) const { return now_ms - lastValidMs_; }
  void reset() { dc_.reset(); valid_ = false; }
};

// =============================================================================
//  SECTION 10 -- Validity wrapper
// =============================================================================
//
// Wraps any scalar so a stale reading cannot survive on screen. Once a value
// ages past its timeout it reverts to invalid, and the UI prints "--".
class TimedValue {
  float    v_;
  uint32_t stampMs_, timeoutMs_;
  bool     valid_;
public:
  explicit TimedValue(uint32_t timeout_ms = 8000UL)
    : v_(NAN), stampMs_(0), timeoutMs_(timeout_ms), valid_(false) {}

  void set(float v, uint32_t now_ms) {
    if (isnan(v)) return;
    v_ = v; stampMs_ = now_ms; valid_ = true;
  }
  void invalidate() { valid_ = false; v_ = NAN; }

  bool valid(uint32_t now_ms) const {
    return valid_ && (now_ms - stampMs_) < timeoutMs_;
  }
  float value(uint32_t now_ms) const { return valid(now_ms) ? v_ : NAN; }
  float raw() const { return v_; }
};

#endif  // FILTERS_H
