  // =============================================================================
//  MULTI-VITAL MONITOR  --  ESP32
//
//  Board    : ESP32 devkit (classic, dual core)
//  Display  : 320x240 SPI TFT via TFT_eSPI
//             CS 5   DC 16   RST 17   SCK 18   MOSI 15
//  MAX30102 : I2C bus 0    SDA 21   SCL 22       (HR, SpO2, respiration)
//  MLX90614 : I2C bus 1    SDA 25   SCL 26       (surface temperature)
//  AD8232   : OUT 33   LO+ 35   LO- 32           (ECG, HRV)
//  MQ3      : AOUT 39                            (relative alcohol response)
//
//  Libraries: TFT_eSPI, MAX3010x (eepj), Adafruit_MLX90614
//
//  HARDWARE IS UNCHANGED FROM THE ORIGINAL BUILD. This is a firmware-only
//  rewrite. No pin moved, nothing added.
//
// -----------------------------------------------------------------------------
//  HONESTY POLICY -- read this before changing anything on screen
// -----------------------------------------------------------------------------
//  Every displayed number is either measured or absent. There is no smoothing
//  of missing data into something plausible, no last-known-good value left on
//  screen to age quietly, and no derived figure presented as more certain than
//  its inputs.
//
//    * A metric with no valid measurement shows "--". Never 0, never stale.
//    * SpO2 is marked EST until calibrated against a reference oximeter.
//      The default coefficients are literature placeholders, not measurements.
//    * Alcohol is reported as Rs/R0 -- resistance relative to this sensor's own
//      clean-air baseline. It is NOT blood alcohol content and cannot be
//      without a certified breathalyser to calibrate against.
//    * Temperature is SURFACE temperature. Skin is not core body temperature.
//    * HRV appears only after 30 accepted R-R intervals, and only while the
//      ECG signal quality gate is satisfied.
//    * Respiration is the least reliable output here and says so.
//    * Measured sample rates are shown on screen. If they drift from 250/100,
//      the filter corner frequencies are wrong and every number is suspect.
//
//  The previous firmware's ECG "Critical / Danger / Stable" indicator has been
//  removed rather than fixed. It classified the 100 ms sample-to-sample delta,
//  which labels a normal QRS complex as Critical and a flatline as Stable --
//  exactly backwards. There is no honest version of that display, so it is gone.
// =============================================================================

#include <SPI.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MAX3010x.h>
#include <Adafruit_MLX90614.h>
#include <Preferences.h>
#include <ctype.h>
#include <esp_system.h>
#include "filters.h"

// =============================================================================
//  CONFIGURATION
// =============================================================================

// --- Pins (unchanged) ---
#define MAX_SDA   21
#define MAX_SCL   22
#define MLX_SDA   25
#define MLX_SCL   26
#define MQ3_PIN   39
#define ECG_OUT   33
#define LO_PLUS   35
#define LO_MINUS  32
// NOTE: the original sketch drove ECG_SDN on GPIO 34. GPIO 34 is input-only on
// the classic ESP32 -- it has no output driver, so that write never did
// anything. The AD8232 breakout holds SDN high by itself, which is why the ECG
// worked regardless. The pin is deliberately not declared here.

// --- Sample rates ---
#define ECG_FS_HZ      250.0f     // ECG acquisition, fixed by the task tick
#define ECG_PERIOD_MS  4          // 1000 / 250
#define PPG_FS_HZ      100.0f     // MAX30102 FIFO rate
#define MAINS_HZ       50.0f      // 50 for India/EU, 60 for the Americas

// --- Detection thresholds ---
#define FINGER_IR_MIN     20000.0f   // MAX30102 IR DC floor for "finger present"
#define PPG_PEAK_FRACTION 0.55f      // of tracked envelope
#define PPG_REFRACTORY_MS 300        // caps detection at 200 bpm
#define PPG_MIN_AMPLITUDE 20.0f      // absolute AC floor, counts
#define ECG_MIN_QRS_AMP   25.0f      // band-passed QRS floor, counts
#define MIN_PERFUSION_IDX 0.05f      // below this, PPG numbers are not reported

// --- Display geometry -------------------------------------------------------
// ROTATION 1 and 3 are both 320x240 (3 is 1 flipped 180 degrees). Switching
// between those two needs NOTHING below changed -- edit TFT_ROTATION only.
// ROTATION 0 and 2 are 240x320 portrait: set LAYOUT_PORTRAIT to 1 and every
// dimension below re-derives. Nothing else in the file hardcodes a pixel.
#define TFT_ROTATION      2  //3
#define LAYOUT_PORTRAIT   2

#if LAYOUT_PORTRAIT
  #define SCR_W     240
  #define SCR_H     320
#else
  #define SCR_W     320
  #define SCR_H     240
#endif

#define BAR_H        13
#if LAYOUT_PORTRAIT
  #define BLK_H      70          // 320 px of height buys taller blocks
#else
  #define BLK_H      62
#endif
#define BLK_W       ((SCR_W - 4) / 2)
#define BLK_STATUS_Y (BLK_H - 18)

// Blocks narrow from 158 to 118 px in portrait, so the thin row drops to the
// small font -- at font 2 a value like "29/30" collides with its own label.
#if LAYOUT_PORTRAIT
  #define THIN_FONT  1
#else
  #define THIN_FONT  2
#endif
#define R1_Y         15
#define R2_Y        (R1_Y + BLK_H + 2)
#define R3_Y        (R2_Y + BLK_H + 2)
#define R3_H         22
#define THIN_W      ((SCR_W - 2) / 3)
#define THIN_X1       0
#define THIN_X2     (THIN_W + 1)
#define THIN_X3     (2 * (THIN_W + 1))
#define ECG_Y       (R3_Y + R3_H + 2)
#define ECG_H       (SCR_H - ECG_Y)
#define ECG_SPR_W   (SCR_W - 4)
#define ECG_SPR_H   (ECG_H - 4)

#define COL_BG       TFT_BLACK
#define COL_BORDER   0x39C7
#define COL_LABEL    0x7BEF
#define COL_GOOD     TFT_GREEN
#define COL_WARN     0xFD20
#define COL_BAD      TFT_RED
#define COL_IDLE     0x7BEF
#define COL_INFO     0x05FF
#define COL_TRACE    TFT_GREEN
#define COL_GRID     0x18C3
#define COL_GRIDC    0x3186
// Horizontal division spacing, chosen to match the original firmware's look.
#define ECG_HGRID    22
// Vertical time divisions. Off by default -- the original had none and that is
// the look being matched. Set to 1 for a full grid; spacing is 16 columns,
// which at 83.3 columns/s is 192 ms, NOT the 200 ms a clinical monitor uses.
#define ECG_SHOW_VGRID 0
#define ECG_VGRID    16

// =============================================================================
//  OBJECTS
// =============================================================================

TFT_eSPI    tft = TFT_eSPI();
// Sweep rendering. The old version rebuilt and pushed the whole 236x133 sprite
// every frame -- 62 KB over SPI (~18 ms) just to shift the picture left by 3.33
// columns. Non-integer, so the trace lurched 3,3,4,3,3,4 px per frame. A narrow
// segment sprite draws only the new columns plus a blank gap ahead, leaves the
// rest of the trace untouched, and costs 4 KB instead of 62 KB.
#define ECG_SEG_W  8
#define ECG_GAP_W  8
#define ECG_SEG_FULL (ECG_SEG_W + ECG_GAP_W)
TFT_eSprite ecgSeg = TFT_eSprite(&tft);
bool        ecgSegOk = false;

MAX30105              pulseSensor;
Adafruit_MLX90614     mlx = Adafruit_MLX90614();
Preferences           prefs;

// --- ECG chain ---
RPeakDetector  rpeak(ECG_FS_HZ, MAINS_HZ, ECG_MIN_QRS_AMP);
NotchFilter    ecgDispNotch(MAINS_HZ, ECG_FS_HZ);
LowPass2       ecgDispLp(30.0f, ECG_FS_HZ);
// Baseline tracker, NOT a high-pass. The AD8232 already high-passes internally,
// so its output arrives centred near VS/2 -- which is why the original firmware
// could map raw counts straight to pixels and look right. A biquad high-pass
// here is redundant AND starts from zero state, so it rings for a second on a
// 2048-count step and poisons anything measuring amplitude. A one-pole low-pass
// initialises to its first sample instead, so it has no start-up transient at
// all; subtracting it recentres the trace with nothing to ring.
// 0.20 Hz. Fast enough to converge before the gain is measured, slow enough to
// leave the ST segment and T wave intact. The AD8232 has already removed
// everything below ~0.5 Hz, so this only has to catch the residual VS/2 offset,
// which moves with the supply rail.
LowPassFilter  ecgBase(0.20f, ECG_FS_HZ);
// Measured over seconds 6-11 after the leads connect, then held. The wait is
// deliberate: fitting earlier means measuring the baseline tracker converging
// rather than the heartbeat, which is what produced a two-pixel-tall trace.
EcgGain        ecgGain(ECG_FS_HZ, ((ECG_SPR_H / 2) - 4) * 0.84f, 6.0f, 5.0f);
HRVCalculator<128> hrv;
RailProxy      rail(ECG_FS_HZ);

// --- PPG chain ---
LowPassFilter  ppgDcIr(0.5f, PPG_FS_HZ);
LowPassFilter  ppgDcRed(0.5f, PPG_FS_HZ);
BandPassFilter ppgAcIr(0.5f, 5.0f, PPG_FS_HZ);
BandPassFilter ppgAcRed(0.5f, 5.0f, PPG_FS_HZ);
AmplitudePeakDetector ppgPeak(PPG_PEAK_FRACTION, PPG_MIN_AMPLITUDE, PPG_REFRACTORY_MS);
WindowStatistic ppgBeatIr, ppgBeatRed;
SpO2Estimator   spo2Est;
RespirationEstimator respEst(PPG_FS_HZ);
MovingAverageFilter<6> ppgBpmAvg;

// --- Environment ---
SkinTempEstimator skinTemp;
MQ3Monitor        mq3(90000UL);

// =============================================================================
//  SHARED STATE  (written by acquisition on core 0, read by render on core 1)
// =============================================================================

struct Metrics {
  TimedValue hrEcg{6000};
  TimedValue hrPpg{6000};
  TimedValue spo2{8000};
  TimedValue tempC{8000};
  TimedValue mqRatio{5000};
  TimedValue rmssd{20000};
  TimedValue sdnn{20000};
  TimedValue respRate{45000};
  TimedValue perfusion{5000};
  TimedValue railV{15000};

  bool  fingerOn     = false;
  bool  spo2Cal      = false;
  bool  mqWarm       = false;
  uint32_t mqWarmLeft = 0;
  int   mqBaseN      = 0;
  int   hrvCount     = 0;
  float ecgRateMeas  = 0.0f;
  float ppgRateMeas  = 0.0f;
};

Metrics          M;
SemaphoreHandle_t mLock;

// Single writer (acquisition), single reader (render), word-sized: no lock
// needed, and they must refresh every tick rather than every 2 s so the
// "leads off" banner is not up to two seconds stale.
volatile bool g_leadsOn     = false;
volatile bool g_ecgSignalOk = false;

// --- ECG display ring (single producer, single consumer) ---
#define ECG_DECIM 3
#define ECG_RING  512                 // power of two
// One value per column, in a proper single-producer / single-consumer FIFO.
// Monotonic head published last, so the render task never reads a half-written
// column.
int16_t  ecgRing[ECG_RING];
volatile uint32_t ecgHead = 0;        // producer (ecgTask)
uint32_t          ecgTail = 0;        // consumer (renderTask only)

// --- Boot diagnostics ---
struct BootInfo {
  esp_reset_reason_t resetReason;
  bool maxOk = false;
  bool mlxOk = false;
  uint32_t freeHeap = 0;
} boot;

volatile bool rawStream = false;

// =============================================================================
//  SMALL HELPERS
// =============================================================================

const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_EXT:      return "external";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "int watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "sdio";
    default:               return "unknown";
  }
}

inline void lockM()   { xSemaphoreTake(mLock, portMAX_DELAY); }
inline void unlockM() { xSemaphoreGive(mLock); }

// Explicit prototypes. The Arduino build normally injects these itself, but
// relying on that hides ordering mistakes and breaks the moment the file is
// compiled by anything else.
void processPpgSample(float red, float ir, uint32_t now);
void readEnvironment(uint32_t now);
void ecgTask(void*);
void sensorTask(void*);
void renderTask(void*);
enum EcgView { EV_NONE, EV_LEADS_OFF, EV_NO_SIGNAL, EV_TRACE };
void drawEcg(uint32_t now);
void ecgSegBackground(int absStart, int width);
void ecgTftBackground(int absStart, int width);
void ecgPaintBackground();
void ecgEraseAhead(int startCol, int width);
void ecgBanner(const char* msg, uint16_t colour);
void drawNumbers(uint32_t now);
void printHelp();
void printStatus();
void fitSpO2();
void loadCalibration();
void handleSerial();
void drawStaticChrome();
void showBootScreen();

// =============================================================================
//  ACQUISITION TASK  (core 0)
//
//  Runs on a fixed 4 ms tick. Everything time-critical lives here and NOTHING
//  here touches the display -- that separation is the whole point. In the
//  original firmware the sample loop was paced by SPI writes, so the true rate
//  was roughly 100-150 Hz while every filter was designed for 400 Hz, putting
//  each corner frequency about 2.7x lower than intended.
// =============================================================================

void ecgTask(void*) {
  TickType_t nextWake = xTaskGetTickCount();
  TickType_t period   = pdMS_TO_TICKS(ECG_PERIOD_MS);
  // If the FreeRTOS tick is slower than 250 Hz, pdMS_TO_TICKS(4) truncates to
  // zero and vTaskDelayUntil stops blocking entirely. Clamp so the task can
  // never busy-spin; the measured rate on screen will show the real cadence.
  if (period == 0) period = 1;

  uint32_t count = 0, stamp = millis();
  int   decimN = 0;
  float decAcc = 0.0f;

  for (;;) {
    vTaskDelayUntil(&nextWake, period);
    const uint32_t now = millis();

    // NOTHING in this task may touch I2C or SPI. The previous version drained
    // the MAX30102 FIFO here, and readSample() blocks ~10 ms waiting on the
    // sensor -- four of those per iteration paced the whole task at 25 Hz
    // instead of 250, which put every ECG filter corner 10x too low.
    const bool leadsOn = !(digitalRead(LO_PLUS) || digitalRead(LO_MINUS));
    const int  rawEcg  = analogRead(ECG_OUT);
    count++;
    g_leadsOn     = leadsOn;
    g_ecgSignalOk = leadsOn && rpeak.signalPresent();

    rail.process(rawEcg, leadsOn, now);

    if (leadsOn) {
      if (rpeak.process((float)rawEcg)) {
        const float rr = rpeak.lastRR();
        if (hrv.addInterval(rr)) {
          lockM();
          M.hrEcg.set(60000.0f / rr, now);
          if (hrv.valid()) { M.rmssd.set(hrv.rmssd(), now); M.sdnn.set(hrv.sdnn(), now); }
          M.hrvCount = hrv.count();
          unlockM();
        }
      }

      const float f = ecgDispLp.process(ecgDispNotch.process((float)rawEcg));
      const float d = f - ecgBase.process(f);         // recentre, no ringing
      ecgGain.process(d);
      decAcc += d;
      if (++decimN >= ECG_DECIM) {
        decimN = 0;
        const float avg = decAcc / (float)ECG_DECIM;
        decAcc = 0.0f;
        if (!ecgGain.ready()) continue;               // baseline still settling
        // Fixed counts-per-pixel, exactly like the original firmware's map().
        // Margin of 4 px so a full-scale excursion stops short of the border.
        const int lim = (ECG_SPR_H / 2) - 4;
        ecgRing[ecgHead & (ECG_RING - 1)] =
            (int16_t)constrain((int)(-avg / ecgGain.gain()), -lim, lim);
        ecgHead++;
      }
    } else {
      rpeak.reset(); ecgDispLp.reset(); ecgDispNotch.reset(); ecgBase.reset();
      hrv.reset(); ecgGain.refit();          // re-measure on every reconnection
      decimN = 0; decAcc = 0.0f;
      lockM();
      M.hrEcg.invalidate(); M.rmssd.invalidate(); M.sdnn.invalidate();
      M.hrvCount = 0;
      unlockM();
    }

    if (now - stamp >= 2000) {
      const float dt = (now - stamp) / 1000.0f;
      lockM();
      M.ecgRateMeas = count / dt;
      const float v = rail.volts();
      if (!isnan(v)) M.railV.set(v, now);
      unlockM();
      count = 0; stamp = now;
    }
  }
}

// All I2C lives here: the MAX30102 FIFO plus the 1 Hz environment sensors.
// This task is paced by the PPG sensor itself, which is exactly what we want --
// and it can no longer drag the ECG clock down with it.
void sensorTask(void*) {
  uint32_t ppgCount = 0, rateStamp = millis(), lastSlow = millis();
  mq3.begin(millis());

  for (;;) {
    auto sample = pulseSensor.readSample(20);
    const uint32_t now = millis();

    if (sample.valid) {
      ppgCount++;
      processPpgSample((float)sample.red, (float)sample.ir, now);
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));      // sensor absent or idle: never spin
    }

    // Wall-clock scheduling, not a loop counter. The old `if (++slowDiv >= 250)`
    // silently became a 10 s cadence the moment the loop rate changed, which is
    // what stranded the MQ3 baseline and the temperature settle counter.
    if (now - lastSlow >= 1000) { lastSlow = now; readEnvironment(now); }

    if (now - rateStamp >= 2000) {
      const float dt = (now - rateStamp) / 1000.0f;
      lockM(); M.ppgRateMeas = ppgCount / dt; unlockM();
      ppgCount = 0; rateStamp = now;
    }
  }
}

// --- One MAX30102 sample -----------------------------------------------------
void processPpgSample(float red, float ir, uint32_t now) {
  const float dcIr  = ppgDcIr.process(ir);
  const float dcRed = ppgDcRed.process(red);

  if (dcIr < FINGER_IR_MIN) {                    // no finger
    ppgAcIr.reset(); ppgAcRed.reset(); ppgPeak.reset();
    ppgBeatIr.reset(); ppgBeatRed.reset();
    spo2Est.reset(); respEst.reset(); ppgBpmAvg.reset();
    lockM();
    M.fingerOn = false;
    M.hrPpg.invalidate();
    M.spo2.invalidate();
    M.perfusion.invalidate();
    M.respRate.invalidate();
    unlockM();
    return;
  }

  const float acIr  = ppgAcIr.process(ir);
  const float acRed = ppgAcRed.process(red);
  ppgBeatIr.process(acIr);
  ppgBeatRed.process(acRed);
  respEst.process(dcIr, now);

  // The optical pulse is a DOWNWARD deflection: more blood absorbs more light,
  // so the raw count FALLS at systole. Peak detection therefore runs on the
  // inverted AC signal. Detecting on the raw sign finds the diastolic recoil
  // instead and undercounts by roughly 10-20% at normal rates.
  const bool beat = ppgPeak.process(-acIr, now);

  // Perfusion index: pulsatile fraction of the DC level. This is the honest
  // gate on whether any PPG-derived number is worth showing.
  const float pi = (dcIr > 0.0f) ? (ppgPeak.amplitude() / dcIr * 100.0f) : 0.0f;

  lockM();
  M.fingerOn = true;
  M.perfusion.set(pi, now);
  unlockM();

  if (!beat) return;

  static uint32_t lastBeatMs = 0;
  if (lastBeatMs && pi >= MIN_PERFUSION_IDX) {
    const float bpm = 60000.0f / (float)(now - lastBeatMs);
    if (bpm >= 30.0f && bpm <= 220.0f) {
      const float sm = ppgBpmAvg.process(bpm);
      if (ppgBpmAvg.count() >= 3) { lockM(); M.hrPpg.set(sm, now); unlockM(); }
    }
  }
  lastBeatMs = now;

  if (pi >= MIN_PERFUSION_IDX) {
    const float v = spo2Est.processBeat(ppgBeatRed.peakToPeak(), dcRed,
                                        ppgBeatIr.peakToPeak(),  dcIr);
    if (!isnan(v)) { lockM(); M.spo2.set(v, now); M.spo2Cal = spo2Est.calibrated(); unlockM(); }
  }
  ppgBeatIr.reset();
  ppgBeatRed.reset();

  if (respEst.valid()) { lockM(); M.respRate.set(respEst.breathsPerMinute(), now); unlockM(); }
}

// --- MLX90614 + MQ3 at 1 Hz --------------------------------------------------
void readEnvironment(uint32_t now) {
  const float tObj = mlx.readObjectTempC();
  const float tAmb = mlx.readAmbientTempC();
  // The original code let NAN through: NAN + offset is NAN, and both
  // `NAN < 20.0` and `NAN > 45.0` evaluate false, so a failed I2C read printed
  // "nan" straight to the panel.
  const float t = (isnan(tObj) || isnan(tAmb)) ? NAN : skinTemp.process(tObj, tAmb);
  lockM();
  if (isnan(t)) M.tempC.invalidate(); else M.tempC.set(t, now);
  unlockM();

  mq3.process(analogRead(MQ3_PIN), now);
  const float r = mq3.ratio();
  lockM();
  M.mqWarm     = mq3.warmedUp(now) && mq3.baselineSet();
  M.mqWarmLeft = mq3.warmupRemainingS(now);
  M.mqBaseN    = mq3.baselineProgress();
  if (isnan(r)) M.mqRatio.invalidate(); else M.mqRatio.set(r, now);
  unlockM();

  if (rawStream) {
    Serial.printf("RAW t=%.2f amb=%.2f mq=%.0f rail=%.2f ecgRate=%.1f ppgRate=%.1f\n",
                  tObj, tAmb, mq3.lastAdc(), rail.volts(), M.ecgRateMeas, M.ppgRateMeas);
  }
}

// =============================================================================
//  RENDER TASK  (core 1)
// =============================================================================

struct Cache {
  char hr[12] = "", spo2[12] = "", temp[16] = "", alc[12] = "";
  char hrv[16] = "", resp[12] = "", pi[12] = "", bar[48] = "";
  uint16_t hrC = 0, spo2C = 0, tempC = 0, alcC = 0;
} cache;

void drawFrame(int x, int y, int w, int h) {
  tft.drawRoundRect(x, y, w, h, 4, COL_BORDER);
}

void drawBlock(int x, int y, const char* label, const char* unit,
               const char* value, const char* status, uint16_t colour,
               char* cacheVal, uint16_t* cacheCol) {
  if (!strcmp(cacheVal, value) && *cacheCol == colour) return;  // nothing changed
  strncpy(cacheVal, value, 11); cacheVal[11] = 0;
  *cacheCol = colour;

  tft.fillRect(x + 2, y + 11, BLK_W - 4, BLK_H - 13, COL_BG);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.drawString(label, x + 5, y + 2, 1);
  if (unit && *unit) {
    tft.setTextDatum(TR_DATUM);
    tft.drawString(unit, x + BLK_W - 5, y + 2, 1);
  }
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(colour, COL_BG);
  tft.drawString(value, x + 5, y + 13, 4);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.drawString(status, x + 5, y + BLK_STATUS_Y, 1);
}

void drawThin(int x, int w, const char* label, const char* value, char* cacheVal) {
  if (!strcmp(cacheVal, value)) return;
  strncpy(cacheVal, value, 11); cacheVal[11] = 0;
  tft.fillRect(x + 2, R3_Y + 2, w - 4, R3_H - 4, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.drawString(label, x + 5, R3_Y + 6, 1);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(TFT_WHITE, COL_BG);
  tft.drawString(value, x + w - 5, R3_Y + (THIN_FONT == 1 ? 7 : 5), THIN_FONT);
}

// Status colours describe MEASUREMENT CONFIDENCE, not clinical severity.
uint16_t hrColour(bool valid)   { return valid ? COL_GOOD : COL_IDLE; }

void renderTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(20);      // 50 Hz -- affordable now
  TickType_t nextWake = xTaskGetTickCount();
  uint32_t   lastSlow = 0;

  for (;;) {
    vTaskDelayUntil(&nextWake, period);
    const uint32_t now = millis();

    drawEcg(now);

    if (now - lastSlow >= 250) {                    // 4 Hz for the numbers
      lastSlow = now;
      drawNumbers(now);
    }
  }
}

// Grid: horizontal every 16 px, a brighter centreline, and vertical time
// divisions keyed to the ABSOLUTE column so they line up across sweeps.
void ecgSegBackground(int absStart, int width) {
  ecgSeg.fillSprite(COL_BG);
  for (int i = ECG_HGRID; i < ECG_SPR_H; i += ECG_HGRID)
    ecgSeg.drawFastHLine(0, i, width, COL_GRID);
  ecgSeg.drawFastHLine(0, ECG_SPR_H / 2, width, COL_GRIDC);
#if ECG_SHOW_VGRID
  for (int k = 0; k < width; k++)
    if (((absStart + k) % ECG_VGRID) == 0)
      ecgSeg.drawFastVLine(k, 0, ECG_SPR_H, COL_GRID);
#endif
}

void ecgTftBackground(int absStart, int width) {
  tft.fillRect(2 + absStart, ECG_Y + 2, width, ECG_SPR_H, COL_BG);
  for (int i = ECG_HGRID; i < ECG_SPR_H; i += ECG_HGRID)
    tft.drawFastHLine(2 + absStart, ECG_Y + 2 + i, width, COL_GRID);
  tft.drawFastHLine(2 + absStart, ECG_Y + 2 + ECG_SPR_H / 2, width, COL_GRIDC);
#if ECG_SHOW_VGRID
  for (int k = 0; k < width; k++)
    if (((absStart + k) % ECG_VGRID) == 0)
      tft.drawFastVLine(2 + absStart + k, ECG_Y + 2, ECG_SPR_H, COL_GRID);
#endif
}

void ecgPaintBackground() { ecgTftBackground(0, ECG_SPR_W); }

void ecgEraseAhead(int startCol, int width) {
  int c = startCol, rem = width;
  while (rem > 0) {
    if (c >= ECG_SPR_W) c -= ECG_SPR_W;
    int w = ECG_SPR_W - c;
    if (w > rem) w = rem;
    ecgTftBackground(c, w);
    rem -= w;
    c   += w;
  }
}

void ecgBanner(const char* msg, uint16_t colour) {
  tft.fillRect(2, ECG_Y + 2, ECG_SPR_W, ECG_SPR_H, COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(colour, COL_BG);
  tft.drawString(msg, 2 + ECG_SPR_W / 2, ECG_Y + 2 + ECG_SPR_H / 2, 2);
}

void drawEcg(uint32_t now) {
  static EcgView  shown      = EV_NONE;
  static int      cursor     = 0;
  static int      prevY      = -1;
  static uint32_t lastGoodMs = 0;

  // Leads physically detached.
  if (!g_leadsOn) {
    if (shown != EV_LEADS_OFF) { ecgBanner("ECG leads off", COL_BAD); shown = EV_LEADS_OFF; }
    ecgTail = ecgHead; cursor = 0; prevY = -1; lastGoodMs = 0;
    return;
  }

  // Leads attached but nothing cardiac coming through -- electrodes not on
  // skin, dried gel, bad contact. Say so instead of sweeping a flat line, which
  // is indistinguishable from a broken display.
  if (lastGoodMs == 0)  lastGoodMs = now;
  if (g_ecgSignalOk)    lastGoodMs = now;
  if (now - lastGoodMs > 4000) {
    if (shown != EV_NO_SIGNAL) { ecgBanner("no ECG signal - check pads", COL_WARN); shown = EV_NO_SIGNAL; }
    ecgTail = ecgHead; cursor = 0; prevY = -1;
    return;
  }

  if (shown != EV_TRACE) {
    ecgPaintBackground();
    shown = EV_TRACE; cursor = 0; prevY = -1; ecgTail = ecgHead;
  }
  if (!ecgSegOk) return;

  uint32_t head = ecgHead;
  if (head - ecgTail > ECG_RING) ecgTail = head - ECG_RING;   // overrun guard
  if (ecgTail == head) return;                                // nothing new

  const int mid = ECG_SPR_H / 2;
  const int yLo = 2, yHi = ECG_SPR_H - 3;

  while (ecgTail != head) {
    if (cursor + ECG_SEG_W > ECG_SPR_W) { cursor = 0; prevY = -1; }

    int n = (int)(head - ecgTail);
    if (n > ECG_SEG_W) n = ECG_SEG_W;

    ecgSegBackground(cursor, ECG_SEG_W);
    for (int k = 0; k < n; k++) {
      const int y = constrain(mid + ecgRing[ecgTail & (ECG_RING - 1)], yLo, yHi);
      if (prevY < 0)   ecgSeg.drawPixel(k, y, COL_TRACE);
      else if (k == 0) ecgSeg.drawLine(0, prevY, 0, y, COL_TRACE);
      else             ecgSeg.drawLine(k - 1, prevY, k, y, COL_TRACE);
      prevY = y;
      ecgTail++;
    }
    ecgSeg.pushSprite(2 + cursor, ECG_Y + 2);
    cursor += n;
  }
  ecgEraseAhead(cursor, ECG_GAP_W);
}

void drawNumbers(uint32_t now) {
  Metrics m;
  lockM(); m = M; unlockM();

  char buf[24], st[24];

  // ---- Heart rate: ECG preferred, PPG as fallback, source always shown -----
  const bool eOk = m.hrEcg.valid(now);
  const bool pOk = m.hrPpg.valid(now);
  if (eOk)      { snprintf(buf, sizeof buf, "%d", (int)lroundf(m.hrEcg.value(now))); strcpy(st, "from ECG"); }
  else if (pOk) { snprintf(buf, sizeof buf, "%d", (int)lroundf(m.hrPpg.value(now))); strcpy(st, "from PPG"); }
  else          { strcpy(buf, "--"); strcpy(st, (g_leadsOn || m.fingerOn) ? "acquiring" : "no signal"); }
  // When both are live, disagreement is the most useful thing on the screen.
  if (eOk && pOk) {
    const float d = fabsf(m.hrEcg.value(now) - m.hrPpg.value(now));
    if (d > 8.0f) snprintf(st, sizeof st, "E/P differ %d", (int)lroundf(d));
    else          strcpy(st, "ECG+PPG agree");
  }
  drawBlock(0, R1_Y, "HEART RATE", "bpm", buf, st,
            hrColour(eOk || pOk), cache.hr, &cache.hrC);

  // ---- SpO2 ----------------------------------------------------------------
  if (m.spo2.valid(now)) {
    snprintf(buf, sizeof buf, "%d", (int)lroundf(m.spo2.value(now)));
    strcpy(st, m.spo2Cal ? "calibrated" : "estimate only");
  } else { strcpy(buf, "--"); strcpy(st, m.fingerOn ? "acquiring" : "no finger"); }
  drawBlock(SCR_W - BLK_W, R1_Y, "BLOOD OXYGEN", "%", buf, st,
            m.spo2.valid(now) ? (m.spo2Cal ? COL_GOOD : COL_WARN) : COL_IDLE,
            cache.spo2, &cache.spo2C);

  // ---- Surface temperature -------------------------------------------------
  if (m.tempC.valid(now)) {
    const float c = m.tempC.value(now);
    snprintf(buf, sizeof buf, "%.1f", c);
    snprintf(st, sizeof st, "surface  %.1fF", c * 9.0f / 5.0f + 32.0f);
  } else { strcpy(buf, "--"); strcpy(st, "no target"); }
  drawBlock(0, R2_Y, "TEMPERATURE", "C", buf, st,
            m.tempC.valid(now) ? COL_GOOD : COL_IDLE, cache.temp, &cache.tempC);

  // ---- Alcohol: ratio only, never a BAC figure -----------------------------
  if (!m.mqWarm) {
    strcpy(buf, "--");
    // Two distinct states. "warming up 0s" was a contradiction on screen.
    if (m.mqWarmLeft > 0) snprintf(st, sizeof st, "heater %us", (unsigned)m.mqWarmLeft);
    else                  snprintf(st, sizeof st, "baseline %d/%d",
                                   m.mqBaseN, MQ3Monitor::baselineNeeded());
  } else if (m.mqRatio.valid(now)) {
    snprintf(buf, sizeof buf, "%.2f", m.mqRatio.value(now));
    strcpy(st, "Rs/R0  not BAC");
  } else { strcpy(buf, "--"); strcpy(st, "no baseline"); }
  drawBlock(SCR_W - BLK_W, R2_Y, "ALCOHOL", "ratio", buf, st,
            m.mqWarm && m.mqRatio.valid(now)
              ? (m.mqRatio.value(now) < 0.6f ? COL_WARN : COL_GOOD)
              : COL_IDLE,
            cache.alc, &cache.alcC);

  // ---- Thin row: HRV / respiration / perfusion -----------------------------
  if (m.rmssd.valid(now)) snprintf(buf, sizeof buf, "%dms", (int)lroundf(m.rmssd.value(now)));
  else                    snprintf(buf, sizeof buf, "%d/30", m.hrvCount);
  drawThin(THIN_X1, THIN_W, "HRV", buf, cache.hrv);

  if (m.respRate.valid(now)) snprintf(buf, sizeof buf, "%d", (int)lroundf(m.respRate.value(now)));
  else                       strcpy(buf, "--");
  drawThin(THIN_X2, THIN_W, "RESP", buf, cache.resp);

  if (m.perfusion.valid(now)) snprintf(buf, sizeof buf, "%.2f", m.perfusion.value(now));
  else                        strcpy(buf, "--");
  drawThin(THIN_X3, THIN_W, "PERF", buf, cache.pi);

  // ---- Status bar ----------------------------------------------------------
  char bar[48];
  // The ECG quality marker lives here, not in the waveform box -- anything
  // drawn inside the box gets wiped by the sweep cursor on its next pass.
  const char* q = (g_leadsOn && !g_ecgSignalOk) ? " LOW" : "";
  if (m.railV.valid(now))
    snprintf(bar, sizeof bar, "%.2fV %d/%dHz%s", m.railV.value(now),
             (int)lroundf(m.ecgRateMeas), (int)lroundf(m.ppgRateMeas), q);
  else
    snprintf(bar, sizeof bar, "--.--V %d/%dHz%s",
             (int)lroundf(m.ecgRateMeas), (int)lroundf(m.ppgRateMeas), q);
  if (strcmp(bar, cache.bar)) {
    strncpy(cache.bar, bar, sizeof(cache.bar) - 1);
    tft.fillRect(0, 0, SCR_W, BAR_H, COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(COL_LABEL, COL_BG);
    tft.drawString("VITALS", 3, 2, 1);
    tft.setTextDatum(TR_DATUM);
    // Rate deviation is flagged, not hidden: wrong rate means wrong filters.
    const bool rateOk = fabsf(m.ecgRateMeas - ECG_FS_HZ) < 12.0f;
    tft.setTextColor(rateOk ? COL_LABEL : COL_WARN, COL_BG);
    tft.drawString(bar, SCR_W - 3, 2, 1);
  }
}

// =============================================================================
//  SERIAL CALIBRATION CONSOLE
//
//  The only honest way to turn the R ratio into an SpO2 percentage, or the
//  MLX reading into a surface temperature, is to fit against a reference
//  instrument. These commands do that and persist the result to NVS.
// =============================================================================

#define CAL_MAX 12
float calR[CAL_MAX], calRef[CAL_MAX];
int   calN = 0;

void loadCalibration() {
  prefs.begin("vitals", true);
  const float a = prefs.getFloat("spo2_a", NAN);
  const float b = prefs.getFloat("spo2_b", NAN);
  const bool  c = prefs.getBool("spo2_cal", false);
  if (!isnan(a) && !isnan(b)) spo2Est.setCoefficients(a, b, c);
  const float f = prefs.getFloat("temp_f", NAN);
  if (!isnan(f)) skinTemp.setFillFraction(f);
  const float s = prefs.getFloat("rail_fs", NAN);
  if (!isnan(s)) rail.setScale(s);
  const float g = prefs.getFloat("ecg_gain", NAN);
  if (!isnan(g) && g >= 1.0f) ecgGain.setManual(g);
  prefs.end();
}

void saveFloat(const char* key, float v) {
  prefs.begin("vitals", false); prefs.putFloat(key, v); prefs.end();
}

void fitSpO2() {
  if (calN < 3) { Serial.println("need at least 3 points"); return; }
  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int i = 0; i < calN; i++) {
    sx += calR[i]; sy += calRef[i];
    sxx += calR[i] * calR[i]; sxy += calR[i] * calRef[i];
  }
  const float den = calN * sxx - sx * sx;
  if (fabsf(den) < 1e-9f) { Serial.println("points too clustered - vary SpO2 more"); return; }
  const float slope = (calN * sxy - sx * sy) / den;
  const float icept = (sy - slope * sx) / calN;
  if (slope >= 0.0f) { Serial.println("slope positive - check your pairings"); return; }
  spo2Est.setCoefficients(icept, -slope, true);
  prefs.begin("vitals", false);
  prefs.putFloat("spo2_a", icept);
  prefs.putFloat("spo2_b", -slope);
  prefs.putBool("spo2_cal", true);
  prefs.end();
  Serial.printf("fitted: SpO2 = %.2f - %.2f * R  (n=%d) -- saved\n", icept, -slope, calN);
}

void printStatus() {
  Metrics m; lockM(); m = M; unlockM();
  const uint32_t now = millis();
  Serial.println(F("---------------- status ----------------"));
  Serial.printf("last reset      : %s\n", resetReasonName(boot.resetReason));
  Serial.printf("sensors         : MAX30102 %s | MLX90614 %s\n",
                boot.maxOk ? "ok" : "FAIL", boot.mlxOk ? "ok" : "FAIL");
  Serial.printf("free heap       : %u bytes (%u at boot)\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)boot.freeHeap);
  Serial.printf("ecg sprite      : %s (%d x %d, %d bytes)\n",
                ecgSegOk ? "allocated" : "FAILED",
                ECG_SEG_FULL, ECG_SPR_H, ECG_SEG_FULL * ECG_SPR_H * 2);
  Serial.printf("freertos tick   : %d Hz (need >= 250 for a 250 Hz ECG task)\n", (int)configTICK_RATE_HZ);
  Serial.printf("measured rates  : ECG %.1f Hz (target %.0f) | PPG %.1f Hz (target %.0f)\n",
                m.ecgRateMeas, ECG_FS_HZ, m.ppgRateMeas, PPG_FS_HZ);
  Serial.printf("leads / finger  : %s / %s\n", g_leadsOn ? "on" : "off", m.fingerOn ? "on" : "off");
  Serial.printf("ecg quality     : %s (qrs amp %.0f, snr %.1f)\n",
                rpeak.signalPresent() ? "ok" : "LOW", rpeak.qrsAmplitude(), rpeak.snr());
  Serial.printf("rail proxy      : %.3f V (scale %.3f) %s\n",
                rail.volts(), rail.scale(), rail.valid() ? "" : "[no valid baseline yet]");
  Serial.printf("hrv             : %d intervals, %u rejected, RMSSD %.1f SDNN %.1f\n",
                hrv.count(), (unsigned)hrv.rejected(), hrv.rmssd(), hrv.sdnn());
  Serial.printf("spo2            : R=%.4f  coef a=%.2f b=%.2f  %s\n",
                spo2Est.lastRatio(), spo2Est.coefA(), spo2Est.coefB(),
                spo2Est.calibrated() ? "CALIBRATED" : "uncalibrated (estimate only)");
  Serial.printf("temp model      : fill fraction f=%.3f\n", skinTemp.fillFraction());
  Serial.printf("ecg display     : gain %.1f counts/px (%s), peak %.0f, full scale +/-%.0f counts\n",
                ecgGain.gain(), ecgGain.manual() ? "manual" :
                  (ecgGain.fitted() ? "auto-fitted" : "measuring"),
                ecgGain.peak(), ecgGain.gain() * ((ECG_SPR_H / 2) - 4));
  Serial.printf("mq3             : warm=%d baseline=%.0f ratio=%.3f\n",
                (int)mq3.warmedUp(now), mq3.baselineAdc(), mq3.ratio());
  Serial.printf("resp            : %d breaths buffered, valid=%d\n",
                respEst.breathCount(), (int)respEst.valid());
  Serial.println(F("----------------------------------------"));
}

void printHelp() {
  Serial.println(F(
    "commands:\n"
    "  status              full diagnostic dump\n"
    "  spo2 ref <pct>      record current R against a reference oximeter reading\n"
    "  spo2 fit            least-squares fit + save (needs 3+ spread-out points)\n"
    "  spo2 clear          discard collected points and stored coefficients\n"
    "  temp ref <degC>     solve the FOV fill fraction from a known surface temp\n"
    "  rail ref <volts>    set ADC full-scale from a multimeter reading of 3V3\n"
    "  mq3 zero            re-zero the clean-air baseline (do this in clean air)\n"
    "  gain auto           re-measure the ECG display gain over 6 s\n"
    "  gain <counts/px>    fix the ECG display gain by hand\n"
    "  gain show           report the current ECG gain\n"
    "  raw on|off          stream slow-sensor values to serial\n"
    "  help                this list"));
}

void handleSerial() {
  static char line[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (!n) continue;
      line[n] = 0; n = 0;

      if (!strcmp(line, "help"))   { printHelp(); }
      else if (!strcmp(line, "status")) { printStatus(); }
      else if (!strcmp(line, "raw on"))  { rawStream = true;  Serial.println("raw on"); }
      else if (!strcmp(line, "raw off")) { rawStream = false; Serial.println("raw off"); }
      else if (!strcmp(line, "gain auto")) {
        ecgGain.refit();
        prefs.begin("vitals", false); prefs.remove("ecg_gain"); prefs.end();
        Serial.println("re-measuring ECG gain - hold still with the leads on for 6 s");
      }
      else if (!strcmp(line, "gain show")) {
        Serial.printf("ECG gain %.1f counts/px (%s), measured peak %.0f counts, full scale +/- %.0f counts\n",
                      ecgGain.gain(), ecgGain.manual() ? "manual" :
                        (ecgGain.fitted() ? "auto-fitted" : "measuring"),
                      ecgGain.peak(), ecgGain.gain() * ((ECG_SPR_H / 2) - 4));
      }
      else if (!strncmp(line, "gain ", 5) && isdigit((unsigned char)line[5])) {
        const float g = atof(line + 5);
        if (g < 1.0f || g > 400.0f) Serial.println("gain must be 1-400 counts per pixel");
        else { ecgGain.setManual(g); saveFloat("ecg_gain", g);
               Serial.printf("ECG gain fixed at %.1f counts/px -- saved\n", g); }
      }
      else if (!strcmp(line, "mq3 zero")) { mq3.rezero(); Serial.println("mq3 baseline cleared - hold in clean air"); }
      else if (!strcmp(line, "spo2 fit")) { fitSpO2(); }
      else if (!strcmp(line, "spo2 clear")) {
        calN = 0;
        spo2Est.setCoefficients(110.0f, 25.0f, false);
        prefs.begin("vitals", false); prefs.putBool("spo2_cal", false); prefs.end();
        Serial.println("spo2 calibration cleared");
      }
      else if (!strncmp(line, "spo2 ref ", 9)) {
        const float ref = atof(line + 9);
        const float r   = spo2Est.lastRatio();
        if (isnan(r))                    Serial.println("no valid R ratio right now - keep your finger on the sensor");
        else if (ref < 70 || ref > 100)  Serial.println("reference must be 70-100");
        else if (calN >= CAL_MAX)        Serial.println("calibration buffer full");
        else { calR[calN] = r; calRef[calN] = ref; calN++;
               Serial.printf("point %d: R=%.4f -> %.1f%%\n", calN, r, ref); }
      }
      else if (!strncmp(line, "temp ref ", 9)) {
        const float ref  = atof(line + 9);
        const float tObj = mlx.readObjectTempC(), tAmb = mlx.readAmbientTempC();
        const float f    = SkinTempEstimator::solveFillFraction(tObj, tAmb, ref);
        if (isnan(f)) Serial.println("could not solve - need more contrast between target and ambient");
        else { skinTemp.setFillFraction(f); skinTemp.reset(); saveFloat("temp_f", f);
               Serial.printf("fill fraction f = %.3f (obj %.2f, amb %.2f, ref %.2f) -- saved\n", f, tObj, tAmb, ref); }
      }
      else if (!strncmp(line, "rail ref ", 9)) {
        const float v = atof(line + 9);
        const float b = rail.baseline();
        if (isnan(b) || b < 500) Serial.println("no valid ECG baseline - attach the leads first");
        else { const float fs = v / (2.0f * (b / 4095.0f));
               rail.setScale(fs); saveFloat("rail_fs", fs);
               Serial.printf("ADC full-scale = %.3f V -- saved\n", fs); }
      }
      else Serial.printf("unknown: '%s' (try 'help')\n", line);
    } else if (n < sizeof(line) - 1) {
      line[n++] = c;
    }
  }
}

// =============================================================================
//  SETUP
// =============================================================================

void drawStaticChrome() {
  tft.fillScreen(COL_BG);
  drawFrame(0, R1_Y, BLK_W, BLK_H);
  drawFrame(SCR_W - BLK_W, R1_Y, BLK_W, BLK_H);
  drawFrame(0, R2_Y, BLK_W, BLK_H);
  drawFrame(SCR_W - BLK_W, R2_Y, BLK_W, BLK_H);
  drawFrame(THIN_X1, R3_Y, THIN_W, R3_H);
  drawFrame(THIN_X2, R3_Y, THIN_W, R3_H);
  drawFrame(THIN_X3, R3_Y, THIN_W, R3_H);
  drawFrame(0, ECG_Y, SCR_W, ECG_H);
}

void showBootScreen() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.drawString("VITALS MONITOR", 8, 8, 4);

  int y = 48;
  char b[64];
  const bool brownout = (boot.resetReason == ESP_RST_BROWNOUT);
  tft.setTextColor(brownout ? COL_BAD : COL_LABEL, COL_BG);
  snprintf(b, sizeof b, "last reset : %s", resetReasonName(boot.resetReason));
  tft.drawString(b, 8, y, 2); y += 20;

  tft.setTextColor(boot.maxOk ? COL_GOOD : COL_BAD, COL_BG);
  tft.drawString(boot.maxOk ? "MAX30102   : ok" : "MAX30102   : NOT FOUND", 8, y, 2); y += 20;
  tft.setTextColor(boot.mlxOk ? COL_GOOD : COL_BAD, COL_BG);
  tft.drawString(boot.mlxOk ? "MLX90614   : ok" : "MLX90614   : NOT FOUND", 8, y, 2); y += 20;

  tft.setTextColor(COL_LABEL, COL_BG);
  snprintf(b, sizeof b, "free heap  : %u", (unsigned)boot.freeHeap);
  tft.drawString(b, 8, y, 2); y += 20;
  snprintf(b, sizeof b, "ecg sprite : %s", ecgSegOk ? "ok" : "FAILED");
  tft.drawString(b, 8, y, 2); y += 24;

  if (brownout) {
    tft.setTextColor(COL_BAD, COL_BG);
    tft.drawString("brownout - check battery / supply", 8, y, 2);
  } else {
    tft.setTextColor(COL_LABEL, COL_BG);
    tft.drawString("serial 115200 - type 'help'", 8, y, 2);
  }
  delay(2500);
}

void setup() {
  Serial.begin(115200);
  delay(50);

  boot.resetReason = esp_reset_reason();
  boot.freeHeap    = ESP.getFreeHeap();

  pinMode(LO_PLUS,  INPUT);
  pinMode(LO_MINUS, INPUT);
  analogReadResolution(12);

  mLock = xSemaphoreCreateMutex();

  tft.init();
  tft.setRotation(TFT_ROTATION);   // see the geometry block for portrait
  tft.fillScreen(COL_BG);

  ecgSeg.setColorDepth(16);
  ecgSegOk = (ecgSeg.createSprite(ECG_SEG_FULL, ECG_SPR_H) != nullptr);

  // --- MAX30102 on bus 0 ---
  Wire.begin(MAX_SDA, MAX_SCL);
  Wire.setClock(400000);       // at 100 kHz a FIFO read ate a third of the budget
  boot.maxOk = pulseSensor.begin();
  if (boot.maxOk) {
    pulseSensor.setSamplingRate(pulseSensor.SAMPLING_RATE_100SPS);
    // 100 SPS is ample for a 0.5-5 Hz pulse waveform and frees I2C bandwidth.
  } else {
    Serial.println(F("MAX30102 not found on bus 0"));
  }

  // --- MLX90614 on bus 1 ---
  Wire1.begin(MLX_SDA, MLX_SCL);
  boot.mlxOk = mlx.begin(0x5A, &Wire1);
  if (!boot.mlxOk) Serial.println(F("MLX90614 not found on bus 1"));

  loadCalibration();

  Serial.printf("\nboot: reset=%s heap=%u sprite=%s\n",
                resetReasonName(boot.resetReason), (unsigned)boot.freeHeap,
                ecgSegOk ? "ok" : "FAILED");
  printHelp();

  showBootScreen();
  drawStaticChrome();

  xTaskCreatePinnedToCore(ecgTask,    "ecg",    8192, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(sensorTask, "sensor", 8192, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(renderTask, "render", 8192, nullptr, 1, nullptr, 1);
}

void loop() {
  handleSerial();
  delay(20);
}
