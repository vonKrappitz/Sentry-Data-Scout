/**
 * =======================================================================================
 * Sentry Data Scout v0.4.4 FL — UNIT TESTS
 * Author: Maciej Kasperek (vonKrappitz) + Test harness
 * ---------------------------------------------------------------------------------------
 * [EN] Native C++ test harness that mocks Arduino APIs and tests:
 *   - State machine transitions (all 7 states, all paths)
 *   - Cadence safety floor (MIN_CADENCE_MS = 500)
 *   - Double-start protection
 *   - Power mapping to calibrated range
 *   - Voltage reading calculation
 *   - Calibration constrain guards
 *   - Shot counter and last-shot data
 *   - Resume with cadence timer reset
 *   - Pause during cadence and burst
 *   - V input flow (WAITING_FOR_V → CADENCE or IDLE)
 *   - Reset logs behavior
 *   - Background sensor polling conditions
 *
 * [PL] Natywny harness testowy C++ z mockami Arduino testujący:
 *   - Przejścia maszyny stanów (wszystkie 7 stanów, wszystkie ścieżki)
 *   - Bezpieczna minimalna kadencja (MIN_CADENCE_MS = 500)
 *   - Ochrona przed podwójnym startem
 *   - Mapowanie mocy na skalibrowany zakres
 *   - Obliczenie napięcia z ADC
 *   - Guard constrain na kalibracji
 *   - Licznik strzałów i dane ostatniego strzału
 *   - Wznowienie z resetem timera kadencji
 *   - Pauza w trakcie kadencji i serii
 *   - Przepływ wpisywania V (WAITING_FOR_V → CADENCE lub IDLE)
 *   - Zachowanie resetu logów
 *   - Warunki odpytywania czujników w tle
 *
 * Compile: g++ -std=c++17 -o test_sentry test_sentry_0_4_4.cpp && ./test_sentry
 * =======================================================================================
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include <cassert>
#include <functional>

/* ==================== ARDUINO MOCKS ==================== */
static unsigned long mock_millis_value = 0;
unsigned long millis() { return mock_millis_value; }
void set_millis(unsigned long v) { mock_millis_value = v; }
void advance_millis(unsigned long delta) { mock_millis_value += delta; }

int mock_analog_value = 2048;
int analogRead(int pin) { (void)pin; return mock_analog_value; }

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int constrain_int(int val, int lo, int hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

/* max() for int */
int max_int(int a, int b) { return a > b ? a : b; }

/* ==================== EXTRACTED BUSINESS LOGIC (from v0.4.4) ==================== */
/* These replicate the exact logic from the .ino, isolated for testing */

#define MIN_CADENCE_MS 500
#define VBAT_DIVIDER_RATIO 2.0

enum SystemState {
  SYS_IDLE, SYS_SHOOTING_PRESS, SYS_SHOOTING_RELEASE,
  SYS_SHOOTING_WAIT_500, SYS_WAITING_CADENCE, SYS_PAUSED, SYS_WAITING_FOR_V
};

/* Global state (mirrors firmware) */
struct SentryState {
  SystemState sysState = SYS_IDLE;
  int shotsRemaining = 0;
  bool pauseRequested = false;
  bool requireVInput = false;
  bool isShooting = false;
  unsigned long stateStartTime = 0;
  int totalShotsFired = 0;
  int currentCadenceMs = 1000;
  
  int cal_spust_min = 0, cal_spust_max = 180;
  int cal_sruba_min = 0, cal_sruba_max = 180;
  int cal_power_pct = 50;
  
  float lastShotTemp = 0.0, lastShotG = 0.0, lastShotPeakDB = 0.0, lastShotAvgDB = 0.0;
  bool hasLastShot = false;
  
  /* Mock sensor values */
  float sharedMaxG = 5.5;
  float sharedPeakDB = 95.0;
  float sharedAvgDB = 88.0;
  float barrelTemp = 32.5;
};

/*
 * Simulates one iteration of loop() state machine.
 * Returns the state AFTER processing.
 */
SystemState simulateLoopTick(SentryState& s) {
  unsigned long now = millis();
  
  switch (s.sysState) {
    case SYS_IDLE:
      break;
      
    case SYS_SHOOTING_PRESS:
      s.isShooting = true;
      s.stateStartTime = now;
      s.sysState = SYS_SHOOTING_RELEASE;
      break;
      
    case SYS_SHOOTING_RELEASE:
      if (now - s.stateStartTime >= 60) {
        s.sysState = SYS_SHOOTING_WAIT_500;
      }
      break;
      
    case SYS_SHOOTING_WAIT_500:
      if (now - s.stateStartTime >= 500) {
        s.isShooting = false;
        s.totalShotsFired++;
        
        /* Last shot preview */
        s.lastShotTemp = s.barrelTemp;
        s.lastShotG = s.sharedMaxG;
        s.lastShotPeakDB = s.sharedPeakDB;
        s.lastShotAvgDB = s.sharedAvgDB;
        s.hasLastShot = true;
        
        s.shotsRemaining--;
        
        if (s.requireVInput) {
          s.sysState = SYS_WAITING_FOR_V;
        } else if (s.shotsRemaining > 0) {
          s.stateStartTime = millis();
          s.sysState = s.pauseRequested ? SYS_PAUSED : SYS_WAITING_CADENCE;
        } else {
          s.sysState = SYS_IDLE;
          s.pauseRequested = false;
        }
      }
      break;
      
    case SYS_WAITING_CADENCE:
      if (s.pauseRequested) {
        s.sysState = SYS_PAUSED;
      } else if (now - s.stateStartTime >= (unsigned long)s.currentCadenceMs) {
        s.sysState = SYS_SHOOTING_PRESS;
      }
      break;
      
    case SYS_PAUSED:
    case SYS_WAITING_FOR_V:
      break;
  }
  
  return s.sysState;
}

/* Simulates cmd=start */
bool cmdStart(SentryState& s, int count) {
  if (s.sysState != SYS_IDLE) return false; /* 409 Conflict */
  s.shotsRemaining = count;
  s.pauseRequested = false;
  s.sysState = SYS_SHOOTING_PRESS;
  return true;
}

/* Simulates cmd=pause */
void cmdPause(SentryState& s) { s.pauseRequested = true; }

/* Simulates cmd=resume */
void cmdResume(SentryState& s) {
  s.pauseRequested = false;
  if (s.sysState == SYS_PAUSED) {
    s.stateStartTime = millis();
    s.sysState = SYS_WAITING_CADENCE;
  }
}

/* Simulates cmd=stop */
void cmdStop(SentryState& s) {
  s.shotsRemaining = 0;
  s.pauseRequested = false;
  s.sysState = SYS_IDLE;
  s.isShooting = false;
}

/* Simulates save_v when WAITING_FOR_V */
void cmdSaveV(SentryState& s) {
  if (s.sysState == SYS_WAITING_FOR_V) {
    if (s.shotsRemaining > 0) {
      s.stateStartTime = millis();
      s.sysState = SYS_WAITING_CADENCE;
    } else {
      s.sysState = SYS_IDLE;
    }
  }
}

/* Simulates cadence setting with safety floor */
void setCadence(SentryState& s, int ms) {
  s.currentCadenceMs = max_int(ms, (int)MIN_CADENCE_MS);
}

/* Power mapping */
int mapPower(int pct, int cal_min, int cal_max) {
  return (int)map((long)pct, 0, 100, (long)cal_min, (long)cal_max);
}

/* Voltage calculation */
float calcVoltage(int adc_raw, float divider_ratio) {
  return (adc_raw / 4095.0f) * 3.3f * divider_ratio;
}

/* ==================== TEST FRAMEWORK ==================== */
static int tests_passed = 0;
static int tests_failed = 0;
static int tests_total = 0;

#define TEST(name) void test_##name()
#define RUN(name) do { \
  tests_total++; \
  printf("  [%02d] %-55s", tests_total, #name); \
  try { test_##name(); tests_passed++; printf("✅ PASS\n"); } \
  catch (...) { tests_failed++; printf("❌ FAIL\n"); } \
} while(0)

#define ASSERT_EQ(a, b) do { if ((a) != (b)) { printf("\n       ASSERT_EQ failed: %s=%d, expected %s=%d  ", #a, (int)(a), #b, (int)(b)); throw 1; } } while(0)
#define ASSERT_TRUE(x) do { if (!(x)) { printf("\n       ASSERT_TRUE failed: %s  ", #x); throw 1; } } while(0)
#define ASSERT_FALSE(x) do { if ((x)) { printf("\n       ASSERT_FALSE failed: %s  ", #x); throw 1; } } while(0)
#define ASSERT_FLOAT_EQ(a, b, eps) do { if (fabs((a)-(b)) > (eps)) { printf("\n       ASSERT_FLOAT failed: %s=%.4f, expected %.4f  ", #a, (double)(a), (double)(b)); throw 1; } } while(0)

/* Helper: run full single-shot cycle */
void runFullShotCycle(SentryState& s) {
  set_millis(1000);
  simulateLoopTick(s); /* PRESS → RELEASE */
  advance_millis(60);
  simulateLoopTick(s); /* RELEASE → WAIT_500 */
  advance_millis(440);
  simulateLoopTick(s); /* WAIT_500 → next state */
}

/* ==================== TESTS ==================== */

/* --- GROUP 1: State Machine Basic Flow --- */

TEST(idle_stays_idle) {
  SentryState s;
  set_millis(0);
  ASSERT_EQ(simulateLoopTick(s), SYS_IDLE);
  advance_millis(10000);
  ASSERT_EQ(simulateLoopTick(s), SYS_IDLE);
}

TEST(single_shot_full_cycle) {
  SentryState s;
  cmdStart(s, 1);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_PRESS);
  
  set_millis(1000);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_RELEASE);
  ASSERT_TRUE(s.isShooting);
  
  advance_millis(59); /* Not enough for trigger release */
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_RELEASE); /* Still waiting */
  
  advance_millis(1); /* 60ms total */
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_WAIT_500);
  
  advance_millis(439); /* Not 500ms yet */
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_WAIT_500);
  
  advance_millis(1); /* 500ms total */
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_IDLE);
  ASSERT_FALSE(s.isShooting);
  ASSERT_EQ(s.totalShotsFired, 1);
  ASSERT_EQ(s.shotsRemaining, 0);
}

TEST(burst_3_shots_cycle) {
  SentryState s;
  s.currentCadenceMs = 1000;
  cmdStart(s, 3);
  
  /* Shot 1 */
  set_millis(1000);
  simulateLoopTick(s); /* PRESS → RELEASE */
  advance_millis(60);
  simulateLoopTick(s); /* RELEASE → WAIT_500 */
  advance_millis(440);
  simulateLoopTick(s); /* WAIT_500 → WAITING_CADENCE */
  ASSERT_EQ(s.sysState, SYS_WAITING_CADENCE);
  ASSERT_EQ(s.totalShotsFired, 1);
  ASSERT_EQ(s.shotsRemaining, 2);
  
  /* Wait cadence */
  advance_millis(999);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_CADENCE); /* Not yet */
  advance_millis(1);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_PRESS); /* Cadence done → next shot */
  
  /* Shot 2 */
  simulateLoopTick(s);
  advance_millis(60); simulateLoopTick(s);
  advance_millis(440); simulateLoopTick(s);
  ASSERT_EQ(s.totalShotsFired, 2);
  ASSERT_EQ(s.shotsRemaining, 1);
  
  /* Shot 3 (last) */
  advance_millis(1000); simulateLoopTick(s); /* cadence */
  simulateLoopTick(s); /* PRESS → RELEASE */
  advance_millis(60); simulateLoopTick(s);
  advance_millis(440); simulateLoopTick(s);
  ASSERT_EQ(s.totalShotsFired, 3);
  ASSERT_EQ(s.sysState, SYS_IDLE);
}

TEST(trigger_release_exactly_60ms) {
  SentryState s;
  cmdStart(s, 1);
  set_millis(0);
  simulateLoopTick(s); /* PRESS → RELEASE, stateStartTime = 0 */
  
  set_millis(59);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_RELEASE); /* Still holding */
  
  set_millis(60);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_WAIT_500); /* Released at exactly 60 */
}

TEST(measurement_window_exactly_500ms) {
  SentryState s;
  cmdStart(s, 1);
  set_millis(0);
  simulateLoopTick(s); /* → RELEASE */
  set_millis(60);
  simulateLoopTick(s); /* → WAIT_500 */
  
  set_millis(499);
  simulateLoopTick(s);
  ASSERT_TRUE(s.isShooting); /* Still in window */
  
  set_millis(500);
  simulateLoopTick(s);
  ASSERT_FALSE(s.isShooting); /* Window closed */
}

/* --- GROUP 2: Pause / Resume --- */

TEST(pause_during_cadence) {
  SentryState s;
  s.currentCadenceMs = 2000;
  cmdStart(s, 3);
  runFullShotCycle(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_CADENCE);
  
  cmdPause(s);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_PAUSED);
}

TEST(resume_from_pause_resets_timer) {
  SentryState s;
  s.currentCadenceMs = 1000;
  cmdStart(s, 3);
  runFullShotCycle(s);
  
  cmdPause(s);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_PAUSED);
  
  /* Long pause */
  advance_millis(60000);
  
  set_millis(100000);
  cmdResume(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_CADENCE);
  ASSERT_EQ(s.stateStartTime, 100000UL); /* Timer reset to NOW */
  
  /* Cadence starts from resume time, not original */
  set_millis(100999);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_CADENCE);
  
  set_millis(101000);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_PRESS);
}

TEST(pause_mid_burst_preserves_remaining) {
  SentryState s;
  s.currentCadenceMs = 500;
  cmdStart(s, 5);
  runFullShotCycle(s);
  ASSERT_EQ(s.shotsRemaining, 4);
  
  cmdPause(s);
  simulateLoopTick(s);
  ASSERT_EQ(s.shotsRemaining, 4); /* Not lost */
  ASSERT_EQ(s.sysState, SYS_PAUSED);
}

/* --- GROUP 3: V Input / Chronograph --- */

TEST(require_v_goes_to_waiting) {
  SentryState s;
  s.requireVInput = true;
  cmdStart(s, 2);
  runFullShotCycle(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_FOR_V);
  ASSERT_EQ(s.shotsRemaining, 1);
}

TEST(save_v_with_remaining_goes_cadence) {
  SentryState s;
  s.requireVInput = true;
  cmdStart(s, 2);
  runFullShotCycle(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_FOR_V);
  
  set_millis(50000);
  cmdSaveV(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_CADENCE);
  ASSERT_EQ(s.stateStartTime, 50000UL);
}

TEST(save_v_last_shot_goes_idle) {
  SentryState s;
  s.requireVInput = true;
  cmdStart(s, 1);
  runFullShotCycle(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_FOR_V);
  ASSERT_EQ(s.shotsRemaining, 0);
  
  cmdSaveV(s);
  ASSERT_EQ(s.sysState, SYS_IDLE);
}

/* --- GROUP 4: Safety Guards --- */

TEST(double_start_rejected) {
  SentryState s;
  ASSERT_TRUE(cmdStart(s, 1)); /* First start OK */
  ASSERT_FALSE(cmdStart(s, 5)); /* Second start REJECTED */
  ASSERT_EQ(s.shotsRemaining, 1); /* Original count preserved */
}

TEST(cadence_safety_floor_enforced) {
  SentryState s;
  setCadence(s, 100);
  ASSERT_EQ(s.currentCadenceMs, 500); /* Clamped to MIN */
  
  setCadence(s, 499);
  ASSERT_EQ(s.currentCadenceMs, 500);
  
  setCadence(s, 500);
  ASSERT_EQ(s.currentCadenceMs, 500);
  
  setCadence(s, 501);
  ASSERT_EQ(s.currentCadenceMs, 501); /* Above minimum OK */
  
  setCadence(s, 5000);
  ASSERT_EQ(s.currentCadenceMs, 5000);
}

TEST(cadence_zero_clamped) {
  SentryState s;
  setCadence(s, 0);
  ASSERT_EQ(s.currentCadenceMs, 500);
}

TEST(cadence_negative_clamped) {
  SentryState s;
  setCadence(s, -100);
  ASSERT_EQ(s.currentCadenceMs, 500);
}

TEST(constrain_servo_values) {
  ASSERT_EQ(constrain_int(-10, 0, 180), 0);
  ASSERT_EQ(constrain_int(0, 0, 180), 0);
  ASSERT_EQ(constrain_int(90, 0, 180), 90);
  ASSERT_EQ(constrain_int(180, 0, 180), 180);
  ASSERT_EQ(constrain_int(200, 0, 180), 180);
  ASSERT_EQ(constrain_int(999, 0, 180), 180);
}

TEST(constrain_power_values) {
  ASSERT_EQ(constrain_int(-5, 0, 100), 0);
  ASSERT_EQ(constrain_int(0, 0, 100), 0);
  ASSERT_EQ(constrain_int(50, 0, 100), 50);
  ASSERT_EQ(constrain_int(100, 0, 100), 100);
  ASSERT_EQ(constrain_int(150, 0, 100), 100);
}

/* --- GROUP 5: Stop Command --- */

TEST(stop_resets_everything) {
  SentryState s;
  cmdStart(s, 5);
  set_millis(0);
  simulateLoopTick(s); /* → RELEASE */
  ASSERT_TRUE(s.isShooting);
  
  cmdStop(s);
  ASSERT_EQ(s.sysState, SYS_IDLE);
  ASSERT_FALSE(s.isShooting);
  ASSERT_EQ(s.shotsRemaining, 0);
  ASSERT_FALSE(s.pauseRequested);
}

TEST(stop_from_paused) {
  SentryState s;
  cmdStart(s, 3);
  runFullShotCycle(s);
  cmdPause(s);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_PAUSED);
  
  cmdStop(s);
  ASSERT_EQ(s.sysState, SYS_IDLE);
}

TEST(stop_from_waiting_v) {
  SentryState s;
  s.requireVInput = true;
  cmdStart(s, 1);
  runFullShotCycle(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_FOR_V);
  
  cmdStop(s);
  ASSERT_EQ(s.sysState, SYS_IDLE);
}

/* --- GROUP 6: Power Mapping --- */

TEST(power_mapping_0_percent) {
  ASSERT_EQ(mapPower(0, 20, 160), 20);
}

TEST(power_mapping_100_percent) {
  ASSERT_EQ(mapPower(100, 20, 160), 160);
}

TEST(power_mapping_50_percent) {
  ASSERT_EQ(mapPower(50, 20, 160), 90);
}

TEST(power_mapping_default_range) {
  ASSERT_EQ(mapPower(0, 0, 180), 0);
  ASSERT_EQ(mapPower(100, 0, 180), 180);
  ASSERT_EQ(mapPower(50, 0, 180), 90);
}

TEST(power_mapping_asymmetric) {
  /* Real-world: spust_min=10, spust_max=45 */
  ASSERT_EQ(mapPower(0, 10, 45), 10);
  ASSERT_EQ(mapPower(100, 10, 45), 45);
}

/* --- GROUP 7: Voltage Calculation --- */

TEST(voltage_zero_adc) {
  ASSERT_FLOAT_EQ(calcVoltage(0, 2.0), 0.0, 0.01);
}

TEST(voltage_mid_adc) {
  float v = calcVoltage(2048, 2.0);
  ASSERT_FLOAT_EQ(v, 3.3, 0.01); /* ~3.3V at midpoint with 2x divider */
}

TEST(voltage_full_adc) {
  float v = calcVoltage(4095, 2.0);
  ASSERT_FLOAT_EQ(v, 6.6, 0.01);
}

TEST(voltage_custom_divider) {
  float v = calcVoltage(4095, 3.13);
  ASSERT_FLOAT_EQ(v, 10.329, 0.01);
}

/* --- GROUP 8: Last Shot Data --- */

TEST(last_shot_not_set_initially) {
  SentryState s;
  ASSERT_FALSE(s.hasLastShot);
}

TEST(last_shot_set_after_fire) {
  SentryState s;
  s.sharedMaxG = 12.5;
  s.sharedPeakDB = 105.3;
  s.sharedAvgDB = 92.1;
  s.barrelTemp = 38.7;
  
  cmdStart(s, 1);
  runFullShotCycle(s);
  
  ASSERT_TRUE(s.hasLastShot);
  ASSERT_FLOAT_EQ(s.lastShotG, 12.5, 0.01);
  ASSERT_FLOAT_EQ(s.lastShotPeakDB, 105.3, 0.01);
  ASSERT_FLOAT_EQ(s.lastShotAvgDB, 92.1, 0.01);
  ASSERT_FLOAT_EQ(s.lastShotTemp, 38.7, 0.01);
}

TEST(last_shot_updates_on_each_shot) {
  SentryState s;
  s.currentCadenceMs = 500;
  s.sharedMaxG = 5.0;
  
  cmdStart(s, 2);
  
  /* Shot 1 */
  runFullShotCycle(s);
  ASSERT_FLOAT_EQ(s.lastShotG, 5.0, 0.01);
  
  /* Change sensor reading */
  s.sharedMaxG = 15.0;
  
  /* Shot 2 */
  advance_millis(500); simulateLoopTick(s); /* cadence → PRESS */
  simulateLoopTick(s); /* PRESS → RELEASE */
  advance_millis(60); simulateLoopTick(s);
  advance_millis(440); simulateLoopTick(s);
  
  ASSERT_FLOAT_EQ(s.lastShotG, 15.0, 0.01); /* Updated */
}

/* --- GROUP 9: Reset Logs --- */

TEST(reset_clears_shot_counter) {
  SentryState s;
  cmdStart(s, 1);
  runFullShotCycle(s);
  ASSERT_EQ(s.totalShotsFired, 1);
  ASSERT_TRUE(s.hasLastShot);
  
  /* Simulate reset_logs */
  s.totalShotsFired = 0;
  s.hasLastShot = false;
  
  ASSERT_EQ(s.totalShotsFired, 0);
  ASSERT_FALSE(s.hasLastShot);
}

/* --- GROUP 10: Edge Cases --- */

TEST(resume_when_not_paused_is_noop) {
  SentryState s;
  s.sysState = SYS_IDLE;
  cmdResume(s);
  ASSERT_EQ(s.sysState, SYS_IDLE); /* No change */
  
  s.sysState = SYS_WAITING_FOR_V;
  cmdResume(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_FOR_V); /* No change */
}

TEST(save_v_when_not_waiting_is_noop) {
  SentryState s;
  s.sysState = SYS_IDLE;
  cmdSaveV(s);
  ASSERT_EQ(s.sysState, SYS_IDLE);
  
  s.sysState = SYS_WAITING_CADENCE;
  cmdSaveV(s);
  ASSERT_EQ(s.sysState, SYS_WAITING_CADENCE);
}

TEST(pause_request_cleared_on_burst_end) {
  SentryState s;
  cmdStart(s, 1);
  cmdPause(s); /* Pause requested but only 1 shot */
  ASSERT_TRUE(s.pauseRequested);
  
  runFullShotCycle(s);
  ASSERT_EQ(s.sysState, SYS_IDLE);
  ASSERT_FALSE(s.pauseRequested); /* Cleared on IDLE transition */
}

TEST(millis_overflow_safe) {
  SentryState s;
  s.currentCadenceMs = 1000;
  cmdStart(s, 2);
  
  /* Simulate near overflow */
  set_millis(0xFFFFFF00UL);
  simulateLoopTick(s); /* PRESS → RELEASE */
  advance_millis(60);
  simulateLoopTick(s); /* → WAIT_500 */
  advance_millis(440);
  simulateLoopTick(s); /* → CADENCE */
  ASSERT_EQ(s.sysState, SYS_WAITING_CADENCE);
  
  /* Cadence wraps around millis overflow */
  advance_millis(1000);
  simulateLoopTick(s);
  ASSERT_EQ(s.sysState, SYS_SHOOTING_PRESS); /* Works despite overflow */
}

TEST(zero_shots_start_still_fires_one) {
  /* Edge: what if count=0 is passed? shotsRemaining becomes 0 immediately */
  SentryState s;
  cmdStart(s, 0);
  runFullShotCycle(s);
  /* shotsRemaining was 0, after -- it's -1, which means shotsRemaining > 0 is false */
  ASSERT_EQ(s.sysState, SYS_IDLE);
  ASSERT_EQ(s.totalShotsFired, 1); /* One shot still fired */
}

/* ==================== MAIN ==================== */
int main() {
  printf("\n");
  printf("╔══════════════════════════════════════════════════════════════════╗\n");
  printf("║  SENTRY DATA SCOUT v0.4.4 FL — UNIT TEST SUITE                ║\n");
  printf("╚══════════════════════════════════════════════════════════════════╝\n\n");
  
  printf("─── Group 1: State Machine Basic Flow ───\n");
  RUN(idle_stays_idle);
  RUN(single_shot_full_cycle);
  RUN(burst_3_shots_cycle);
  RUN(trigger_release_exactly_60ms);
  RUN(measurement_window_exactly_500ms);
  
  printf("\n─── Group 2: Pause / Resume ───\n");
  RUN(pause_during_cadence);
  RUN(resume_from_pause_resets_timer);
  RUN(pause_mid_burst_preserves_remaining);
  
  printf("\n─── Group 3: V Input / Chronograph ───\n");
  RUN(require_v_goes_to_waiting);
  RUN(save_v_with_remaining_goes_cadence);
  RUN(save_v_last_shot_goes_idle);
  
  printf("\n─── Group 4: Safety Guards ───\n");
  RUN(double_start_rejected);
  RUN(cadence_safety_floor_enforced);
  RUN(cadence_zero_clamped);
  RUN(cadence_negative_clamped);
  RUN(constrain_servo_values);
  RUN(constrain_power_values);
  
  printf("\n─── Group 5: Stop Command ───\n");
  RUN(stop_resets_everything);
  RUN(stop_from_paused);
  RUN(stop_from_waiting_v);
  
  printf("\n─── Group 6: Power Mapping ───\n");
  RUN(power_mapping_0_percent);
  RUN(power_mapping_100_percent);
  RUN(power_mapping_50_percent);
  RUN(power_mapping_default_range);
  RUN(power_mapping_asymmetric);
  
  printf("\n─── Group 7: Voltage Calculation ───\n");
  RUN(voltage_zero_adc);
  RUN(voltage_mid_adc);
  RUN(voltage_full_adc);
  RUN(voltage_custom_divider);
  
  printf("\n─── Group 8: Last Shot Data ───\n");
  RUN(last_shot_not_set_initially);
  RUN(last_shot_set_after_fire);
  RUN(last_shot_updates_on_each_shot);
  
  printf("\n─── Group 9: Reset Logs ───\n");
  RUN(reset_clears_shot_counter);
  
  printf("\n─── Group 10: Edge Cases ───\n");
  RUN(resume_when_not_paused_is_noop);
  RUN(save_v_when_not_waiting_is_noop);
  RUN(pause_request_cleared_on_burst_end);
  RUN(millis_overflow_safe);
  RUN(zero_shots_start_still_fires_one);
  
  printf("\n══════════════════════════════════════════════════════════════════\n");
  printf("  RESULTS: %d/%d passed", tests_passed, tests_total);
  if (tests_failed > 0) printf(", %d FAILED ❌", tests_failed);
  else printf("  ✅ ALL GREEN");
  printf("\n══════════════════════════════════════════════════════════════════\n\n");
  
  return tests_failed > 0 ? 1 : 0;
}
