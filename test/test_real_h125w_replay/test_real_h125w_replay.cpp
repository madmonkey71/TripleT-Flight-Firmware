// End-to-end software-in-the-loop replay of the flight simulated by flight_simulation_h125w.py
// (Aerotech H125W: 17.7 g peak, burnout 2.8 s @ 195 m/s, apogee 1936 m AGL @ 19.8 s, drogue ~8.5 m/s,
// main at 100 m AGL, landing ~272 s). The physics below is a line-for-line port of that script; the
// resulting barometric altitude / specific force are fed - with sensor noise, at 10 Hz - through the
// REAL flight_logic / pyro / state-management code, and the state machine's decisions are checked
// against the simulated events.
#include <unity.h>
#include <vector>
#include <math.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

// ---- port of flight_simulation_h125w.py ---------------------------------------------------------------
struct TP { double t, n; };
static const TP kThrust[] = {
  {0.053, 276}, {0.161, 241}, {0.270, 216}, {0.378, 199}, {0.488, 189}, {0.597, 182}, {0.705, 176}, {0.814, 169},
  {0.922, 162}, {1.031, 154}, {1.141, 144}, {1.249, 133}, {1.357, 123}, {1.466, 113}, {1.575, 102}, {1.684, 88},
  {1.793, 74}, {1.901, 60}, {2.009, 47}, {2.119, 37}, {2.228, 30}, {2.336, 24}, {2.445, 19}, {2.553, 15},
  {2.663, 11}, {2.772, 0}};
static const double BURN = 2.77, PROP = 0.188, CASE = 0.323 - 0.188, DRY = 1.18, G = 9.81, RHO = 1.225;
static const double CD_ROCKET = 0.5, A_ROCKET = 0.00066, CD_DROGUE = 1.5, A_DROGUE = M_PI * 0.25 * 0.25;
static const double CD_MAIN = 2.2, A_MAIN = M_PI * 0.6 * 0.6, MAIN_ALT = 100.0, DT = 0.1;

static double thrust(double t) {
  const int n = sizeof(kThrust) / sizeof(kThrust[0]);
  if (t <= 0) return kThrust[0].n;
  if (t >= BURN) return 0;
  for (int i = 0; i < n - 1; i++)
    if (kThrust[i].t <= t && t <= kThrust[i + 1].t)
      return kThrust[i].n + (t - kThrust[i].t) / (kThrust[i + 1].t - kThrust[i].t) * (kThrust[i + 1].n - kThrust[i].n);
  return 0;
}
static double mass(double t) { return DRY + CASE + (t >= BURN ? 0.0 : PROP * (1 - t / BURN)); }

struct SimStep { double t, alt, vel, accel_g; int phase; /*0 boost 1 coast 2 drogue 3 main*/ };
static std::vector<SimStep> run_sim(double& burnout_t, double& apogee_t, double& apogee_alt, double& landing_t, double& main_alt_at_deploy) {
  std::vector<SimStep> out;
  double t = 0, alt = 0, vel = 0; int phase = 0;
  burnout_t = apogee_t = apogee_alt = landing_t = main_alt_at_deploy = -1;
  for (int step = 0; step < 4000; step++) {
    const double m = mass(t), T = thrust(t);
    double cd = CD_ROCKET, A = A_ROCKET;
    if (phase == 2) { cd = CD_DROGUE; A = A_DROGUE; } else if (phase == 3) { cd = CD_MAIN; A = A_MAIN; }
    const double drag = 0.5 * cd * RHO * A * vel * vel;
    const double net = (vel > 0) ? T - m * G - drag : T - m * G + drag;
    const double acc = net / m;
    out.push_back({t, alt, vel, acc / G, phase});
    vel += acc * DT; alt += vel * DT;
    if (phase == 0 && T == 0) { phase = 1; burnout_t = t; }
    if (phase == 1 && vel <= 0) { phase = 2; apogee_alt = alt; apogee_t = t; }
    if (phase == 2 && alt <= MAIN_ALT) { phase = 3; main_alt_at_deploy = alt; }
    if (alt <= 0 && t > 1) { landing_t = t; break; }
    t += DT;
  }
  return out;
}

static unsigned s_rng = 777;
static float noise() { s_rng = s_rng * 1664525u + 1013904223u; return ((s_rng >> 8) & 0xFFFF) / 32768.0f - 1.0f; }

void test_replay_of_the_simulated_h125w_flight_through_the_real_state_machine() {
  double burnout_t, apogee_t, apogee_alt, landing_t, main_deploy_alt_sim;
  const std::vector<SimStep> sim = run_sim(burnout_t, apogee_t, apogee_alt, landing_t, main_deploy_alt_sim);
  TEST_ASSERT_TRUE(landing_t > 200.0 && apogee_alt > 1500.0);   // sanity of the port: 1936 m, ~272 s

  const float pad = 100.0f;
  harness_on_pad(pad);
  for (int i = 0; i < 30; i++) { harness_set_baro_alt(pad + 0.15f * noise()); harness_run_ms(100); }   // 3 s on the pad
  g_currentFlightState = ARMED; harness_pass();

  double t_boost = -1, t_coast = -1, t_apogee = -1, t_drogue = -1, t_main = -1, t_landed = -1;
  float agl_at_main_fire = -1;
  bool saw_error = false;
  bool guidance_after_apogee = false;

  for (const SimStep& s : sim) {
    // barometer with ~0.15 m noise; the accelerometer reads |specific force| = |accel_g + 1| (g)
    harness_set_baro_alt(pad + (float)s.alt + 0.15f * noise());
    harness_set_accel_g(fabsf((float)s.accel_g + 1.0f));
    isStationary = false;
    const int runs_before = g_test_guidance_run_steps;
    const bool apogee_before_slice = t_apogee >= 0;   // apogee had already been detected before this 100 ms slice
    harness_run_ms(100);
    if (g_currentFlightState == ERROR) saw_error = true;
    if (t_boost < 0 && g_currentFlightState >= BOOST) t_boost = s.t;
    if (t_coast < 0 && g_currentFlightState >= COAST) t_coast = s.t;
    if (t_apogee < 0 && g_currentFlightState >= APOGEE) t_apogee = s.t;
    if (t_drogue < 0 && g_pin_ever_high[PYRO_CHANNEL_1]) t_drogue = s.t;
    if (t_main < 0 && g_pin_ever_high[PYRO_CHANNEL_2]) { t_main = s.t; agl_at_main_fire = (float)s.alt; }
    if (apogee_before_slice && g_test_guidance_run_steps != runs_before) guidance_after_apogee = true;
  }
  // touchdown: at rest on the ground
  harness_set_baro_alt(pad + 0.15f * noise());
  harness_set_accel_g(1.0f);
  isStationary = true;
  for (int i = 0; i < 200; i++) { harness_set_baro_alt(pad + 0.15f * noise()); harness_run_ms(100); if (t_landed < 0 && g_currentFlightState >= LANDED) t_landed = landing_t + i * 0.1; }

  char ev[256];
  snprintf(ev, sizeof ev, "sim: burnout %.1fs apogee %.1fs@%.0fm landing %.1fs | firmware: boost %.1fs coast %.1fs apogee %.1fs drogue %.1fs main %.1fs (at %.0f m AGL) landed %.1fs",
           burnout_t, apogee_t, apogee_alt, landing_t, t_boost, t_coast, t_apogee, t_drogue, t_main, agl_at_main_fire, t_landed);
  TEST_MESSAGE(ev);
  TEST_ASSERT_FALSE(saw_error);
  TEST_ASSERT_FALSE_MESSAGE(guidance_after_apogee, "guidance must not steer after apogee");

  // liftoff needs N fresh samples of >2 g (the motor is at 17 g from t=0)
  TEST_ASSERT_TRUE_MESSAGE(t_boost >= 0 && t_boost <= 1.0, "liftoff detected within 1 s");
  // burnout at 2.8 s, not during the gradual tail-off
  TEST_ASSERT_TRUE_MESSAGE(t_coast >= burnout_t - 0.3 && t_coast <= burnout_t + 1.2, "burnout detected near 2.8 s");
  // apogee at 19.8 s: detected by the sensors shortly after, well before the 20 s-after-burnout backup timer
  TEST_ASSERT_TRUE_MESSAGE(t_apogee >= apogee_t - 1.0 && t_apogee <= apogee_t + 2.5, "apogee detected within ~2.5 s of the true apogee");
  TEST_ASSERT_TRUE(t_apogee < t_coast + BACKUP_APOGEE_TIME_MS / 1000.0);      // by the sensors, not the backup timer
  TEST_ASSERT_TRUE(t_drogue >= t_apogee - 0.2);
  // main at ~100 m AGL (descending at 8.5 m/s, 3 samples of debounce + 0.1 s steps => up to ~5 m late)
  TEST_ASSERT_TRUE(t_main > t_drogue);
  TEST_ASSERT_TRUE_MESSAGE(agl_at_main_fire <= 105.0f && agl_at_main_fire >= 85.0f, "main deployed at 100 m AGL +-15 m");
  // landing after touchdown, then RECOVERY after LANDED_TIMEOUT
  TEST_ASSERT_TRUE_MESSAGE(t_landed >= landing_t - 1.0 && t_landed <= landing_t + 8.0, "landing detected within 8 s of touchdown");
  TEST_ASSERT_EQUAL(RECOVERY, g_currentFlightState);
  TEST_ASSERT_EQUAL(PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN, g_pyroFiredMask);
}

// The same flight with the barometer dying at apogee: the vehicle must still deploy the main (estimated
// time), and land, with no ERROR.
void test_replay_with_the_barometer_failing_at_apogee_still_deploys_the_main() {
  double burnout_t, apogee_t, apogee_alt, landing_t, main_deploy_alt_sim;
  const std::vector<SimStep> sim = run_sim(burnout_t, apogee_t, apogee_alt, landing_t, main_deploy_alt_sim);
  const float pad = 100.0f;
  harness_on_pad(pad);
  for (int i = 0; i < 30; i++) { harness_set_baro_alt(pad + 0.15f * noise()); harness_run_ms(100); }
  g_currentFlightState = ARMED; harness_pass();

  bool baro_dead = false;
  double t_main = -1; float alt_at_main = -1; bool saw_error = false;
  for (const SimStep& s : sim) {
    if (!baro_dead && s.t >= apogee_t + 6.0) baro_dead = true;       // dies 6 s after apogee, under the drogue
    harness_set_accel_g(fabsf((float)s.accel_g + 1.0f));
    isStationary = false;
    if (!baro_dead) harness_set_baro_alt(pad + (float)s.alt + 0.15f * noise());
    for (int k = 0; k < 10; k++) {                                   // 100 ms in 10 ms passes
      test_advance_ms(10);
      static unsigned ph = 0;
      if (((++ph) % 10) == 0) { if (!baro_dead) harness_new_baro_sample(); harness_new_accel_samples(); }
      harness_pass();
    }
    if (g_currentFlightState == ERROR) saw_error = true;
    if (t_main < 0 && g_pin_ever_high[PYRO_CHANNEL_2]) { t_main = s.t; alt_at_main = (float)s.alt; }
  }
  TEST_ASSERT_FALSE(saw_error);
  TEST_ASSERT_TRUE_MESSAGE(t_main > 0, "main must deploy on the fallback timer");
  TEST_ASSERT_TRUE_MESSAGE(alt_at_main > 150.0f, "deployed while still clear of the ground");
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_replay_of_the_simulated_h125w_flight_through_the_real_state_machine);
  RUN_TEST(test_replay_with_the_barometer_failing_at_apogee_still_deploys_the_main);
  return UNITY_END();
}
