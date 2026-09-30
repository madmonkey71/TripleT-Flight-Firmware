// Audit #14 - the watchdog must be fed around every slow setup step (SD init, log creation, sensor
// inits) so its 5 s window covers ONE step, while loop() keeps its single per-pass feed.
// Exercises the REAL setup_sequence.h; the setup()/loop() wiring is checked against the real source.
#include <unity.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "../support/flight_harness.h"     // stub Arduino core (fake clock) + config.h
#include "../../src/setup_sequence.h"

void setUp() { test_set_ms(100000); }
void tearDown() {}

// ---- a fake watchdog -----------------------------------------------------------------------------
static unsigned long g_last_feed = 0;
static int g_feeds = 0;
static std::vector<std::string> g_log;          // "feed" / step names in execution order
static bool g_expired = false;                  // the timeout elapsed at any observation point
static unsigned long g_max_gap = 0;

static void fake_feed() {
  const unsigned long gap = millis() - g_last_feed;
  if (gap > g_max_gap) g_max_gap = gap;
  if (gap > WATCHDOG_TIMEOUT_MS) g_expired = true;
  g_last_feed = millis();
  g_feeds++;
  g_log.push_back("feed");
}
static void reset_fake() { g_last_feed = millis(); g_feeds = 0; g_log.clear(); g_expired = false; g_max_gap = 0; }

struct Slow { const char* name; unsigned long ms; bool ok; };
static bool run_slow(void* ctx) {
  Slow* s = static_cast<Slow*>(ctx);
  test_advance_ms(s->ms);                        // the blocking library call
  g_log.push_back(s->name);
  return s->ok;
}

// The real bring-up costs (worst case on a slow card): every step is shorter than the timeout but the sum is not.
static Slow k_recover = {"recover", 10, true},   k_sd = {"sd", 3000, true},      k_gps = {"gps", 1500, true},
            k_baro = {"baro", 1000, true},       k_icm = {"icm", 1000, true},    k_kx = {"kx134", 200, true},
            k_filters = {"filters", 10, true},   k_log = {"log", 4000, true};
static SetupStep make(const char* n, Slow* s) { return SetupStep{n, run_slow, s}; }

void test_feed_happens_before_the_first_step_and_after_every_step() {
  reset_fake();
  const SetupStep steps[] = {make("a", &k_recover), make("b", &k_filters), make("c", &k_kx)};
  runSetupSteps(steps, 3, fake_feed);
  const std::vector<std::string> expect = {"feed", "recover", "feed", "filters", "feed", "kx134", "feed"};
  TEST_ASSERT_EQUAL(expect.size(), g_log.size());
  for (size_t i = 0; i < expect.size(); i++) TEST_ASSERT_EQUAL_STRING(expect[i].c_str(), g_log[i].c_str());
  TEST_ASSERT_EQUAL(4, g_feeds);
}

// The whole bring-up (>10 s) never lets the 5 s watchdog expire because each step is fed around.
void test_full_bring_up_longer_than_the_timeout_never_expires_the_watchdog() {
  reset_fake();
  const SetupStep steps[] = {make("recover", &k_recover), make("sd", &k_sd), make("gps", &k_gps), make("baro", &k_baro),
                             make("icm", &k_icm), make("kx", &k_kx), make("filters", &k_filters), make("log", &k_log)};
  const unsigned long t0 = millis();
  runSetupSteps(steps, 8, fake_feed);
  const unsigned long total = millis() - t0;
  TEST_ASSERT_TRUE(total > WATCHDOG_TIMEOUT_MS);                 // the sum would have expired an unfed watchdog...
  TEST_ASSERT_FALSE(g_expired);                                  // ...but it never did
  TEST_ASSERT_EQUAL(4000, g_max_gap);                            // the window only ever covered the slowest single step (log creation)
  TEST_ASSERT_TRUE(g_max_gap < WATCHDOG_TIMEOUT_MS);
}

// Demonstrates the old behaviour: the same bring-up with only the sensors feeding (SD / log creation not)
// - modelled here as no feeds between steps - expires the watchdog.
void test_without_the_step_feeds_the_same_bring_up_would_have_expired_the_watchdog() {
  reset_fake();
  const SetupStep steps[] = {make("sd", &k_sd), make("log", &k_log)};
  runSetupSteps(steps, 2, nullptr);                              // no feed function
  const unsigned long gap = millis() - g_last_feed;
  TEST_ASSERT_TRUE(gap > WATCHDOG_TIMEOUT_MS);
}

void test_a_failing_step_does_not_stop_bring_up_and_is_still_fed_around() {
  reset_fake();
  Slow bad_sd = {"sd", 500, false};
  const SetupStep steps[] = {make("sd", &bad_sd), make("gps", &k_gps), make("log", &k_log)};
  const int failures = runSetupSteps(steps, 3, fake_feed);
  TEST_ASSERT_EQUAL(1, failures);
  TEST_ASSERT_EQUAL(4, g_feeds);
  TEST_ASSERT_EQUAL_STRING("log", g_log[g_log.size() - 2].c_str());   // the later steps still ran
}

// ---- wiring in the real setup() / loop() ---------------------------------------------------------
static std::string read_main() {
  std::string path = __FILE__;
  path = path.substr(0, path.rfind('/')) + "/../../src/TripleT_Flight_Firmware.cpp";
  std::ifstream f(path);
  if (!f.good()) f.open("src/TripleT_Flight_Firmware.cpp");
  std::stringstream ss; ss << f.rdbuf();
  return ss.str();
}
static std::string body_of(const std::string& src, const char* head, const char* next_head) {
  size_t a = src.find(head);
  size_t b = src.find(next_head, a + 1);
  return (a == std::string::npos || b == std::string::npos) ? std::string() : src.substr(a, b - a);
}
static int count(const std::string& hay, const char* needle) {
  int n = 0; for (size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + 1)) n++; return n;
}

void test_setup_runs_sd_init_and_log_creation_through_the_fed_step_sequence() {
  const std::string src = read_main();
  const std::string setup = body_of(src, "void setup() {", "void loop() {");
  TEST_ASSERT_TRUE(!setup.empty());
  TEST_ASSERT_TRUE(setup.find("{\"sd card\",") != std::string::npos);
  TEST_ASSERT_TRUE(setup.find("{\"log file\",") != std::string::npos);
  TEST_ASSERT_TRUE(setup.find("runSetupSteps(kSetupSteps") != std::string::npos);
  // the steps themselves contain the slow calls
  TEST_ASSERT_TRUE(src.find("initSDCard(g_SD, g_sdCardMounted, g_sdCardPresent)") != std::string::npos);
  // the watchdog is started BEFORE the steps run (bring-up hangs are still recovered)
  TEST_ASSERT_TRUE(setup.find("wdt.begin(config)") < setup.find("runSetupSteps(kSetupSteps"));
  // and the slow calls are no longer bare statements in setup() itself
  TEST_ASSERT_EQUAL(0, count(setup, "initSDCard("));
  TEST_ASSERT_EQUAL(0, count(setup, "createNewLogFile("));
}

void test_the_in_flight_watchdog_is_unchanged() {
  const std::string src = read_main();
  const size_t loop_pos = src.find("void loop() {");
  const std::string loop_body = body_of(src, "void loop() {", "// Implementation of initSDCard function");
  TEST_ASSERT_TRUE(loop_pos != std::string::npos);
  TEST_ASSERT_EQUAL_MESSAGE(1, count(loop_body, "wdt.feed()"), "loop() must feed exactly once per pass");
  TEST_ASSERT_EQUAL(0, count(loop_body, "runSetupSteps"));
  // The feed is the first thing loop() does, so a hang anywhere later in the pass is still caught.
  TEST_ASSERT_TRUE(loop_body.find("wdt.feed()") < loop_body.find("pyro_service()"));
  TEST_ASSERT_EQUAL(5000, WATCHDOG_TIMEOUT_MS);                  // timeout not lengthened to make setup fit
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_feed_happens_before_the_first_step_and_after_every_step);
  RUN_TEST(test_full_bring_up_longer_than_the_timeout_never_expires_the_watchdog);
  RUN_TEST(test_without_the_step_feeds_the_same_bring_up_would_have_expired_the_watchdog);
  RUN_TEST(test_a_failing_step_does_not_stop_bring_up_and_is_still_fed_around);
  RUN_TEST(test_setup_runs_sd_init_and_log_creation_through_the_fed_step_sequence);
  RUN_TEST(test_the_in_flight_watchdog_is_unchanged);
  return UNITY_END();
}
