#include <unity.h>
#include <Arduino.h>

// Mock or include the actual header if it's pure C/C++ logic compatible with native
// For this example, we will test a hypothetical math utility or the kalman logic
// Since kalman_filter.cpp uses <math.h> it should be mostly portable.

#include "kalman_filter.h"

// Set up logic before every test
void setUp(void) {
    kalman_init(0.0f, 0.0f, 0.0f);
}

// Clean up logic after every test
void tearDown(void) {
}

void test_kalman_initialization(void) {
    float r, p, y;
    kalman_get_orientation(r, p, y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, y);
}

void test_kalman_predict_simple(void) {
    // Predict with 1 rad/s roll rate for 1 second
    kalman_predict(1.0f, 0.0f, 0.0f, 1.0f);

    float r, p, y;
    kalman_get_orientation(r, p, y);

    // Should be close to 1.0 radian
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, r);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, y);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_kalman_initialization);
    RUN_TEST(test_kalman_predict_simple);
    UNITY_END();
    return 0;
}
