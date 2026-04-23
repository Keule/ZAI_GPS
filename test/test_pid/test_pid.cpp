#include <unity.h>

#include "logic/control.h"

void test_pid_converges_to_setpoint(void) {
    PidState pid;
    pidInit(&pid, 3.0f, 0.1f, 0.01f, 0.0f, 65535.0f);

    float error = 10.0f;
    for (int i = 0; i < 200; i++) {
        const float output = pidCompute(&pid, error, 5);
        error -= (output * 0.001f);
        if (error < 0) error = -error;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, error);
}

void test_pid_output_clamped(void) {
    PidState pid;
    pidInit(&pid, 100.0f, 0.0f, 0.0f, 0.0f, 65535.0f);

    float output = pidCompute(&pid, 1000.0f, 5);
    TEST_ASSERT_TRUE(output <= 65535.0f);

    output = pidCompute(&pid, -1000.0f, 5);
    TEST_ASSERT_TRUE(output >= 0.0f);
}

void test_pid_anti_windup(void) {
    PidState pid;
    pidInit(&pid, 1.0f, 10.0f, 0.0f, 0.0f, 100.0f);

    for (int i = 0; i < 1000; i++) {
        (void)pidCompute(&pid, 50.0f, 5);
    }

    const float output = pidCompute(&pid, 50.0f, 5);
    TEST_ASSERT_TRUE(output <= 100.0f);
}

void test_pid_reset(void) {
    PidState pid;
    pidInit(&pid, 1.0f, 1.0f, 1.0f, 0.0f, 1000.0f);

    (void)pidCompute(&pid, 10.0f, 5);
    (void)pidCompute(&pid, 10.0f, 5);
    pidReset(&pid);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.prev_error);
    TEST_ASSERT_TRUE(pid.first_update);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_converges_to_setpoint);
    RUN_TEST(test_pid_output_clamped);
    RUN_TEST(test_pid_anti_windup);
    RUN_TEST(test_pid_reset);
    return UNITY_END();
}
