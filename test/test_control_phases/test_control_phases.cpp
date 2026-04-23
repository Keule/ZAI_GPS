#include <unity.h>

#include "mock/hal_mock.h"
#include "logic/control.h"
#include "logic/global_state.h"

void test_safety_ok_by_default(void) {
    hal_mock_set_millis(1000);
    hal_mock_set_safety(true);

    const bool ok = controlReadSafety();
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(g_nav.safety.safety_ok);
}

void test_safety_kick(void) {
    hal_mock_set_millis(1000);
    hal_mock_set_safety(false);

    const bool ok = controlReadSafety();
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_FALSE(g_nav.safety.safety_ok);
}

void test_watchdog_triggered(void) {
    TEST_ASSERT_FALSE(controlCheckWatchdog(5000, 4000));
    TEST_ASSERT_TRUE(controlCheckWatchdog(5000, 2000));
}

void test_controlstep_updates_state_and_actuator(void) {
    hal_mock_set_millis(10000);
    hal_mock_set_safety(true);
    hal_mock_set_imu(0.5f, 1.0f, 180.0f, true);
    hal_mock_set_was(15.0f, 500);

    {
        StateLock lock;
        g_nav.sw.work_switch = true;
        g_nav.sw.steer_switch = true;
        g_nav.sw.gps_speed_kmh = 5.0f;
        g_nav.sw.watchdog_timer_ms = 9500;
    }

    desiredSteerAngleDeg = 20.0f;
    controlStep();

    TEST_ASSERT_TRUE(g_nav.safety.safety_ok);
    TEST_ASSERT_EQUAL(500, g_nav.steer.steer_angle_raw);
    TEST_ASSERT_TRUE(hal_mock_get_actuator_cmd() >= 0);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_safety_ok_by_default);
    RUN_TEST(test_safety_kick);
    RUN_TEST(test_watchdog_triggered);
    RUN_TEST(test_controlstep_updates_state_and_actuator);
    return UNITY_END();
}
