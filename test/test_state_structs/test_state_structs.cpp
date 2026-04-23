#include <unity.h>

#include "logic/state_structs.h"
#include "logic/global_state.h"

void test_navigationstate_substructs_exist(void) {
    NavigationState nav;
    nav.imu.heading_deg = 1.0f;
    nav.steer.steer_angle_deg = 2.0f;
    nav.sw.work_switch = true;
    nav.pid.pid_output = 100;
    nav.safety.safety_ok = true;
    nav.gnss.gps_fix_quality = 4;

    TEST_ASSERT_EQUAL_FLOAT(1.0f, nav.imu.heading_deg);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, nav.steer.steer_angle_deg);
    TEST_ASSERT_TRUE(nav.sw.work_switch);
    TEST_ASSERT_EQUAL(100, nav.pid.pid_output);
    TEST_ASSERT_TRUE(nav.safety.safety_ok);
    TEST_ASSERT_EQUAL(4, nav.gnss.gps_fix_quality);
}

void test_navigationstate_size_reasonable(void) {
    static_assert(sizeof(NavigationState) <= 256,
                  "NavigationState too large — check padding/bloat");
    TEST_ASSERT_TRUE(sizeof(NavigationState) <= 256);
}

void test_state_sizes_reasonable(void) {
    TEST_ASSERT_TRUE(sizeof(ImuState) <= 32);
    TEST_ASSERT_TRUE(sizeof(SteerState) <= 16);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_navigationstate_substructs_exist);
    RUN_TEST(test_navigationstate_size_reasonable);
    RUN_TEST(test_state_sizes_reasonable);
    return UNITY_END();
}
