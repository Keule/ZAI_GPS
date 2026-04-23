#include <unity.h>

#include "logic/dependency_policy.h"

void test_isFresh_within_timeout(void) {
    TEST_ASSERT_TRUE(dep_policy::isFresh(1000, 999, 300));
    TEST_ASSERT_TRUE(dep_policy::isFresh(1000, 700, 300));
    TEST_ASSERT_FALSE(dep_policy::isFresh(1000, 699, 300));
    TEST_ASSERT_FALSE(dep_policy::isFresh(1000, 0, 300));
}

void test_steer_angle_plausible(void) {
    TEST_ASSERT_TRUE(dep_policy::isSteerAnglePlausible(0.0f));
    TEST_ASSERT_TRUE(dep_policy::isSteerAnglePlausible(-40.0f));
    TEST_ASSERT_TRUE(dep_policy::isSteerAnglePlausible(40.0f));
    TEST_ASSERT_FALSE(dep_policy::isSteerAnglePlausible(999.0f));
}

void test_heading_plausible(void) {
    TEST_ASSERT_TRUE(dep_policy::isHeadingPlausible(0.0f));
    TEST_ASSERT_TRUE(dep_policy::isHeadingPlausible(359.99f));
    TEST_ASSERT_FALSE(dep_policy::isHeadingPlausible(9999.0f));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_isFresh_within_timeout);
    RUN_TEST(test_steer_angle_plausible);
    RUN_TEST(test_heading_plausible);
    return UNITY_END();
}
