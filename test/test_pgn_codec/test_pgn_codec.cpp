#include <unity.h>

#include "logic/pgn_codec.h"
#include "logic/pgn_types.h"

void test_pgn250_encoding(void) {
    uint8_t buf[32] = {};
    const uint8_t sensor_val = 0x42;

    const size_t len = pgnEncodeFromAutosteer2(buf, sizeof(buf), sensor_val);
    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_EQUAL_HEX8(0x80, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x81, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(250, buf[3]);
}

void test_pgn253_known_values(void) {
    uint8_t buf[32] = {};
    const size_t len = pgnEncodeSteerStatusOut(buf, sizeof(buf),
                                               1500, 1800, 25,
                                               0x83, 128);
    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_EQUAL_HEX8(0x80, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x81, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(253, buf[3]);
}

void test_pgn_hardware_message(void) {
    uint8_t buf[64] = {};
    const size_t len = pgnEncodeHardwareMessage(buf, sizeof(buf),
                                                0x7E, 5, 1,
                                                "ERR IMU: Not Detected");
    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_EQUAL_HEX8(0xDD, buf[3]);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_pgn250_encoding);
    RUN_TEST(test_pgn253_known_values);
    RUN_TEST(test_pgn_hardware_message);
    return UNITY_END();
}
