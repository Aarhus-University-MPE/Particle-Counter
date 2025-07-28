#include <Arduino.h>
#include <SD.h>
#include <unity.h>

#define SD_CS 10  // Change if your SD card CS pin is different

void test_sd_card_init() {
    bool initResult = SD.begin(SD_CS);
    TEST_ASSERT_MESSAGE(initResult, "SD card initialization failed!");
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_sd_card_init);
    UNITY_END();
}

void loop() {
    // not used in unit test
}