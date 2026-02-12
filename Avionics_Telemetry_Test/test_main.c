//test seneryoları ve main fonklar
#include <stdio.h>
#include <string.h>
#include "unity/unity.h"
#include "telemetry.h"
void setUp(void) {}
void tearDown(void) {}
void test_Data_Serialization()
{
    FlyData_t telemtryData;
    uint8_t buffer[sizeof(FlyData_t)];

    telemtryData.timeStamp = 0x11223344; 
    telemtryData.pitch = 1.0f;          
    telemtryData.roll = 0.0f;
    telemtryData.altitude = 100.5f;
    telemtryData.status = 0xFF;

    copyStruct(&telemtryData,buffer);

    TEST_ASSERT_EQUAL_INT(17, sizeof(FlyData_t));

    TEST_ASSERT_EQUAL_HEX8(0x44,buffer[0]);
    TEST_ASSERT_EQUAL_HEX8(0x33,buffer[1]);
    TEST_ASSERT_EQUAL_HEX8(0x22,buffer[2]);
    TEST_ASSERT_EQUAL_HEX8(0x11,buffer[3]);

    TEST_ASSERT_EQUAL_HEX8(0xFF,buffer[16]);
    printf("Test Basarili: Veri paketi boyutu: %lu byte\n", sizeof(telemtryData));
}
int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_Data_Serialization);
    return UNITY_END();
}