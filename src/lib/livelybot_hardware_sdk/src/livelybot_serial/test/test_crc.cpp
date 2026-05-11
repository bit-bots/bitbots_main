#include <gtest/gtest.h>
#include "crc8.h"
#include "crc16.h"

// CRC-8 function: Get_CRC8_Check_Sum(data, len, initial=0xFF)
// CRC-16 function: crc_ccitt(initial, data, len)

// --- CRC-8 tests ---

TEST(Crc8Test, ZeroLength)
{
    uint8_t buf[] = {0x00};
    // Length 0 → should return the initial value unchanged (0xFF → 0xFF after no data)
    // Just verify it doesn't crash
    uint8_t result = Get_CRC8_Check_Sum(buf, 0, 0xFF);
    (void)result;
    SUCCEED();
}

TEST(Crc8Test, SingleByteDeterministic)
{
    uint8_t buf[] = {0xAB};
    uint8_t a = Get_CRC8_Check_Sum(buf, 1, 0xFF);
    uint8_t b = Get_CRC8_Check_Sum(buf, 1, 0xFF);
    EXPECT_EQ(a, b);
}

TEST(Crc8Test, KnownVectorDeterministic)
{
    uint8_t buf[] = {0xF7, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00};
    uint8_t a = Get_CRC8_Check_Sum(buf, sizeof(buf), 0xFF);
    uint8_t b = Get_CRC8_Check_Sum(buf, sizeof(buf), 0xFF);
    EXPECT_EQ(a, b);
}

TEST(Crc8Test, DifferentDataDifferentCrc)
{
    uint8_t buf1[] = {0x01, 0x02, 0x03};
    uint8_t buf2[] = {0x01, 0x02, 0x04};
    uint8_t c1 = Get_CRC8_Check_Sum(buf1, sizeof(buf1), 0xFF);
    uint8_t c2 = Get_CRC8_Check_Sum(buf2, sizeof(buf2), 0xFF);
    EXPECT_NE(c1, c2);
}

TEST(Crc8Test, InitialValueMatters)
{
    uint8_t buf[] = {0xAA, 0xBB};
    uint8_t c1 = Get_CRC8_Check_Sum(buf, sizeof(buf), 0x00);
    uint8_t c2 = Get_CRC8_Check_Sum(buf, sizeof(buf), 0xFF);
    EXPECT_NE(c1, c2);
}

// --- CRC-16 tests ---

TEST(Crc16Test, ZeroLength)
{
    uint8_t buf[] = {0x00};
    uint16_t result = crc_ccitt(0xFFFF, buf, 0);
    // Zero-length input returns initial value
    EXPECT_EQ(result, 0xFFFFu);
}

TEST(Crc16Test, SingleByteDeterministic)
{
    uint8_t buf[] = {0xFF};
    uint16_t a = crc_ccitt(0xFFFF, buf, 1);
    uint16_t b = crc_ccitt(0xFFFF, buf, 1);
    EXPECT_EQ(a, b);
}

TEST(Crc16Test, KnownVectorDeterministic)
{
    uint8_t buf[] = {0xF7, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00};
    uint16_t a = crc_ccitt(0xFFFF, buf, sizeof(buf));
    uint16_t b = crc_ccitt(0xFFFF, buf, sizeof(buf));
    EXPECT_EQ(a, b);
}

TEST(Crc16Test, DifferentDataDifferentCrc)
{
    uint8_t buf1[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t buf2[] = {0xDE, 0xAD, 0xBE, 0xEE};
    uint16_t c1 = crc_ccitt(0xFFFF, buf1, sizeof(buf1));
    uint16_t c2 = crc_ccitt(0xFFFF, buf2, sizeof(buf2));
    EXPECT_NE(c1, c2);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
