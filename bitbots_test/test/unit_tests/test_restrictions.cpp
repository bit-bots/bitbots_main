#include "bitbots_test.h"
#include <cstdlib>
#include <gtest/gtest.h>

class RestrictionTestFixture: public ::testing::Test {
public:
    static void set_on_robot(bool value) {
        if (value) {
            setenv("IS_ROBOT", "true", 1);
        } else {
            setenv("IS_ROBOT", "", 1);
        }
    }
};

TEST_F(RestrictionTestFixture, testIsOnRobot) {
    // positive test
    set_on_robot(true);
    EXPECT_TRUE(bitbots_test::is_on_robot());

    // negative test
    set_on_robot(false);
    EXPECT_FALSE(bitbots_test::is_on_robot());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
