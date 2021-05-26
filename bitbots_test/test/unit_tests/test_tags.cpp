#include "bitbots_test.h"
#include <cstdlib>
#include <gtest/gtest.h>
#include <boost/algorithm/string.hpp>

class TagTestFixture : public ::testing::Test {
public:
    static void set_env_tags(const std::set<std::string> &tags) {
        std::string joined = boost::algorithm::join(tags, ",");
        setenv("TEST_TAGS", joined.c_str(), 1);
    }
};

TEST_F(TagTestFixture, testTaggedTestRunsIfNoTagsAreSpecified) {
    set_env_tags({});
    ASSERT_TRUE(bitbots_test::should_tags_run({"hello", "world"}));
}

TEST_F(TagTestFixture, testTaggedTestGetsRunIfTagIsRequested) {
    set_env_tags({"hello"});
    ASSERT_TRUE(bitbots_test::should_tags_run({"hello", "world"}));
}

TEST_F(TagTestFixture, testTaggedTestDoesNotGetRunIfTagIsNotRequested) {
    set_env_tags({"some", "other", "tags"});
    ASSERT_FALSE(bitbots_test::should_tags_run({"hello", "world"}));
}

TEST_F(TagTestFixture, testTaggedTestGetsRunIfTagIsNotForbiddenAndNoTagsAreRequested) {
    set_env_tags({"!some", "!other", "!tags"});
    ASSERT_TRUE(bitbots_test::should_tags_run({"hello", "world"}));
}

TEST_F(TagTestFixture, testTaggedTestDoesNotGetRunIfTagIsForbiddenAndNoTagsAreRequested) {
    set_env_tags({"!hello", "!some", "!other", "!tags"});
    ASSERT_FALSE(bitbots_test::should_tags_run({"hello", "world"}));
}

TEST_F(TagTestFixture, testTaggedTestDoesNotGetRunIfTagIsForbiddenAndRequested) {
    set_env_tags({"!hello", "world"});
    ASSERT_FALSE(bitbots_test::should_tags_run({"hello", "world"}));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
