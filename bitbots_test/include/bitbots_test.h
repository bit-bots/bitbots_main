#ifndef BITBOTS_TEST_BITBOTS_TEST_H
#define BITBOTS_TEST_BITBOTS_TEST_H

#include <set>
#include <algorithm>
#include <string>
#include <sstream>

namespace bitbots_test {
    namespace impl {
        /**
         * Retrieve which tags the user supplied via the environment variable `TEST_TAGS`
         * @param negated Whether only negated tags (prefixed with !) or only requested tags are returned
         */
        std::set<std::string> get_env_tags(bool negated);

        bool is_test_requested(const std::set<std::string> &applied_tags);

        bool is_test_forbidden(const std::set<std::string> &applied_tags);
    }

    /**
     * Check whether a test with the supplied set of tags should run.
     *
     * This will enable the user to filter test execution to a subset of tests by specifying a list of tags to run as well
     * as a list of tags to forbid. This is done defining the environment variable *TEST_TAGS* as a comma separated list.
     *
     * Behavior when specifying tags on test execution:
     *
     * - When no tags are specified by the user, all tests will be run and this decorator has no effect at all.
     * - When only a simple list of tags is specified by the user, only tests which have this tag will be run.
     * - When only forbidden tags are specified by the user, tests with these tags will not run but all other tests will.
     * - When specifying a list of tags as well as forbidden tags, first tests will be filtered to the allowed tags and then
     *   again all tests with forbidden tags will be removed.
     *
     * @return Whether this test should be executed right now or not
     */
    bool should_tags_run(const std::set<std::string> &tags);
}

#define TEST_TAGS(tags) if (!bitbots_test::should_tags_run(tags)) { GTEST_SKIP(); }

#endif //BITBOTS_TEST_BITBOTS_TEST_H
