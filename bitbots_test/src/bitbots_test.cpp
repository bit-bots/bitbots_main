#include "bitbots_test.h"

namespace bitbots_test {
    namespace impl {
        std::set<std::string> get_env_tags(bool negated) {
            auto env = std::getenv("TEST_TAGS");
            if (env == nullptr) {
                return std::set<std::string>();
            }
            std::stringstream env_stream(env);
            std::set<std::string> result;

            std::string item;
            while (std::getline(env_stream, item, ',')) {
                if (negated && item.at(0) == '!') {
                    result.insert(item.substr(1));
                } else if (!negated && item.at(0) != '!') {
                    result.insert(item);
                }
            }

            return result;
        }

        bool is_test_requested(const std::set<std::string>& applied_tags) {
            auto env_tags = get_env_tags(false);
            std::set<std::string> intersection;
            auto matching_tags = std::set_intersection(
                    applied_tags.begin(), applied_tags.end(),
                    env_tags.begin(), env_tags.end(),
                    std::inserter(intersection, intersection.begin()));

            return !intersection.empty();
        }

        bool is_test_forbidden(const std::set<std::string>& applied_tags) {
            auto env_tags = get_env_tags(true);
            std::set<std::string> intersection;
            auto matching_tags = std::set_intersection(
                    applied_tags.begin(), applied_tags.end(),
                    env_tags.begin(), env_tags.end(),
                    std::inserter(intersection, intersection.begin()));

            return !intersection.empty();
        }
    }

    bool should_tags_run(const std::set<std::string>& tags) {
        // when no tags are specified by the user, the test should simply run
        if (impl::get_env_tags(false).empty() && impl::get_env_tags(true).empty())
            return true;

        // when only a positive list of tags is specified by the user,
        // the test should only run if one of the `tags` is requested
        else if (!impl::get_env_tags(false).empty() && impl::get_env_tags(true).empty())
            return impl::is_test_requested(tags);

        // when only forbidden tags are specified by the user,
        // the test should run by default but not if one of the `tags` is forbidden
        else if (impl::get_env_tags(false).empty() && !impl::get_env_tags(true).empty())
            return !impl::is_test_forbidden(tags);

        // when specifying positive tags as well as forbidden tags,
        // first tests will only run if they are requested but not if they are forbidden as well
        else
            return impl::is_test_requested(tags) && !impl::is_test_forbidden(tags);
    }
}
