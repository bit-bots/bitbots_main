#include <gtest/gtest.h>
#include <cstdlib>

#include "../../util/eigen_util.hpp"
#include "../png.hpp"

namespace Test {

TEST(PNG, bgra_to_rgb) {
    uint8_t orgbytes[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    uint8_t refbytes[] = {2,1,0,  6,5,4,  10,9,8,  14,13,12};

    uint8_t* result = Util::PngImageHolder::bgra_to_rgb(Eigen::MapRMatVec4Ub((Eigen::Vector4Ub*)orgbytes, 2, 2));

    for(unsigned i = 0; i < 12; ++i) {
        ASSERT_EQ(refbytes[i], result[i]);
    }
    delete[] result;
    ++result;
}

TEST(PNG, bgra_to_rgb_big) {
    unsigned h = 500, w = 1000;
    unsigned size3 = 3 * h * w;
    unsigned size4 = 4 * h * w;
    uint8_t* orgbytes = new uint8_t[size4];
    uint8_t* refbytes = new uint8_t[size3];

    //std::srand(1337);

    for(unsigned i = 0; i < h; ++i) {
        for(unsigned j = 0; j < w; ++j) {
            for(uint8_t k = 0; k < 3; ++k) {
                uint8_t r = i + j + k;
                unsigned idx3 = 3 * h * j + 3 * i + (2 - k);
                unsigned idx4 = 4 * h * j + 4 * i + k;
                ASSERT_TRUE(idx3 < size3);
                ASSERT_TRUE(idx4 < size4);
                orgbytes[idx4] = r;
                refbytes[idx3] = r;
            }
            orgbytes[4 * h * j + 4 * i + 3] = i + j + 4;
        }
    }

    uint8_t* result = Util::PngImageHolder::bgra_to_rgb(Eigen::MapRMatVec4Ub((Eigen::Vector4Ub*)orgbytes, h, w));

    for(unsigned i = 0; i < h; ++i) {
        for(unsigned j = 0; j < w; ++j) {
            for(uint8_t k = 0; k < 3; ++k) {
                unsigned idx = 3 * h * j + 3 * i + k;
                ASSERT_TRUE(idx < size3);
                ASSERT_EQ(refbytes[idx], result[idx])<<"i is " << i << " j is " << j << " k is " << (unsigned)k << " idx is " << idx;
            }
        }
    }
    delete[] result;
    delete[] orgbytes;
    delete[] refbytes;
    ++result;
}

} //namespace Test

using namespace Test;

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
