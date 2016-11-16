#ifndef _PNG_HPP__
#define _PNG_HPP__

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <stdexcept>
#include <Eigen/Core>

#include "../util/eigen_util.hpp"

//typedef Eigen::Map<Eigen::Matrix<Eigen::Matrix<uint8_t, 4, 1>,-1,-1,Eigen::RowMajor> > Eigen::MapRMatVec4Ub;
//typedef Eigen::Map<Eigen::Matrix<uint8_t,-1,-1,Eigen::RowMajor> > Eigen::RMatrixXUb;

uint8_t* read_png_file(const char* file_name, int* w, int* h, int* ct);
void write_png(const char* file_name, const uint8_t* c_raw_data, int width, int height, int color_type=0, int bit_depth=8);

namespace Util {

union ImageMat{
    Eigen::MapRMatVec4Ub* rgb;
    Eigen::MapRMatUb* bw;
};

/* There is always dynamicly allocated heap space, which won't be deleted in destructor
 * Take care for yourself!!
 */
class PngImageHolder {
private:
bool m_has_rgb_image;
ImageMat png;
public:
    PngImageHolder() {
        png.bw = nullptr;
    }
    PngImageHolder(const Eigen::MapRMatVec4Ub& rgb_)
    :m_has_rgb_image(true) {
        png.rgb = new Eigen::MapRMatVec4Ub(const_cast<Eigen::MapRMatVec4Ub::Scalar*>(rgb_.data()), rgb_.rows(), rgb_.cols());
    }
    PngImageHolder(const Eigen::MapRMatUb& bw_)
    :m_has_rgb_image(false) {
        png.bw = new Eigen::MapRMatUb(const_cast<Eigen::MapRMatUb::Scalar*>(bw_.data()), bw_.rows(), bw_.cols());
    }
    PngImageHolder(const PngImageHolder& other)
    :m_has_rgb_image(other.m_has_rgb_image) {
        png.bw = other.png.bw;
        const_cast<PngImageHolder&>(other).png.bw = nullptr;
    }
    PngImageHolder(PngImageHolder& other) {
        m_has_rgb_image = other.m_has_rgb_image;
        png.bw = other.png.bw;
        other.png.bw = nullptr;
    }
    PngImageHolder& operator=(const PngImageHolder& other) {
        m_has_rgb_image = other.m_has_rgb_image;
        png.bw = other.png.bw;
        const_cast<PngImageHolder&>(other).png.bw = nullptr;
        return *this;
    }

    ~PngImageHolder() {
        delete png.rgb;
    }

    void delete_internal_data() {
        delete (uint8_t*)png.bw->data();
        png.bw = nullptr;
    }

    bool has_rgb_image() const {
        return m_has_rgb_image;
    }

    const Eigen::MapRMatVec4Ub& get_bgra_image() const {
        assert(png.bw != nullptr);
        assert(has_rgb_image());
        return *(png.rgb);
    }

    const Eigen::MapRMatUb& get_bw_image() const {
        assert(png.bw != nullptr);
        assert(!has_rgb_image());
        return *(png.bw);
    }

    static uint8_t* bgra_to_rgb(const Eigen::MapRMatVec4Ub& bgra) {
        unsigned size = sizeof(Eigen::Vector3Ub) * bgra.cols() * bgra.rows();
        uint8_t* rgb = new uint8_t[size], *rgb_end = rgb + size - 1;
        for(int i = 0; i < bgra.rows(); ++i) {
            for(int j = 0; j < bgra.cols(); ++j) {
                Eigen::Vector3Ub* n = &(((Eigen::Vector3Ub*)rgb)[i * bgra.cols() + j]);
                assert(i * bgra.cols() + j < bgra.cols() * bgra.rows());
                assert((uint8_t*)n < rgb_end);
                *n = (Eigen::Vector3Ub() << bgra(i, j)(2), bgra(i, j)(1), bgra(i, j)(0)).finished();
                #ifndef NDEBUG
                void* m = (void*)&(rgb[3 * j + 3 * i * bgra.cols()]);
                //printf("Pointer %lu soll, aber %lu bei %d, %d\n", (uint64_t)n, (uint64_t)m, i, j);
                assert((void*)n == m);
                #endif
            }
        }
        return rgb;
    }

};

static inline uint8_t* bwa_to_bw_in_place(uint8_t* bwa, unsigned size) {
    for(unsigned r_idx = 2, w_idx = 1; r_idx < size; r_idx += 2, w_idx += 2) {
        bwa[w_idx] = bwa[r_idx];
    }
    return bwa;
}

static Util::PngImageHolder load_png(const char* filename) {
    int width, height, color_type;
    uint8_t* png_data = read_png_file(filename, &width, &height, &color_type);
    //printf("width %d height %d \n", width, height);
    if(color_type == 0) {
        Eigen::MapRMatUb tmp(bwa_to_bw_in_place((uint8_t*)png_data, height * width), height, width);
        return Util::PngImageHolder(tmp);
    } else if(color_type == 4) {
        Eigen::MapRMatUb tmp((uint8_t*) png_data, height, width);
        return Util::PngImageHolder(tmp);
    } else if (color_type == 6){
        Eigen::MapRMatVec4Ub tmp((Eigen::Vector4Ub*) png_data, height, width);
        return Util::PngImageHolder(tmp);
    } else {
        throw std::runtime_error("Unknown color_type");
    }
}

static inline Util::PngImageHolder load_png(const std::string& filename) {
    return load_png(filename.data());
}

static void write_png_file(const char* file_name, const Eigen::RMatrixXUb& image) {
    write_png(file_name, (const uint8_t*)image.data(), image.cols(), image.rows(), 0, 8);
}

} //namespace Util

#endif //_PNG_HPP__
