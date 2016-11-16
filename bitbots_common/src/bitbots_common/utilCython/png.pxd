from libcpp cimport bool
cimport numpy as np
from eigen cimport *

cdef extern from "png.hpp" namespace "Util":
    cppclass PngImageHolder:
        PngImageHolder()
        PngImageHolder(const PngImageHolder& other)
        bool has_rgb_image()
        #get_brga_image()
        Map[RMatrixXUb, unsigned char]& get_bw_image()
        MapRMatVec4Ub& get_bgra_image()
        void delete_internal_data()

    unsigned char* bgra_to_rgb "Util::PngImageHolder::bgra_to_rgb" (const MapRMatVec4Ub& bgra)

    PngImageHolder load_png(const char* filename)
    unsigned char* get_uchar_data_from_map(const Map[RMatrixXUb, unsigned char]&)
    unsigned char* get_uchar_data_from_map(const Map[RMatrixVec3Ub, Matrix[char]]&)
    void write_png_file(const char* filename, const RMatrixXUb& image)

cdef class PngImage:
    cdef PngImageHolder png

cpdef save_png_image(filename, np.ndarray arr)
