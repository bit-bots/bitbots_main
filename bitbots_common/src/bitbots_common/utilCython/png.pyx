from cython.operator cimport address as ref
from libcpp.string cimport string

cimport numpy as np
np.import_array()

from eigen cimport *


cdef class PngImage:

    def __cinit__(self, filename):
        cdef string n = str(filename)
        self.png = load_png(n.data())

    def __dealloc__(self):
        self.png.delete_internal_data()

    def get_png_as_numpy_array(self):
        cdef np.npy_intp shape[3]

        cdef int cn = 2
        cdef np.ndarray  png
        if self.png.has_rgb_image():
            shape[0] = self.png.get_bgra_image().rows()
            shape[1] = self.png.get_bgra_image().cols()
            shape[2] = 1
            png = np.PyArray_SimpleNewFromData(cn, shape, np.NPY_UBYTE, \
                    bgra_to_rgb(self.png.get_bgra_image()))
            return png
        else:
            shape[0] = self.png.get_bw_image().rows()
            shape[1] = self.png.get_bw_image().cols()
            shape[2] = 3
            png = np.PyArray_SimpleNewFromData(cn, shape, np.NPY_UBYTE, \
                    mutable_uchar_ptr(self.png.get_bw_image().data()))
            return png.copy()


cpdef save_png_image(filename, np.ndarray arr):
    if arr.shape[3] != 1:
        raise ValueError("Unsupported shape")

    cdef string n = str(filename)
    cdef char* pixels = np.PyArray_BYTES(arr)
    cdef unsigned char* upixels = <unsigned char*> pixels
    write_png_file(n.data(), <const RMatrixXUb&>MapRMatrixXUb(upixels, int(arr.shape[0]), int(arr.shape[1])))
