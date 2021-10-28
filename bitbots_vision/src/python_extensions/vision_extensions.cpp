#include <Python.h>
#include "numpy/arrayobject.h"
#include <iostream>
#include <csignal>
#include <vector>
#include <utility>
#include <algorithm>
#include <stdexcept>
using namespace std;

// Definition of a candidate
typedef struct Candidate{
    int rx, lx, ly, uy;
} Candidate;


static PyObject* maskImg(PyObject *self, PyObject *args) {
    // Declare image and mask arrays
    PyArrayObject *image, *mask;

    // Parse parameters from python function call
    if (!PyArg_ParseTuple(args, "O!O!", &PyArray_Type, &image, &PyArray_Type, &mask))
        return NULL;
    // Check image array shape and dtype
    if (image->nd != 3 || image->descr->type_num != PyArray_UBYTE) {
        PyErr_SetString(PyExc_ValueError,
            "image must be three-dimensional and of type numpy.uint8");
        return NULL;
    }
    // Check color lookup table array shape and dtype
    if (mask->nd != 3 || mask->descr->type_num != PyArray_UBYTE) {
        PyErr_SetString(PyExc_ValueError,
            "mask must be three-dimensional and of type numpy.uint8");
        return NULL;
    }

    // Shape for mask
    npy_intp maskDims[] = {image->dimensions[0], image->dimensions[1]};
    // Binary image mask is two dimensional (obviously)
    int maskDimsNd = 2;
    // Create new python array for the image mask
    PyArrayObject* maskedImg = (PyArrayObject*) PyArray_SimpleNew(maskDimsNd, maskDims, PyArray_UBYTE);
    // Define a pixel
    unsigned char* pixel;
    // Iterate over the whole image
    for (int y = 0; y < image->dimensions[0]; y++) {
        for (int x = 0; x < image->dimensions[1]; x++) {
            // Get pixel value
            pixel = (unsigned char*) image->data + y * image->strides[0] + x * image->strides[1];
            // Check if pixel is in color lookup table
            if (*(mask->data + pixel[0] * mask->strides[0] + pixel[1] * mask->strides[1] + pixel[2] * mask->strides[2])) {
                // Set this pixel in the image mask to 255
                *(maskedImg->data + y * maskedImg->strides[0] + x * maskedImg->strides[1]) = 255;
            }
            else {
                // Set this pixel in the image mask to 0 if the pixel is not in the lookup table
                *(maskedImg->data + y * maskedImg->strides[0] + x * maskedImg->strides[1]) = 0;
            }
        }
    }

    // Return the image mask
    return PyArray_Return(maskedImg);
}

// Define methods for this module
static PyMethodDef VisionMethods[] = {
    {"maskImg", maskImg, METH_VARARGS, ""},
    {NULL, NULL, 0, NULL}
};

// Python 3 stuff
#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef visionmodule = {
    PyModuleDef_HEAD_INIT,
    "VisionExtensions",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    VisionMethods
};

PyMODINIT_FUNC
PyInit_VisionExtensions(void) {
    import_array();
    return PyModule_Create(&visionmodule);
}
#else
// Python 2 stuff
PyMODINIT_FUNC initVisionExtensions(void) {
    import_array();
    (void) Py_InitModule("VisionExtensions", VisionMethods);
}
#endif

int main(int argc, char *argv[]) {
#if PY_MAJOR_VERSION >= 3
    wchar_t *program = Py_DecodeLocale(argv[0], NULL);
    if (program == NULL) {
        fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
        exit(1);
    }

    /* Add a built-in module, before Py_Initialize */
    PyImport_AppendInittab("VisionExtensions", PyInit_VisionExtensions);
#else
    char *program = argv[0];
#endif

    /* Pass argv[0] to the Python interpreter */
    Py_SetProgramName(program);

    /* Initialize the Python interpreter.  Required. */
    Py_Initialize();

#if PY_MAJOR_VERSION >= 3
    /* Optionally import the module; alternatively,
       import can be deferred until the embedded script
       imports it. */
    PyImport_ImportModule("VisionExtensions");


    PyMem_RawFree(program);
#else
    initVisionExtensions();
#endif
    return 0;
}
