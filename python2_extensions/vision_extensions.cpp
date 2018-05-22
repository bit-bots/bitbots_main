#include <Python.h>
#include "numpy/arrayobject.h"
#include <iostream>
#include <csignal>

static PyObject* maskImg(PyObject *self, PyObject *args) {
    PyArrayObject *image, *mask;
     

    if (!PyArg_ParseTuple(args, "O!O!", &PyArray_Type, &image, &PyArray_Type, &mask))
        return NULL;
    if (image->nd != 3 || image->descr->type_num != PyArray_UBYTE) {
        PyErr_SetString(PyExc_ValueError,
            "image must be three-dimensional and of type numpy.uint8");
        return NULL;
    }
    if (mask->nd != 3 || mask->descr->type_num != PyArray_UBYTE) {
        PyErr_SetString(PyExc_ValueError,
            "mask must be three-dimensional and of type numpy.uint8");
        return NULL;
    }
    

    npy_intp maskDims[] = {image->dimensions[0], image->dimensions[1]};
    int maskDimsNd = 2; // two dimensional output image
    PyArrayObject* maskedImg = (PyArrayObject*) PyArray_SimpleNew(maskDimsNd, maskDims, PyArray_UBYTE);
    unsigned char* pixel; 
    for (int y = 0; y < image->dimensions[0]; y++) {
        for (int x = 0; x < image->dimensions[1]; x++) {
            pixel = (unsigned char*) image->data + y * image->strides[0] + x * image->strides[1];
            if (*(mask->data + pixel[0] * mask->strides[0] + pixel[1] * mask->strides[1] + pixel[2] * mask->strides[2])) {
                *(maskedImg->data + y * maskedImg->strides[0] + x * maskedImg->strides[1]) = 255;
            }
            else {
                *(maskedImg->data + y * maskedImg->strides[0] + x * maskedImg->strides[1]) = 0;
            }
        }
    }

    return PyArray_Return(maskedImg);
}

static PyMethodDef VisionMethods[] = {
    {"maskImg", maskImg, METH_VARARGS, ""},
    {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC initvision(void) {
    import_array();
    (void) Py_InitModule("vision", VisionMethods);
}

int main(int argc, char *argv[]) {

    /* Pass argv[0] to the Python interpreter */
    Py_SetProgramName(argv[0]);

    /* Initialize the Python interpreter.  Required. */
    Py_Initialize();

    initvision();

    return 0;
}
