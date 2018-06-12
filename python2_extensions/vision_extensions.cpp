#include <Python.h>
#include "numpy/arrayobject.h"
#include <iostream>
#include <csignal>
#include <vector>
#include <utility>
#include <algorithm>
#include <stdexcept>
using namespace std;

typedef struct Candidate{
    int rx, lx, ly, uy;
} Candidate;

static bool pointInCandidate(std::pair<int, int> point, int lx, int uy, int rx, int ly)
{
    return (point.first <= rx && point.first >= lx) && (point.second <= ly && point.second >= uy);
}

static PyObject* findSpots(PyObject *self, PyObject *args)
{
    PyArrayObject *binary_image;
    int pointcloud_stepsize, expand_stepsize, refinement_iterations;

    if (!PyArg_ParseTuple(args, "O!iii", &PyArray_Type, &binary_image, &pointcloud_stepsize, &expand_stepsize, &refinement_iterations))
        return NULL;
    if (binary_image->nd != 2 || binary_image->descr->type_num != PyArray_UBYTE) {
        PyErr_SetString(PyExc_ValueError,
            "image must be two-dimensional and of type numpy.uint8");
        return NULL;
    }

    // Build point grid
    std::vector< std::pair<int, int> > points;
    int y = pointcloud_stepsize / 2 , x;
    while (y < binary_image->dimensions[0])
    {
        x = pointcloud_stepsize / 2;
        while (x < binary_image->dimensions[1])
        {
            points.push_back({x, y});
            x += pointcloud_stepsize;
        }
        y += pointcloud_stepsize;
    }
    std::vector<Candidate> candidates;
    // used for every single point
    std::pair<int, int> point;
    int lx, uy, rx, ly, next;
    // iterate through points (gets empty faster when points in candidates get sorted out earlier, thus no for is used)
    while (! points.empty())
    {
        // get and remove the last point of the vector
        point = points.back();
        points.pop_back();


        // std::cout << (int) *(binary_image->data + point.second * binary_image->strides[0] + point.first * binary_image->strides[1]) << "\n";
        // point is not activated
        if (! *(binary_image->data + point.second * binary_image->strides[0] + point.first * binary_image->strides[1]))
        {
            continue;
        }
        // std::cout << "hi!\n";
        // expand the area
        lx = point.first;
        rx = point.first;
        uy = point.second;
        ly = point.second;

        // expand to the left
        next = std::max(lx - expand_stepsize, 0);
        while (next > 0 && *(binary_image->data + point.second * binary_image->strides[0] + next * binary_image->strides[1]))
        {
            lx = next;
            next = std::max(lx - expand_stepsize, 0);
        }

        // expand to the right
        next = std::min(rx + expand_stepsize, (int) binary_image->dimensions[1] - 1);
        while (next < binary_image->dimensions[1] - 1 && *(binary_image->data + point.second * binary_image->strides[0] + next * binary_image->strides[1]))
        {
            rx = next;
            next = std::min(rx + expand_stepsize, (int) binary_image->dimensions[1] - 1);
        }

        // expand upwards
        next = std::max(uy - expand_stepsize, 0);
        while (next > 0 && *(binary_image->data + next * binary_image->strides[0] + point.first * binary_image->strides[1]))
        {
            uy = next;
            next = std::max(uy - expand_stepsize, 0);
        }

        // expand downwards
        next = std::min(ly + expand_stepsize, (int) binary_image->dimensions[0] - 1);
        while (next < binary_image->dimensions[0] - 1 && *(binary_image->data + next * binary_image->strides[0] + point.first * binary_image->strides[1]))
        {
            ly = next;
            next = std::min(ly + expand_stepsize, (int) binary_image->dimensions[0] - 1);
        }

        // TODO: the refinement stuff

        // remove points in the found candidate
        auto iterator = std::remove_if(points.begin(), points.end(), [lx, uy, rx, ly](std::pair<int, int> p){return pointInCandidate(p, lx, uy, rx, ly);});
        points.erase(iterator, points.end());
        candidates.push_back({rx, lx, ly, uy});

    }
	if( candidates.empty() )
    {
        return PyList_New( 0 );
    }
    PyObject* candidateList = PyList_New( candidates.size() );
    Py_INCREF(candidateList);
    //iterate through candidates
	if (!candidateList) throw logic_error("Unable to allocate memory for candidate list");
	for (unsigned int i = 0; i < candidates.size(); i++) {
	    // create tuple


		PyObject* pyCandidate = PyTuple_New(4);
        Py_INCREF(pyCandidate);
	    if (!pyCandidate) throw logic_error("Unable to allocate memory for candidate");

        PyObject *py_rx = PyInt_FromLong(candidates[i].rx);
        PyObject *py_lx = PyInt_FromLong(candidates[i].lx);
        PyObject *py_uy = PyInt_FromLong(candidates[i].uy);
        PyObject *py_ly = PyInt_FromLong(candidates[i].ly);
        if (! (py_rx && py_lx && py_uy && py_ly))
        {
            Py_DECREF(pyCandidate);
            throw logic_error("Unable to allocate memory for candidate");
        }
        PyTuple_SET_ITEM(pyCandidate, 0, py_rx);
        PyTuple_SET_ITEM(pyCandidate, 1, py_lx);
        PyTuple_SET_ITEM(pyCandidate, 2, py_uy);
        PyTuple_SET_ITEM(pyCandidate, 3, py_ly);
	    PyList_SET_ITEM(candidateList, i, pyCandidate);
	}
	return candidateList;
}

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
    {"findSpots", findSpots, METH_VARARGS, ""},
    {"maskImg", maskImg, METH_VARARGS, ""},
    {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC initVisionExtensions(void) {
    import_array();
    (void) Py_InitModule("VisionExtensions", VisionMethods);
}

int main(int argc, char *argv[]) {

    /* Pass argv[0] to the Python interpreter */
    Py_SetProgramName(argv[0]);

    /* Initialize the Python interpreter.  Required. */
    Py_Initialize();

    initVisionExtensions();

    return 0;
}
