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

static PyObject* expandPoints(PyObject *self, PyObject *args)
{
    PyArrayObject *binary_image;
    std::pair<int, int> point;
    int expand_stepsize;
    float expand_threshold;

    if (!PyArg_ParseTuple(args, "O!(ii)fi", &PyArray_Type, &binary_image, &point.first, &point.second, &expand_threshold, &expand_stepsize))
        return NULL;
    if (binary_image->nd != 2 || binary_image->descr->type_num != PyArray_FLOAT) {
        PyErr_SetString(PyExc_ValueError,
            "image must be two-dimensional and of type float");
        return NULL;
    }

    int lx, uy, rx, ly, next;


    // std::cout << (int) *(binary_image->data + point.second * binary_image->strides[0] + point.first * binary_image->strides[1]) << "\n";
    // point is not activated
    if (! (*(binary_image->data + point.second * binary_image->strides[0] + point.first * binary_image->strides[1]) > expand_threshold))
    {
        Py_RETURN_NONE;
    }
    // std::cout << "hi!\n";
    // expand the area
    lx = point.first;
    rx = point.first;
    uy = point.second;
    ly = point.second;

    // expand to the left
    next = std::max(lx - expand_stepsize, 0);
    while (next > 0 && *(binary_image->data + point.second * binary_image->strides[0] + next * binary_image->strides[1]) > expand_threshold)
    {
        lx = next;
        next = std::max(lx - expand_stepsize, 0);
    }

    // expand to the right
    next = std::min(rx + expand_stepsize, (int) binary_image->dimensions[1] - 1);
    while (next < binary_image->dimensions[1] - 1 && *(binary_image->data + point.second * binary_image->strides[0] + next * binary_image->strides[1]) > expand_threshold)
    {
        rx = next;
        next = std::min(rx + expand_stepsize, (int) binary_image->dimensions[1] - 1);
    }

    // expand upwards
    next = std::max(uy - expand_stepsize, 0);
    while (next > 0 && *(binary_image->data + next * binary_image->strides[0] + point.first * binary_image->strides[1]) > expand_threshold)
    {
        uy = next;
        next = std::max(uy - expand_stepsize, 0);
    }

    // expand downwards
    next = std::min(ly + expand_stepsize, (int) binary_image->dimensions[0] - 1);
    while (next < binary_image->dimensions[0] - 1 && *(binary_image->data + next * binary_image->strides[0] + point.first * binary_image->strides[1]) > expand_threshold)
    {
        ly = next;
        next = std::min(ly + expand_stepsize, (int) binary_image->dimensions[0] - 1);
    }

    PyObject* pyCandidate = PyTuple_New(4);
    Py_INCREF(pyCandidate);
    if (!pyCandidate) throw logic_error("Unable to allocate memory for candidate");

#if PY_MAJOR_VERSION >= 3
    PyObject *py_rx = PyLong_FromLong(rx);
    PyObject *py_lx = PyLong_FromLong(lx);
    PyObject *py_uy = PyLong_FromLong(uy);
    PyObject *py_ly = PyLong_FromLong(ly);
#else
    PyObject *py_rx = PyInt_FromLong(rx);
    PyObject *py_lx = PyInt_FromLong(lx);
    PyObject *py_uy = PyInt_FromLong(uy);
    PyObject *py_ly = PyInt_FromLong(ly);
#endif
    if (! (py_rx && py_lx && py_uy && py_ly))
    {
        Py_DECREF(pyCandidate);
        throw logic_error("Unable to allocate memory for candidate");
    }
    PyTuple_SET_ITEM(pyCandidate, 0, py_rx);
    PyTuple_SET_ITEM(pyCandidate, 1, py_lx);
    PyTuple_SET_ITEM(pyCandidate, 2, py_uy);
    PyTuple_SET_ITEM(pyCandidate, 3, py_ly);

	return pyCandidate;
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

#if PY_MAJOR_VERSION >= 3
        PyObject *py_rx = PyLong_FromLong(candidates[i].rx);
        PyObject *py_lx = PyLong_FromLong(candidates[i].lx);
        PyObject *py_uy = PyLong_FromLong(candidates[i].uy);
        PyObject *py_ly = PyLong_FromLong(candidates[i].ly);
#else
        PyObject *py_rx = PyInt_FromLong(candidates[i].rx);
        PyObject *py_lx = PyInt_FromLong(candidates[i].lx);
        PyObject *py_uy = PyInt_FromLong(candidates[i].uy);
        PyObject *py_ly = PyInt_FromLong(candidates[i].ly);
#endif
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
    {"expandPoints", expandPoints, METH_VARARGS, ""},
    {"maskImg", maskImg, METH_VARARGS, ""},
    {NULL, NULL, 0, NULL}
};

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

//#if PY_MAJOR_VERSION >= 3
//PyMODINIT_FUNC initVisionExtensions(void) {
    //import_array();
    //(void) Py_InitModule("VisionExtensions", VisionMethods);
//}

//int main(int argc, char *argv[]) {

//     Pass argv[0] to the Python interpreter 
    //Py_SetProgramName(argv[0]);

  //   Initialize the Python interpreter.  Required. 
    //Py_Initialize();

    //initVisionExtensions();

    //return 0;
//}
//#endif
