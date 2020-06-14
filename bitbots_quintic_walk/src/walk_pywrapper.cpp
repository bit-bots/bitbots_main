//
// Created by nfiedler on 6/13/20.
//

#include "bitbots_quintic_walk/walk_pywrapper.h"

char const* greet()
{
   return "hello, world";
}

BOOST_PYTHON_MODULE(py_quintic_walk)
{
    using namespace boost::python;
    def("greet", greet);
}
