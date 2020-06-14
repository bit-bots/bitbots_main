//
// Created by nfiedler on 6/13/20.
//

#include "bitbots_quintic_walk/walk_pywrapper.h"

char const* greet()
{
   return "hello, world";
}

PyWalk::PyWalk() {
  std::map<std::string, std::string> empty;
  ros::init(empty, "PyWalking");
  //
  walk_node_.reset(new bitbots_quintic_walk::WalkNode());
}

int PyWalk::step(int i){
    return walk_node_->step(i);
  }

void PyWalk::reset() {
  walk_node_->reset();
}

BOOST_PYTHON_MODULE(py_quintic_walk)
{
    using namespace boost::python;
    using namespace bitbots_quintic_walk;
    def("greet", greet);

    class_<PyWalk>("PyWalk")
        .def("step", &PyWalk::step)
        .def("reset", &PyWalk::reset);
}
