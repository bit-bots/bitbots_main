
cdef extern from "boost/shared_ptr.hpp" namespace "boost":
    cdef cppclass shared_ptr[T]:
        T* get()

cdef extern from "boost/ref.hpp" namespace "boost":
    cdef cppclass reference_wrapper[T]:
        T* get_pointer()
        T& get()
    
