from bitbots_common.utilCython.datavector cimport DataVector, IntDataVector

cdef class PyDataVector:
    cdef DataVector *vec

    cdef void set_data_vector(self, DataVector* other)
    cdef DataVector* get_data_vector(self)

    cpdef set(self, float x, float y, float z)
    cpdef float get_x(self)
    cpdef float get_y(self)
    cpdef float get_z(self)

    cdef void set_x(self, float x)
    cdef void set_y(self, float y)
    cdef void set_z(self, float z)

    cpdef float max(self)
    cpdef float absmax(self)

    cpdef float squared_norm(self)
    cpdef float norm(self)

cdef PyDataVector wrap_data_vector(DataVector& data)

cdef class PyIntDataVector:
    cdef IntDataVector *vec

    cdef void set_data_vector(self, IntDataVector* other)
    cdef IntDataVector* get_data_vector(self)

    cpdef set(self, int x, int y, int z)
    cpdef int get_x(self)
    cpdef int get_y(self)
    cpdef int get_z(self)

    cdef void set_x(self, int x)
    cdef void set_y(self, int y)
    cdef void set_z(self, int z)

    cpdef int max(self)
    cpdef int absmax(self)

    cpdef int squared_norm(self)
    cpdef int norm(self)

cdef PyIntDataVector wrap_int_data_vector(IntDataVector& data)

