from libc.math cimport sqrt, fabs

cdef inline float max2(float a, float b):
    return a if a > b else b

cdef inline float max3(float a, float b, float c):
    return max2(a, max2(b, c))

cdef inline int max2i(int a, int b):
    return a if a > b else b

cdef inline int max3i(int a, int b, int c):
    return max2i(a, max2i(b, c))

cdef class PyDataVector:
    def __cinit__(self, *args, **kwargs):
        self.vec = new DataVector()

    def __init__(self, float x, float y, float z):
        self.vec.set_x(x)
        self.vec.set_y(y)
        self.vec.set_z(z)

    def __dealloc__(self):
        del self.vec

    cdef void set_data_vector(self, DataVector* other):
        self.vec.copy(other)

    cdef DataVector* get_data_vector(self):
        return self.vec

    cpdef set(self, float x, float y, float z):
        self.vec.set_x(x)
        self.vec.set_y(y)
        self.vec.set_z(z)

    property x:
        def __get__(self): return self.vec.get_x()
        def __set__(self, float v): self.vec.set_x(v)

    property y:
        def __get__(self): return self.vec.get_y()
        def __set__(self, float v): self.vec.set_y(v)

    property z:
        def __get__(self): return self.vec.get_z()
        def __set__(self, float v): self.vec.set_z(v)

    property r:
        def __get__(self): return self.vec.get_x()
        def __set__(self, float v): self.vec.set_x(v)

    property g:
        def __get__(self): return self.vec.get_y()
        def __set__(self, float v): self.vec.set_y(v)

    property b:
        def __get__(self): return self.vec.get_z()
        def __set__(self, float v): self.vec.set_z(v)

    cpdef float get_x(self):
        return self.vec.get_x()

    cpdef float get_y(self):
        return self.vec.get_y()

    cpdef float get_z(self):
        return self.vec.get_z()

    cdef void set_x(self, float x):
        self.vec.set_x(x)

    cdef void set_y(self, float y):
        self.vec.set_y(y)

    cdef void set_z(self, float z):
        self.vec.set_z(z)

    cpdef float max(self):
        return max3(self.vec.get_x(), self.vec.get_y(), self.vec.get_z())

    cpdef float absmax(self):
        return max3(fabs(self.vec.get_x()),
            fabs(self.vec.get_y()),
            fabs(self.vec.get_z()))

    property xyz:
        def __get__(self):
            return (self.vec.get_x(), self.vec.get_y(), self.vec.get_z())
        def __set__(self, tuple v):
            if len(v) != 3:
                raise ValueError("Three elements needed")

            self.vec.set_x(<float>v[0])
            self.vec.set_y(<float>v[1])
            self.vec.set_z(<float>v[2])

    def __repr__(self):
        return "<vec x=%s, y=%s, z=%s>" % self.xyz

    def __getitem__(self, int n):
        if n == 0: return self.vec.get_x()
        if n == 1: return self.vec.get_y()
        if n == 2: return self.vec.get_z()

        raise KeyError("Invalid index, not in range 0 to 2: %d" % n)

    def __len__(self):
        return 3

    def __iter__(self):
        return iter(self.xyz)

    cpdef float squared_norm(self):
        cdef float x = self.vec.get_x()
        cdef float y = self.vec.get_y()
        cdef float z = self.vec.get_z()
        return x*x + y*y + z*z

    cpdef float norm(self):
        return sqrt(self.squared_norm())

    def __add__(PyDataVector self, other):
        cdef PyDataVector vec
        if other.__class__ is PyDataVector:
            vec = other
            return PyDataVector(
                self.get_x() + vec.get_x(),
                self.get_y() + vec.get_y(),
                self.get_z() + vec.get_z())

        return PyDataVector(
            self.get_x() + other[0],
            self.get_y() + other[1],
            self.get_z() + other[2])

    def __sub__(PyDataVector self, other):
        cdef PyDataVector vec
        if other.__class__ is PyDataVector:
            vec = other
            return PyDataVector(
                self.get_x() - vec.get_x(),
                self.get_y() - vec.get_y(),
                self.get_z() - vec.get_z())

        return PyDataVector(
            self.get_x() - other[0],
            self.get_y() - other[1],
            self.get_z() - other[2])

    def __mul__(first, second):
        if isinstance(first, (float, int)):
            first, second = second, first

        cdef PyDataVector self = first
        cdef PyDataVector vec
        if second.__class__ is PyDataVector:
            vec = second
            return  self.get_x() * vec.get_x() + \
                self.get_y() * vec.get_y() + \
                self.get_z() * vec.get_z()

        if isinstance(second, (float, int)):
            return PyDataVector(
                self.get_x() * second,
                self.get_y() * second,
                self.get_z() * second)

        return self.get_x() * second[0] + \
            self.get_y() * second[1] + \
            self.get_z() * second[2]

    def __abs__(PyDataVector self):
        return PyDataVector(fabs(self.get_x()),
            fabs(self.get_y()),
            fabs(self.get_z()))



from cython.operator cimport address as ref
cdef PyDataVector wrap_data_vector(const DataVector& data):
    return PyDataVector(data.get_x(), data.get_y(), data.get_z())


cdef class PyIntDataVector:
    def __cinit__(self, *args, **kwargs):
        self.vec = new IntDataVector()

    def __init__(self, int x, int y, int z):
        self.vec.set_x(x)
        self.vec.set_y(y)
        self.vec.set_z(z)

    def __dealloc__(self):
        del self.vec

    cdef void set_data_vector(self, IntDataVector* other):
        self.vec.copy(other)

    cdef IntDataVector* get_data_vector(self):
        return self.vec

    cpdef set(self, int x, int y, int z):
        self.vec.set_x(x)
        self.vec.set_y(y)
        self.vec.set_z(z)

    property x:
        def __get__(self): return self.vec.get_x()
        def __set__(self, int v): self.vec.set_x(v)

    property y:
        def __get__(self): return self.vec.get_y()
        def __set__(self, int v): self.vec.set_y(v)

    property z:
        def __get__(self): return self.vec.get_z()
        def __set__(self, int v): self.vec.set_z(v)

    property r:
        def __get__(self): return self.vec.get_x()
        def __set__(self, int v): self.vec.set_x(v)

    property g:
        def __get__(self): return self.vec.get_y()
        def __set__(self, int v): self.vec.set_y(v)

    property b:
        def __get__(self): return self.vec.get_z()
        def __set__(self, int v): self.vec.set_z(v)

    cpdef int get_x(self):
        return self.vec.get_x()

    cpdef int get_y(self):
        return self.vec.get_y()

    cpdef int get_z(self):
        return self.vec.get_z()

    cdef void set_x(self, int x):
        self.vec.set_x(x)

    cdef void set_y(self, int y):
        self.vec.set_y(y)

    cdef void set_z(self, int z):
        self.vec.set_z(z)

    cpdef int max(self):
        return max3i(self.vec.get_x(), self.vec.get_y(), self.vec.get_z())

    cpdef int absmax(self):
        return max3i(abs(self.vec.get_x()),
            abs(self.vec.get_y()),
            abs(self.vec.get_z()))

    property xyz:
        def __get__(self):
            return (self.vec.get_x(), self.vec.get_y(), self.vec.get_z())
        def __set__(self, tuple v):
            if len(v) != 3:
                raise ValueError("Three elements needed")

            self.vec.set_x(<int>v[0])
            self.vec.set_y(<int>v[1])
            self.vec.set_z(<int>v[2])

    def __repr__(self):
        return "<vec x=%s, y=%s, z=%s>" % self.xyz

    def __getitem__(self, int n):
        if n == 0: return self.vec.get_x()
        if n == 1: return self.vec.get_y()
        if n == 2: return self.vec.get_z()

        raise KeyError("Invalid index, not in range 0 to 2: %d" % n)

    def __len__(self):
        return 3

    def __iter__(self):
        return iter(self.xyz)

    cpdef int squared_norm(self):
        cdef int x = self.vec.get_x()
        cdef int y = self.vec.get_y()
        cdef int z = self.vec.get_z()
        return x*x + y*y + z*z

    cpdef int norm(self):
        return int(sqrt(float(self.squared_norm())))

    def __add__(PyIntDataVector self, other):
        cdef PyIntDataVector vec
        if other.__class__ is PyIntDataVector or other.__class__ is PyDataVector:
            vec = other
            return PyIntDataVector(
                self.get_x() + int(vec.get_x()),
                self.get_y() + int(vec.get_y()),
                self.get_z() + int(vec.get_z()))

        return PyIntDataVector(
            self.get_x() + int(other[0]),
            self.get_y() + int(other[1]),
            self.get_z() + int(other[2]))

    def __sub__(PyIntDataVector self, other):
        cdef PyIntDataVector vec
        if other.__class__ is PyIntDataVector:
            vec = other
            return PyIntDataVector(
                self.get_x() - vec.get_x(),
                self.get_y() - vec.get_y(),
                self.get_z() - vec.get_z())

        return PyIntDataVector(
            self.get_x() - other[0],
            self.get_y() - other[1],
            self.get_z() - other[2])

    def __mul__(first, second):
        if isinstance(first, (int, int)):
            first, second = second, first

        cdef PyIntDataVector self = first
        cdef PyIntDataVector vec
        if second.__class__ is PyIntDataVector or second.__class__ is PyDataVector:
            vec = second
            return  self.get_x() * int(vec.get_x()) + \
                self.get_y() * int(vec.get_y()) + \
                self.get_z() * int(vec.get_z())

        if isinstance(second, (int, int)):
            return PyIntDataVector(
                self.get_x() * second,
                self.get_y() * second,
                self.get_z() * second)

        if isinstance(second, (float, float)):
            return PyIntDataVector(
                int(self.get_x() * second),
                int(self.get_y() * second),
                int(self.get_z() * second))

        return self.get_x() * int(second[0]) + \
            self.get_y() * int(second[1]) + \
            self.get_z() * int(second[2])

    def __abs__(PyIntDataVector self):
        return PyIntDataVector(abs(self.get_x()),
            abs(self.get_y()),
            abs(self.get_z()))


cdef PyIntDataVector wrap_int_data_vector(const IntDataVector& data):
    return PyIntDataVector(data.get_x(), data.get_y(), data.get_z())


