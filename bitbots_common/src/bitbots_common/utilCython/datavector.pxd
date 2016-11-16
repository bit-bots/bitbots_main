cdef import from "datavector.hpp":
    cdef cppclass DataVector "Util::DataVector<float>":
        DataVector()
        DataVector(float, float, float)
        float get_x()
        float get_y()
        float get_z()

        void set_x(float x)
        void set_y(float y)
        void set_z(float z)

        void copy(DataVector* other)

    cdef cppclass IntDataVector "Util::DataVector<int>":
        IntDataVector()
        IntDataVector(int, int, int)
        int get_x()
        int get_y()
        int get_z()

        void set_x(int x)
        void set_y(int y)
        void set_z(int z)

        void copy(IntDataVector* other)

