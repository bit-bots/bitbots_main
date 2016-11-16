#ifndef _DATA_VECTOR
#define _DATA_VECTOR

namespace Util {

template<class T=float>
class DataVector {
    private:
        T x, y, z;

    public:
        DataVector() : x(0), y(0), z(0) {}
        DataVector(T x, T y, T z) : x(x), y(y), z(z) {}

        inline
        T get_x() const {
            return x;
        }

        inline
        T get_y() const {
            return y;
        }

        inline
        T get_z() const {
            return z;
        }

        inline
        void set_x(T x) {
            this->x = x;
        }

        inline
        void set_y(T y) {
            this->y = y;
        }

        inline
        void set_z(T z) {
            this->z = z;
        }

        inline
        void copy(DataVector *other) {
            *this = *other;
        }
};

} //namespace

#endif

