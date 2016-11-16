#ifndef _BITBOTS_EIGEN_UTIL_HPP__
#define _BITBOTS_EIGEN_UTIL_HPP__

#include <Eigen/Core>

namespace Eigen {

typedef Matrix<char, 3, 1> Vector3b;
typedef Matrix<uint8_t, 3, 1> Vector3Ub;
typedef Matrix<char, 4, 1> Vector4b;
typedef Matrix<uint8_t, 4, 1> Vector4Ub;
#define TYPEDEF_VECTOR1(TYPE, LETTER) \
typedef Eigen::Matrix<TYPE, 1, 1> Vector1 ## LETTER; \
typedef Eigen::Matrix<TYPE, 1, 1> Matrix1 ## LETTER;
TYPEDEF_VECTOR1(double, d)
TYPEDEF_VECTOR1(float, f)
TYPEDEF_VECTOR1(int, i)
#undef TYPEDEF_VECTOR1
typedef Matrix<char, Dynamic, Dynamic, RowMajor> RMatrixXb;
typedef Matrix<uint8_t, Dynamic, Dynamic, RowMajor> RMatrixXUb;
typedef Matrix<Vector3b, Dynamic, Dynamic, RowMajor> RMatrixVec3b;
typedef Matrix<Vector3Ub, Dynamic, Dynamic, RowMajor> RMatrixVec3Ub;
typedef Matrix<Vector4Ub, Dynamic, Dynamic, RowMajor> RMatrixVec4Ub;
typedef Map<RMatrixXb> MapRMatb;
typedef Map<RMatrixXUb> MapRMatUb;
typedef Map<RMatrixVec4Ub> MapRMatVec4Ub;
typedef Map<RMatrixVec3Ub> MapRMatVec3Ub;

/**
 * This is a class, that is written to add some small functions to an Eigen matrix, as it's needed in the debug framework
 */
template<class Derived>
class MatrixHolder : public Derived{
private:
public:
    typedef typename Derived::Scalar Scalar;
    typedef Derived Base;
    MatrixHolder()
    {}
    template<class Derived2>
    MatrixHolder(Derived2& matrix)
    :Base(matrix){}
    MatrixHolder(size_t rows, size_t cols)
    :Base(rows, cols){}
    void create(size_t rows, size_t cols) {
        assert(((Derived*)this)->rows() == 0 && ((Derived*)this)->cols() == 0);
        ((Derived*)this)->operator=(Derived(rows, cols));
    }
    const Derived& get() {
        return *this;
    }
    const Scalar* ptr(size_t row=0) {
        return &(this->operator() (row, 0));
    }
    int elemSize() {
        return sizeof(Scalar);
    }
    size_t size() {
        return ((Derived*)this)->rows() * ((Derived*)this)->cols() * sizeof(Scalar);
    }
};

} //namespace Eigen

#endif //_BITBOTS_EIGEN_UTIL_HPP__
