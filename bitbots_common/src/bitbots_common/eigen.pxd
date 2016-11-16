"""
@Robert 27.05.2014  Major Refactoring, defining the base classes and only use
                    ctypedefs to acces members of eigen classes

How to use Eigen in cython:
    In here are the general interfaces of some major Eigen classes, the matrix
    and the map. These classes have, in comparison to their C++ implementation
    a uncomplete or even false template parameter list. The given template
    parameters are there to define as much as possible from the public class
    interface with the given Eigen templates. I could not apply the templates
    as in the C++ implementation, because cyton cannot handle numeric constant
    template parameters. The way it's implemented here allows me to specify a
    "hidden" or "implicid" template parameter of the map.
    Using various numbers of ctypedefs allows me to correct the template
    parameters that are generated in the cython precompiled .cxx files.
    Otherwise there would be no way to use all the matrix specializytions than
    defining each vector, matrix, or map as independent cython class and duplicate
    the public matrix interface. At least there is one disadvantage: cython cannot
    differ between most of the vector classes, because they are all ctypedefs
    of the same matrix template specialization. But I don't mind this due to the
    advantages like the option to define most methods of the public class interface
    easily in cython.

    As described above, NEVER instanciate a matrix[] or map[] directly. Cython
    will produce a wrong or incomplete template parameter list an you will get
    .cxx file compile errors. ALWAYS use a given ctypetef to use any matrix or
    map class. It's easy to add another type. But be careful with the possibilities
    of the interface. It's not the best to call the ".z()" on two member vector.
"""

cdef extern from "Eigen/Geometry" namespace "Eigen":
    cdef cppclass AngleAxis[T, Matrix]:
        AngleAxis(T, Matrix)
        Matrix operator*(Matrix)

cdef extern from "eigen_util.hpp" namespace "Eigen":

    cdef cppclass CommaInitializer[T, S]:
        CommaInitializer add "operator," (const S&)
        CommaInitializer add "operator," (const T&)

    cdef cppclass Matrix[T]:
        Matrix() nogil
        Matrix(int, int) nogil
        Matrix(T, T) nogil
        Matrix(T, T, T) nogil
        Matrix(T, T, T, T) nogil
        #general accessor interface
        int rows() nogil
        int cols() nogil
        #Matrix accessors
        T at "operator()" (int) nogil
        T at "operator()" (int, int) nogil
        T operator() (int) nogil
        T operator() (int, int) nogil
        T x() nogil
        T y() nogil
        T z() nogil
        T* data() nogil
        Matrix col(int) nogil
        Matrix row(int) nogil
        Matrix head(int) nogil
        Matrix tail(int) nogil
        Matrix block(int, int, int, int)
        Matrix set "block<1,1>" (int, int)
        T norm() nogil
        #interface to work with matrices
        Matrix transpose() nogil
        Matrix operator/(T) nogil
        Matrix operator*(const Matrix[T]&) nogil
        Matrix operator*(const AngleAxis[T, Matrix[T]]&) nogil
        Matrix operator*(T&) nogil
        Matrix operator+(const Matrix[T]&) nogil
        Matrix operator-(const Matrix[T]&) nogil
        T dot(const Matrix[T]&) nogil
        #Initializer
        CommaInitializer[Matrix[T], T] insert "operator<<" (const T&) nogil
        CommaInitializer[Matrix[T], T] insert "operator<<" (const Matrix[T]&) nogil
        #casts
        Matrix[char] to_char "cast<char>" () nogil
        Matrix[unsigned char] to_uchar "cast<unsigned char>" () nogil
        Matrix[int] to_int "cast<int>" () nogil
        Matrix[float] to_float "cast<float>" () nogil
        Matrix[double] to_double "cast<double>" () nogil

#Some static Matrix members
    cdef Matrix[double] unitX3d "Eigen::Vector3d::UnitX"()
    cdef Matrix[float] unitX3f "Eigen::Vector3f::UnitX"()
    cdef Matrix[double] nUnitX3d " - Eigen::Vector3d::UnitX"()
    cdef Matrix[double] unitY3d "Eigen::Vector3d::UnitY"()
    cdef Matrix[float] unitY3f "Eigen::Vector3f::UnitY"()
    cdef Matrix[double] nUnitY3d " - Eigen::Vector3d::UnitY"()
    cdef Matrix[double] unitZ3d "Eigen::Vector3d::UnitZ"()
    cdef Matrix[double] nUnitZ3d " - Eigen::Vector3d::UnitZ"()
    cdef Matrix[float] unitZ3f "Eigen::Vector3f::UnitZ"()
    cdef Matrix[double] zeroMatd "Eigen::MatrixXd::Zero"(int, int)
    cdef Matrix[float] zeroMatf "Eigen::MatrixXf::Zero"(int, int)
    cdef Matrix[double] identityd "Eigen::MatrixXd::Identity" (int, int)
    cdef Matrix[float] identityf "Eigen::MatrixXf::Identity" (int, int)

    cdef cppclass Transform[T]:
        Transform() nogil
        Transform(Matrix[T]) nogil
        Matrix[T] matrix() nogil
        Matrix[T] linear() nogil
        Matrix[T] translation() nogil
        Transform operator*(Transform) nogil

    cdef cppclass MatrixHolder[T, MatrixT]:
        MatrixHolder() nogil
        MatrixHolder(int, int) nogil
        MatrixHolder(const T&) nogil
        int cols() nogil
        int rows() nogil
        const T& get() nogil
        MatrixT* ptr() nogil
        int elemSize() nogil
        int size() nogil
        T& get() nogil

    cdef cppclass Map[T, MatrixT]:
        Map(void*, int) nogil
        Map(void*, int, int) nogil
        MatrixT at "operator()" (int, int) nogil
        MatrixT operator() (int, int) nogil
        int rows() nogil
        int cols() nogil
        MatrixT* data() nogil

    ctypedef AngleAxis[float, Matrix[float]] AngleAxisf "Eigen::AngleAxisf"
    ctypedef AngleAxis[double, Matrix[double]] AngleAxisd "Eigen::AngleAxisd"
    ctypedef Matrix[char] Vector3b "Eigen::Vector3b"
    ctypedef Matrix[int] Vector2i "Eigen::Vector2i"
    ctypedef Matrix[float] Vector2f "Eigen::Vector2f"
    ctypedef Matrix[double] Vector2d "Eigen::Vector2d"
    ctypedef Matrix[int] Vector3i "Eigen::Vector3i"
    ctypedef Matrix[float] Vector3f "Eigen::Vector3f"
    ctypedef Matrix[double] Vector3d "Eigen::Vector3d"
    ctypedef Matrix[int] Vector4i "Eigen::Vector4i"
    ctypedef Matrix[float] Vector4f "Eigen::Vector4f"
    ctypedef Matrix[double] Vector4d "Eigen::Vector4d"
    ctypedef Matrix[float] VectorXf "Eigen::VectorXf"
    ctypedef Matrix[double] VectorXd "Eigen::VectorXd"
    ctypedef Matrix[float] Matrix2f "Eigen::Matrix2f"
    ctypedef Matrix[double] Matrix2d "Eigen::Matrix2d"
    ctypedef Matrix[float] Matrix3f "Eigen::Matrix3f"
    ctypedef Matrix[double] Matrix3d "Eigen::Matrix3d"
    ctypedef Matrix[float] Matrix4f "Eigen::Matrix4f"
    ctypedef Matrix[double] Matrix4d "Eigen::Matrix4d"
    ctypedef Matrix[float] Matrix3_2f "Eigen::Matrix<float, 3, 2>"
    ctypedef Matrix[double] Matrix3_2d "Eigen::Matrix<double, 3, 2>"
    ctypedef Matrix[int] MatrixXi "Eigen::MatrixXi"
    ctypedef Matrix[float] MatrixXf "Eigen::MatrixXf"
    ctypedef Matrix[double] MatrixXd "Eigen::MatrixXd"
    ctypedef Matrix[unsigned char] RMatrixXUb "Eigen::RMatrixXUb"
    ctypedef Matrix[char] RMatrixXb "Eigen::RMatrixXb"
    ctypedef Matrix[Matrix[char]] RMatrixVec4Ub "Eigen::RMatrixVec4Ub"
    ctypedef Matrix[Matrix[char]] RMatrixVec4b "Eigen::RMatrixVec4b"
    ctypedef Matrix[Matrix[char]] RMatrixVec3Ub "Eigen::RMatrixVec3Ub"
    ctypedef Matrix[Matrix[char]] RMatrixVec3b "Eigen::RMatrixVec3b"

    ctypedef Map[RMatrixXb, char] MapRMatrixXb "Eigen::MapRMatrixXb"
    ctypedef Map[VectorXd, double] MapVectorXd "Eigen::Map<Eigen::VectorXd >"
    ctypedef Map[RMatrixXb, char] MapRMatrixXb "Eigen::Map<Eigen::RMatrixXb >"
    ctypedef Map[RMatrixXUb, char] MapRMatrixXUb "Eigen::Map<Eigen::RMatrixXUb >"
    ctypedef Map[RMatrixVec4Ub, Matrix[char]] MapRMatVec4Ub "Eigen::Map<Eigen::RMatrixVec4Ub >"

    ctypedef Transform[double] Affine3d "Eigen::Affine3d"
    ctypedef Transform[float] Affine3f "Eigen::Affine3f"

    #const definitions
    ctypedef Vector3f const_Vector3f "const Eigen::Vector3f"
    ctypedef Vector4f const_Vector4f "const Eigen::Vector4f"
    ctypedef Vector3d const_Vector3f "const Eigen::Vector3d"
    ctypedef Vector4d const_Vector4f "const Eigen::Vector4d"
    ctypedef VectorXf const_VectorXf "const Eigen::VectorXf"

    ctypedef Matrix2f const_Matrix2f "const Eigen::Matrix2f"
    ctypedef Matrix3_2f const_Matrix3_2f "const Eigen::Matrix<float,3,2>"
    ctypedef Matrix4f const_Matrix4f "const Eigen::Matrix4f"
    ctypedef Matrix4d const_Matrix4d "const Eigen::Matrix4d"
    ctypedef MatrixXf const_MatrixXf "const Eigen::MatrixXf"
    ctypedef RMatrixXb const_RMatrixXb "const Eigen::RMatrixXb"
    ctypedef RMatrixVec3b const_RMatrixVec3b "const Eigen::RMatrixVec3b"

    ctypedef MatrixHolder[RMatrixXb, unsigned char] RMatrixXbHolder "Eigen::MatrixHolder<Eigen::RMatrixXb>"
    ctypedef MatrixHolder[RMatrixVec3b, Vector3b] RMatrixVec3bHolder "Eigen::MatrixHolder<Eigen::RMatrixVec3b>"

cdef extern from "eigen_util.hpp":
    char* mutable_char_ptr "const_cast<char*>" (const char* ptr)
    unsigned char* mutable_uchar_ptr "const_cast<unsigned char*>" (const unsigned char* ptr)

cimport cython
from libcpp.string cimport string

ctypedef fused matrixT:
    RMatrixXb
    MatrixXi
    MatrixXf
    MatrixXd

cdef inline string mat_to_string(matrixT m):
    cdef string s
    for i in range(m.cols()):
        for j in range(m.rows()):
            s = s + str(m.at(i, j)) + str("\t")
        s = s + str("\n")
    return s
