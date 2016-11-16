#ifndef NEW_KINAMATIC_JOINT_HPP__
#define NEW_KINAMATIC_JOINT_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "../util/math_constants.h"

#if defined NDEBUG && ! defined DO_NOT_INLINE
    #define BITBOTS_INLINE inline
#else
    #define BITBOTS_INLINE
#endif

namespace Robot{
namespace Kinematics{

class KRobot;

/**
 * This class represents a joint, as it's used in the robot representing the kinematic chains.
 * The joint holds multiple vectors and matrices representing this joint.
 * It holds the current angle and it's bounds, The motor_id, the transform from it's parent, the inverse transform from
 * it's follower, a given mass with offset from the fixed point of the joint, the current chain matrix and the given global mass endpoint.
 */
class KJoint {
private:

    typedef Eigen::Matrix4d Matrix;
    typedef Eigen::Matrix3d Matrix3;
    typedef Eigen::Vector4d Vector;
    typedef Eigen::Vector3d Vector3;
    typedef Eigen::AngleAxis<double> Axis;

    /*const*/ Eigen::Affine3d m_transform;
    Eigen::Affine3d m_inverse_transform;
    Eigen::Affine3d m_chain_matrix;
    /*const*/ Vector m_mass_offset;
    Vector m_mass_endpoint;
    Vector m_chain_masspoint;
    double m_angle;
    /*const*/ double m_min_angle, m_max_angle;
    /*const*/ int m_motor_id;

    /**
     * Internal method to verify, that the angle is in range of the given restrictions
     * \param allow_jump_to_opposite: default false, allows to apply the opposite extreme angle, when the angle to be applied is far from the limits
     */
    template<bool allow_jump_to_opposite=false>
    BITBOTS_INLINE void check_angle();

    friend class KRobot;

public:
    typedef std::vector<Vector, Eigen::aligned_allocator<Vector> > Masses;
    enum AngleCheck{check=true, unchecked=false, Undef=-1};
    enum AxisType{XAxis=0,YAxis=1,ZAxis=2,Position=3,None=-1};
    enum KinematicType{AxisTask=true, COG=false};//COG mean centre of gravity
    enum InvType{is_inverse=true, is_direct=false};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KJoint();

    KJoint(int motor_id);

    KJoint(const Vector3& transform_from_parent, const Vector3& rotations, const Masses& masses, const Vector3& angles, int motor_id);

    KJoint(const KJoint& other);

    inline double get_mass() const {
        return m_mass_offset(3);
    }

    inline bool is_static_joint() const {
        return m_min_angle == m_max_angle && m_min_angle == 0;
    }

    bool all_data_valid() const;

    /**
     * Updates the internal angle. It's possible to restrict the update to the given bounds or allow any angle.
     */
    template<AngleCheck checked=AngleCheck::Undef>
    void update_angle(const double degree, const AngleCheck p_checked=checked);

    template<AngleCheck checked=AngleCheck::Undef>
    void update_angle_rad(const double radians, const AngleCheck p_checked=checked);

    inline int get_id() const {
        return m_motor_id;
    }

    inline const Eigen::Affine3d& get_transform() const {
        return m_transform;
    }

    inline const Eigen::Affine3d& get_inverse_transform() const {
        return m_inverse_transform;
    }

    /**
     * This method is only useful, when the chain masspoint was updated with the following joint
     * in the chain. This chain masspoint is necessary to calculate the local cog gradient.
     */
    inline const Vector& get_chain_masspoint() const {
        return m_chain_masspoint;
    }

    inline const Vector& get_local_chain_cog() const {
        return m_chain_masspoint;
    }

    /**
     * This method returns the endpoint of this joint. You can choose, whether it returns the positional endpoint, or an endpoint according to an axis.
     */
    template<int size = 4, AxisType axis=AxisType::None>
    inline const Eigen::Matrix<double, size, 1> get_endpoint(AxisType p_axis=AxisType::Position) const {
        return axis != AxisType::None ? m_chain_matrix.matrix().col(axis).head<size>() : m_chain_matrix.matrix().col(p_axis).head<size>();
    }

    /**
     * Takes a global endpoint vector and calculates the local gradient belonging to joint manipulation with given endpoint
     */
    BITBOTS_INLINE const Vector3 get_local_gradient(const Vector3& endpoint, const AxisType axis=AxisType::Position) const;

    /**
     * Calculates an gradient for a given mass manipulation
     */
    BITBOTS_INLINE const Vector3 get_local_cog_gradient() const;

    /**
     * @Return The applied angle in radians
     */
    inline double get_angle() const {
        return m_angle;
    }
    inline double& get_angle() {
        return m_angle;
    }

    inline const Eigen::Affine3d& get() const {
        return get_transform();
    }

    inline const Eigen::Affine3d& get_chain_matrix() const {
        return m_chain_matrix;
    }

    /**
     * Returns the own mass with an offset vector from the defined centre
     */
    inline const Vector& get_centre_of_gravity() const {
        return m_mass_endpoint;
    }
    /**
     * Returns the normalized an offset vector from the defined centre.
     */
    inline const Vector get_normalized_centre_of_gravity() const {
        return (Vector()<<m_mass_endpoint.head<3>(),1).finished();
    }

    /**
     * If you want to use inverse calculations with using masses, the mass endpoint needs to be updated this way. It's important that
     * the chain is updated in reverse order.
     * \param next: The following joint in the given chain. Though, this update must be in reverse chain order, according to the robots chains
     * \param add_existing: The flag, whether the current chain masspoint should be respected
     * \param inverse: If the joint is an inverse joint or not
     */
    template<bool add_existing=false, InvType inverse=is_direct>
    BITBOTS_INLINE void update_chain_masspoint_with_follower(const KJoint& next);

    /**
     * This method sets and creates the inverse transform for a joint using the following joint of the chain.
     * \param next: The following joint of the chain, so that the reverse ordered ways in the kinematic chains are possible.
     */
    void create_inverse_transform_with_follower(const KJoint& next);

    Eigen::Affine3d get_chain_matrix_inverse() const;

    void print_joint_for_debug() const;

    template<AngleCheck checked=AngleCheck::Undef, InvType inverse=is_direct>
    BITBOTS_INLINE const Eigen::Affine3d& update_chain_matrix(const Eigen::Affine3d& chain_matrix, const AngleCheck p_checked=checked);

    bool operator==(const KJoint& other) const {
        #define EQ(X) X == other.X
        return EQ(m_angle) && EQ(m_chain_masspoint) && EQ(m_chain_matrix.matrix()) && EQ(m_inverse_transform.matrix())
                && EQ(m_mass_endpoint) && EQ(m_mass_offset) && EQ(m_max_angle) && EQ(m_min_angle)
                && EQ(m_motor_id) && EQ(m_transform.matrix());
        #undef EQ
    }

    #ifdef KINEMATIC_IMPL
    double& angle_ref() {
        return m_angle;
    }
    #endif

};

inline KJoint::AxisType int_to_axis(int a) {
    switch(a) {
        case(0): {
            return KJoint::AxisType::XAxis;
        }
        case(1): {
            return KJoint::AxisType::YAxis;
        }
        case(2): {
            return KJoint::AxisType::ZAxis;
        }
        default: {
            return KJoint::AxisType::Position;
        }
    }
}

} } //namespace

#endif
