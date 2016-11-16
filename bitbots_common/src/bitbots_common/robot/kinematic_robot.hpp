#ifndef KINEMATIC_REPRESENTATION_LIKE_THE_FUMANOIDS_HPP__
#define KINEMATIC_REPRESENTATION_LIKE_THE_FUMANOIDS_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <string>
#include <assert.h>

#include "../utilCython/eigen_util.hpp"
#include "jointids.hpp"
#include "kinematic_joint.hpp"
#include "chain_member.hpp"
#include "../pose/pose.hpp"

namespace Robot{
namespace Kinematics{

class KinematicTask;

namespace intern {
    template<unsigned int i>
    struct DynamicMultipleTarget{
        enum{Dynamic=Eigen::Dynamic};
    };
    template<>
    struct DynamicMultipleTarget<1>{
        enum{Dynamic=1};
    };
    template<>
    struct DynamicMultipleTarget<0>{
    };
}
#ifndef MAX_MULTIPLE_INVERSE_COMPUTATION_TARGET
    #define MAX_MULTIPLE_INVERSE_COMPUTATION_TARGET 4
#endif

class KRobot;
extern const std::string default_file;
extern const std::string default_installed_file;
KRobot* load_robot_from_file(const std::string& file=default_file);

/**
 * This is the kinematic robot implementation represented by various chains, and joints.
 * The robot holds containers for all the given joints, furthermore there are small pointer based representations for the chains, containing these joints.
 * Some additional convenience containers for mapping joints to chains are also stored.
 * Every joint and chain can be accessed by a configurable name or id. Please make sure the ids fit to those defined in "jointids.hpp".
 */
class KRobot {
public:
enum TemplateNumbers{max_multiple_target=MAX_MULTIPLE_INVERSE_COMPUTATION_TARGET, Dynamic=intern::DynamicMultipleTarget<max_multiple_target>::Dynamic};
enum UpdateChainFlags{NOP=0x0, reset_start_chain_matrix=0x1, update_masses=0x2, both=0x3};
enum ChainIdBit:uint16_t{first=0x1, second=0x2, third=0x4, fourth=0x8, fifth=0x10, sixth=0x20};
enum KinematicScaling{PositionalScaling=4};
typedef uint16_t ChainIdType;
typedef KJoint::AxisType AxisType;
typedef std::vector<KJointChainMember>Chain;
typedef std::vector<Chain> Chains;
typedef Eigen::MatrixXd JacobiType;
typedef Eigen::MatrixXd InverseJacobiType;
//typedef Eigen::Matrix<JacobiType, Dynamic, 1, Eigen::ColMajor, max_multiple_target, 1> MultipleJacobiType;
//typedef Eigen::Matrix<InverseJacobiType, Dynamic, 1, Eigen::ColMajor, Dynamic, 1> InverseMultipleJacobiType;
typedef Eigen::VectorXd AngleMatrixType;
//typedef Eigen::Matrix<double, Eigen::Dynamic, Dynamic, Eigen::ColMajor, Eigen::Dynamic, max_multiple_target> MultipleAngleMatrixType;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 3 * max_multiple_target, 1> MultipleTargetType;
typedef Eigen::Matrix<KJoint::AxisType, 1, 1> SingleAxisTypeMat;
typedef Eigen::Matrix<KJoint::AxisType, Dynamic, 1, Eigen::ColMajor, max_multiple_target, 1> MultipleAxisType;
typedef Eigen::Array<double, 1, Dynamic, Eigen::RowMajor, 1, max_multiple_target> MultipleErrorType;
typedef std::vector<KJoint, Eigen::aligned_allocator<KJoint> > Joints;
typedef std::map<std::string, int> IdMapping;
typedef std::map<std::string, int> ChainMapping;
typedef std::vector<std::vector<int> > ChainsTemplate;
typedef std::pair<short, uint16_t> JointMapType;
typedef std::vector<JointMapType> JointChainMappingType;

private:
typedef Eigen::Vector3d Vector3;
typedef Eigen::Vector4d Vector4;
typedef Eigen::Matrix4d Matrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 3 * max_multiple_target, 1> MultileDeltaType;
typedef Eigen::Array<double, 1, Dynamic, Eigen::RowMajor, 1, max_multiple_target> DeltaNormType;

typedef KJoint::AngleCheck AngleCheck;

private:
Joints m_joints;
Chains m_chains;
IdMapping m_id_mapping;
ChainMapping m_chain_mapping;
JointChainMappingType m_joint_chain_mapping;
int m_max_motor_id;
double m_mass;

    void init_chains(const Robot::Kinematics::KRobot::ChainsTemplate& chains_template, const int max_id);

    void init_mass();

    void create_chain_template_and_init_chains(const KRobot& other);

    template<AngleCheck checked=AngleCheck::check, unsigned update_flags=UpdateChainFlags::NOP, bool skip_static_start=false>
    BITBOTS_INLINE void update_chain(Chain& chain);

    template<AngleCheck checked=AngleCheck::check>
    BITBOTS_INLINE void update_chain(const unsigned int chain_id);

    template<AngleCheck checked=AngleCheck::check>
    BITBOTS_INLINE void update_chains();

    /*
    template<bool addition>
    BITBOTS_INLINE void manipulate_chain_angles(AngleMatrixType& angleOff, Chain& chain, const unsigned single_jacobi_cols);
    template<bool addition>
    BITBOTS_INLINE void manipulate_chain_angles(AngleMatrixType& angleOff, Chain& chain, const KinematicTask::IdPair& range);
    */

    template<bool use_local_cog=false>
    int inverse_cog_chain(Chain& chain, const Eigen::Vector3d& target, const Eigen::Vector3i& ignore_axis, const double error, const int max_it=100, const double min_angle_d=1e-5, bool allow_angle_manipulation=true);

    template<AngleCheck check>
    void update_chain(Chain& chain, unsigned int update_flags);
    void update_chain(Chain& chain, AngleCheck check, unsigned int update_flags);

public:

    friend class KinematicTask;
    /*
     * Is currently not needed, because every Eigen class members are stored in aligned containers
     * EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     */

    KRobot();

    /**
     * The robot is a complex object requiring various parameters to create the chains in the right way
     * \param idMapping: a map to enable joint access by names
     * \param ChainMapping: a map to enable chain access by names
     * \param joints: the vector containing the raw joints without their connections
     * \param chains_template: A nested list containing the jointids except the root joint of every chain
     * \param max_id the maximum number of an active joint
     */
    KRobot(const IdMapping& idMapping, const ChainMapping& chainMapping, const Joints& joints, const ChainsTemplate& chains_template, int max_id);

    KRobot(const KRobot& other);

    KRobot(KRobot&& other);

    /**
     * Updates the robots joint angles, and endpoints.
     * \param pose: the pose, the angles are taken from
     * \param positional_update: a flag to reduce the computational amount by skipping the update of the joint positions,
     *                           this can be useful, when working with customized chains and without access on joint endpoints
     */
    void update(const Pose& pose, const bool positional_update=true);
    void update_from_goals(const Pose& pose, const bool positional_update=true);

    void update(const Eigen::VectorXd& angles, const bool positional_update=true);

    /**
     * This function performs an update of all components masses and masspoints
     */
    void update_robot_masses();

    /**
     * Applies angles to a pose.
     * \param pose: the pose
     * \param chain_id: The id of a chain, that should be set to pose or -1 for all chains
     * \param time: the time left to reach the applied angles
     */
    void set_angles_to_pose(Pose& pose, const int chain_id=-1, const float time=-1);

    inline const Chains& get_chains() const {
        return m_chains;
    }

    bool all_data_valid() const;

    inline Chain& get_chain_by_bit(uint16_t chain_bit) {
        #define CHAIN_BY_BIT \
        uint16_t chain_id = 0; \
        while(! ((1<<chain_id) & chain_bit)) ++chain_id; \
        return m_chains[chain_id];
        CHAIN_BY_BIT
    }

    inline const Chain& get_chain_by_bit(uint16_t chain_bit) const {
        CHAIN_BY_BIT
        #undef CHAIN_BY_BIT
    }

    inline Chain& get_chain_by_id(unsigned id) {
        return m_chains[id];
    }

    inline const Chain& get_chain_by_id(unsigned id) const {
        return m_chains[id];
    }

    inline Chain& get_chain(unsigned id) {
        return get_chain_by_id(id);
    }

    inline const Chain& get_chain(unsigned id) const {
        return get_chain_by_id(id);
    }

    inline const Chain& get_chain_by_name(const std::string& name) const {
        assert(m_chain_mapping.find(name) != m_chain_mapping.end());
        return m_chains[m_chain_mapping.find(name)->second];
    }

    inline KJoint& get_joint_by_id(unsigned id) {
        assert(id < m_joints.size());
        return m_joints[id];
    }

    inline const KJoint& get_joint_by_id(unsigned id) const {
        assert(id < m_joints.size());
        return m_joints[id];
    }

    /**
     * The joints are accessible over the [] operator by name and id
     */
    inline const KJoint& operator[](unsigned id) const {
        return get_joint_by_id(id);
    }

    //! \copydoc KRobot::operator[](unsigned)
    inline KJoint& operator[](unsigned id) {
        return get_joint_by_id(id);
    }

    inline KJoint& get_joint_by_name(const std::string& name) {
        assert(m_id_mapping.find(name) != m_id_mapping.end());
        return m_joints[m_id_mapping.find(name)->second];
    }

    inline const KJoint& get_joint_by_name(const std::string& name) const {
        assert(m_id_mapping.find(name) != m_id_mapping.end());
        return m_joints[m_id_mapping.find(name)->second];
    }

    /**
     * The joints are accessible over the [] operator by name and id
     */
    inline KJoint& operator[](const std::string& name) {
        return get_joint_by_name(name);
    }

    //! \copydoc KRobot::operator[](const std::string&)
    inline const KJoint& operator[](const std::string& name) const {
        return get_joint_by_name(name);
    }

    /**
     * The wrapper for get_joint_by_id and get_joint_by_name
     */
    template<class T>
    inline KJoint& get_joint(const T& t) {
        return (*this)[t];
    }
    //! \copydoc KRobot::get_joint(const T&)
    template<class T>
    inline const KJoint& get_joint(const T& t) const {
        return (*this)[t];
    }

    inline double get_mass() const{
        return m_mass;
    }

    Vector4 get_centre_of_gravity() const;

    Vector4 get_centre_of_gravity(const Matrix& offset) const;

    /**
     * A mapping for easy access on the chains, a joint is member of
     */
    const JointChainMappingType& get_joint_chain_mapping() const {
        return m_joint_chain_mapping;
    }

    /**
     * Calculates an inverse computation task for a given chain.
     * This method uses an iterative algorithm using local derivates to approximate an positional or angular change on the endpoints position for an angular manipulation of
     * any joint. This is used to manipulate the robots jointangles. After performing a check of having reached the position, it's returned or continued.
     * You can specify some termination criteria like the proximity to the target, the maximum number of iterations or a minimal angular change.
     * \param chain: The chain, that is used to perform the inversion
     * \param target: the target position, the end of chain will read
     * \param error: the error value, to finish the task
     * \param max_it: maximum number of iterations
     * \param axis: the axis, describing the target position from the chains start and endpoint, default is the position relative to the start joint
     * \param min_angle_d: another stop criterion: the minimal value, the angles must change to continue iterating
     */
    //int inverse_chain(KinematicTask& task, int it, double positional_scaling=PositionalScaling, std::unique_ptr<Eigen::MatrixXd> N=std::unique_ptr<Eigen::MatrixXd>());
    int inverse_chain(KinematicTask& task, int it, double positional_scaling=PositionalScaling, Eigen::MatrixXd* N=nullptr, std::pair<int, int>* range= nullptr);

    /**
     * It is possible, to give the centred root joint an initial angle, such as gathered from IMU data. But keep in mind, the these initial angles will be disturbing, when
     * calculating kinematic tasks, with the feet on flat ground.
     * Never forget calling reset_initial_angles.
     */
    void set_initial_angles(const Vector3& rpy, const int chain=-1);

    /**
     * \brief Check whether a target is in range of a chain or not
     *
     * This is just a simple check method to ensure basic reachability of a target position.
     * This check is separated in two parts: First the static part is evaluated using this
     * last joints endpoint, second the dynamic part. The transformation of the remaining joints
     * is summed up. Finally it's the difference between the static fixed position and the target,
     * that must be lass than the sum of the transformation of the dynamic part.
     * \param chain: The chain that is used to perform the check
     * \param target: The target, the chain endpoint should reach
     */
    static bool plausibility_check(const Chain& chain, const Eigen::Vector3d& target);

    /*
     * Resets a given angle in the root joint after an initial root angle was set.
     * This method must be called after set_initial_angles was called, to reset the robot into a valid state, when performing normal tasks.
     */
    BITBOTS_INLINE void reset_initial_angles()
    {
        m_joints[JointIds::Root].m_chain_matrix = Matrix::Identity();
        m_joints[JointIds::Root].m_mass_endpoint = m_joints[JointIds::Root].m_mass_offset;
    }

    void print_robot_data_for_debug() const;

    int get_max_motor_id() const {
        return m_max_motor_id;
    }
};

/**
 * The test interface for the creation of the jacobi matrix used to determine the derivatives of the joints.
 * Finally the jacobi matrix will be inverted to calculate the difference angles.
 * \param jacobi: The jacobi matrix that will be filled
 * \param chain: The underlying chain used to fill the matrix
 * \param axis: The used axis the calculate the local derivates.
 */
void fill_jacobi_matrix(Eigen::Block<KRobot::JacobiType> jacobi, const KRobot::Chain& chain, const KRobot::AxisType axis=KRobot::AxisType::Position, unsigned offset=1);
//! \copydoc fill_jacobi_matrix
void fill_jacobi_matrix_l(KRobot::JacobiType& jacobi, const KRobot::Chain& chain, const KJoint::AxisType axis=KJoint::AxisType::Position);


} } //namespace

#endif
