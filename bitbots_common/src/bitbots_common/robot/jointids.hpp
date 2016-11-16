#ifndef ROBOT_JOINT_IDS_HPP__
#define ROBOT_JOINT_IDS_HPP__

#include <string>
#include <stdexcept>

namespace Robot {
namespace Kinematics {

/**
 * This is the representation of the real robots JointIds, so that this is the only file, where any joint id has a number.
 * This should be a capsule, so that the underlying joints may change, without having to correct the algorithms.
 * Same is done for the robots chains.
 */

struct DarwinJointIDs {
enum Ids {
    Root=0,
    RShoulderPitch = 1,
    LShoulderPitch = 2,
    RShoulerRoll = 3,
    LShoulderRoll = 4,
    RElbow= 5,
    LElbow= 6,
    RHipYaw = 7,
    LHipYaw = 8,
    RHipRoll = 9,
    LHipRoll = 10,
    RHipPitch = 11,
    LHipPitch = 12,
    RKnee = 13,
    LKnee = 14,
    RAnklePitch = 15,
    LAnklePitch = 16,
    RAnkleRoll = 17,
    LAnkleRoll = 18,
    Neck = 19,
    Head = 20,
    Camera = 31,
    RArmEndpoint = 32,
    LArmEndpoint = 33,
    RFootEndpoint = 34,
    LFootEndpoint = 35,
    CameraHolder = 36,
    RHandRoll = 21,
    LHandRoll = 22,
    RHandPitch = 23,
    LHandPitch = 24};
};

struct GOALJointIDs{
enum Ids {
    Root=0,
    RShoulderPitch = 1,
    LShoulderPitch = 2,
    RShoulerRoll = 3,
    LShoulderRoll = 4,
    RElbow= 5,
    LElbow= 6,
    RHipYaw = 7,
    LHipYaw = 8,
    RHipRoll = 9,
    LHipRoll = 10,
    RHipPitch = 11,
    LHipPitch = 12,
    RKnee = 13,
    LKnee = 14,
    RAnklePitch = 15,
    LAnklePitch = 16,
    RAnkleRoll = 17,
    LAnkleRoll = 18,
    Neck = 19,
    Head = 20,
    RHandRoll = 21,
    LHandRoll = 22,
    RHandPitch = 23,
    LHandPitch = 24,
    RShoulderYaw = 25,
    LShoulderYaw = 26,
    RToe = 27,
    LToe = 28,
    BellyPitch = 29,
    BellyRoll = 30,
    Camera = 31,
    RArmEndpoint = 32,
    LArmEndpoint = 33,
    RFootEndpoint = 34,
    LFootEndpoint = 35,
    CameraHolder = 36};

};

struct DarwinChainIDs {
enum Ids {
    HeadChain=0,
    RArmChain=1,
    LArmChain=2,
    RLegChain=3,
    LLegChain=4,};
};

// Darwin and GOAL have the same chains
typedef DarwinChainIDs GOALChainIDs;
// Make some typedefs to decrease enum depth
typedef DarwinJointIDs::Ids DarwinJointIds;
typedef DarwinChainIDs::Ids DarwinChainIds;
typedef GOALJointIDs::Ids GOALJointIds;
typedef GOALChainIDs::Ids GOALChainIds;

#ifdef USE_DARWIN_JOINTS
typedef DarwinJointIds JointIds;
typedef DarwinChainIds ChainIds;
#else
typedef GOALJointIds JointIds;
typedef GOALChainIds ChainIds;
#endif

namespace intern {

template<typename T1, typename T2>
struct is_same_type {
    enum{value=false};
};
template<typename T1>
struct is_same_type<T1, T1> {
    enum{value=true};
};

static_assert(is_same_type<GOALJointIds, JointIds>::value ^ is_same_type<DarwinJointIds, JointIds>::value, "Must be either darwin or goal joint Id's");

}

enum JointAndChainNumbers{DarwinJointNumber=25, GOALJointNumber=39, JointNumber=(intern::is_same_type<JointIds, GOALJointIds>::value? GOALJointNumber : intern::is_same_type<JointIds, DarwinJointIds>::value? DarwinJointNumber: -1),
                          ActiveDarwinJoints=20, ActiveGOALJoints=30, ActiveJoints=(intern::is_same_type<JointIds, GOALJointIds>::value? ActiveGOALJoints : intern::is_same_type<JointIds, DarwinJointIds>::value? ActiveDarwinJoints: -1),
                          DarwinChainNumber=5, GOALChainNumber=5, ChainNumber=(intern::is_same_type<ChainIds, GOALChainIds>::value? GOALChainNumber : intern::is_same_type<ChainIds, DarwinChainIds>::value? DarwinChainNumber: -1),
                          IDsValid = (JointNumber != -1 && ChainNumber != -1)};

static_assert(IDsValid, "Joint and Chain Ids must be valid");

inline JointIds int_to_JointID(int id) {
    switch(id) {
        #define CASE(NAME) \
        case(JointIds::NAME): \
            return JointIds::NAME;
        CASE(Root)
        CASE(RShoulderPitch)
        CASE(LShoulderPitch)
        CASE(RShoulerRoll)
        CASE(LShoulderRoll)
        CASE(RElbow)
        CASE(LElbow)
        CASE(RHipYaw)
        CASE(LHipYaw)
        CASE(RHandRoll)
        CASE(LHipRoll)
        CASE(RHipPitch)
        CASE(LHipPitch)
        CASE(RKnee)
        CASE(LKnee)
        CASE(RAnklePitch)
        CASE(LAnklePitch)
        CASE(RAnkleRoll)
        CASE(LAnkleRoll)
        CASE(Neck)
        CASE(Head)
        CASE(Camera)
        #ifndef USE_DARWIN_JOINTS
        CASE(RToe)
        CASE(LToe)
        #endif
        CASE(RArmEndpoint)
        CASE(LArmEndpoint)
        CASE(RFootEndpoint)
        CASE(LFootEndpoint)
        default: {
            std::string msg = std::string("This method has no option for ") + std::to_string(id) + " " + __FILE__ + ": " + std::to_string(__LINE__);
            throw std::runtime_error(msg);
        }
    }
}
#undef CASE

inline ChainIds int_to_ChainID(int id) {
    switch(id){
        #define CASE(NAME) \
        case(ChainIds::NAME): \
            return ChainIds::NAME;
        CASE(HeadChain)
        CASE(RArmChain)
        CASE(LArmChain)
        CASE(RLegChain)
        CASE(LLegChain)
        default: {
            std::string msg = std::string("This method has no option for ") + std::to_string(id) + " " + __FILE__ + ": " + std::to_string(__LINE__);
            throw std::runtime_error(msg);
        }
    }
}
#undef CASE

} } //namespace

#endif
