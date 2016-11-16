#include <set>
#include <Eigen/Core>
#include <boost/foreach.hpp>

#include "kinematic_task.hpp"
#include "kinematic_joint.hpp"
#include "chain_member.hpp"
#include "jointids.hpp"
#include "../debug/debugmacro.h"

#define STATIC

using namespace Robot;
using namespace Kinematics;
using namespace std;
using namespace Eigen;


inline KinematicTask::ChainHolder KinematicTask::create_chain_intern(JointIds from, JointIds to) {
    KRobot::Chain* chain = nullptr;
    const KRobot::JointChainMappingType& joint_chain_mapping = m_robot.get_joint_chain_mapping();
    bool new_chain = true;
    if(from == JointIds::Root) {
        //standard robot's chain
        chain = & const_cast<KRobot::Chain&>(m_robot.get_chain_by_id(joint_chain_mapping[to].first));
        if(chain->back()->get_id() != to) {
            chain = new KRobot::Chain((const KRobot::Chain)*chain);
            while(chain->back()->get_id() != to) {
                chain->pop_back();
            }
        } else {
            new_chain = false;
        }
    } else if (joint_chain_mapping[from].second & joint_chain_mapping[to].second) {
        //Chain where from and to are in on chain
        uint16_t chain_bit = joint_chain_mapping[from].second & joint_chain_mapping[to].second;
        const KRobot::Chain& org = m_robot.get_chain_by_bit(chain_bit);
        chain = new KRobot::Chain();
        chain->reserve(org.size());
        int idx = 0;
        bool inverse = false;
        while(org[idx]->get_id() != from && org[idx]->get_id() != to) {
            ++idx;
        }
        if(org[idx]->get_id() == to) {
            inverse = true;
            while(org[idx]->get_id() != from)
                ++idx;
            chain->push_back(org[idx]);
            --idx;
            while(org[idx]->get_id() != to ) {
                chain->push_back(org[idx]);
                --idx;
            }
            chain->push_back(org[idx]);
        } else {
            chain->push_back(org[idx]);
            ++idx;
            while(org[idx]->get_id() != to) {
                chain->push_back(org[idx]);
                ++idx;
            }
            chain->push_back(org[idx]);
        }
        KJointChainMember::OptionType option = KJointChainMember::SingleOptions::is_inverse;

        if(inverse) {
            for(KJointChainMember& mem : *chain) {
                mem.set_option(option);
            }
        }
    } else {
        //Dynamic chain with from and to in different chains
        int inv_chain_chain_id = joint_chain_mapping[from].first, org_chain_chain_id = joint_chain_mapping[to].first;
        const KRobot::Chain& inv_chain = m_robot.get_chain_by_id(inv_chain_chain_id),
        org_chain = m_robot.get_chain_by_id(org_chain_chain_id);
        chain = new KRobot::Chain();
        chain->reserve(inv_chain.size() + org_chain.size() - 1);
        bool started = false;
        KJointChainMember::OptionType option = KJointChainMember::SingleOptions::is_inverse;
        for(unsigned i = inv_chain.size() - 1; i != (unsigned) -1; --i) {
            const KJointChainMember& mem = inv_chain[i];
            if(started || mem->get_id() == from) {
                started = true;
                chain->push_back(mem);
                chain->back().set_option(option);
                if(mem.evaluate_option(KJointChainMember::SingleOptions::is_in_multiple_chains) && i < inv_chain.size() - 1) {
                    chain->back()->create_inverse_transform_with_follower(*(inv_chain[i + 1]));
                }
            }
        }
        for(unsigned i = 1; i < org_chain.size(); ++i) {
            chain->push_back(org_chain[i]);
            if((*chain)[i]->get_id() == to) {
                break;
            }
        }
        //check for joints that are in both chains
        KRobot::ChainIdType inv_chain_chain_bit = 1 << inv_chain_chain_id;
        KRobot::ChainIdType org_chain_chain_bit = 1 << org_chain_chain_id;
        KRobot::ChainIdType chain_bits = inv_chain_chain_bit | org_chain_chain_bit;
        for(KJointChainMember& mem: *chain) {
            if((joint_chain_mapping[mem->get_id()].second & chain_bits) == chain_bits) {
                mem.set_option(KJointChainMember::SingleOptions::is_duplicate);
            }
        }
    }
    return KinematicTask::ChainHolder(*chain, new_chain);
}

STATIC KinematicTask::ChainHolder KinematicTask::create_chain(const KRobot& robot, JointIds from, JointIds to) {
    KinematicTask t(robot);
    return t.create_chain_intern(from, to);
}

void KinematicTask::set_target_values(const KRobot::MultipleAxisType& axis, JointIds from, JointIds to,
                                      const KRobot::MultipleTargetType& target_position, const KRobot::MultipleErrorType& error) {
    ChainHolder ch = KinematicTask::create_chain(m_robot, from, to);
    m_parameters = std::unique_ptr<Parameters>(new Parameters(axis, ch, target_position, error, KinematicTask::update_robot_chain_,
                                                              Robot::Kinematics::fill_jacobi_matrix));
}


KinematicTask::ChainHolder KinematicTask::create_centre_of_gravity_chain (ChainIds chain_id) {
    KRobot::Chain* chain = new KRobot::Chain();
    const KRobot::Chain& ref_chain = m_robot.get_chain_by_id(chain_id);
    for(unsigned i = ref_chain.size() - 1; i < ref_chain.size(); --i) {
        KJointChainMember mem = ref_chain[i];
        mem.set_option(KJointChainMember::SingleOptions::is_inverse);
        if(i != ref_chain.size() - 1 && mem.evaluate_option(KJointChainMember::SingleOptions::is_in_multiple_chains)) {
            mem->create_inverse_transform_with_follower(*ref_chain[i + 1]);
        }
        chain->push_back(mem);
    }
    for(unsigned i = 0; i < m_robot.get_chains().size(); ++i) {
        if(i == chain_id)
            continue;
        const KRobot::Chain& cur_chain = m_robot.get_chain_by_id(i);
        KJointChainMember duplicat_root = cur_chain[0];
        duplicat_root.set_option(KJointChainMember::SingleOptions::is_duplicate | KJointChainMember::SingleOptions::is_restart);
        chain->push_back(duplicat_root);
        for(unsigned j = 1; j < cur_chain.size(); ++j) {
            KJointChainMember cur_mem = cur_chain[j];
            chain->push_back(cur_mem);
        }
    }
    return ChainHolder(*chain, true);
}

void KinematicTask::assure_chain_correctness (KRobot::Chain& chain) {
    for(unsigned i = 0; i < chain.size(); ++i) {
        KJointChainMember& mem = chain[i];
        if(i && mem.evaluate_option(KJointChainMember::SingleOptions::is_in_multiple_chains) && mem.evaluate_option(KJointChainMember::SingleOptions::is_inverse)) {
            mem->create_inverse_transform_with_follower(*chain[i - 1]);
        }
    }
}


void KinematicTask::update_robot_chain(KRobot::Chain& chain, const unsigned update_flags) {
    assure_chain_correctness(chain);
    const_cast<KRobot&>(m_robot).update_chain(chain, KJoint::AngleCheck::check, update_flags);
}
void KinematicTask::update_robot_chain(ChainHolder& chain, const unsigned update_flags) {
    assure_chain_correctness(*chain);
    const_cast<KRobot&>(m_robot).update_chain(*chain, KJoint::AngleCheck::check, update_flags);
}
void KinematicTask::update_robot_chain(KRobot& robot, KRobot::Chain& chain, const unsigned update_flags) {
    assure_chain_correctness(chain);
    robot.update_chain(chain, KJoint::AngleCheck::check, update_flags);
}
void KinematicTask::update_robot_chain(KRobot& robot,ChainHolder& chain, const unsigned update_flags) {
    assure_chain_correctness(*chain);
    robot.update_chain(*chain, KJoint::AngleCheck::check, update_flags);
}

