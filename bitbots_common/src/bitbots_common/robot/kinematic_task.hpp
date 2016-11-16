#ifndef _KINEMATIC_ROBOT_TASK_HPP__
#define _KINEMATIC_ROBOT_TASK_HPP__

#include <set>
#include <memory>
#include <Eigen/Core>
#include <iostream>
#include <cmath>

#include "kinematic_robot.hpp"
#include "kinematic_joint.hpp"
#include "jointids.hpp"

namespace Robot {
namespace Kinematics {

    using std::max;
    using std::min;

class KRobot;

/**
 * \brief This is a wrapper to create KinematicTasks for the robot implementation
 *
 * This KinematicTask creates kinematic chain and performs them.
 * There are various ways, the used chains can be created or used. Most of all, the chains are
 * build dynamically and there are some options, you can apply on the chain members.
 */
class KinematicTask {
public:

    typedef std::pair<int, int> IdPair;

    friend class KRobot;
    /**
     * A simple small wrapper to hold and export the chain.
     */
    class ChainHolder : public std::pair<KRobot::Chain*, bool> {
    public:
        ChainHolder()
        :pair(nullptr, false)
        {
            std::cout<<"Empty Constructor"<<std::endl;
        }

        ChainHolder(const ChainHolder& other)
        :pair(other.first, true)
        {
            first = new KRobot::Chain(*first);
            std::cout<<"Copy Constructor"<<std::endl;
        }

        ChainHolder(ChainHolder&& other)
        :pair(other.first, other.second)
        {
            other.first = nullptr;
            other.second = false;
            std::cout<<"Move Constructor"<<std::endl;
        }

        ChainHolder& operator=(const ChainHolder& other) {
            std::cout<<"Copy Operator="<<std::endl;
            this->~ChainHolder();
            first = new KRobot::Chain(*other.first);
            second = true;
            return *this;
        }

        ChainHolder& operator=(ChainHolder&& other) {
            std::cout<<"Move Operator="<<std::endl;
            this->~ChainHolder();
            first = other.first;
            second = other.second;
            other.first = nullptr;
            other.second = false;
            return *this;
        }

        ChainHolder(KRobot::Chain& chain, const bool new_chain)
        :pair(&chain, new_chain)
        {
            std::cout<<"Default Constructor"<<std::endl;
        }

        ~ChainHolder() {
            if(second)
                delete first;
        }

        size_t size() {
            return first? first->size():0;
        }

        IdPair& id_range(IdPair& min_max) {
            if(first) {
                for(unsigned i = 1; i < first->size(); ++i) {
                    const KJointChainMember& mem = (*first)[i];
                    if(mem->is_static_joint())
                        continue;
                    int id = mem->get_id();
                    min_max.first = min(id, min_max.first);
                    min_max.second = max(id, min_max.second);
                }
            }
            return min_max;
        }

        KRobot::Chain& operator*() {
            return *first;
        }

        /**
         * This operator enables some nice syntax, as this chain holder is primary holding a Chain pointer.
         * Using this operator, there are expressions like the following are possible:
         * holder->size(), for calling the size method of the Chain.
         * This avoids the confusing indirection of the get method or the bracket horror using the * operator.
         */
        KRobot::Chain* operator->() {
            return first;
        }

        KRobot::Chain& get() {
            return *first;
        }

        KRobot::Chain::value_type& operator[](const unsigned idx) {
            return (*first)[idx];
        }

        KJoint& get_joint(unsigned idx) {
            return *first->at(idx);
        }

    };

private:
    const KRobot& m_robot;


    struct Parameters {
        KRobot::MultipleAxisType m_axis;
        ChainHolder m_chain;
        KRobot::MultipleTargetType m_target;
        KRobot::MultipleErrorType m_error;
        void (*update_func)(KRobot& robot, KRobot::Chain& chain, const uint flags);
        void (*jacobi_func)(Eigen::Block<KRobot::JacobiType>, const KRobot::Chain&, KRobot::AxisType, unsigned);

        Parameters(const KRobot::MultipleAxisType& axis, ChainHolder& chain, const KRobot::MultipleTargetType& target, const KRobot::MultipleErrorType& error,
            void (*update_func)(KRobot& robot, KRobot::Chain& chain, const uint flags),
            void (*jacobi_func)(Eigen::Block<KRobot::JacobiType>, const KRobot::Chain&, KRobot::AxisType, unsigned))
        : m_axis(axis), m_chain(chain), m_target(target), m_error(error), update_func(update_func), jacobi_func(jacobi_func){}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
    std::unique_ptr<Parameters> m_parameters;

    std::unique_ptr<KinematicTask> m_subtask;

    inline ChainHolder create_chain_intern(JointIds from, JointIds to);


public:

    //! \defgroup public kinematic interfaces

    KinematicTask(const KRobot& robot)
    :m_robot(robot) {}


    static ChainHolder create_chain(const KRobot& robot, JointIds from, JointIds to);
    ChainHolder create_chain(JointIds from, JointIds to) {
        return KinematicTask::create_chain(m_robot, from, to);
    }

    IdPair get_id_range() {
        IdPair p(1<<30, 0);
        return get_id_range(p);
    }
    IdPair& get_id_range(IdPair& min_max) {
        min_max = m_parameters->m_chain.id_range(min_max);
        if (m_subtask)
            return m_subtask->get_id_range(min_max);
        return min_max;
    }

    ChainHolder create_centre_of_gravity_chain(ChainIds chain_id);


    void set_target_values(const KRobot::MultipleAxisType& axis, JointIds from, JointIds to,
                           const KRobot::MultipleTargetType& target_position, const KRobot::MultipleErrorType& error);
    void set_target_values(const KRobot::AxisType& axis, JointIds from, JointIds to,
                           const Eigen::Vector3d& target_position, const double& error) {
        set_target_values((KRobot::MultipleAxisType(1,1)<<axis).finished(), from, to, target_position, (KRobot::MultipleErrorType(1,1)<<error).finished());
    }

    void update_target(const KRobot::MultipleTargetType& target) {
        assert(m_parameters);
        m_parameters->m_target = target;
    }

    void set_subtask(KinematicTask* subtaskt) {
        m_subtask=std::unique_ptr<KinematicTask>(subtaskt);
    }

    static void assure_chain_correctness(KRobot::Chain& chain);

    void update_robot_chain(ChainHolder& chain, const unsigned update_flags/*=KRobot::UpdateChainFlags::NOP*/);
    void update_robot_chain(KRobot::Chain& chain, const unsigned update_flags/*=KRobot::UpdateChainFlags::NOP*/);
    static void update_robot_chain(KRobot& robot, ChainHolder& chain, const unsigned update_flags=KRobot::UpdateChainFlags::NOP);
    static void update_robot_chain(KRobot& robot, KRobot::Chain& chain, const unsigned update_flags=KRobot::UpdateChainFlags::NOP);
    static void update_robot_chain_(KRobot& robot, KRobot::Chain& chain, const unsigned update_flags=KRobot::UpdateChainFlags::NOP) {
        update_robot_chain(robot, chain, update_flags | KRobot::UpdateChainFlags::reset_start_chain_matrix);
    }

    int execute(int it) {
        return const_cast<KRobot&>(m_robot).inverse_chain(*this, it);
    }

};



} } //namespace

#endif //_KINEMATIC_ROBOT_TASK_HPP__

