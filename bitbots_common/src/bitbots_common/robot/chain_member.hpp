#ifndef _KJOINT_CHAIN_MEMBER_HPP__
#define _KJOINT_CHAIN_MEMBER_HPP__

#include <assert.h>
#include "kinematic_joint.hpp"

namespace Robot {
namespace Kinematics {

/**
 * This class represents a chainmember of the robots chains. This member is used to store some additional options for any joint in a given chain.
 * When there is an dynamically created chain, then there are some options you would like to set, like there is a small inverse part of the chain or
 * there are some joints, that should not be used for a given kind of task.
 */
class KJointChainMember {
public:
    typedef uint32_t OptionType;
    typedef uint16_t IdType;
    enum SingleOptions : OptionType {no_flag=0, is_inactive=0x1, is_in_multiple_chains=0x2, is_duplicate=0x4, is_restart=0x8, is_end=0x10,
        not_for_XAxis=0x20, not_for_YAxis=0x40, not_for_ZAxis=0x80, not_for_Pos=0x100, not_for_COG=0x200, is_inverse=0x400, is_static=0x800
    };
    #define SO SingleOptions
    #define O Options
    enum Options : OptionType {Default=SO::is_inactive,
        need_to_ignore=SO::is_inactive | SO::is_duplicate | SO::is_restart | SO::is_static,
        not_for_any_axis = not_for_COG | not_for_Pos | not_for_XAxis | not_for_YAxis | not_for_ZAxis,
        ignore_any_axis = need_to_ignore | not_for_any_axis,
    };
private:
    OptionType m_options;
    KJoint* j;

public:

    KJointChainMember()
    :m_options(O::Default), j(nullptr){
    }
    KJointChainMember(KJoint& j)
    :m_options(O::Default), j(&j){
        if(j.is_static_joint()) {
            m_options |= SO::is_static;
        }
    }
    KJointChainMember(KJoint& j, OptionType options)
    :m_options(options), j(&j){
        if(j.is_static_joint()) {
            m_options |= SO::is_static;
        }
    }
    #undef SO
    #undef O


    OptionType evaluate_option(OptionType o) const {
        return m_options & o;
    }

    void set_option(OptionType o) {
        m_options |= o;
    }

    void disable_option(OptionType o) {
        m_options &= ~o;
    }

    /**
     * This method sets a joint to active, if it's id is lesser equal than the reference value, otherwise it's set inactive
     */
    void set_active_with_id(int max_id) {
        if(j->get_id() != 0 && j->get_id() <= max_id) {
            m_options &= ~is_inactive;
            assert(!(m_options & is_inactive));
        }
    }

    void set_inactive() {
        m_options |= is_inactive;
    }

    /**
     * Evaluates, wheather this joint should be ignored for kinematic centre of gravity tasks
     */
    bool ignore_for_cog() const {
        OptionType o = Options::need_to_ignore | SingleOptions::not_for_COG;
        return m_options & o;
    }


    /**
     * Evaluates, whether this joint should be ignored for kinematic tasks using a given axis
     */
    bool ignore_for_axis(KJoint::AxisType axis) const {
        OptionType o = Options::need_to_ignore;
        #define SWITCH_OVER_ALL_AXIS_AND_PERFORM_LOGICAL_OR(VAR) \
        do { \
            switch(axis) { \
                case(KJoint::AxisType::XAxis): { \
                    VAR |= SingleOptions::not_for_XAxis;break; \
                } \
                case(KJoint::AxisType::YAxis): { \
                    VAR |= SingleOptions::not_for_YAxis;break; \
                } \
                case(KJoint::AxisType::ZAxis): { \
                    VAR |= SingleOptions::not_for_ZAxis;break; \
                } \
                case(KJoint::AxisType::Position): { \
                    VAR |= SingleOptions::not_for_Pos;break; \
                } \
                default: { \
                    VAR |= SingleOptions::not_for_COG;break; \
                } \
            } \
        } while(false)
        SWITCH_OVER_ALL_AXIS_AND_PERFORM_LOGICAL_OR(o);
        return m_options & o;
    }

    /**
     * Sets the ignorance flag for a given axis
     */
    void set_ignorance_for_axis(KJoint::AxisType axis) {
        SWITCH_OVER_ALL_AXIS_AND_PERFORM_LOGICAL_OR(m_options);
    }

    const KJoint& operator*() const {
        assert(j);
        return *j;
    }
    KJoint& operator*() {
        assert(j);
        return *j;
    }

    /**
     * This operator enables some nice syntax, as this chain member is primary holding a KJoint pointer.
     * Using this operator, there are expressions like the following are possible:
     * member->get_id(), for calling the get_id method of the KJoint.
     * This avoids the confusing indirection of the get method or the bracket horror using the * operator.
     */
    const KJoint* operator->() const {
        assert(j);
        return j;
    }

    //! \copydoc KJoint::operator->() const
    KJoint* operator->() {
        assert(j);
        return j;
    }

    KJoint& get() {
        assert(j);
        return *j;
    }

    const KJoint& get() const {
        assert(j);
        return *j;
    }

    KJoint* ptr() {
        assert(j);
        return j;
    }

    const KJoint* ptr() const {
        assert(j);
        return j;
    }

    bool operator==(const KJointChainMember& other) const {
        return m_options == other.m_options && ((*j) == (*other.j));
    }
};

// Disable the -Wc++1y-extensions Warning for this statement
#pragma clang diagnostic ignored "-Wc++1y-extensions"
inline constexpr KJointChainMember::SingleOptions get_ignorance_axis(KJoint::AxisType axis) {
    return axis == KJoint::AxisType::XAxis? KJointChainMember::SingleOptions::not_for_XAxis:
           axis == KJoint::AxisType::YAxis? KJointChainMember::SingleOptions::not_for_YAxis:
           axis == KJoint::AxisType::ZAxis? KJointChainMember::SingleOptions::not_for_ZAxis:
           axis == KJoint::AxisType::Position? KJointChainMember::SingleOptions::not_for_Pos:
           KJointChainMember::SingleOptions::no_flag;
}

} } //namespace

#endif //_KJOINT_CHAIN_MEMBER_HPP__
