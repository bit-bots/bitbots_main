/** Definitions of enums and structures for the Mixed Team Protocol.
 **
 */

#ifndef MITECOM_DATA_H__
#define MITECOM_DATA_H__

#include <inttypes.h>
#include <stdlib.h>

#include <map>


/// the magic value for network packets
const static uint32_t MITECOM_MAGIC   = 'MXTC'; // little endian

/// the version number
const static uint32_t MITECOM_VERSION = 1;

namespace MiTeCom {

/*------------------------------------------------------------------------------------------------*/

/// enum for the global roles
enum MixedTeamRoleEnum {
	/// robot is not doing anything
	ROLE_IDLING                                  = 0,

	/// undefined role, no role or some weird stuff
	ROLE_OTHER                                   = 1,

	/// A striker is actively pursuing the ball.
	ROLE_STRIKER                                 = 2,

	/// A supporter is positioning itself close to the striker or the
	/// ball to be able to take over the striker role if necessary,
	/// and/or to block access to the ball or to be the first line of
	/// defense once the striker is removed/fails.
	ROLE_SUPPORTER                               = 3,

	/// A defender is positioning itself not too far away from the goal
	/// in order to defend the goal against opponent strikers
	ROLE_DEFENDER                                = 4,

	/// A goalie is positioned inside the penalty box, and the last line
	/// of defense. It may touch the ball, and it has special protection.
	ROLE_GOALIE                                  = 5
};

typedef MixedTeamRoleEnum MixedTeamRole;


/*------------------------------------------------------------------------------------------------*/

/// enum describing the current action performed by the robot
enum MixedTeamActionEnum {
	/// undefined action (if nothing else matches)
	ACTION_UNDEFINED                             = 0,

	/// robot is trying to position at a certain position
	ACTION_POSITIONING                           = 1,

	/// robot is trying to reach the ball
	ACTION_GOING_TO_BALL                         = 2,

	/// robot is in the possession of the ball (i.e. ball is in front)
	/// and is actively trying to move the ball into the opponent goal
	/// by e.g. dribbling or kicking ...
	ACTION_TRYING_TO_SCORE                       = 3,

	/// robot is waiting for an event (e.g. ball coming closer)
	ACTION_WAITING                               = 4
};

typedef MixedTeamActionEnum MixedTeamAction;


/*------------------------------------------------------------------------------------------------*/

/// the playing state of the robot
enum MixedTeamStateEnum {
	/// robot is not doing anything or incapable of doing anything
	STATE_INACTIVE                                  = 0,

	/// the robot is ready to play or is playing already
	STATE_ACTIVE                                    = 1,

	/// The robot is penalized
	STATE_PENALIZED                                 = 2,

};

typedef MixedTeamStateEnum MixedTeamState;


/*------------------------------------------------------------------------------------------------*/

static const uint32_t MITECOM_RANGE_STATE        = 0x00000000;
static const uint32_t MITECOM_RANGE_COGNITION    = 0x00010000;
static const uint32_t MITECOM_RANGE_CAPABILITIES = 0x00020000;
static const uint32_t MITECOM_RANGE_USERDEFINED  = 0x10000000; // individual settings

/// enum for addressing the different values the team shares
enum MixedTeamKeyEnum {

	/* ******************************************************************
	** CURRENT ROBOT STATE AND PLANNING
	** *****************************************************************/

	/// current role
	ROBOT_CURRENT_ROLE                           = MITECOM_RANGE_STATE + 0,

	/// current action
	ROBOT_CURRENT_ACTION                         = MITECOM_RANGE_STATE + 1,

	/// current state (init, active, inactive)
	ROBOT_CURRENT_STATE                          = MITECOM_RANGE_STATE + 2,


	/* ******************************************************************
	** modelling and percept data
	** *****************************************************************/

	/// absolute position on the field, x-coordinate (in mm)
	ROBOT_ABSOLUTE_X                             =   MITECOM_RANGE_COGNITION + 0,

	/// absolute position on the field, y-coordinate (in mm)
	ROBOT_ABSOLUTE_Y                             =   MITECOM_RANGE_COGNITION + 1,

	/// orientation on the field, in degree
	ROBOT_ABSOLUTE_ORIENTATION                   =   MITECOM_RANGE_COGNITION + 2,

	/// belief of absolute position on the field (0..255), with 0 = no confidence
	/// and 255 = highest confidence
	ROBOT_ABSOLUTE_BELIEF                        =   MITECOM_RANGE_COGNITION + 3,

	/// relative position of ball to robot, x-coordinate (in mm)
	BALL_RELATIVE_X                              =   MITECOM_RANGE_COGNITION + 4,

	/// relative position of ball to robot, y-coordinate (in mm)
	BALL_RELATIVE_Y                              =   MITECOM_RANGE_COGNITION + 5,


	/* ******************************************************************
	** ROBOT CAPABILITIES
	** *****************************************************************/

	/// average walking speed of robot (in cm/s)
	ROBOT_AVG_WALKING_SPEED_IN_CM_PER_SECOND     =  MITECOM_RANGE_CAPABILITIES + 1,

	/// expected time (in seconds) for robot to reach the ball and
	/// position itself behind it correctly (i.e. aligned to opp goal)
	ROBOT_TIME_TO_POSITION_AT_BALL_IN_SECONDS    =  MITECOM_RANGE_CAPABILITIES + 2,

	/// the maximum (realistic) distance the ball can be kicked
	ROBOT_MAX_KICKING_DISTANCE                   =  MITECOM_RANGE_CAPABILITIES + 3,

    /* *******************************************************************
    ** TEAM DATA
    ** ******************************************************************/
    /// The Direction of the Kickoff
    KICKOFF_OFFENCE_SIDE                         = MITECOM_RANGE_USERDEFINED + 80 + 1,

    // Position of an opponent robot, x-coordinate (in mm)
    OPPONENT_ROBOT_X                             = MITECOM_RANGE_USERDEFINED + 80 + 2,

    // Position of an opponent robot, y-coordinate (in mm)
    OPPONENT_ROBOT_Y                             = MITECOM_RANGE_USERDEFINED + 80 + 3,

    // Position of an teammate, x-coordinate (in mm)
    TEAM_MATE_X                                  = MITECOM_RANGE_USERDEFINED + 80 + 4,

    // Position of an teammate, y-coordinate (in mm)
    TEAM_MATE_Y                                  = MITECOM_RANGE_USERDEFINED + 80 + 5,

    OPPONENT_ROBOT_3							 = MITECOM_RANGE_USERDEFINED + 80 + 6,

    OPPONENT_ROBOT_4							 = MITECOM_RANGE_USERDEFINED + 80 + 7,

    OPPONENT_ROBOT_5							 = MITECOM_RANGE_USERDEFINED + 80 + 8,

    OPPONENT_ROBOT_6							 = MITECOM_RANGE_USERDEFINED + 80 + 9,

    OPPONENT_ROBOT_7							 = MITECOM_RANGE_USERDEFINED + 80 + 10,

    OPPONENT_ROBOT_8							 = MITECOM_RANGE_USERDEFINED + 80 + 11,

    TEAM_MATE_3                                  = MITECOM_RANGE_USERDEFINED + 80 + 12,

    TEAM_MATE_4                                  = MITECOM_RANGE_USERDEFINED + 80 + 13,

    TEAM_MATE_5                                  = MITECOM_RANGE_USERDEFINED + 80 + 14,

    TEAM_MATE_6                                  = MITECOM_RANGE_USERDEFINED + 80 + 15,

    TEAM_MATE_7                                  = MITECOM_RANGE_USERDEFINED + 80 + 16,

    TEAM_MATE_8                                  = MITECOM_RANGE_USERDEFINED + 80 + 17,




};

typedef MixedTeamKeyEnum MixedTeamKey;


/*------------------------------------------------------------------------------------------------*/

/** Structure for a single value
 **
 */

typedef uint32_t MITECOM_KEYTYPE;
typedef  int32_t MITECOM_DATATYPE;

struct MixedTeamCommValueStruct {
	MITECOM_KEYTYPE  key;      /// key (MixedTeamKey)
	MITECOM_DATATYPE value;    /// value
};

typedef MixedTeamCommValueStruct MixedTeamCommValue;


/*------------------------------------------------------------------------------------------------*/

/** The message transmitted over the network.
 **
 */

struct MixedTeamCommMessageStruct {
	uint32_t messageMagic;      /// protocol header magic bytes 'MXTC', little endian

	uint16_t messageVersion;    /// the version of the message protocol, little endian
	uint16_t messageLength;     /// number of values, little endian

	uint32_t messageFlags;      /// flags

	uint16_t teamID;            /// ID of the team, little endian
	uint16_t robotID;           /// ID of the sending robot, little endian

	MixedTeamCommValueStruct values[];
};

typedef MixedTeamCommMessageStruct MixedTeamCommMessage;


/*------------------------------------------------------------------------------------------------*/

typedef std::map<MITECOM_KEYTYPE, MITECOM_DATATYPE> MixedTeamMateData;

struct MixedTeamMateStruct {
	uint16_t     robotID;           /// ID of the sending robot
	uint64_t     lastUpdate;        /// time of last update (in milliseconds)

	MixedTeamMateData data;
};

typedef MixedTeamMateStruct MixedTeamMate;


/*------------------------------------------------------------------------------------------------*/

typedef std::map<uint16_t, MixedTeamMate> MixedTeamMates;


/*------------------------------------------------------------------------------------------------*/
} //namespace MiTeCom

#endif // MITECOM_DATA_H__
