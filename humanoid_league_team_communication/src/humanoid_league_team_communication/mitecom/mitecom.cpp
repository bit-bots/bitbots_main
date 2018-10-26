#include "mitecom.hpp"

#include <sys/time.h>

#define BUFF_SIZE 5000

using namespace MiTeCom;

uint64_t getCurrentTime () {
	struct timeval tv;
	gettimeofday(&tv, 0);
	return static_cast<uint64_t>( static_cast<uint64_t>(tv.tv_sec) * 1000 + tv.tv_usec / 1000 );
}



void TeamMateData::setData(MixedTeamMate mate)
{
    data = mate;
}

int TeamMateData::get_id(void)
{
    return data.robotID;
}

int TeamMateData::get_role(void)
{
    return data.data[ROBOT_CURRENT_ROLE];
}

int TeamMateData::get_action(void)
{
    return data.data[ROBOT_CURRENT_ACTION];
}

int TeamMateData::get_state(void)
{
    return data.data[ROBOT_CURRENT_STATE];
}


int TeamMateData::get_absolute_x(void)
{
    return data.data[ROBOT_ABSOLUTE_X];
}

int TeamMateData::get_absolute_y(void)
{
    return data.data[ROBOT_ABSOLUTE_Y];
}

int TeamMateData::get_absolute_orientation(void)
{
    return data.data[ROBOT_ABSOLUTE_ORIENTATION];
}

int TeamMateData::get_absolute_belief(void)
{
    return data.data[ROBOT_ABSOLUTE_BELIEF];
}

int TeamMateData::get_relative_ball_x(void)
{
    return data.data[BALL_RELATIVE_X];
}

int TeamMateData::get_relative_ball_y(void)
{
    return data.data[BALL_RELATIVE_Y];
}

int TeamMateData::get_ball_belief(void)
{
    return data.data[BALL_RELATIVE_Y];
}

int TeamMateData::get_oppgoal_relative_x(void)
{
    return data.data[OPPGOAL_RELATIVE_X];
}

int TeamMateData::get_oppgoal_relative_y(void)
{
    return data.data[OPPGOAL_RELATIVE_Y];
}

int TeamMateData::get_oppgoal_belief(void)
{
    return data.data[OPPGOAL_RELATIVE_BELIEF];
}

int TeamMateData::get_opponent_robot_a_x(void)
{
    return data.data[OPPONENT_ROBOT_A_X];
}

int TeamMateData::get_opponent_robot_a_y(void)
{
    return data.data[OPPONENT_ROBOT_A_Y];
}

int TeamMateData::get_opponent_robot_a_belief(void)
{
    return data.data[OPPONENT_ROBOT_A_BELIEF];
}

int TeamMateData::get_opponent_robot_b_x(void)
{
    return data.data[OPPONENT_ROBOT_B_X];
}

int TeamMateData::get_opponent_robot_b_y(void)
{
    return data.data[OPPONENT_ROBOT_B_Y];
}

int TeamMateData::get_opponent_robot_b_belief(void)
{
    return data.data[OPPONENT_ROBOT_B_BELIEF];
}

int TeamMateData::get_opponent_robot_c_x(void)
{
    return data.data[OPPONENT_ROBOT_C_X];
}

int TeamMateData::get_opponent_robot_c_y(void)
{
    return data.data[OPPONENT_ROBOT_C_Y];
}

int TeamMateData::get_opponent_robot_c_belief(void)
{
    return data.data[OPPONENT_ROBOT_C_BELIEF];
}

int TeamMateData::get_opponent_robot_d_x(void)
{
    return data.data[OPPONENT_ROBOT_D_X];
}

int TeamMateData::get_opponent_robot_d_y(void)
{
    return data.data[OPPONENT_ROBOT_D_Y];
}

int TeamMateData::get_opponent_robot_d_belief(void)
{
    return data.data[OPPONENT_ROBOT_D_BELIEF];
}


int TeamMateData::get_team_robot_a_x(void)
{
    return data.data[TEAM_ROBOT_A_X];
}

int TeamMateData::get_team_robot_a_y(void)
{
    return data.data[TEAM_ROBOT_A_Y];
}

int TeamMateData::get_team_robot_a_belief(void)
{
    return data.data[TEAM_ROBOT_A_BELIEF];
}

int TeamMateData::get_team_robot_b_x(void)
{
    return data.data[TEAM_ROBOT_B_X];
}

int TeamMateData::get_team_robot_b_y(void)
{
    return data.data[TEAM_ROBOT_B_Y];
}

int TeamMateData::get_team_robot_b_belief(void)
{
    return data.data[TEAM_ROBOT_B_BELIEF];
}

int TeamMateData::get_team_robot_c_x(void)
{
    return data.data[TEAM_ROBOT_C_X];
}

int TeamMateData::get_team_robot_c_y(void)
{
    return data.data[TEAM_ROBOT_C_Y];
}

int TeamMateData::get_team_robot_c_belief(void)
{
    return data.data[TEAM_ROBOT_C_BELIEF];
}

int TeamMateData::get_avg_walking_speed(void)
{
    return data.data[ROBOT_AVG_WALKING_SPEED_IN_CM_PER_SECOND];
}

int TeamMateData::get_time_to_ball(void)
{
    return data.data[ROBOT_TIME_TO_POSITION_AT_BALL_IN_SECONDS];
}

int TeamMateData::get_max_kicking_distance(void)
{
    return data.data[ROBOT_MAX_KICKING_DISTANCE];
}

int TeamMateData::get_offensive_side(void)
{
    return data.data[OFFENSIVE_SIDE];
}


void TeamMateData::set_lastUpdate(uint64_t lastUpdate)
{
    m_lastUpdate = lastUpdate;
}


uint64_t TeamMateData::get_lastUpdate(void)
{
    return m_lastUpdate;
}

mitecom::mitecom(void)
{
    ownData = new MixedTeamMate();
    m_teamID = -1;
}

mitecom::~mitecom(void)
{
    delete ownData;
}

void mitecom::open_socket(int port)
{
    m_port = port;
    m_sock = mitecom_open(port);
}

void mitecom::send_data(void)
{
    ownData->robotID = m_robotID;
    //ownData->data[ROBOT_MAX_KICKING_DISTANCE] = 250000;
    //ownData->data[ROBOT_AVG_WALKING_SPEED_IN_CM_PER_SECOND] = 1;

    MixedTeamCommMessage *messageDataPtr = NULL;
    uint32_t messageDataLength = 0;

    // serialize and broadcast data
    messageDataPtr = MixedTeamParser::create(&messageDataLength, *ownData, m_teamID, m_robotID);
    mitecom_broadcast(m_sock, m_port , messageDataPtr, messageDataLength);
    free(messageDataPtr);

    // expire team mates we haven't seen in a while
    for (TeamRobotData::iterator it = teamRobotData.begin(); it != teamRobotData.end(); )
    {
        if ((it->second)->get_lastUpdate() + 3000 < getCurrentTime())
        {
            printf("Mitecom: I didn't hear from %d for a while. Good bye, %d.\n", it->first, it->first); //todo change to ROS debug
            delete teamRobotData[it->first];
            teamRobotData.erase(it++);
            //delete teamRobotData[teamMate.robotID]
            //teamRobotData.erase(teamMate.robotID]
            // TODO
        }
        else
        {
            it++;
        }
    }

    // gesendete Daten wieder vergessen
    delete ownData;
    ownData = new MixedTeamMate();
}

void mitecom::recieve_data(void)
{
    char buff[BUFF_SIZE];
    ssize_t messageLength = mitecom_receive(m_sock, buff, BUFF_SIZE);
    if (messageLength > 0)
    {
        // message received, process it
        MixedTeamMate teamMate = MixedTeamParser::parseIncoming(buff, messageLength, m_teamID);
        if (teamMate.robotID == -1)
        {
            printf("Mitecom: Invalid Data\n"); //todo change to ROS debug
        }
        else if (teamMate.robotID > 30)
        {
            printf("Mitecom: Too high RobotID: %d\n", teamMate.robotID); //todo change to ROS debug
        }
        else if (teamMate.robotID != m_robotID)
        {
            if (teamRobotData.find(teamMate.robotID) == teamRobotData.end()) {
                printf("Mitecom: Adding robot %d to my list of team mates. Welcome.\n", teamMate.robotID); //todo change to ROS debug
                teamRobotData[teamMate.robotID] = new TeamMateData();
            }

            // add team mate to our map
            teamRobotData[teamMate.robotID]->setData(teamMate);
            // remember the last time (i.e. now) that we heard from this robot
            teamRobotData[teamMate.robotID]->set_lastUpdate(getCurrentTime());
        }
    }
    else
    {
        // add some delay
        //usleep(100*1000 /* microseconds */);
    }
}

void mitecom::set_robot_id(int ID)
{
    m_robotID = ID;
}

void mitecom::set_team_id(int id)
{
    m_teamID = id;
}

void mitecom::set_role(int role)
{
    ownData->data[ROBOT_CURRENT_ROLE] = role;
}

void mitecom::set_action(int action)
{
    ownData->data[ROBOT_CURRENT_ACTION] = action;
}

void mitecom::set_state(int state)
{
    ownData->data[ROBOT_CURRENT_STATE] = state;
}

void mitecom::set_pos(int x, int y, int orientation, int belief)
{
    ownData->data[ROBOT_ABSOLUTE_X] = x;
    ownData->data[ROBOT_ABSOLUTE_Y] = y;
    ownData->data[ROBOT_ABSOLUTE_ORIENTATION] = orientation;
    ownData->data[ROBOT_ABSOLUTE_BELIEF] = belief;

}

void mitecom::set_relative_ball(int x, int y, int belief)
{
    ownData->data[BALL_RELATIVE_X] = x;
    ownData->data[BALL_RELATIVE_Y] = y;
}

void mitecom::set_opp_goal_relative(int x, int y, int belief)
{
    ownData->data[OPPGOAL_RELATIVE_X] = x;
    ownData->data[OPPGOAL_RELATIVE_Y] = y;
    ownData->data[OPPGOAL_RELATIVE_BELIEF] = belief;
}

void mitecom::set_opponent_robot_a(int x, int y, int belief)
{
    ownData->data[OPPONENT_ROBOT_A_X] = x;
    ownData->data[OPPONENT_ROBOT_A_Y] = y;
    ownData->data[OPPONENT_ROBOT_A_BELIEF] = belief;
}

void mitecom::set_opponent_robot_b(int x, int y, int belief)
{
    ownData->data[OPPONENT_ROBOT_B_X] = x;
    ownData->data[OPPONENT_ROBOT_B_Y] = y;
    ownData->data[OPPONENT_ROBOT_B_BELIEF] = belief;
}
void mitecom::set_opponent_robot_c(int x, int y, int belief)
{
    ownData->data[OPPONENT_ROBOT_C_X] = x;
    ownData->data[OPPONENT_ROBOT_C_Y] = y;
    ownData->data[OPPONENT_ROBOT_C_BELIEF] = belief;
}

void mitecom::set_opponent_robot_d(int x, int y, int belief)
{
    ownData->data[OPPONENT_ROBOT_D_X] = x;
    ownData->data[OPPONENT_ROBOT_D_Y] = y;
    ownData->data[OPPONENT_ROBOT_D_BELIEF] = belief;
}

void mitecom::set_team_robot_a(int x, int y, int belief)
{
    ownData->data[TEAM_ROBOT_A_X] = x;
    ownData->data[TEAM_ROBOT_A_Y] = y;
    ownData->data[TEAM_ROBOT_A_BELIEF] = belief;
}

void mitecom::set_team_robot_b(int x, int y, int belief)
{
    ownData->data[TEAM_ROBOT_B_X] = x;
    ownData->data[TEAM_ROBOT_B_Y] = y;
    ownData->data[TEAM_ROBOT_B_BELIEF] = belief;
}

void mitecom::set_team_robot_c(int x, int y, int belief)
{
    ownData->data[TEAM_ROBOT_C_X] = x;
    ownData->data[TEAM_ROBOT_C_Y] = y;
    ownData->data[TEAM_ROBOT_C_BELIEF] = belief;
}

void mitecom::set_get_avg_walking_speed(int speed)
{
    ownData->data[ROBOT_AVG_WALKING_SPEED_IN_CM_PER_SECOND] = speed;
}

void mitecom::set_time_to_ball(int sec)
{
    ownData->data[ROBOT_TIME_TO_POSITION_AT_BALL_IN_SECONDS] = sec;
}

void mitecom::set_max_kicking_distance(int distance)
{
    ownData->data[ROBOT_MAX_KICKING_DISTANCE] = distance;
}


void mitecom::set_offensive_side(int side)
{
    ownData->data[OFFENSIVE_SIDE] = side;
}


const TeamRobotData* mitecom::get_data()
{
    return &teamRobotData;
}
