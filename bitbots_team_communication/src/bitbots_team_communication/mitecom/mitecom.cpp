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

int TeamMateData::get_relative_ball_x(void)
{
    return data.data[BALL_RELATIVE_X];
}

int TeamMateData::get_relative_ball_y(void)
{
    return data.data[BALL_RELATIVE_Y];
}

int TeamMateData::get_kickoff_offence_side(void)
{
    return data.data[KICKOFF_OFFENCE_SIDE];
}

int TeamMateData::get_ball_time(void)
{
    return data.data[ROBOT_TIME_TO_POSITION_AT_BALL_IN_SECONDS];
}

void TeamMateData::set_lastUpdate(uint64_t lastUpdate)
{
    m_lastUpdate = lastUpdate;
}


int TeamMateData::get_opponent_robot_x(void)
{
    return data.data[OPPONENT_ROBOT_X];
}

int TeamMateData::get_opponent_robot_y(void)
{
    return data.data[OPPONENT_ROBOT_Y];
}

int TeamMateData::get_opponent_robot_3(void)
{
    return data.data[OPPONENT_ROBOT_3];
}

int TeamMateData::get_opponent_robot_4(void)
{
    return data.data[OPPONENT_ROBOT_4];
}

int TeamMateData::get_opponent_robot_5(void)
{
    return data.data[OPPONENT_ROBOT_5];
}

int TeamMateData::get_opponent_robot_6(void)
{
    return data.data[OPPONENT_ROBOT_6];
}

int TeamMateData::get_opponent_robot_7(void)
{
    return data.data[OPPONENT_ROBOT_7];
}

int TeamMateData::get_opponent_robot_8(void)
{
    return data.data[OPPONENT_ROBOT_8];
}

int TeamMateData::get_team_mate_x(void)
{
    return data.data[TEAM_MATE_X];
}

int TeamMateData::get_team_mate_y(void)
{
    return data.data[TEAM_MATE_Y];
}

int TeamMateData::get_team_mate_3(void)
{
    return data.data[TEAM_MATE_3];
}

int TeamMateData::get_team_mate_4(void)
{
    return data.data[TEAM_MATE_4];
}

int TeamMateData::get_team_mate_5(void)
{
    return data.data[TEAM_MATE_5];
}

int TeamMateData::get_team_mate_6(void)
{
    return data.data[TEAM_MATE_6];
}

int TeamMateData::get_team_mate_7(void)
{
    return data.data[TEAM_MATE_7];
}

int TeamMateData::get_team_mate_8(void)
{
    return data.data[TEAM_MATE_8];
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

void mitecom::open_socket(int port)
{
    m_port = port;
    m_sock = mitecom_open(port);
}

void mitecom::set_team_id(int id)
{
    m_teamID = id;
}

mitecom::~mitecom(void)
{
    delete ownData;
}

void mitecom::set_robot_id(int ID)
{
    m_robotID = ID;
}

void mitecom::set_pos(int x, int y, int orientation, int belief)
{
    ownData->data[ROBOT_ABSOLUTE_X] = x;
    ownData->data[ROBOT_ABSOLUTE_Y] = y;
    ownData->data[ROBOT_ABSOLUTE_ORIENTATION] = orientation;
    ownData->data[ROBOT_ABSOLUTE_BELIEF] = belief;

}

void mitecom::set_action(int action)
{
    ownData->data[ROBOT_CURRENT_ACTION] = action;
}

void mitecom::set_role(int role)
{
    ownData->data[ROBOT_CURRENT_ROLE] = role;
}

void mitecom::set_state(int state)
{
    ownData->data[ROBOT_CURRENT_STATE] = state;
}

void mitecom::set_relative_ball(int x, int y)
{
    ownData->data[BALL_RELATIVE_X] = x;
    ownData->data[BALL_RELATIVE_Y] = y;
}

void mitecom::set_ball_time(int sec)
{
    ownData->data[ROBOT_TIME_TO_POSITION_AT_BALL_IN_SECONDS] = sec;
}

void mitecom::set_kickoff_offence_side(int val)
{
    ownData->data[KICKOFF_OFFENCE_SIDE] = val;
}

void mitecom::set_opponent_robot(int x, int y, int op3, int op4, int op5, int op6, int op7, int op8)
{
    ownData->data[OPPONENT_ROBOT_X] = x;
    ownData->data[OPPONENT_ROBOT_Y] = y;
    ownData->data[OPPONENT_ROBOT_3] = op3;
    ownData->data[OPPONENT_ROBOT_4] = op4;
    ownData->data[OPPONENT_ROBOT_5] = op5;
    ownData->data[OPPONENT_ROBOT_6] = op6;
    ownData->data[OPPONENT_ROBOT_7] = op7;
    ownData->data[OPPONENT_ROBOT_8] = op8;
}

void mitecom::set_team_mate(int x ,int y, int tm3, int tm4, int tm5, int tm6, int tm7, int tm8)
{
    ownData->data[TEAM_MATE_X] = x;
    ownData->data[TEAM_MATE_Y] = y;
    ownData->data[TEAM_MATE_3] = tm3;
    ownData->data[TEAM_MATE_4] = tm4;
    ownData->data[TEAM_MATE_5] = tm5;
    ownData->data[TEAM_MATE_6] = tm6;
    ownData->data[TEAM_MATE_7] = tm7;
    ownData->data[TEAM_MATE_8] = tm8;
}

void mitecom::send_data(void)
{
    ownData->robotID = m_robotID;
    ownData->data[ROBOT_MAX_KICKING_DISTANCE] = 250000;
    ownData->data[ROBOT_AVG_WALKING_SPEED_IN_CM_PER_SECOND] = 1;

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
            printf("Mitecom: I didn't hear from %d for a while. Good bye, %d.\n", it->first, it->first);
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
            printf("Mitecom: Invalid Data\n");
        }
        else if (teamMate.robotID > 30)
        {
            printf("Mitecom: Too high RobotID: %d\n", teamMate.robotID);
        }
        else if (teamMate.robotID != m_robotID)
        {
            if (teamRobotData.find(teamMate.robotID) == teamRobotData.end()) {
                printf("Mitecom: Adding robot %d to my list of team mates. Welcome.\n", teamMate.robotID);
                teamRobotData[teamMate.robotID] = new TeamMateData();
            }

            // add team mate to our map
            //teamMates[teamMate.robotID] = teamMate;
            teamRobotData[teamMate.robotID]->setData(teamMate);
            teamRobotData[teamMate.robotID]->set_lastUpdate(getCurrentTime());

            // remember the last time (i.e. now) that we heard from this robot
            //teamMates[teamMate.robotID].lastUpdate = getCurrentTime();
        }

    }
    else
    {
        // add some delay
        //usleep(100*1000 /* microseconds */);
    }
}

const TeamRobotData* mitecom::get_data()
{
    return &teamRobotData;
}
