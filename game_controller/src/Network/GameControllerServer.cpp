/*
 * GameControllerServer.cpp
 *
 *  Created on: 27.06.2009
 *      Author: Stefan
 */

#include <sys/time.h>
#include "GameControllerServer.h"
//#include "../Config.h"
#include "../Time.h"

#define GAMECONTROLLER_BUFFER_LENGTH	1024
#define KEEPALIVE_INTERVAL	250 // ms
#define KICKOFF_KEEPOUT		10000 // ms
#define GAMECONTROLER_LOST_TIMEOUT	20000 // ms

GameControllerServer::GameControllerServer(Game* game) {
	/*ConfigFile *config = Config::getInstance();
	//mTeamID = config->get<uint8_t>("Game", "teamID", 22);
	mTeamID = config->get<int>("Game", "teamID", 22);*/
	mPreviousGameState = 255;
	for (int i = 0; i < MAX_NUM_PLAYERS; i++) {
		mPreviousPenaltyState[i] = 255;
	}
	mPreviousTeamColor = 255;
	mGame = game;
	mWaitingForKickOffDelay = false;
    isWifiConnected = false;
	mKickOffUnparalyzeTime = 0;
	mLastMsgReceived = 0;
	//Debugger::DEBUG("GameControllerServer", "Team: %d, Port: %d", mGame->getTeamID(), GAMECONTROLLER_PORT);
	ROS_DEBUG("Team: %d, Port: %d", mGame->getTeamID(), GAMECONTROLLER_DATA_PORT);
	mNetwork = new Network(0, GAMECONTROLLER_RETURN_PORT, INADDR_ANY, GAMECONTROLLER_DATA_PORT);
	this->start(this);
}

GameControllerServer::~GameControllerServer() {
	this->stop();
	delete mNetwork;
	mGame = NULL;
}

void GameControllerServer::execute(void* arg) {
    char networkBuffer[GAMECONTROLLER_BUFFER_LENGTH];
    struct timeval timeout;     // timeout for socket access

    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

	uint64_t lastSendTime = 0;
	uint64_t currentTime = 0;

	do {
		if (mNetwork == NULL) {
			break;
		}

		if (mNetwork->receiveData(networkBuffer, (size_t)GAMECONTROLLER_BUFFER_LENGTH, &timeout, NULL, NULL) > 0) {
			// Check header
			if (   networkBuffer[0] == 'R'
				&& networkBuffer[1] == 'G'
				&& networkBuffer[2] == 'm'
				&& networkBuffer[3] == 'e') {
					HandlePacket(&networkBuffer[0]);
			}
		}
		currentTime = getCurrentTime();
		if (mWaitingForKickOffDelay) {
			if (currentTime >= mKickOffUnparalyzeTime) {
				//Debugger::INFO("GameControllerServer", "Kick-off keepout over");
				ROS_INFO("Kick-off keepout over");
				mGame->setBotAllowedToMove(true);
				mGame->setGameState(Game::PLAYING);
				mWaitingForKickOffDelay = false;
			}
		}
		if ((currentTime - lastSendTime) >= KEEPALIVE_INTERVAL) {
			SendKeepAlive();
            //We check each KeepAlive step if the wifi is connected
            if(mNetwork->isWifiConnected()){
                //ROS_ERROR("NO CONNECTION");
                isWifiConnected = true;
            }else{
                isWifiConnected = false;
            }
			lastSendTime = currentTime;
		}
		if (mLastMsgReceived != 0) {
			if (currentTime - mLastMsgReceived >= GAMECONTROLER_LOST_TIMEOUT) {
				//Debugger::WARN("GameControllerServer", "No game controller messages received for %d ms, allowing robot to move", GAMECONTROLER_LOST_TIMEOUT);
				ROS_WARN("No game controller messages received for %d ms, allowing robot to move", GAMECONTROLER_LOST_TIMEOUT);
				mGame->setBotAllowedToMove(true);
				mGame->setGameState(Game::PLAYING);

				mLastMsgReceived = 0;
			}
		}
	} while (IsRunning());
}

void GameControllerServer::HandlePacket(char* data) {
	//Debugger::INFO"GameControllerServer", "Received RGme packet");
    ROS_DEBUG( "Received RGme packet");
	//lint -e(826)
	struct RoboCupGameControlData* msg = (struct RoboCupGameControlData*)data;

	// Check version
	if (msg->version == GAMECONTROLLER_STRUCT_VERSION) {
		// Check if we are one of the teams
		struct TeamInfo* ourTeam = NULL;
		struct TeamInfo* rivalTeam = NULL;
		uint8_t teamId = mGame->getTeamID();
		if (msg->teams[0].teamNumber == teamId) {
			ourTeam = &msg->teams[0];
			rivalTeam = &msg->teams[1];
		} else if (msg->teams[1].teamNumber == teamId) {
			ourTeam = &msg->teams[1];
			rivalTeam = &msg->teams[0];
		}

		if (ourTeam != NULL) {
			//Debugger::DEBUG("GameControllerServer", "Packet received");
			ROS_DEBUG( "Packet received");
			mLastMsgReceived = getCurrentTime();
			// Check if game state changed
			if (msg->state != mPreviousGameState) {
				//Debugger::DEBUG("GameControllerServer", "Game state changed to %i", msg->state);
				 ROS_DEBUG("Game state changed to %i", msg->state);
				switch (msg->state) {
					case STATE_INITIAL:
						//Debugger::INFO("GameControllerServer", "GameState: INITIAL");
						ROS_INFO("GameState: INITIAL");
						mGame->setBotAllowedToMove(false);
						mGame->setGameState(Game::INITIAL);
						break;
					case STATE_READY:
						//Debugger::INFO("GameControllerServer", "GameState: READY");
						ROS_INFO("GameState: READY");
						mGame->setBotAllowedToMove(false);
						mGame->setGameState(Game::READY);
						/*if (msg->kickOffTeam == teamId) {
							mGame->setKickoff(true);
						}else{
							mGame->setKickoff(false);
						}*/
						// update score
						if( rivalTeam != NULL) {
							mGame->setScore(ourTeam->score, rivalTeam->score);
						}
						break;
					case STATE_SET:
						//Debugger::INFO("GameControllerServer", "GameState: SET");
						ROS_INFO("GameState: SET");
						mGame->setBotAllowedToMove(false);
						mGame->setGameState(Game::SET);
						break;
					case STATE_PLAYING:
						//Debugger::INFO("GameControllerServer", "GameState: PLAYING");
						//Debugger::DEBUG("GameControllerServer", "Kickoff for team %d", msg->kickOffTeam);
						ROS_INFO("GameState: PLAYINGs");
						ROS_INFO("KickoffTeam %d",msg->kickOffTeam);
						if (msg->kickOffTeam <  128 ) {
                            ROS_INFO("KickoffTeam %d",msg->kickOffTeam);
                            if (msg->kickOffTeam== teamId) {
								//Debugger::INFO("GameControllerServer", "We got kick-off!");
								ROS_INFO("We got kick-off!");
								timeval kickoffTime;
								gettimeofday(&kickoffTime, 0);
								mGame->setKickoff(true, kickoffTime);
								mGame->setBotAllowedToMove(true);
								mWaitingForKickOffDelay = false;
							} else {
								ROS_INFO("Other team has kick-off, waiting for %d ms", KICKOFF_KEEPOUT);
								//Debugger::INFO("GameControllerServer", "Other team has kick-off, waiting for %d ms", KICKOFF_KEEPOUT);
								mKickOffUnparalyzeTime = mLastMsgReceived;
								mKickOffUnparalyzeTime += KICKOFF_KEEPOUT;
								mWaitingForKickOffDelay = true;
								timeval kickoffTime;
								kickoffTime.tv_sec = 0; kickoffTime.tv_usec = 0;
								mGame->setKickoff(false, kickoffTime);
							}
						} else {
							//Debugger::INFO("GameControllerServer", "Drop ball");
							ROS_INFO("Drop ball");
							timeval kickoffTime;
							kickoffTime.tv_sec = 0; kickoffTime.tv_usec = 0;
							mGame->setKickoff(false, kickoffTime);
							mGame->setBotAllowedToMove(true);
							mWaitingForKickOffDelay = false;
						}
						mGame->setGameState(Game::PLAYING);
						break;
					case STATE_FINISHED:
						//Debugger::INFO("GameControllerServer", "GameState: FINISHED");
						ROS_INFO( "GameState: FINISHED");
						mGame->setBotAllowedToMove(true);
						mGame->setGameState(Game::FINISHED);
						break;
					default:
						//Debugger::WARN("GameControllerServer", "Unknown game state (%d)!", msg->state);
						ROS_WARN("Unknown game state (%d)!", msg->state);
						break;
				}
				mPreviousGameState = msg->state;
			}

			// Check if penalty state changed
			uint8_t botID = mGame->getBotID() - 1;
			if (botID < MAX_NUM_PLAYERS) {
				uint8_t penalty = ourTeam->players[botID].penalty;
				if (penalty != mPreviousPenaltyState[botID]) {
				/*	Debugger::INFO("GameControllerServer",
							"Penalty state of bot %i changed to %i",
							(botID + 1), penalty);*/
					ROS_INFO("Penalty state of bot %i changed to %i",
							(botID + 1), penalty);
					if (penalty > 0) {
						mGame->setBotPenalized(true);
						mGame->setBotAllowedToMove(false);
					} else {
						mGame->setBotPenalized(false);
						if (mGame->getGameState() == Game::PLAYING || mGame->getGameState() == Game::FINISHED) {
							mGame->setBotAllowedToMove(true);
						}
					}
					mPreviousPenaltyState[botID] = penalty;
				}
			}

			// Check our team color
			if (ourTeam->teamColour != mPreviousTeamColor) {
				//Debugger::INFO("GameControllerServer", "Team color changed to %i", ourTeam->teamColour);
				ROS_INFO("Team color changed to %i", ourTeam->teamColour);
				if (ourTeam->teamColour == TEAM_CYAN) {
					mGame->setTeamColor(Game::CYAN);
				} else if (ourTeam->teamColour == TEAM_MAGENTA) {
					mGame->setTeamColor(Game::MAGENTA);
				}
				mPreviousTeamColor = ourTeam->teamColour;
			}

			mGame->setIsFirstHalf(msg->firstHalf == 1);
		} else {
			//Debugger::INFO("GameControllerServer", "Message was for other teams (not %d)", mGame->getTeamID());
			ROS_INFO("Message was for other teams (not %d)", mGame->getTeamID());
		}
	} else {
		//Debugger::DEBUG("GameControllerServer", "Wrong RGme protocol version (%d:%d)!", msg->version, GAMECONTROLLER_STRUCT_VERSION);
		ROS_DEBUG("Wrong RGme protocol version (%d:%d)!", msg->version, GAMECONTROLLER_STRUCT_VERSION);
	}
}

void GameControllerServer::SendKeepAlive(void) {
	struct RoboCupGameControlReturnData msg;

	msg.header[0] = 'R';
	msg.header[1] = 'G';
	msg.header[2] = 'r';
	msg.header[3] = 't';
	msg.version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
	msg.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
	msg.team = mGame->getTeamID();
	msg.player = mGame->getBotID();

	if (mNetwork != NULL) {
		if( !mNetwork->sendData((char*)&msg, sizeof(struct RoboCupGameControlReturnData))) {
			printf("<2> [GameControllerServer] sendKeepAlive: error\n");
		} else {
			//Debugger::DEBUG("GameControllerServer", "KeepAlive sent");
		}
	}
}
