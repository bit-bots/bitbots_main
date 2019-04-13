/*
 * GameControllerServer.h
 *
 *  Created on: 27.06.2009
 *      Author: Stefan
 */

#ifndef GAMECONTROLLERSERVER_H_
#define GAMECONTROLLERSERVER_H_

#include <iostream>
#include "../Thread.h"
#include "../Game.h"
#include "Network.h"
#include "RoboCupGameControlData.h"
#include "ros/ros.h"


/**
 * Server for communication with the GameController
 */
class GameControllerServer : public Thread {
public:
    /**
     * Constructor
     * @param game		the data of the game
     * @param net_interface name of the network interface to use
     */
    GameControllerServer(Game *game, std::string net_interface);

    virtual ~GameControllerServer();

    bool isWifiConnected;
private:
    GameControllerServer(const GameControllerServer &cSource);

    GameControllerServer &operator=(const GameControllerServer &cSource);

    void execute(void *arg);

    void HandlePacket(char *data);

    void SendKeepAlive(void);

    Network *mNetwork;
    Game *mGame;
    uint8_t mPreviousGameState;
    uint8_t mPreviousSecondaryState;
    uint8_t mPreviousPenaltyState[MAX_NUM_PLAYERS];
    uint8_t mPreviousTeamColor;
    uint64_t mLastMsgReceived;
    uint64_t mKickOffUnparalyzeTime;

    bool mWaitingForKickOffDelay;
};

#endif /* GAMECONTROLLERSERVER_H_ */
