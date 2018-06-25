/*
 * game_controller_Node.cpp
 *
 *  Created on: May 9, 2016
 *      Author: Tobi
 */

#include "ros/ros.h"
#include "Network/GameControllerServer.h"
#include "Game.h"
#include "IGame.h"
#include "std_msgs/Bool.h"
#include "humanoid_league_msgs/GameState.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "Game_Controller_Node");

    ros::NodeHandle n;

	GameControllerServer* mGameController;
	Game* mGame;
	mGame = new Game();

    // set IDs coresponding to the config
    int bot_id;
    int team_id;
    n.getParam("bot_id", bot_id);
    n.getParam("team_id", team_id);

    mGame->setTeamId(team_id);
    mGame->setBotId(bot_id);
	mGame->setBotAllowedToMove(true);
	mGame->setGameState(Game::PLAYING);

	mGameController = new GameControllerServer(mGame);

	mGame->isAllowedToMove();

    ros::Publisher networkStatePub = n.advertise<std_msgs::Bool>("wifi_connected", 1);
	ros::Publisher gameStatePub = n.advertise<humanoid_league_msgs::GameState>("gamestate", 1);
	ros::Rate r(3);
	 while (ros::ok())
	 {
		 humanoid_league_msgs::GameState gameState;
                 //gameState.teamColor=mGame->getTeamColor();
         gameState.header.stamp = ros::Time::now();
		 gameState.secondsRemaining = mGame->getSecondsRemaining();
		 gameState.gameState = mGame->getGameState();
		 gameState.secondaryState = mGame->getSecondaryState();
                 //gameState.gameResult=mGame->getGameResult();
                 //gameState.botId=mGame->getBotID();
                 //gameState.teamId=mGame->getTeamID();
		 gameState.ownScore=mGame->getOwnScore();
		 gameState.rivalScore=mGame->getRivalScore();
                // gameState.goalDifference=mGame->getGoalDifference();
		 gameState.allowedToMove = mGame->isAllowedToMove();
		 gameState.hasKickOff = mGame->haveKickOff();
		 gameState.penalized = mGame->isPenalized();
		 gameState.secondsTillUnpenalized = mGame->getSecondsTillUnpenalized();
		 gameState.firstHalf = mGame->isFirstHalf();
		 gameState.penaltyShot = mGame->getPenaltyShot();
		 gameState.singleShots = mGame->getSingleShots();
         const bool isWifi = mGameController->isWifiConnected;
         std_msgs::Bool wifiMsg;
         wifiMsg.data = isWifi;
                 //gameState.kickoff_sec = mGame->getKickOffTime().tv_sec;
         networkStatePub.publish(wifiMsg);
                 //gameState.kickoff_nsec = mGame->getKickOffTime().tv_usec;
		 gameStatePub.publish(gameState);
		 ros::spinOnce();
		 r.sleep();
	 }


	delete mGameController;
	delete mGame;
}

