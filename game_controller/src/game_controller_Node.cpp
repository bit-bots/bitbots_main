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

    mGame->setTeamId(bot_id);
    mGame->setBotId(team_id);
	mGame->setBotAllowedToMove(true);
	mGame->setGameState(Game::PLAYING);

	mGameController = new GameControllerServer(mGame);

	mGame->isAllowedToMove();

	ros::Publisher gameStatePub = n.advertise<humanoid_league_msgs::GameState>("gamestate", 1);
	 while (ros::ok())
	 {
		 humanoid_league_msgs::GameState gameState;
                 //gameState.teamColor=mGame->getTeamColor();
		 gameState.gameState=mGame->getGameState();
                 //gameState.gameResult=mGame->getGameResult();
                 //gameState.botId=mGame->getBotID();
                 //gameState.teamId=mGame->getTeamID();
		 gameState.ownScore=mGame->getOwnScore();
		 gameState.rivalScore=mGame->getRivalScore();
                // gameState.goalDifference=mGame->getGoalDifference();
		 gameState.allowedToMove = mGame->isAllowedToMove();
		 gameState.hasKickOff = mGame->haveKickOff();
		 gameState.penalized = mGame->isPenalized();
		 gameState.firstHalf = mGame->isFirstHalf();
                 //gameState.kickoff_sec = mGame->getKickOffTime().tv_sec;
                 //gameState.kickoff_nsec = mGame->getKickOffTime().tv_usec;
		 gameStatePub.publish(gameState);
		 ros::spinOnce();
		 ros::Duration(0.05).sleep();
	 }


	delete mGameController;
	delete mGame;
}

