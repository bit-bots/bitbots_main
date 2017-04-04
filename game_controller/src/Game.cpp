/*
 * Game.cpp
 *
 *  Created on: 02.07.2009
 *      Author: Stefan
 */
#include <string>
#include <algorithm>
#include <cctype>

#include "Game.h"
//#include "Debugging/Debugger.h"
//#include "Config.h"
//#include "External/SettingConfig.h"

using namespace std;

Game::Game()
:	mGameState( Game::PLAYING),
	mTeamColor( Game::CYAN),
	mGameResult( Game::DRAW),
	mOwnScore( 0),
	mRivalScore( 0),
	mGoalDifference( 0),
	mTeamID( 0),
	mBotID( 0),
	mHaveKickOff( false),
	mIsAllowedToMove( true),
	mIsPenalized( false),
	mIsFirstHalf(true)
{
	//ConfigFile *config = Config::getInstance();
	//ConfigFile *setting = SettingConfig::getInstance();


	//mTeamID = setting->get<uint8_t>("Robot", "teamID", 23);
	ros::param::get("/network/robot/teamID",mTeamID);

	//mBotID  = config->get<uint8_t>("Game", "botID", 1);
	ros::param::get("/network/game/botID",mBotID);


	//string color = config->getString("Game", "teamColor", "CYAN");
	string color = "CYAN";
	ros::param::get("/network/game/teamColor",color);

	//lint --e{864}
	transform(color.begin(), color.end(), color.begin(), ::toupper);
	mTeamColor = color == "MAGENTA" ? Game::MAGENTA : Game::CYAN;

	mKickOffTime.tv_sec = 0;
	mKickOffTime.tv_usec = 0;
}

Game::~Game()
{
	mListenerList.clear();
}

uint8_t Game::getTeamID() const {
	return mTeamID;
}

uint8_t Game::getBotID() const {
	return mBotID;
}

Game::GameState Game::getGameState() const {
	return mGameState;
}

Game::GameResult Game::getGameResult() const {
	return mGameResult;
}

bool Game::isAllowedToMove() const {
	return mIsAllowedToMove;
}

bool Game::isPenalized() const {
	return mIsPenalized;
}

bool Game::isFirstHalf() const {
	return mIsFirstHalf;
}

Game::TeamColor Game::getTeamColor() const {
	return mTeamColor;
}

uint8_t Game::getOwnScore() const {
	return mOwnScore;
}

uint8_t Game::getRivalScore() const {
	return mRivalScore;
}

int8_t Game::getGoalDifference() const {
	return mGoalDifference;
}

void Game::setBotAllowedToMove(bool state) {
	mIsAllowedToMove = state;
	if (state) {

		//Debugger::INFO("Game", "Bot is now allowed to move");
		ROS_INFO("Bot is now allowed to move");
	} else {
		//Debugger::WARN("Game", "Bot is now disallowed to move");
		ROS_WARN("Bot is now disallowed to move");
	}
}

void Game::setBotPenalized(bool state) {
	//lint --e{731}
	if( (mIsPenalized != state)) {
		mIsPenalized = state;
		if (state) {
			//Debugger::WARN("Game", "Bot is now penalized");
			ROS_WARN("Bot is now penalized");
		} else {
			//Debugger::INFO("Game", "Bot is now un-penalized");
			ROS_INFO( "Bot is now un-penalized");
		}
		std::vector<GameEventListener*>::const_iterator it;
		for( it = mListenerList.begin(); it != mListenerList.end(); ++it) {
			(*it)->isPenalizedHasChanged(mIsPenalized);
		}
	}
}

void Game::setTeamColor(TeamColor color) {
	mTeamColor = color;
	//Debugger::WARN("Game", "Now playing as team %s", mTeamColor == Game::CYAN ? "CYAN" : "MAGENTA");
	ROS_WARN( "Now playing as team %s", mTeamColor == Game::CYAN ? "CYAN" : "MAGENTA");
}

void Game::setGameState(GameState state) {
	if( mGameState != state) {
		mGameState = state;
		std::vector<GameEventListener*>::const_iterator it;
		for( it = mListenerList.begin(); it != mListenerList.end(); ++it) {
			(*it)->gameStateHasChanged(mGameState);
		}
		//Debugger::DEBUG("Game", "GameState changed to: %d", mGameState);
		ROS_DEBUG("GameState changed to: %d", mGameState);
	}
}

void Game::setIsFirstHalf(bool state) {
	if( mIsFirstHalf != state) {
		mIsFirstHalf = state;
		if( mIsFirstHalf) {
			//Debugger::INFO("Game", "First half");
			ROS_INFO("First half");
		} else {
			//Debugger::INFO("Game", "Second half");
			ROS_INFO("Second half");
		}
	}
}

void Game::setScore(uint8_t ownScore, uint8_t rivalScore) {
	// score has changed
	if( (mOwnScore != ownScore) || (mRivalScore != rivalScore)) {
		bool gameResultChanged = false;
		mOwnScore = ownScore;
		mRivalScore = rivalScore;
		mGoalDifference = (int8_t)mOwnScore - (int8_t)mRivalScore;
		//Debugger::DEBUG("Game", "Score has changed to: %d : %d (%d)", mOwnScore, mRivalScore, mGoalDifference);
		ROS_DEBUG("Score has changed to: %d : %d (%d)", mOwnScore, mRivalScore, mGoalDifference);
		if( mGoalDifference > 0) {
			if( mGameResult != Game::WIN) {
				mGameResult = Game::WIN;
				gameResultChanged = true;
				//Debugger::INFO("Game", "GameResult has changed to: WIN");
				ROS_INFO("GameResult has changed to: WIN");
			}
		} else if( mGoalDifference < 0) {
			if( mGameResult != Game::LOSE) {
				mGameResult = Game::LOSE;
				gameResultChanged = true;
				ROS_INFO("GameResult has changed to: LOSE");
			}
		} else {
			if( mGameResult != Game::DRAW) {
				mGameResult = Game::DRAW;
				gameResultChanged = true;
				ROS_INFO("GameResult has changed to: DRAW");
			}
		}
		if( gameResultChanged) {
			std::vector<GameEventListener*>::const_iterator it;
			for( it = mListenerList.begin(); it != mListenerList.end(); ++it) {
				(*it)->gameResultHasChanged(mGameResult);
			}
		}
	}
}

void Game::setTeamId(int id){
    mTeamID = id;
    ROS_INFO("Now I have the team ID %d", id);
}

void Game::setBotId(int id){
    mBotID = id;
    ROS_INFO("Now I have the bot ID %d", id);
}

void Game::setKickoff(bool haveKickoff, const timeval& kickoffTime) {
	//lint --e{731}
	if( (mHaveKickOff != haveKickoff)) {
		mHaveKickOff = haveKickoff;
		mKickOffTime = kickoffTime;
		if( mHaveKickOff) {
			ROS_INFO("We have KickOff!");
		} else {
			ROS_INFO( "Other team has KickOff!");
		}
		std::vector<GameEventListener*>::const_iterator it;
		for( it = mListenerList.begin(); it != mListenerList.end(); ++it) {
			(*it)->kickOffHasChanged(mHaveKickOff);
		}
	}
}

bool Game::haveKickOff() const {
	return mHaveKickOff;
}

timeval Game::getKickOffTime() const {
	return mKickOffTime;
}

bool Game::save() {
	//TODO Save in yaml File
	//ConfigFile *config = Config::getInstance();
	//config->set<int>("Game", "botID", mBotID);
	ros::param::set("/network/game/botID",mBotID);
	if( mTeamColor == Game::CYAN) {
		ros::param::set("/network/game/teamColor","CYAN");
		//config->set("Game", "teamColor", "CYAN");
	} else {
		//config->set("Game", "teamColor", "MAGENTA");
		ros::param::set("/network/game/teamColor","MAGENTA");
	}
	//config->save();
	return true;
}

void Game::addGameEventListener(GameEventListener *listener)
{
	mListenerList.push_back(listener);
}
