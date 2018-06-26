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

using namespace std;

Game::Game()
        : mGameState(Game::PLAYING),
          mSecondaryState(Game::NORMAL),
          mTeamColor(Game::CYAN),
          mGameResult(Game::DRAW),
          mOwnScore(0),
          mRivalScore(0),
          mGoalDifference(0),
          mTeamID(0),
          mBotID(0),
          mHaveKickOff(false),
          mIsAllowedToMove(true),
          mIsPenalized(false),
          mIsFirstHalf(true) {
    mKickOffTime.tv_sec = 0;
    mKickOffTime.tv_usec = 0;
}

Game::~Game() {
    mListenerList.clear();
}

uint8_t Game::getTeamID() const {
    return mTeamID;
}

uint8_t Game::getBotID() const {
    return mBotID;
}

uint16_t Game::getSecondsRemaining() const {
    return mSecondsRemaining;
}

uint8_t Game::getPenaltyShot() const {
    return mPenaltyShot;
}

uint16_t Game::getSingleShots() const {
    return mSingleShots;
}

Game::GameState Game::getGameState() const {
    return mGameState;
}

Game::SecondaryState Game::getSecondaryState() const {
    return mSecondaryState;
}

uint8_t Game::getSecondsTillUnpenalized() const {
    return mSecondsTillUnpenalized;
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
        ROS_INFO("Bot is now allowed to move");
    } else {
        ROS_WARN("Bot is now disallowed to move");
    }
}

void Game::setBotPenalized(bool state) {
    if ((mIsPenalized != state)) {
        mIsPenalized = state;
        if (state) {
            ROS_WARN("Bot is now penalized");
        } else {
            ROS_INFO("Bot is now un-penalized");
        }
        std::vector<GameEventListener *>::const_iterator it;
        for (it = mListenerList.begin(); it != mListenerList.end(); ++it) {
            (*it)->isPenalizedHasChanged(mIsPenalized);
        }
    }
}

void Game::setSecondsTillUnpenalized(uint8_t seconds) {
    mSecondsTillUnpenalized = seconds;
}

void Game::setPenaltyShot(uint8_t number) {
    mPenaltyShot = number;
}

void Game::setSingleShots(uint16_t pattern) {
    mSingleShots = pattern;
}

void Game::setTeamColor(TeamColor color) {
    mTeamColor = color;
    ROS_WARN("Now playing as team %s", mTeamColor == Game::CYAN ? "CYAN" : "MAGENTA");
}

void Game::setGameState(GameState state) {
    if (mGameState != state) {
        mGameState = state;
        std::vector<GameEventListener *>::const_iterator it;
        for (it = mListenerList.begin(); it != mListenerList.end(); ++it) {
            (*it)->gameStateHasChanged(mGameState);
        }
        ROS_DEBUG("GameState changed to: %d", mGameState);
    }
}

void Game::setSecondaryState(SecondaryState state) {
    if (mSecondaryState != state) {
        mSecondaryState = state;
        ROS_DEBUG("SecondaryState changed to: %d", mSecondaryState);
    }
}

void Game::setIsFirstHalf(bool state) {
    if (mIsFirstHalf != state) {
        mIsFirstHalf = state;
        if (mIsFirstHalf) {
            ROS_INFO("First half");
        } else {
            ROS_INFO("Second half");
        }
    }
}

void Game::setScore(uint8_t ownScore, uint8_t rivalScore) {
    // score has changed
    if ((mOwnScore != ownScore) || (mRivalScore != rivalScore)) {
        bool gameResultChanged = false;
        mOwnScore = ownScore;
        mRivalScore = rivalScore;
        mGoalDifference = (int8_t) mOwnScore - (int8_t) mRivalScore;
        ROS_DEBUG("Score has changed to: %d : %d (%d)", mOwnScore, mRivalScore, mGoalDifference);
        if (mGoalDifference > 0) {
            if (mGameResult != Game::WIN) {
                mGameResult = Game::WIN;
                gameResultChanged = true;
                ROS_INFO("GameResult has changed to: WIN");
            }
        } else if (mGoalDifference < 0) {
            if (mGameResult != Game::LOSE) {
                mGameResult = Game::LOSE;
                gameResultChanged = true;
                ROS_INFO("GameResult has changed to: LOSE");
            }
        } else {
            if (mGameResult != Game::DRAW) {
                mGameResult = Game::DRAW;
                gameResultChanged = true;
                ROS_INFO("GameResult has changed to: DRAW");
            }
        }
        if (gameResultChanged) {
            std::vector<GameEventListener *>::const_iterator it;
            for (it = mListenerList.begin(); it != mListenerList.end(); ++it) {
                (*it)->gameResultHasChanged(mGameResult);
            }
        }
    }
}

void Game::setTeamId(int id) {
    mTeamID = id;
    ROS_INFO("Now I have the team ID %d", id);
}

void Game::setBotId(int id) {
    mBotID = id;
    ROS_INFO("Now I have the bot ID %d", id);
}

void Game::setSecondsRemaining(uint16_t seconds) {
    mSecondsRemaining = static_cast<int16_t>(seconds);
}

void Game::setKickoff(bool haveKickoff) {
    mHaveKickOff = haveKickoff;
    if (mHaveKickOff) {
        ROS_INFO("We have KickOff!");
    } else {
        ROS_INFO("Other team has KickOff!");
    }
}

void Game::setKickoff(bool haveKickoff, const timeval &kickoffTime) {
    if ((mHaveKickOff != haveKickoff)) {
        mHaveKickOff = haveKickoff;
        mKickOffTime = kickoffTime;
        if (mHaveKickOff) {
            ROS_INFO("We have KickOff!");
        } else {
            ROS_INFO("Other team has KickOff!");
        }
        std::vector<GameEventListener *>::const_iterator it;
        for (it = mListenerList.begin(); it != mListenerList.end(); ++it) {
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
    // TODO Save in yaml File
    return true;
}

void Game::addGameEventListener(GameEventListener *listener) {
    mListenerList.push_back(listener);
}
