/*
 * Game.h
 *
 *  Created on: 02.07.2009
 *      Author: Stefan
 */

#ifndef GAME_H_
#define GAME_H_

#include "ros/ros.h"
#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <sys/time.h>
#include "GameEventListener.h"
#include "IGame.h"

/**
 * Game represents all the data from the game published by the GameController.
 * So the current score, kick-off, penalizing, ...
 */
class Game : public IGame {
public:
    Game();

    virtual ~Game();

    /**
     * add a new GameEventListener
     * @param listener		the new listener to register
     */
    void addGameEventListener(GameEventListener *listener);

    /**
     * get the team ID
     * @return ID of the own team
     */
    uint8_t getTeamID() const;

    /**
     * get the bot ID
     * @return ID of me
     */
    uint8_t getBotID() const;

    /**
     * get the number of seconds remaining in the half
     * @return remaining seconds
     */
    uint16_t getSecondsRemaining() const;

    /**
	 * get the number of the current penalty shot during penalty shootout
	 * @return number of the penalty shot
	 */
    uint8_t getPenaltyShot() const;

    /**
     * get a binary pattern indicating the success of each penalty shot,
     * 1 is success, 0 is failure, starting at the least significant bit
     * @return the penalty pattern
     */
    uint16_t getSingleShots() const;

    /**
     * get the own team goal score
     * @return own goal score
     */
    uint8_t getOwnScore() const;

    /**
     * get the rival team goal score
     * @return rival goal score
     */
    uint8_t getRivalScore() const;

    /**
     * get the goal difference (my team - rival team)
     * @return goal difference
     */
    int8_t getGoalDifference() const;

    /**
     * get the time from kick-off
     * @return kick-off time
     */
    timeval getKickOffTime() const;

    /**
     * get the current game state
     * @return game state
     */
    GameState getGameState() const;

    /**
     * get the current game state
     * @return game state
     */
    SecondaryState getSecondaryState() const;

    /**
     * get the seconds until the robot is no more penalized
     * @return remaining seconds
     */
    uint8_t getSecondsTillUnpenalized() const;

    /**
     * get the current game result
     * @return game result
     */
    GameResult getGameResult() const;

    /**
     * get the own team color
     * @return team color
     */
    TeamColor getTeamColor() const;

    /**
     * check if our team has kick-off
     * @return true if our team has kick-off
     */
    bool haveKickOff() const;

    /**
     * check if I am allowed to move
     * @return true if I am allowed to move
     */
    bool isAllowedToMove() const;

    /**
     * check if I am penalized
     * @return true if I am penalized
     */
    bool isPenalized() const;

    /**
     * check if it is first half
     * @return true if it is first half
     */
    bool isFirstHalf() const;

    /**
     * set the state to allow to move
     * @param state		true if I am allowed to move
     */
    void setBotAllowedToMove(bool state);

    /**
     * set the state of p#ifdef WIN32
        WSACleanup();
#endifenalize
     * @param state		true if penalized
     */
    void setBotPenalized(bool state);

    /**
     * set my team color
     * @param color		my new team color
     */
    void setTeamColor(TeamColor color);

    /**
     * set current game state
     * @param state 	current game state
     */
    void setGameState(GameState state);

    /**
     * set current secondary state
     * @param state 	current secondary state
     */
    void setSecondaryState(SecondaryState state);

    /**
     * set the remaining seconds in the half
     * @param seconds   the remaining seconds
     */
    void setSecondsRemaining(uint16_t seconds);

    /**
     * set the number of the current penalty shot
     * @param number    the number of the penalty shot
     */
    void setPenaltyShot(uint8_t number);

    /**
     * set the binary penalty shot pattern, where 1 indicates
     * success and 0 failure (starting at the least significant bit)
     * @param pattern   the penalty shot patternm
     */
    void setSingleShots(uint16_t pattern);

    /**
     * set the seconds until the robot is no more penalized
     * @param seconds   the remaining seconds
     */
    void setSecondsTillUnpenalized(uint8_t seconds);

    /**
     * set the score
     * @param ownScore		my own team goal score
     * @param rivalScore	rival team goal score
     */
    void setScore(uint8_t ownScore, uint8_t rivalScore);

    void setTeamId(int id);

    void setBotId(int id);

    void setKickoff(bool haveKickoff);

    /**
     * set the kick-off data
     * @param haveKickoff	true if our team has kick-off
     * @param kickoffTime	time for kick-off
     */
    void setKickoff(bool haveKickoff, const timeval &kickoffTime);

    /**
     * set first or second half
     * @param state		true if first half
     */
    void setIsFirstHalf(bool state);

    /**
     * save the changes
     * @return true if success
     */
    bool save();

private:
    std::vector<GameEventListener *> mListenerList;
    GameState mGameState;
    SecondaryState mSecondaryState;
    TeamColor mTeamColor;
    GameResult mGameResult;
    timeval mKickOffTime;
    int16_t mSecondsRemaining;
    uint8_t mPenaltyShot;
    uint16_t mSingleShots;
    uint8_t mSecondsTillUnpenalized;
    uint8_t mOwnScore;
    uint8_t mRivalScore;
    int8_t mGoalDifference;
    int mTeamID;
    int mBotID;
    bool mHaveKickOff;
    bool mIsAllowedToMove;
    bool mIsPenalized;
    bool mIsFirstHalf;
};

#endif /* GAME_H_ */
