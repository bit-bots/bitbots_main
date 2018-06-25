/*
 * IGame.h
 *
 *  Created on: 25.02.2014
 *      Author: Oliver
 */

#ifndef IGAME_H_
#define IGAME_H_

/**
 * Interface for Game
 */
class IGame
{
public:
	/**
	 * team colors
	 */
	enum TeamColor {
		CYAN,			//!< cyan
		MAGENTA			//!< magenta
	};

	/**
	 * game states
	 */
	enum GameState {
		INITIAL = 0,	//!< initial, state before READY
		READY = 1,		//!< READY, robots should take their positions
		SET = 2,		//!< SET, robots should be in position now
		PLAYING = 3,	//!< PLAY, game is in play
		FINISHED = 4	//!< game is finished
	};

	/**
	 * result of a game
	 */
	enum GameResult {
		LOSE = -1,		//!< our team is loosing (rival team has more goals than us)
		DRAW = 0,		//!< the game is drawn (we have the same number of goals like the rival team)
		WIN = 1			//!< our team is winning (we have more goals than the rival team)
	};

	virtual ~IGame() {}

	/**
	 * get the team ID
	 * @return ID of the own team
	 */
	virtual uint8_t getTeamID() const = 0;

	/**
	 * get the bot ID
	 * @return ID of me
	 */
	virtual uint8_t getBotID() const = 0;

	/**
	 * get the own team goal score
	 * @return own goal score
	 */
	virtual uint8_t getOwnScore() const = 0;

	/**
	 * get the rival team goal score
	 * @return rival goal score
	 */
	virtual uint8_t getRivalScore() const = 0;
	
	/**
	 * get the goal difference (my team - rival team)
	 * @return goal difference
	 */
	virtual int8_t getGoalDifference() const = 0;
	
	/**
	 * get the time from kick-off
	 * @return kick-off time
	 */
	virtual timeval getKickOffTime() const = 0;
	
	/**
	 * get the current game state
	 * @return game state
	 */
	virtual GameState getGameState() const = 0;
	
	/**
	 * get the current game result
	 * @return game result
	 */
	virtual GameResult getGameResult() const = 0;
	
	/**
	 * get the own team color
	 * @return team color
	 */
	virtual TeamColor getTeamColor() const = 0;
	
	/**
	 * check if our team has kick-off
	 * @return true if our team has kick-off
	 */
	virtual bool haveKickOff() const = 0;
	
	/**
	 * check if I am allowed to move
	 * @return true if I am allowed to move
	 */
	virtual bool isAllowedToMove() const = 0;
	
	/**
	 * check if I am penalized
	 * @return true if I am penalized
	 */
	virtual bool isPenalized() const = 0;
	
	/**
	 * check if it is first half
	 * @return true if it is first half
	 */
	virtual bool isFirstHalf() const = 0;

	/**
	 * set the state to allow to move
	 * @param state		true if I am allowed to move
	 */
	virtual void setBotAllowedToMove(bool state) = 0;
	
	/**
	 * set the state of penalize
	 * @param state		true if penalized
	 */
	virtual void setBotPenalized(bool state) = 0;
	
	/**
	 * set my team color
	 * @param color		my new team color
	 */
	virtual void setTeamColor(TeamColor color) = 0;
	
	/**
	 * set current game state
	 * @param state 	current game state
	 */
	virtual void setGameState(GameState state) = 0;
	
	/**
	 * set the score
	 * @param ownScore		my own team goal score
	 * @param rivalScore	rival team goal score
	 */
	virtual void setScore(uint8_t ownScore, uint8_t rivalScore) = 0;
	
	/**
	 * set the kick-off data
	 * @param haveKickoff	true if our team has kick-off
	 * @param kickoffTime	time for kick-off
	 */
	virtual void setKickoff(bool haveKickoff, const timeval& kickoffTime) = 0;
	virtual void setKickoff(bool haveKickoff) = 0;
	
	/**
	 * set first or second half
	 * @param state		true if first half
	 */
	virtual void setIsFirstHalf(bool state) = 0;
};

#endif /* IGAME_H_ */
