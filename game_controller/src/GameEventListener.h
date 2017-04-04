/*
 * GameEventListener.h
 *
 *  Created on: 25.02.2014
 *      Author: Oliver Krebs
 */

#ifndef GAMEEVENTLISTENER_H_
#define GAMEEVENTLISTENER_H_

#include "IGame.h"

/**
 * Listener for Game Events
 */
class GameEventListener
{
public:
	GameEventListener() {}
	virtual ~GameEventListener() {}

	/**
	 * the penalized status has changed
	 * @param penalized		new status, true if robot is penalized (not allowed to move)
	 */
	virtual void isPenalizedHasChanged(bool penalized) = 0;

	/**
	 * the game result has changed
	 * @param result		the new result of the game (lose, draw, win)
	 */
	virtual void gameResultHasChanged(IGame::GameResult result) = 0;

	/**
	 * the game state has changed
	 * @param state			the new state (initial, ready, set, playing, finished)
	 */
	virtual void gameStateHasChanged(IGame::GameState state) = 0;

	/**
	 * kickoff has changed
	 * @param haveKickOff	new state, true if the own team has kickoff now
	 */
	virtual void kickOffHasChanged(bool haveKickOff) = 0;
};

#endif /* GAMEEVENTLISTENER_H_ */
