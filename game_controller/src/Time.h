/*
 * Time.h
 *
 *  Created on: 08.04.2014
 *      Author: Oliver Krebs
 */

#ifndef TIME_H_
#define TIME_H_

#include <sys/time.h>

/*
 * returns the current time in milliseconds
 */

inline uint64_t getCurrentTime() {
	struct timeval tv;
	gettimeofday(&tv, 0);
	return (uint64_t) tv.tv_sec * 1000 + (uint64_t) tv.tv_usec / 1000;
}


#endif /* TIME_H_ */
