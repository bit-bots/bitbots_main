#ifndef MITECOM_HANDLER_H__
#define MITECOM_HANDLER_H__

#include "mitecom-data.h"


/*------------------------------------------------------------------------------------------------*/

class MixedTeamParser {
public:
	static MixedTeamMate parseIncoming(const void* messageData, uint32_t messageLength, int teamID);
	static MixedTeamCommMessage *create(uint32_t *messageSizePtr, const MixedTeamMate &mate, uint16_t teamID, uint16_t robotID);
};


#endif
