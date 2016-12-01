#include "mitecom-handler.h"
#include "mitecom-data.h"

#include <assert.h>
#include <stdio.h>
#include <arpa/inet.h>


/*------------------------------------------------------------------------------------------------*/

/**
 * Take an incoming message and check if the message was received correctly and store the data in
 * data struct defined in @mitecom-data.h
 * @param messageData
 * @param messageLength
 * @param teamID
 * @return
 */
using namespace MiTeCom;

MixedTeamMate MixedTeamParser::parseIncoming(const void* messageData, uint32_t messageLength, int teamID) {
	// test endianness, the default code only supports little endian, so a conversion between host byte order
	// and network byte order should cause a mismatch to the original value
	assert(htonl(0x12345678) != 0x12345678);

	MixedTeamMate mate;
	mate.robotID = -1; // mark as invalid

	const MixedTeamCommMessage *message = (const MixedTeamCommMessage*)messageData;

	// check magic bytes in header
	if ('MXTC' != message->messageMagic) {
		fprintf(stderr, "Magic value mismatch in received message.");
		return mate;
	}

	// we currently support version 1 only
	if (1 != message->messageVersion) {
		fprintf(stderr, "Unsupported protocol received.");
		return mate;
	}

	// check that we got the full message
	if (messageLength != sizeof(MixedTeamCommMessage) + sizeof(MixedTeamCommValueStruct) * message->messageLength) {
		fprintf(stderr, "Mismatched message length.");
		return mate;
	}

	// only handle messages from members of our own team
	if (message->teamID != teamID) {
		return mate;
	}

	/*
	 * create a mate and store its corresponding values
	 */
	mate.robotID = message->robotID;

	for (uint16_t index = 0;
	     index < message->messageLength;
	     index++)
	{
		MITECOM_KEYTYPE  key   = message->values[index].key;
		MITECOM_DATATYPE value = message->values[index].value;
		mate.data[key] = value;
	}
	return mate;
}



/*------------------------------------------------------------------------------------------------*/

/**
 ** Create the serialization of a team mate's information. The result of this
 ** function can be directly broadcasted.
 **
 ** @param messageSizePtr   pointer to an integer to store the size of the serialized message
 ** @param mate             the team member data to serialize (see @mitecom-data.h)
 ** @param teamID           the ID of the team this team mate belongs to
 ** @param robotID          the ID of the robot
 **
 ** @return pointer to allocated structure (must be released with free())
 */

MixedTeamCommMessage *MixedTeamParser::create(uint32_t *messageSizePtr, const MixedTeamMate &mate, uint16_t teamID, uint16_t robotID) {
	uint32_t messageSize = sizeof(MixedTeamCommMessage) + mate.data.size() * sizeof(MixedTeamCommValue);
	*messageSizePtr = messageSize;

	MixedTeamCommMessage *msgPtr = (MixedTeamCommMessage*)malloc(messageSize); // FIXME

	msgPtr->messageMagic    = 'MXTC';
	msgPtr->messageVersion  = 1;

	msgPtr->messageLength   = mate.data.size();
	msgPtr->messageFlags    = 0;

	msgPtr->teamID          = teamID;
	msgPtr->robotID         = robotID;

	uint32_t index = 0;
	for (MixedTeamMateData::const_iterator it = mate.data.begin(); it != mate.data.end(); it++) {
		msgPtr->values[index].key      = it->first;
		msgPtr->values[index].value    = it->second;
		index++;
	}


	return msgPtr;
}
