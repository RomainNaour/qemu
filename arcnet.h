/*
 * QEMU ARCnet generic declarations
 */

#ifndef ARCNET_H
#define ARCNET_H

#include "qemu-common.h"

/* arcnet packet size
 * The packets can have only 2 sizes: short or long */
#define ARCNET_SHORT_SIZE 256
#define ARCNET_LONG_SIZE  512
#define ARCNET_MAX_SIZE ARCNET_LONG_SIZE

/* arcnet header
 * The source and destination bytes are node ids (0 for broadcast).
 * If the short start byte is 0, it is a long packet.
 * The start byte is the offset of the first data byte. */
enum arcnet_packet_offsets {
	ARCNET_SOURCE = 0,
	ARCNET_DESTINATION,
	ARCNET_SHORT_START,
	ARCNET_SHORT_HEADER_SIZE,
	ARCNET_LONG_START = ARCNET_SHORT_HEADER_SIZE,
	ARCNET_LONG_HEADER_SIZE
} ;

/* get the size of the packet */
#define ARCNET_SIZE(buffer) (((uint8_t*)buffer)[ARCNET_SHORT_START] != 0 \
	? ARCNET_SHORT_SIZE \
	: ARCNET_LONG_SIZE \
)
/* get the size of the header for a packet size */
#define ARCNET_HEADER_SIZE(size) (size == ARCNET_SHORT_SIZE \
	? ARCNET_SHORT_HEADER_SIZE \
	: ARCNET_LONG_HEADER_SIZE \
)
/* get the start byte offset for a packet size */
#define ARCNET_START(size) (size == ARCNET_SHORT_SIZE \
	? ARCNET_SHORT_START \
	: ARCNET_LONG_START \
)

/* The body of a packet coontains padding and data to the end.
 * The data sizes ranges are not contiguous. */
#define ARCNET_SHORT_DATA_MIN_SIZE 1
#define ARCNET_SHORT_DATA_MAX_SIZE (ARCNET_SHORT_SIZE - ARCNET_SHORT_HEADER_SIZE)
#define ARCNET_LONG_DATA_MIN_SIZE  (ARCNET_LONG_SIZE  - 0xff)
#define ARCNET_LONG_DATA_MAX_SIZE  (ARCNET_LONG_SIZE  - ARCNET_LONG_HEADER_SIZE)
/* get the data pointer of a packet */
#define ARCNET_DATA(buffer) (ARCNET_SIZE(buffer) == ARCNET_SHORT_SIZE \
	? ((uint8_t*)buffer) + ((uint8_t*)buffer)[ARCNET_SHORT_START] \
	: ((uint8_t*)buffer) + ((uint8_t*)buffer)[ARCNET_LONG_START ] \
)
/* get the data length of a packet */
#define ARCNET_DATA_LENGTH(buffer) (ARCNET_SIZE(buffer) == ARCNET_SHORT_SIZE \
	? ARCNET_SHORT_SIZE - ((uint8_t*)buffer)[ARCNET_SHORT_START] \
	: ARCNET_LONG_SIZE  - ((uint8_t*)buffer)[ARCNET_LONG_START ] \
)

#endif /* ARCNET_H */
