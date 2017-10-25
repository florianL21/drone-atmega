/*
 * HelperFunctions.h
 *
 * Created: 29.09.2017 21:44:29
 *  Author: Markus Lorenz
 */ 


#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_

#include "sam.h"
#include <stdbool.h>
#include <stdlib.h>

struct queue_node {
	struct queue_node *next;
	uint8_t* data;
	uint16_t Length;
};
typedef struct queue_node queue_node;

struct Queue {
	struct queue_node *front;
	struct queue_node *back;
};
typedef struct Queue Queue;

/*
 *	Maps the value x from an input range to an output range
 *	Parameters:
 *		x		- the value to be mapped
 *		in_min	- lower bound of input range
 *		in_max	- upper bound of input range
 *		out_min	- lower bound of output range
 *		out_max	- upper bound of output range
 */
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

void _Delay(uint32_t delayCycles);

/*
 * 
 * Creates a queue for further usage.
 * returns a Queue* pointer to the new queue
 * Returns NULL if an error occurs
 */
Queue* queue_new();

/*
 * 
 * Returns true if the queue is empty
 * 
 */
bool queue_is_empty(Queue* queue);

/*
 * 
 * Returns the value from the top of the queue and deletes it
 * returns 0 if an error occures
 */
queue_node queue_read(Queue* readQueue);

/*
 * 
 * Writes a Value to the top of the queue
 * returns 0 if an error occures
 */
bool queue_write(Queue* writeQueue, uint8_t* writeWalue, uint16_t Length, bool CleanupRequired);

/*
 * 
 * Returns the count of stored values in the queue
 * 
 */
uint16_t queue_get_item_count(Queue* queue);

/*
 * 
 * Deletes the queue and all its elements.
 * 
 */
bool queue_delete(Queue* queue);




#endif /* HELPERFUNCTIONS_H_ */
