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
#include "../../config.h"
#include "../ErrorHandling/ErrorHandling.h"

struct queue_node {
	struct queue_node *next;
	uint8_t* data;
	uint16_t Length;
};
typedef struct queue_node queue_node;

struct Queue {
	struct queue_node *front;
	struct queue_node *back;
	uint32_t queueLength;
	uint32_t queueMaxLength;
};
typedef struct Queue Queue;

/*
struct MedianFilter {
	uint16_t* FilterValues;
	uint8_t FilterSize;
};
typedef struct MedianFilter MedianFilter;
*/

/*
 *	Maps the value x from an input range to an output range
 *	Parameters:
 *		x		- the value to be mapped
 *		in_min	- lower bound of input range
 *		in_max	- upper bound of input range
 *		out_min	- lower bound of output range
 *		out_max	- upper bound of output range
 */
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
float map_float(float x, float in_min, float in_max, float out_min, float out_max);


/*
 * 
 * Returns true if the queue is empty
 * 
 */
bool queue_is_empty(Queue* queue);

/*
* returns true if there is space in the queue
*/
bool queue_has_space(Queue *queue);

/*
 * 
 * Creates a queue for further usage.
 * returns a Queue* pointer to the new queue
 * Returns NULL if an error occurs
 */
Queue *queue_new(uint32_t maxQueueLength);

/*
 * 
 * Returns the value from the top of the queue and deletes it
 * returns 0 if an error occures
 */
queue_node queue_read(Queue *queue);

/*
 * 
 * Writes a Value to the top of the queue
 * returns 0 if an error occures
 */
ErrorCode queue_write(Queue* writeQueue, uint8_t* writeWalue, uint16_t Length);

/*
 * 
 * Returns the count of stored values in the queue
 * 
 */
uint32_t queue_get_item_count(Queue* queue);

/*
 * 
 * Deletes the queue and all its elements.
 * 
 */
ErrorCode queue_delete(Queue* queue);


/*
ErrorCode median_filter_new(MedianFilter* newFilter, uint8_t size, uint16_t initValue);
ErrorCode median_filter_add(MedianFilter* Filter, uint16_t newValue);
uint16_t median_filter_get(MedianFilter* Filter);

*/


#endif /* HELPERFUNCTIONS_H_ */
