/*
 * HelperFunctions.c
 *
 * Created: 29.09.2017 21:44:56
 *  Author: Markus Lorenz
 */ 

#include "HelperFunctions.h"

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void _Delay(uint32_t delayCycles)
{
	for(uint32_t i = 0; i < delayCycles; i++)
	{
		asm("nop");
	}
}

bool queue_delete(Queue *queue) {
	if (queue == NULL) {
		return false;
	}
	while (queue->front != NULL) {
		struct queue_node *node = queue->front;
		queue->front = node->next;
		free(node);
	}
	free(queue);
	return true;
}

bool queue_is_empty(Queue *queue) {
	if (queue == NULL || queue->front == NULL) {
		return true;
	} else {
		return false;
	}
}

Queue *queue_new(void) {
	Queue *queue = malloc(sizeof(*queue));
	if (queue == NULL) {
		return NULL;
	}
	queue->front = queue->back = NULL;
	return queue;
}

queue_node queue_read(Queue *queue) {
	queue_node returnStruct;
	returnStruct.Length = 0;
	if (queue == NULL || queue->front == NULL) {
		return returnStruct;
	}
	struct queue_node *node = queue->front;
	returnStruct.data = node->data;
	returnStruct.Length = node->Length;
	queue->front = node->next;
	if (queue->front == NULL) {
		queue->back = NULL;
	}
	free(node);
	return returnStruct;
}

bool queue_write(Queue *queue, uint8_t* data, uint16_t Length) {
	if (queue == NULL) {
		return false;
	}
	struct queue_node *node = malloc(sizeof(*node));
	if (node == NULL) {
		return false;
	}
	node->data = data;
	node->Length = Length;
	node->next = NULL;
	if (queue->back == NULL) {
		queue->front = queue->back = node;
	} else {
		queue->back->next = node;
		queue->back = node;
	}
	return true;
}

