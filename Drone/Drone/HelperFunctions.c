/*
 * HelperFunctions.c
 *
 * Created: 29.09.2017 21:44:56
 *  Author: Markus Lorenz
 */ 

#include "HelperFunctions.h"

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
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

bool queue_has_space(Queue *queue)
{
	if (queue == NULL) {
		return false;
	}
	return (queue->queueLength < queue->queueMaxLength);
}

bool queue_delete(Queue *queue) {
	if (queue == NULL) {
		return false;
	}
	while (queue->front != NULL) {
		queue_node *node = queue->front;
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

Queue *queue_new(uint32_t maxQueueLength) {
	Queue *queue = malloc(sizeof(*queue));
	if (queue == NULL) {
		return NULL;
	}
	queue->front = queue->back = NULL;
	queue->queueMaxLength = maxQueueLength;
	queue->queueLength = 0;
	return queue;
}

queue_node queue_read(Queue *queue) {
	queue_node returnStruct;
	returnStruct.Length = 0;
	if (queue == NULL || queue->front == NULL) {
		return returnStruct;
	}
	queue_node *node = queue->front;
	returnStruct.data = node->data;
	returnStruct.Length = node->Length;
	queue->front = node->next;
	if (queue->front == NULL) {
		queue->back = NULL;
	}
	free(node);
	queue->queueLength--;
	return returnStruct;
}

bool queue_write(Queue *queue, uint8_t* data, uint16_t Length, bool CleanupRequired) {
	if (queue == NULL) 
	{
		return false;
	}
	queue_node *node = malloc(sizeof(*node));
	if (node == NULL) 
	{
		return false;
	}
	if(!queue_has_space(queue))
	{
		return false;
	}
	node->data = malloc(Length * sizeof(uint8_t));
	if (node->data == NULL) 
	{
		return false;
	}
	for(uint16_t dataIndex = 0; dataIndex < Length; dataIndex++)
	{
		node->data[dataIndex] = data[dataIndex];
	}
	if(CleanupRequired == true)
	{
		if(data!=NULL)
			free(data);
	}
	node->Length = Length;
	node->next = NULL;
	if (queue->back == NULL) {
		queue->front = queue->back = node;
	} else {
		queue->back->next = node;
		queue->back = node;
	}
	queue->queueLength++;
	return true;
}
