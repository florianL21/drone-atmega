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


uint32_t queue_get_item_count(Queue* queue)
{
	if (queue == NULL) 
	{
		return 0;
	}
	return queue->queueLength;

}

BoolStatusCode queue_is_empty(Queue *queue) 
{
	if(queue == NULL)
		return HelperFunctions_ERROR_GOT_NULL_POINTER;
	return (queue->front == NULL);
}

BoolStatusCode queue_has_space(Queue *queue)
{
	if (queue == NULL) 
	{
		return HelperFunctions_ERROR_GOT_NULL_POINTER;
	}
	return (queue->queueLength < queue->queueMaxLength);
}

StatusCode queue_delete(Queue *queue) {
	if (queue == NULL) {
		return HelperFunctions_ERROR_GOT_NULL_POINTER;
	}
	while (queue->front != NULL) {
		queue_node *node = queue->front;
		queue->front = node->next;
		free(node);
	}
	free(queue);
	return SUCCESS;
}

StatusCode queue_new(Queue* queue, uint32_t maxQueueLength) {
	queue = malloc(sizeof(*queue));
	if (queue == NULL) {
		return HelperFunctions_ERROR_MALLOC_RETURNED_NULL;
	}
	queue->front = queue->back = NULL;
	queue->queueMaxLength = maxQueueLength;
	queue->queueLength = 0;
	return SUCCESS;
}

StatusCode queue_read(Queue *queue, queue_node* read_element) {
	read_element->Length = 0;
	if (queue == NULL) {
		return HelperFunctions_ERROR_GOT_NULL_POINTER;
	}
	if (queue->front == NULL) {
		return HelperFunctions_ERROR_QUEUE_WAS_EMPTY;
	}
	queue_node *node = queue->front;
	read_element->data = node->data;
	read_element->Length = node->Length;
	queue->front = node->next;
	if (queue->front == NULL) {
		queue->back = NULL;
	}
	free(node);
	queue->queueLength--;
	return SUCCESS;
}

StatusCode queue_write(Queue *queue, uint8_t* data, uint16_t Length) {
	if (queue == NULL) 
	{
		return HelperFunctions_ERROR_GOT_NULL_POINTER;
	}
	queue_node *node = malloc(sizeof(*node));
	if (node == NULL) 
	{
		return HelperFunctions_ERROR_MALLOC_RETURNED_NULL;
	}
	if(!queue_has_space(queue))
	{
		return HelperFunctions_ERROR_NOT_READY_FOR_OPERATION;
	}
	node->data = malloc(Length * sizeof(uint8_t));
	if (node->data == NULL) 
	{
		return HelperFunctions_ERROR_MALLOC_RETURNED_NULL;
	}
	for(uint16_t dataIndex = 0; dataIndex < Length; dataIndex++)
	{
		node->data[dataIndex] = data[dataIndex];
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
	return SUCCESS;
}
