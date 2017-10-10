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

Queue* queue_new()
{
	Queue* newQueue = malloc(sizeof(*newQueue));
	if(newQueue == NULL)
	{
		return NULL;
	} else
  {
    newQueue->start = NULL;
    newQueue->start = newQueue->end = NULL;
    newQueue->elementCount = 0;
    return newQueue;
  }
}

bool queue_is_empty(Queue* queue)
{
  if(queue != NULL)
    return queue->start == NULL;
  else
    return true;
}

uint8_t queue_read(Queue* readQueue)
{
  if(readQueue->start != NULL && readQueue != NULL)
  {
    uint8_t readData;
    Queue_element* workElement = readQueue->start;
    readData = workElement->Data;
    Queue_element* nextStart = workElement->pNext;
    free(readQueue->start);
    readQueue->start = nextStart;
    readQueue->elementCount--;
    return readData;
  } else
  {
    return 0;
  }
}

bool queue_write(Queue* writeQueue, uint8_t writeWalue)
{
  if(writeQueue != NULL)
  {
    if(writeQueue->end == NULL && writeQueue->start == NULL)//first element in queue
    {
      Queue_element* newElement = malloc(sizeof(*newElement));
      if(newElement != NULL)
      {
        newElement->Data = writeWalue;
        newElement->pNext = NULL;
        writeQueue->start = writeQueue->end = newElement;
        writeQueue->elementCount++;
        return true;
      } else
      {
        return false;
      }
    } else
    {
      Queue_element* lastElementInQueue = writeQueue->end;
      if(lastElementInQueue != NULL)
      {
        Queue_element* newElement = malloc(sizeof(*newElement));
        newElement->Data = writeWalue;
        newElement->pNext = NULL;
        lastElementInQueue->pNext = newElement;
        writeQueue->end = newElement;
        writeQueue->elementCount++;
        return true;
      } else
      {
        return false;
      }
    }
  }else
  {
    return false;
  }
}

uint8_t queue_get_item_count(Queue* queue)
{
  if(queue != NULL)
    return queue->elementCount;
  else
    return 0;
}

bool queue_delete(Queue* queue)
{
  if(queue != NULL && queue_is_empty(queue) == false)
  {
    Queue_element* lastPos;
    Queue_element* currentPos = queue->start;
    if(currentPos != NULL)
    {
      uint8_t elementCounter;
      for(elementCounter = 0; elementCounter < queue->elementCount; elementCounter++)
      {
        lastPos = currentPos;
        currentPos = currentPos->pNext;
        if(lastPos != NULL)
        {
          free(lastPos);
        } else
        {
          return false;
        }
      }
      queue->start = NULL;
      queue->end = NULL;
      queue->elementCount = 0;
      free(queue);
      queue = NULL;
    }else
    {
      return false;
    }
  } else
  {
    return false;
  }
  return true;
}
