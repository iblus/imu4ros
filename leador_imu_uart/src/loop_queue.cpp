#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "loop_queue.h"

/****************************************************************
Return value:	 	None
****************************************************************/
void initialize_loop_queue(LOOP_QUEUE *loopQueue)
{
    bzero(loopQueue, sizeof(LOOP_QUEUE));
}

/****************************************************************
Return value:	 	1	--	loop queue is empty
					0	--	loop queue is not empty
****************************************************************/
int loop_queue_is_empty(LOOP_QUEUE *loopQueue)
{
    return (loopQueue->front == loopQueue->rear);
}

/****************************************************************
Return value:	 	1	--	loop queue is full
					0	--	loop queue is not full
****************************************************************/
int loop_queue_is_full(LOOP_QUEUE *loopQueue)
{
    return ((loopQueue->rear + 1) % (sizeof(loopQueue->data) / sizeof(loopQueue->data[0])) == loopQueue->front);
}

/****************************************************************
Return value:	 	0	--	loop queue is full
					others	--	loop queue avaliable items
****************************************************************/
unsigned int loop_queue_avaliable_items_count(LOOP_QUEUE *loopQueue)
{
    if (loopQueue->rear > loopQueue->front)
    {
        return (loopQueue->rear - loopQueue->front);
    }
    else if (loopQueue->rear < loopQueue->front)
    {
        return (sizeof(loopQueue->data) / sizeof(loopQueue->data[0])) - (loopQueue->front - loopQueue->rear);
    }
    else
    {
        return 0;
    }
}

/****************************************************************
Return value:	 	None
****************************************************************/
void loop_queue_in(LOOP_QUEUE *loopQueue, char *ptr, int len)
{
    unsigned short min = (sizeof(loopQueue->data) / sizeof(loopQueue->data[0])) - loopQueue->rear;

    if (min >= len)
    {
        memcpy(&loopQueue->data[loopQueue->rear], ptr, len);
    }
    else
    {
        memcpy(&loopQueue->data[loopQueue->rear], ptr, min);
        memcpy(&loopQueue->data[(loopQueue->rear + min) % (sizeof(loopQueue->data) / sizeof(loopQueue->data[0]))], (ptr + min), (len - min));
    }

    loopQueue->rear = (loopQueue->rear + len) % (sizeof(loopQueue->data) / sizeof(loopQueue->data[0]));
}
/****************************************************************
Return value:	 	None
****************************************************************/
void loop_queue_out(LOOP_QUEUE *loopQueue, char *ptr, int len)
{
    memset(ptr, 0, len);
    unsigned short min = (sizeof(loopQueue->data) / sizeof(loopQueue->data[0])) - loopQueue->front;
    if (min >= len)
    {
        memcpy(ptr, &loopQueue->data[loopQueue->front], len);
    }
    else
    {
        memcpy(ptr, &loopQueue->data[loopQueue->front], min);
        memcpy((ptr + min), &loopQueue->data[(loopQueue->front + min) % (sizeof(loopQueue->data) / sizeof(loopQueue->data[0]))], (len - min));
    }

    loopQueue->front = (loopQueue->front + len) % (sizeof(loopQueue->data) / sizeof(loopQueue->data[0]));
}
/****************************************************************
Return value:	 	None
param:flag--:0    --> remove data from queue
			 other--> keep data in queue
****************************************************************/
void loop_queue_out_preview(LOOP_QUEUE *loopQueue, char *ptr, int len)
{
    memset(ptr, 0, len);
    unsigned short min = (sizeof(loopQueue->data) / sizeof(loopQueue->data[0])) - loopQueue->front;
    if (min >= len)
    {
        memcpy(ptr, &loopQueue->data[loopQueue->front], len);
    }
    else
    {
        memcpy(ptr, &loopQueue->data[loopQueue->front], min);
        memcpy((ptr + min), &loopQueue->data[(loopQueue->front + min) % (sizeof(loopQueue->data) / sizeof(loopQueue->data[0]))], (len - min));
    }
}
