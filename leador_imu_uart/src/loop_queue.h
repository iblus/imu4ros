#ifndef __LOOP_QUEUE_H__
#define	__LOOP_QUEUE_H__

typedef struct loop_queue
{
	#define LOOP_QUEUE_LEN					1024
	#define LOOP_QUEUE_MASK 				(LOOP_QUEUE_LEN - 1)

	char data[LOOP_QUEUE_LEN];	/*the data queue*/
	volatile unsigned short front;					/*the position of first added item, which can be used to read next item*/
	volatile unsigned short rear;					/*the position of newly added item*/	
} LOOP_QUEUE;		/* the loop queue for store the status report */

extern void initialize_loop_queue(LOOP_QUEUE *loopQueue);
extern int	loop_queue_is_empty(LOOP_QUEUE *loopQueue);
extern int loop_queue_is_full(LOOP_QUEUE *loopQueue);
extern unsigned int loop_queue_avaliable_items_count(LOOP_QUEUE *loopQueue);
extern void loop_queue_in(LOOP_QUEUE *loopQueue, char *ptr, int len);
extern void loop_queue_out(LOOP_QUEUE *loopQueue, char *ptr, int len);
extern void loop_queue_out_preview(LOOP_QUEUE *loopQueue, char *ptr, int len);

#endif

