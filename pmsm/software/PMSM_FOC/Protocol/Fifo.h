/**
 * @file Fifo.h
 * @brief FIFO 缓冲区。
 */
#ifndef __FIFO_H
#define __FIFO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define FIFO_SIZE 512   /* Must be power of 2 */

typedef struct {
    uint8_t  buffer[FIFO_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} Fifo_t;

void     Fifo_Init(Fifo_t *fifo);
void     Fifo_Clear(Fifo_t *fifo);
bool     Fifo_IsEmpty(Fifo_t *fifo);
bool     Fifo_IsFull(Fifo_t *fifo);
uint16_t Fifo_Count(Fifo_t *fifo);
uint16_t Fifo_Free(Fifo_t *fifo);

bool     Fifo_Enqueue(Fifo_t *fifo, uint8_t data);
bool     Fifo_Dequeue(Fifo_t *fifo, uint8_t *data);
bool     Fifo_Peek(Fifo_t *fifo, uint8_t *data);

uint16_t Fifo_EnqueueBatch(Fifo_t *fifo, const uint8_t *data, uint16_t len);
uint16_t Fifo_DequeueBatch(Fifo_t *fifo, uint8_t *data, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif /* __FIFO_H */
