#include "fifo.h"
/**
 * @brief 初始化 FIFO 缓冲区。
 * @param fifo FIFO 缓冲区指针。
 */
void Fifo_Init(Fifo_t *fifo) {
    fifo->head = 0;
    fifo->tail = 0;
}
/**
 * @brief 清空 FIFO 缓冲区。
 * @param fifo FIFO 缓冲区指针。
 */
void Fifo_Clear(Fifo_t *fifo) {
    fifo->head = 0;
    fifo->tail = 0;
}
/**
 * @brief 检查 FIFO 缓冲区是否为空。
 * @param fifo FIFO 缓冲区指针。
 * @return true 如果 FIFO 缓冲区为空，否则返回 false。
 */
bool Fifo_IsEmpty(Fifo_t *fifo) {
    return fifo->head == fifo->tail;
}
/**
 * @brief 检查 FIFO 缓冲区是否已满。
 * @param fifo FIFO 缓冲区指针。
 * @return true 如果 FIFO 缓冲区已满，否则返回 false。
 */
bool Fifo_IsFull(Fifo_t *fifo) {
    return ((fifo->head + 1) & (FIFO_SIZE - 1)) == fifo->tail;
}
/**
 * @brief 获取 FIFO 缓冲区中已使用的元素数量。
 * @param fifo FIFO 缓冲区指针。
 * @return FIFO 缓冲区中已使用的元素数量。
 */
uint16_t Fifo_Count(Fifo_t *fifo) {
    return (fifo->head - fifo->tail) & (FIFO_SIZE - 1);
}
/**
 * @brief 获取 FIFO 缓冲区中空闲元素数量。
 * @param fifo FIFO 缓冲区指针。
 * @return FIFO 缓冲区中空闲元素数量。
 */
uint16_t Fifo_Free(Fifo_t *fifo) {
    return FIFO_SIZE - 1 - Fifo_Count(fifo);
}
/**
 * @brief 向 FIFO 缓冲区入队一个元素。
 * @param fifo FIFO 缓冲区指针。
 * @param data 要入队的元素。
 * @return true 如果入队成功，否则返回 false。
 */
bool Fifo_Enqueue(Fifo_t *fifo, uint8_t data) {
    uint16_t next = (fifo->head + 1) & (FIFO_SIZE - 1);
    if (next == fifo->tail) {
        return false;
    }
    fifo->buffer[fifo->head] = data;
    fifo->head = next;
    return true;
}
/**
 * @brief 从 FIFO 缓冲区出队一个元素。
 * @param fifo FIFO 缓冲区指针。
 * @param data 要出队的元素指针。
 * @return true 如果出队成功，否则返回 false。
 */
bool Fifo_Dequeue(Fifo_t *fifo, uint8_t *data) {
    if (fifo->head == fifo->tail) {
        return false;
    }
    *data = fifo->buffer[fifo->tail];
    fifo->tail = (fifo->tail + 1) & (FIFO_SIZE - 1);
    return true;
}
/**
 * @brief 查看 FIFO 缓冲区队头元素。
 * @param fifo FIFO 缓冲区指针。
 * @param data 要查看的元素指针。
 * @return true 如果查看成功，否则返回 false。
 */
bool Fifo_Peek(Fifo_t *fifo, uint8_t *data) {
    if (fifo->head == fifo->tail) {
        return false;
    }
    *data = fifo->buffer[fifo->tail];
    return true;
}
/**
 * @brief 向 FIFO 缓冲区入队多个元素。
 * @param fifo FIFO 缓冲区指针。
 * @param data 要入队的元素指针。
 * @param len 元素数量。
 * @return 入队成功的元素数量。
 */
uint16_t Fifo_EnqueueBatch(Fifo_t *fifo, const uint8_t *data, uint16_t len) {
    uint16_t cnt = 0;
    while (cnt < len && Fifo_Enqueue(fifo, data[cnt])) {
        cnt++;
    }
    return cnt;
}
/**
 * @brief 从 FIFO 缓冲区出队多个元素。
 * @param fifo FIFO 缓冲区指针。
 * @param data 要出队的元素指针。
 * @param len 元素数量。
 * @return 出队成功的元素数量。
 */
uint16_t Fifo_DequeueBatch(Fifo_t *fifo, uint8_t *data, uint16_t len) {
    uint16_t cnt = 0;
    while (cnt < len && Fifo_Dequeue(fifo, &data[cnt])) {
        cnt++;
    }
    return cnt;
}