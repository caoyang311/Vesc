#ifndef APP_KEY_H
#define APP_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define KEY_SCAN_PERIOD_MS        (5U)
#define KEY_FILTER_TIME_MS        (20U)
#define KEY_FILTER_COUNT          \
    ((KEY_FILTER_TIME_MS + KEY_SCAN_PERIOD_MS - 1U) / KEY_SCAN_PERIOD_MS)

typedef enum
{
    KEY_ID_0 = 0U,
    KEY_ID_1,
    KEY_ID_2,
    KEY_ID_COUNT
} Key_IdType;

typedef uint8_t Key_StateType;

#define KEY_STATE_RELEASED ((Key_StateType)0U)
#define KEY_STATE_PRESSED  ((Key_StateType)1U)
#define KEY_STATE_MASK(id)  ((uint8_t)(1U << (id)))

void Key_Init(void);
void Key_Scan(void);
Key_StateType Key_GetState(Key_IdType KeyId);
uint8_t Key_GetStateMask(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_KEY_H */
