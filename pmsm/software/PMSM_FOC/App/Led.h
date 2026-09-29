#ifndef APP_LED_H
#define APP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    LED_ID_0 = 0U,
    LED_ID_1,
    LED_ID_COUNT
} Led_IdType;

typedef enum
{
    LED_RESULT_OK = 0U,
    LED_RESULT_INVALID_PARAMETER
} Led_ResultType;

#define LED_STATE_OFF ((uint8_t)0U)
#define LED_STATE_ON  ((uint8_t)1U)

void Led_Init(void);
Led_ResultType Led_On(Led_IdType LedId);
Led_ResultType Led_Off(Led_IdType LedId);
Led_ResultType Led_Blink(Led_IdType LedId, uint32_t PeriodMs);
void Led_MainFunction(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LED_H */
