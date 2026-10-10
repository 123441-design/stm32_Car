#ifndef PS2_USART5_H
#define PS2_USART5_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* Call once after MX_UART5_Init() to start receiving one byte at a time. */
HAL_StatusTypeDef ps2_usart5_start(void);

/* Copy up to max_len received bytes into data; return the number copied.
 * No data available: return 0 immediately (this function does not wait).
 */
uint16_t ps2_usart5_read(uint8_t *data, uint16_t max_len);

#endif /* PS2_USART5_H */
