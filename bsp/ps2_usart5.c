#include "ps2_usart5.h"
#include "usart.h"

#define PS2_USART5_RX_BUFFER_SIZE 256U

static uint8_t rx_byte;
static volatile uint8_t rx_buffer[PS2_USART5_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

HAL_StatusTypeDef ps2_usart5_start(void)
{
    return HAL_UART_Receive_IT(&huart5, &rx_byte, 1U);
}

uint16_t ps2_usart5_read(uint8_t *data, uint16_t max_len)
{
    uint16_t count = 0U;

    if (data == NULL) 
    {
        return 0U;
    }

    while ((count < max_len) && (rx_tail != rx_head)) 
    {
        data[count] = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % PS2_USART5_RX_BUFFER_SIZE);
        count++;
    }

    return count;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != UART5) {
        return;
    }

    next = (uint16_t)((rx_head + 1U) % PS2_USART5_RX_BUFFER_SIZE);
    if (next != rx_tail) {
        rx_buffer[rx_head] = rx_byte;
        rx_head = next;
    }

    /* HAL_UART_Receive_IT receives only the requested byte. Arm it again. */
    (void)HAL_UART_Receive_IT(&huart5, &rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) {
        /* Restart reception after an overrun or other UART receive error. */
        (void)HAL_UART_Receive_IT(&huart5, &rx_byte, 1U);
    }
}
