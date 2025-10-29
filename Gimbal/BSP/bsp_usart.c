#include "usart.h"

void (*USART_CallBack[6])();

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        USART_CallBack[0]();
    } else if (huart->Instance == USART3) {
        USART_CallBack[2]();
    } else if (huart->Instance == USART6) {
        USART_CallBack[5]();
    }
}