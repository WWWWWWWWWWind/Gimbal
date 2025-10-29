#include "bsp_can.h"

void BspCanInit() {
    //列表模式，在板间通信时接收端可以有效排除电机相关报文
    CAN_FilterTypeDef filterConfig;

    filterConfig.FilterBank = 0;//第0组过滤器
    filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;//列表模式
    filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;//采用16位过滤器
    filterConfig.FilterIdHigh = 0x12 << 5;//一组过滤器最多可配置4个ID
    filterConfig.FilterIdLow = 0x206 << 5;
    filterConfig.FilterMaskIdHigh = 0x207 << 5;
    //filterConfig.FilterMaskIdLow = 0x20A << 5;
    filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filterConfig.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK) {
        Error_Handler();
    }

    // filterConfig.FilterBank = 1;
    // filterConfig.FilterIdHigh = 0x10 << 5;
    // filterConfig.FilterIdLow  = 0x12 << 5;
    // filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    // filterConfig.FilterActivation = ENABLE;
    // if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK) {
    //     Error_Handler();
    // }

    //可接收范围 100-10F
    filterConfig.FilterBank = 14;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 使用掩码模式
    filterConfig.FilterIdHigh = 0x100 << 5;           // 0x100 基准 ID
    filterConfig.FilterMaskIdHigh = 0x7F0 << 5;       // 掩码：前 7 位匹配，后 4 位可变
    filterConfig.FilterIdLow = 0x10F << 5;           // 0x100 基准 ID
    filterConfig.FilterMaskIdLow = 0x7F0 << 5;       // 掩码：前 7 位匹配，后 4 位可变
    filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    if (HAL_CAN_ConfigFilter(&hcan2, &filterConfig) != HAL_OK) {
        Error_Handler();
    }


    filterConfig.FilterBank = 15;
    filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;//列表模式
    filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;//采用16位过滤器
    filterConfig.FilterIdHigh = 0x10 << 5;
    filterConfig.FilterIdLow  = 0x202 << 5;
    filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    filterConfig.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan2, &filterConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}
