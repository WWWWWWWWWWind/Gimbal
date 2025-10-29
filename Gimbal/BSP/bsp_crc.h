
#ifndef CODE_BSP_CRC_H
#define CODE_BSP_CRC_H
#ifdef __cplusplus
extern "C" {
#endif // DEBUG

#include "main.h"

// CRC8
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8);

// CRC16
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

#ifdef __cplusplus
}
#endif
#endif //CODE_BSP_CRC_H
