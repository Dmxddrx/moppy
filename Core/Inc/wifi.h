#ifndef WIFI_H_
#define WIFI_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

// Function Prototypes
int8_t WIFI_Init(void);
int8_t WIFI_Connect(char* ssid, char* password);
void WIFI_Disconnect(void);
void WIFI_SendCommand(char* command);
int8_t WIFI_WaitForResponse(char* expected_response, uint32_t timeout);
int8_t WIFI_GetIP(char* ip_out);

int8_t WIFI_StartUDP(char* target_ip, uint16_t port);
int8_t WIFI_SendUDPData(char* data);

#endif /* WIFI_H_ */
