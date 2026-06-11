#include "wifi.h"
#include "main.h" // ADDED: Needed for the ESP_RST_Pin definition
#include "oled.h"

//extern IWDG_HandleTypeDef hiwdg;

char buffer[512];

void WIFI_SendCommand(const char* command) {
	HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), 100);
}

int8_t WIFI_WaitForResponse(const char* expected_response, uint32_t timeout) {
    uint32_t startTime = HAL_GetTick();
    memset(buffer, 0, sizeof(buffer));
    uint16_t index = 0;

    while (HAL_GetTick() - startTime < timeout) {

    	//HAL_IWDG_Refresh(&hiwdg);

        uint8_t data;
        if (HAL_UART_Receive(&huart1, &data, 1, 1) == HAL_OK) {
            // SAFETY FIX: Prevent buffer overflow if expected response never arrives
            if (index < 511) {
                buffer[index++] = data;
            }
            if (strstr(buffer, expected_response)) {
                return 1; // Success
            }
        }
    }
    return 0; // Timeout
}

int8_t WIFI_Init(void) {
    // --- ADVANCED HARDWARE BOOT & RESET SEQUENCE ---

    // 1. Force the ESP into Normal Boot Mode by pulling GPIO0 HIGH
    HAL_GPIO_WritePin(GPIOA, ESP_IO0_Pin, GPIO_PIN_SET);

    // 2. Pull RST LOW to hard-reset the ESP
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

    // 3. Pull RST HIGH to let it boot (while GPIO0 is still held HIGH)
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);

    // 4. Give the ESP-01S 1 second to fully boot up
    HAL_Delay(800);

    // 5. Release GPIO0 (Set it back to an input)
    // This prevents the STM32 from interfering if the ESP tries to use the pin later.
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ESP_IO0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 6. Flush the UART data register to clear out the boot garbage
    // (The ESP spits out a bunch of random text at 74880 baud when it first turns on)
    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    // ------------------------------------------------

    // Now proceed with normal AT Commands
    // Inside wifi.c - WIFI_Init
    WIFI_SendCommand("AT\r\n");
    if (!WIFI_WaitForResponse("OK", 500)) {
        // Optional: Print the first 10 characters of whatever the ESP actually sent
        OLED_Print(0, 50, buffer);
        OLED_Update();
        return 0;
    }

    WIFI_SendCommand("AT+CWMODE=1\r\n");
    if (!WIFI_WaitForResponse("OK", 500)) return 0;

    return 1; // Init successful
}

int8_t WIFI_Connect(const char* ssid, const char* password) {
    char conn_cmd[128];
    sprintf(conn_cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);

    WIFI_SendCommand(conn_cmd);

    // Give it 15 seconds to connect to the router
    if (WIFI_WaitForResponse("WIFI GOT IP", 15000)) {
        return 1;
    }
    return 0;
}

// Cleanly disconnects from any stuck connection attempts
void WIFI_Disconnect(void) {
    WIFI_SendCommand("AT+CWQAP\r\n");
    WIFI_WaitForResponse("OK", 1000);
}

int8_t WIFI_GetIP(char* ip_out) {
    WIFI_SendCommand("AT+CIFSR\r\n");

    // The response looks like: +CIFSR:STAIP,"192.168.1.15"
    if (WIFI_WaitForResponse("OK", 1000)) {
        char* start = strstr(buffer, "STAIP,\"");
        if (start) {
            start += 7; // Move past STAIP,"
            char* end = strchr(start, '\"');
            if (end) {
                size_t len = end - start;
                strncpy(ip_out, start, len);
                ip_out[len] = '\0'; // Null terminate
                return 1;
            }
        }
    }
    return 0;
}

// WIFI_StartUDP tells the ESP to target your PC.
int8_t WIFI_StartUDP(const char* target_ip, uint16_t port) {
    char cmd[128];
    // Command format: AT+CIPSTART="UDP","192.168.1.50",8080
    sprintf(cmd, "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n", target_ip, port);

    WIFI_SendCommand(cmd);

    // Wait up to 2 seconds for the ESP to confirm the connection
    return WIFI_WaitForResponse("OK", 2000);
}


// WIFI_SendUDPData handles the tricky 2-step process of asking the ESP for permission to send, waiting for the >, and then pushing the data.
int8_t WIFI_SendUDPData(const char* data) {
    char cmd[32];
    uint16_t len = strlen(data);

    // 1. Tell ESP how many bytes we want to send (e.g., AT+CIPSEND=15)
    sprintf(cmd, "AT+CIPSEND=%d\r\n", len);
    WIFI_SendCommand(cmd);

    // 2. Wait for the '>' prompt from the ESP indicating it is ready
    if (WIFI_WaitForResponse(">", 5)) {

        // 3. Send the actual sensor data string
        WIFI_SendCommand(data);

        // 4. Wait for confirmation that it was sent over Wi-Fi
        if (WIFI_WaitForResponse("SEND OK", 5)) {
            return 1; // Success
        }
    }
    return 0; // Failed
}
