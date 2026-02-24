#include "sa818s.h"

static uint8_t rxBuffer[200];


static void SA818_Send(char *cmd)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), SA818_TIMEOUT);
}

HAL_StatusTypeDef SA818_WaitFor(char *expected)
{
    uint8_t ch;
    uint16_t idx = 0;
    uint32_t start = HAL_GetTick();

    memset(rxBuffer, 0, sizeof(rxBuffer));

    while ((HAL_GetTick() - start) < 1000)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 10) == HAL_OK)
        {
            if (idx < sizeof(rxBuffer)-1)
            {
                rxBuffer[idx++] = ch;
                rxBuffer[idx] = 0;

                if (strstr((char*)rxBuffer, expected) != NULL)
                    return HAL_OK;
            }
        }
    }

    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef SA818_Handshake(void)
{
    SA818_Send("AT+DMOCONNECT\r\n");
    return SA818_WaitFor("+DMOCONNECT:0");

}

static HAL_StatusTypeDef SA818_SetGroup(void)
{
	SA818_Send("AT+DMOSETGROUP=0,433.6500,433.6500,0019,4,0019\r\n");	//433.6500,433.6500,(tx and rx RF frequency) 0000,4,0019(tx ctcss value, squelch,rx ctcss value)
    return SA818_WaitFor("+DMOSETGROUP:0");
}

HAL_StatusTypeDef SA818_SetVolume(void)
{
    for(int i = 0; i < 3; i++)
    {
        SA818_Send("AT+DMOSETVOLUME=6\r\n");
        if (SA818_WaitFor("+DMOSETVOLUME:0") == HAL_OK)
            return HAL_OK;

        HAL_Delay(100);
    }
    return HAL_ERROR;
}
static HAL_StatusTypeDef SA818_enableTail(void)
{
    SA818_Send("AT+SETTAIL=1\r\n");
    return SA818_WaitFor("+DMOSETTAIL:0");
}

HAL_StatusTypeDef SA818_Init(void)
{
    HAL_Delay(500);

    if(SA818_Handshake() != HAL_OK) return HAL_ERROR;
    HAL_Delay(500);
    if(SA818_SetGroup() != HAL_OK) return HAL_ERROR;
    HAL_Delay(500);
    if(SA818_SetVolume() != HAL_OK) return HAL_ERROR;
    HAL_Delay(500);
    if(SA818_enableTail() != HAL_OK) return HAL_ERROR;
    HAL_Delay(500);

    return HAL_OK;
}
void SA818_SetHighPower(void)
{
    HAL_GPIO_WritePin(SA818_HL_PORT, SA818_HL_PIN, GPIO_PIN_SET);
}

void SA818_SetLowPower(void)
{
    HAL_GPIO_WritePin(SA818_HL_PORT, SA818_HL_PIN, GPIO_PIN_RESET);
}
