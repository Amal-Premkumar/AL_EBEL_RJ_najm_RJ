#include "motor.h"


/* ================= INIT ================= */
void Motor_Init(void)
{
    /* Motor disabled by default */
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_INH_PIN, GPIO_PIN_RESET);

    /* Stop motor */
    Motor_Stop();
}

/* ================= ENABLE / DISABLE ================= */
void Motor_Enable(void)
{
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_INH_PIN, GPIO_PIN_SET);
}

void Motor_Disable(void)
{
    Motor_Stop();
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_INH_PIN, GPIO_PIN_RESET);
}

/* ================= DIRECTION ================= */
void Motor_SetDirection(MotorDir dir)
{
    switch (dir)
    {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_R_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_L_PIN, GPIO_PIN_RESET);
            break;

        case MOTOR_REVERSE:
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_R_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_L_PIN, GPIO_PIN_SET);
            break;

        default: /* STOP */
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_R_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_L_PIN, GPIO_PIN_RESET);
            break;
    }
}

/* ================= STOP ================= */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_PORT,
                      MOTOR_IN_R_PIN | MOTOR_IN_L_PIN,
                      GPIO_PIN_RESET);
}

/* ================= ADC READS ================= */
uint16_t Motor_ReadVoltage(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t v = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}

uint16_t Motor_ReadCurrent(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t i = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return i;
}
