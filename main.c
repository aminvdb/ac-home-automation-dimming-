#include "main.h"
#include <stdio.h>
#include <string.h>

/* ================= HANDLES ================= */
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* ================= DEFINES ================= */
#define MY_ID           0xA1
#define HALF_CYCLE_US   10000
#define TRIAC_PULSE_US  500
#define TIMER_STEP_US   50

/* ================= VARIABLES ================= */
uint8_t rxData[2];
uint8_t txData[] = "test1";

uint8_t newAddress = 0x13;
volatile uint8_t t = 0;
volatile uint8_t pending_tx = 0;

/* 0 = uit, 95 = volledig aan */
volatile uint8_t dim_value1 = 0;
volatile uint8_t dim_value2 = 0;

volatile uint32_t zero_count = 0;
volatile uint32_t tim2_us = 0;

volatile uint32_t fire_time1 = 11000;
volatile uint32_t fire_time2 = 11000;

volatile uint8_t triac1_fired = 0;
volatile uint8_t triac2_fired = 0;

volatile uint32_t triac1_off_time = 0;
volatile uint32_t triac2_off_time = 0;

/* ================= PROTOTYPES ================= */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

uint32_t Dim_To_Delay_us(uint8_t dim);
void Set_New_Address(uint8_t addr);
void Restart_I2C_Receive(void);
void Error_Handler(void);

/* ================= I2C ================= */
void Restart_I2C_Receive(void)
{
    HAL_I2C_Slave_Receive_IT(&hi2c1, rxData, 2);
}

void Set_New_Address(uint8_t addr)
{
    newAddress = addr;

    HAL_I2C_DeInit(&hi2c1);

    hi2c1.Init.OwnAddress1 = newAddress << 1;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();

    Restart_I2C_Receive();
}

/* ================= DIM DELAY ================= */
/*
   0  = volledig uit
   1  = heel zwak
   95 = volledig aan

   MOC3021 is random-phase, dus geschikt voor phase-cut.
*/
uint32_t Dim_To_Delay_us(uint8_t dim)
{
    if (dim == 0)
        return 11000;   // buiten halve cyclus = nooit vuren

    if (dim > 95)
        dim = 95;

    if (dim >= 95)
        return 200;     // bijna direct na zero-cross = vol aan

    /*
       dim 1  -> ongeveer 9800 us = bijna uit
       dim 95 -> ongeveer 200 us  = bijna vol aan
    */
    return 9800 - (((uint32_t)(dim - 1) * 9600) / 94);
}

/* ================= MAIN ================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();

    HAL_GPIO_WritePin(triac1_GPIO_Port, triac1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(triac2_GPIO_Port, triac2_Pin, GPIO_PIN_RESET);

    Restart_I2C_Receive();

    while (1)
    {
    }
}

/* ================= I2C RX COMPLETE ================= */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        uint8_t cmd = rxData[0];
        uint8_t value = rxData[1];

        if (t == 0)
        {
            if (HAL_GPIO_ReadPin(enable_GPIO_Port, enable_Pin) == GPIO_PIN_SET)
            {
                if (cmd == MY_ID)
                {
                    t = 1;
                    Set_New_Address(value);
                    return;
                }
            }

            Restart_I2C_Receive();
            return;
        }

        if (cmd == 0x01)
        {
            pending_tx = 1;
            HAL_I2C_Slave_Transmit_IT(&hi2c1, txData, 5);
            return;
        }
        else if (cmd == 0x02)
        {
            if (value > 95)
                value = 95;

            dim_value1 = value;
            HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
        }
        else if (cmd == 0x03)
        {
            if (value > 95)
                value = 95;

            dim_value2 = value;
            HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
        }

        Restart_I2C_Receive();
    }
}

/* ================= I2C TX COMPLETE ================= */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        pending_tx = 0;
        Restart_I2C_Receive();
    }
}

/* ================= I2C ERROR ================= */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        Restart_I2C_Receive();
    }
}

/* ================= ZERO CROSS INTERRUPT ================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_15)
    {
        zero_count++;

        fire_time1 = Dim_To_Delay_us(dim_value1);
        fire_time2 = Dim_To_Delay_us(dim_value2);

        tim2_us = 0;

        triac1_fired = 0;
        triac2_fired = 0;
        triac1_off_time = 0;
        triac2_off_time = 0;

        HAL_GPIO_WritePin(triac1_GPIO_Port, triac1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(triac2_GPIO_Port, triac2_Pin, GPIO_PIN_RESET);

        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_Base_Start_IT(&htim2);
    }
}

/* ================= TIM2 CALLBACK ================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        tim2_us += TIMER_STEP_US;

        if (dim_value1 > 0 && !triac1_fired && tim2_us >= fire_time1)
        {
            HAL_GPIO_WritePin(triac1_GPIO_Port, triac1_Pin, GPIO_PIN_SET);
            triac1_fired = 1;
            triac1_off_time = tim2_us + TRIAC_PULSE_US;
        }

        if (triac1_fired && triac1_off_time && tim2_us >= triac1_off_time)
        {
            HAL_GPIO_WritePin(triac1_GPIO_Port, triac1_Pin, GPIO_PIN_RESET);
            triac1_off_time = 0;
        }

        if (dim_value2 > 0 && !triac2_fired && tim2_us >= fire_time2)
        {
            HAL_GPIO_WritePin(triac2_GPIO_Port, triac2_Pin, GPIO_PIN_SET);
            triac2_fired = 1;
            triac2_off_time = tim2_us + TRIAC_PULSE_US;
        }

        if (triac2_fired && triac2_off_time && tim2_us >= triac2_off_time)
        {
            HAL_GPIO_WritePin(triac2_GPIO_Port, triac2_Pin, GPIO_PIN_RESET);
            triac2_off_time = 0;
        }

        if ((triac1_fired || dim_value1 == 0) &&
            triac1_off_time == 0 &&
            (triac2_fired || dim_value2 == 0) &&
            triac2_off_time == 0)
        {
            HAL_TIM_Base_Stop_IT(&htim2);
        }

        if (tim2_us >= 9900)
        {
            HAL_TIM_Base_Stop_IT(&htim2);
            HAL_GPIO_WritePin(triac1_GPIO_Port, triac1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(triac2_GPIO_Port, triac2_Pin, GPIO_PIN_RESET);
        }
    }
}

/* ================= CLOCK ================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

/* ================= I2C INIT ================= */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0x00 << 1;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/* ================= TIM2 INIT ================= */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 7;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 49;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        Error_Handler();

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* ================= GPIO INIT ================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, led_Pin | triac1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(triac2_GPIO_Port, triac2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = led_Pin | triac1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = triac2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(triac2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = enable_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(enable_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ================= ERROR HANDLER ================= */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}
