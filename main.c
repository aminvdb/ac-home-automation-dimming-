#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* ================= HANDLES ================= */
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;
/* ================= OLED ================= */
#define OLED_ADDR (0x3C << 1)
/* ================= UART RX INTERRUPT HC-05 ================= */
uint8_t rxByte;
char rxBuffer[64];
volatile uint8_t rxIndex = 0;
volatile uint8_t btNewData = 0;
char btMessage[64];
/* ================= SLAVE DETECT ================= */
uint8_t slave1_found = 0;   // slave 1 = 0x12
uint8_t slave2_found = 0;   // slave 2 = 0x13
uint32_t last_scan = 0;
uint32_t last_poll = 0;
/* ================= HC-05 STATE ================= */
/*
 * PB15 leest de STATE-uitgang van de HC-05.
 * LOW  = app/Bluetooth niet verbonden -> buttons mogen werken.
 * HIGH = app/Bluetooth verbonden     -> buttons uitgeschakeld.
 */
uint8_t hc05_connected = 0;
uint8_t hc05_prev_connected = 0;
/* Zorgt dat 0 slechts één keer per slave wordt gestuurd bij app-modus */
uint8_t hc05_zero_sent_s1 = 0;
uint8_t hc05_zero_sent_s2 = 0;
/* ================= BUTTON DIMMING ================= */
/*
 * SWITCH1 = PB5 -> slave 2 (0x13)
 * SWITCH2 = PB4 -> slave 1 (0x12)
 *
 * Elke korte druk verandert de dimwaarde met 5:
 *
 * 0 -> 5 -> 10 -> ... -> 90 -> 95
 *                              |
 * 0 <- 5 <- 10 <- ... <- 90 <-+
 *
 * Bij 95 draait de richting om naar beneden.
 * Bij 0 draait de richting om naar boven.
 *
 * Buttons werken alleen wanneer HC-05 NIET verbonden is en de slave WEL verbonden is.
 */
uint8_t dim_slave1 = 0;
uint8_t dim_slave2 = 0;
uint8_t direction_slave1 = 1;   // 1 = omhoog, 0 = omlaag
uint8_t direction_slave2 = 1;   // 1 = omhoog, 0 = omlaag
GPIO_PinState old_switch1 = GPIO_PIN_SET;
GPIO_PinState old_switch2 = GPIO_PIN_SET;
/* ================= FONT ================= */
uint8_t font5x7[][5] = {
    {0x3E,0x51,0x49,0x45,0x3E},  // 0
    {0x00,0x42,0x7F,0x40,0x00},  // 1
    {0x42,0x61,0x51,0x49,0x46},  // 2
    {0x21,0x41,0x45,0x4B,0x31},  // 3
    {0x18,0x14,0x12,0x7F,0x10},  // 4
    {0x27,0x45,0x45,0x45,0x39},  // 5
    {0x3C,0x4A,0x49,0x49,0x30},  // 6
    {0x01,0x71,0x09,0x05,0x03},  // 7
    {0x36,0x49,0x49,0x49,0x36},  // 8
    {0x06,0x49,0x49,0x29,0x1E},  // 9
    {0x48,0x54,0x54,0x54,0x20},  // S
    {0x7F,0x41,0x41,0x22,0x1C},  // D
    {0x1F,0x20,0x40,0x20,0x1F},  // V
    {0x3E,0x41,0x41,0x41,0x3E},  // O
    {0x7F,0x08,0x14,0x22,0x41},  // K
    {0x7F,0x49,0x49,0x49,0x41},  // E
    {0x08,0x04,0x04,0x04,0x08},  // r / :
    {0x00,0x00,0x00,0x00,0x00}   // spatie
};
/* ================= PROTOTYPES ================= */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void OLED_WriteCommand(uint8_t cmd);
void OLED_WriteData(uint8_t *data, uint16_t size);
void OLED_Print(char *str);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Show_Dim(uint8_t slave, uint8_t dim);
void Assign_Address(uint8_t id, uint8_t newAddr);
void I2C_ClearBus(void);
void HC05_ProcessData(char *data);
void Update_HC05_State(void);
void Send_Dim_To_Slave(uint8_t slave, uint8_t channel, uint8_t pwm);
HAL_StatusTypeDef Send_Button_Dim(uint8_t addr, uint8_t dim);
void Process_Buttons(void);
/* ================= PRINTF UART ================= */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
    return len;
}
/* ============================================================
   BLUETOOTH DIMMING
   pwm = 1..255
   ============================================================ */
void Send_Dim_To_Slave(uint8_t slave, uint8_t channel, uint8_t pwm)
{
    uint8_t addr;
    if (slave == 1)
        addr = 0x12;
    else if (slave == 2)
        addr = 0x13;
    else
        return;
    uint8_t dim = 100 - ((uint32_t)pwm * 100 / 255);
    if (dim < 5)
        dim = 5;
    if (dim > 95)
        dim = 95;
    uint8_t cmd;
    if (channel == 0)
        cmd = 0x02;      // triac 1
    else if (channel == 1)
        cmd = 0x03;      // triac 2
    else
        return;
    uint8_t data[2] = {cmd, dim};
    HAL_I2C_Master_Transmit(&hi2c2, (addr << 1), data, 2, 100);
}
/* ============================================================
   BUTTON DIMMING
   Stuurt rechtstreeks dim 0..95 naar triac 1.
   ============================================================ */
HAL_StatusTypeDef Send_Button_Dim(uint8_t addr, uint8_t dim)
{
    if (dim > 95)
        dim = 95;
    uint8_t data[2];
    data[0] = 0x02;   // triac 1
    data[1] = dim;
    return HAL_I2C_Master_Transmit(&hi2c2, (addr << 1), data, 2, 100);
}
/* ============================================================
   OLED DIM WAARDE
   B0 = status S1
   B2 = status S2
   B4 = dimwaarde S1
   B6 = dimwaarde S2
   ============================================================ */
void OLED_Show_Dim(uint8_t slave, uint8_t dim)
{
    char text[20];
    if (slave == 1)
        OLED_WriteCommand(0xB4);
    else if (slave == 2)
        OLED_WriteCommand(0xB6);
    else
        return;
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x10);
    sprintf(text, "S%d D1 V %d      ", slave, dim);
    OLED_Print(text);
}
/* ============================================================
   HC-05 STATE
   PB15 = STATE van HC-05
   Bij verbinding met de app:
   - buttons worden meteen uitgeschakeld;
   - button-dimwaarden worden 0;
   - beide slaves krijgen één keer dimwaarde 0;
   - zolang HC-05 verbonden blijft, veranderen de buttonwaarden niet.
   Na verbreken van Bluetooth:
   - buttons worden opnieuw actief;
   - eerste druk gaat van 0 naar 5.
   ============================================================ */
void Update_HC05_State(void)
{
    uint8_t new_state;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET)
        new_state = 1;
    else
        new_state = 0;
    /* Net verbonden: lokale buttonmodus resetten naar 0 */
    if (new_state == 1 && hc05_prev_connected == 0)
    {
        hc05_connected = 1;
        dim_slave1 = 0;
        dim_slave2 = 0;
        direction_slave1 = 1;
        direction_slave2 = 1;
        hc05_zero_sent_s1 = 0;
        hc05_zero_sent_s2 = 0;
        OLED_Show_Dim(1, 0);
        OLED_Show_Dim(2, 0);
        printf("HC05 connected -> buttons OFF, button values = 0\r\n");
    }
    /*
     * Zolang de app verbonden is:
     * stuur 0 precies één keer naar elke slave zodra die beschikbaar is.
     * Daarna wordt 0 NIET opnieuw gestuurd, zodat de app vrij kan dimmen.
     */
    if (new_state == 1)
    {
        if (!slave1_found)
        {
            hc05_zero_sent_s1 = 0;
        }
        else if (!hc05_zero_sent_s1)
        {
            if (Send_Button_Dim(0x12, 0) == HAL_OK)
                hc05_zero_sent_s1 = 1;
        }
        if (!slave2_found)
        {
            hc05_zero_sent_s2 = 0;
        }
        else if (!hc05_zero_sent_s2)
        {
            if (Send_Button_Dim(0x13, 0) == HAL_OK)
                hc05_zero_sent_s2 = 1;
        }
    }
    /* Net losgekoppeld */
    if (new_state == 0 && hc05_prev_connected == 1)
    {
        hc05_connected = 0;
        hc05_zero_sent_s1 = 0;
        hc05_zero_sent_s2 = 0;
        /*
         * Knoptoestand opnieuw inlezen zodat loskoppelen
         * niet als een druk wordt gezien.
         */
        old_switch1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
        old_switch2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
        printf("HC05 disconnected -> buttons ON\r\n");
    }
    hc05_connected = new_state;
    hc05_prev_connected = new_state;
}
/* ============================================================
   BUTTONS
   SWITCH1 PB5 -> slave 2 = 0x13
   SWITCH2 PB4 -> slave 1 = 0x12
   Voorwaarden:
   - HC-05/app NIET verbonden
   - bijbehorende slave WEL verbonden
   Dimverloop per druk:
   0 -> 5 -> 10 -> ... -> 95 -> 90 -> ... -> 0 -> 5 ...
   ============================================================ */
void Process_Buttons(void)
{
    GPIO_PinState switch1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
    GPIO_PinState switch2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
    /*
     * App verbonden:
     * buttons doen niets en de lokale buttonwaarden blijven 0.
     */
    if (hc05_connected)
    {
        dim_slave1 = 0;
        dim_slave2 = 0;
        direction_slave1 = 1;
        direction_slave2 = 1;
        old_switch1 = switch1;
        old_switch2 = switch2;
        return;
    }
    /* ================= SWITCH1 PB5 -> SLAVE 2 ================= */
    if (old_switch1 == GPIO_PIN_SET &&
        switch1 == GPIO_PIN_RESET)
    {
        if (slave2_found)
        {
            if (direction_slave2)
            {
                if (dim_slave2 < 95)
                {
                    dim_slave2 += 5;
                }
                else
                {
                    direction_slave2 = 0;
                    dim_slave2 -= 5;
                }
            }
            else
            {
                if (dim_slave2 > 0)
                {
                    dim_slave2 -= 5;
                }
                else
                {
                    direction_slave2 = 1;
                    dim_slave2 += 5;
                }
            }
            if (Send_Button_Dim(0x13, dim_slave2) == HAL_OK)
            {
                OLED_Show_Dim(2, dim_slave2);
                printf("SW1 PB5 -> S2 D1 = %d\r\n", dim_slave2);
            }
            else
            {
                slave2_found = 0;
                OLED_WriteCommand(0xB2);
                OLED_WriteCommand(0x00);
                OLED_WriteCommand(0x10);
                OLED_Print("S2 Error");
            }
        }
        HAL_Delay(50);
    }
    /* ================= SWITCH2 PB4 -> SLAVE 1 ================= */
    if (old_switch2 == GPIO_PIN_SET &&
        switch2 == GPIO_PIN_RESET)
    {
        if (slave1_found)
        {
            if (direction_slave1)
            {
                if (dim_slave1 < 95)
                {
                    dim_slave1 += 5;
                }
                else
                {
                    direction_slave1 = 0;
                    dim_slave1 -= 5;
                }
            }
            else
            {
                if (dim_slave1 > 0)
                {
                    dim_slave1 -= 5;
                }
                else
                {
                    direction_slave1 = 1;
                    dim_slave1 += 5;
                }
            }
            if (Send_Button_Dim(0x12, dim_slave1) == HAL_OK)
            {
                OLED_Show_Dim(1, dim_slave1);
                printf("SW2 PB4 -> S1 D1 = %d\r\n", dim_slave1);
            }
            else
            {
                slave1_found = 0;
                OLED_WriteCommand(0xB0);
                OLED_WriteCommand(0x00);
                OLED_WriteCommand(0x10);
                OLED_Print("S1 Error");
            }
        }
        HAL_Delay(50);
    }
    old_switch1 = switch1;
    old_switch2 = switch2;
}
/* ================= HC-05 DATA FUNCTIE ================= */
void HC05_ProcessData(char *data)
{
    int screen;
    int channel;
    int value;
    char oledText[20];
    /*
     * De app mag alleen dimcommando's sturen wanneer
     * de HC-05 STATE aangeeft dat Bluetooth verbonden is.
     *
     * Formaat vanuit de app:
     *
     * slave:channel:value
     *
     * Voorbeeld:
     * 1:0:128
     *
     * screen  1 = slave 1 = 0x12
     * screen  2 = slave 2 = 0x13
     * channel 0 = dimmer 1
     * channel 1 = dimmer 2
     * value     = 1..255
     */
    if (!hc05_connected)
        return;
    if (sscanf(data, "%d:%d:%d", &screen, &channel, &value) == 3)
    {
        if (screen < 1 || screen > 2)
            return;
        if (channel < 0 || channel > 1)
            return;
        if (value < 1)
            value = 1;
        if (value > 255)
            value = 255;
        /* Alleen sturen als de gekozen slave verbonden is */
        if (screen == 1 && !slave1_found)
            return;
        if (screen == 2 && !slave2_found)
            return;
        /* Dimcommando via I2C naar de juiste slave */
        Send_Dim_To_Slave((uint8_t)screen, (uint8_t)channel, (uint8_t)value);
        /*
         * Toon EXACT de waarde die via UART/app binnenkwam
         * op het OLED.
         *
         * App stuurt bijvoorbeeld:
         * 1:0:128
         *
         * OLED toont:
         * S1 D1 V128
         *
         * App stuurt:
         * 2:1:200
         *
         * OLED toont:
         * S2 D2 V200
         */
        if (screen == 1)
            OLED_WriteCommand(0xB4);
        else
            OLED_WriteCommand(0xB6);
        OLED_WriteCommand(0x00);
        OLED_WriteCommand(0x10);
        sprintf(oledText, "S%d D%d V%d      ", screen, channel + 1, value);
        OLED_Print(oledText);
        printf("APP -> S%d D%d V%d\r\n", screen, channel + 1, value);
        /*
         * De lokale buttonwaarden blijven 0 zolang
         * de app verbonden is.
         * Daardoor nemen de fysieke knoppen de app niet over.
         */
    }
}
/* ================= UART RX CALLBACK ================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rxByte == '\n' || rxByte == '\r')
        {
            if (rxIndex > 0)
            {
                rxBuffer[rxIndex] = '\0';
                strncpy(btMessage, rxBuffer, sizeof(btMessage) - 1);
                btMessage[sizeof(btMessage) - 1] = '\0';
                btNewData = 1;
                rxIndex = 0;
            }
        }
        else
        {
            if (rxIndex < sizeof(rxBuffer) - 1)
            {
                rxBuffer[rxIndex++] = rxByte;
            }
            else
            {
                rxIndex = 0;
            }
        }
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}
/* ================= BUS RECOVERY ================= */
void I2C_ClearBus(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_I2C_DeInit(&hi2c2);
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* 9 clockpulsen */
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
    /* STOP-conditie */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1);
    MX_I2C2_Init();
}
/* ================= ADDRESS ASSIGN ================= */
void Assign_Address(uint8_t id, uint8_t newAddr)
{
    uint8_t data[2] = {id, newAddr};
    if (hi2c2.State == HAL_I2C_STATE_BUSY ||
        HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_NONE)
    {
        I2C_ClearBus();
    }
    HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit( &hi2c2, (0x00 << 1), data, 2, 200 );
    if (status == HAL_OK)
    {
        printf("Assigned addr 0x%02X to ID 0x%02X\r\n", newAddr, id);
    }
    else
    {
        I2C_ClearBus();
    }
    HAL_Delay(50);
    HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
}
/* ================= OLED COMMAND ================= */
void OLED_WriteCommand(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, data, 2, 100);
}
/* ================= OLED DATA ================= */
void OLED_WriteData(uint8_t *data, uint16_t size)
{
    uint8_t buffer[129];
    if (size > 128)
        size = 128;
    buffer[0] = 0x40;
    memcpy(&buffer[1], data, size);
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buffer, size + 1, 100);
}
/* ================= OLED PRINT ================= */
void OLED_Print(char *str)
{
    for (int i = 0; str[i] != '\0'; i++)
    {
        uint8_t idx = 17;
        if (str[i] >= '0' && str[i] <= '9')
        {
            idx = str[i] - '0';
        }
        else
        {
            switch (str[i])
            {
                case 'S':
                    idx = 10;
                    break;
                case 'D':
                    idx = 11;
                    break;
                case 'V':
                    idx = 12;
                    break;
                case 'O':
                    idx = 13;
                    break;
                case 'K':
                    idx = 14;
                    break;
                case 'E':
                    idx = 15;
                    break;
                case 'r':
                    idx = 16;
                    break;
                case ':':
                    idx = 16;
                    break;
                case ' ':
                    idx = 17;
                    break;
                default:
                    idx = 17;
                    break;
            }
        }
        OLED_WriteData(font5x7[idx], 5);
        uint8_t space = 0x00;
        OLED_WriteData(&space, 1);
    }
}
/* ================= OLED INIT ================= */
void OLED_Init(void)
{
    HAL_Delay(200);
    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0x20);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0x7F);
    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0xAF);
}
/* ================= OLED CLEAR ================= */
void OLED_Clear(void)
{
    uint8_t zero[128] = {0};
    for (uint8_t page = 0; page < 8; page++)
    {
        OLED_WriteCommand(0xB0 + page);
        OLED_WriteCommand(0x00);
        OLED_WriteCommand(0x10);
        OLED_WriteData(zero, 128);
    }
}
/* ================= MAIN ================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_USART1_UART_Init();
    OLED_Init();
    OLED_Clear();
    /* status slave 1 */
    OLED_WriteCommand(0xB0);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x10);
    OLED_Print("S1 Error");
    /* status slave 2 */
    OLED_WriteCommand(0xB2);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x10);
    OLED_Print("S2 Error");
    /* beginwaarden dimming */
    OLED_Show_Dim(1, dim_slave1);
    OLED_Show_Dim(2, dim_slave2);
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    printf("MASTER START\r\n");
    I2C_ClearBus();
    uint8_t cmdData[2] = {0x01, 0x00};
    uint8_t rxData[6] = {0};
    uint8_t rxData1[6] = {0};
    char msg1[] = "S1\n";
    char msg2[] = "S2\n";
    while (1)
    {
        /* HC-05 STATE controleren */
        Update_HC05_State();
        /* Buttons zijn alleen actief als de app niet verbonden is */
        Process_Buttons();
        /* ================= BLUETOOTH ================= */
        if (btNewData)
        {
            btNewData = 0;
            HC05_ProcessData(btMessage);
        }
        /* ================= ADDRESS ASSIGN ================= */
        if (HAL_GetTick() - last_scan >= 2000)
        {
            last_scan = HAL_GetTick();
            if (!slave1_found)
            {
                Assign_Address(0xA2, 0x12);
            }
            if (!slave2_found)
            {
                Assign_Address(0xA1, 0x13);
            }
        }
        /* ================= SLAVE POLLING ================= */
        if (HAL_GetTick() - last_poll >= 300)
        {
            last_poll = HAL_GetTick();
            /* ================= SLAVE 1 ================= */
            if (HAL_I2C_Master_Transmit(
                    &hi2c2,
                    (0x12 << 1),
                    cmdData,
                    2,
                    50) == HAL_OK)
            {
                HAL_Delay(5);
                if (HAL_I2C_Master_Receive(
                        &hi2c2,
                        (0x12 << 1),
                        rxData,
                        5,
                        50) == HAL_OK)
                {
                    slave1_found = 1;
                    rxData[5] = '\0';
                    OLED_WriteCommand(0xB0);
                    OLED_WriteCommand(0x00);
                    OLED_WriteCommand(0x10);
                    OLED_Print("S1 OK   ");
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg1, strlen(msg1), 100);
                }
                else
                {
                    slave1_found = 0;
                    OLED_WriteCommand(0xB0);
                    OLED_WriteCommand(0x00);
                    OLED_WriteCommand(0x10);
                    OLED_Print("S1 Error");
                }
            }
            else
            {
                slave1_found = 0;
                OLED_WriteCommand(0xB0);
                OLED_WriteCommand(0x00);
                OLED_WriteCommand(0x10);
                OLED_Print("S1 Error");
                I2C_ClearBus();
            }
            /* ================= SLAVE 2 ================= */
            if (HAL_I2C_Master_Transmit(
                    &hi2c2,
                    (0x13 << 1),
                    cmdData,
                    2,
                    50) == HAL_OK)
            {
                HAL_Delay(5);
                if (HAL_I2C_Master_Receive(
                        &hi2c2,
                        (0x13 << 1),
                        rxData1,
                        5,
                        50) == HAL_OK)
                {
                    slave2_found = 1;
                    rxData1[5] = '\0';
                    OLED_WriteCommand(0xB2);
                    OLED_WriteCommand(0x00);
                    OLED_WriteCommand(0x10);
                    OLED_Print("S2 OK   ");
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg2, strlen(msg2), 100);
                }
                else
                {
                    slave2_found = 0;
                    OLED_WriteCommand(0xB2);
                    OLED_WriteCommand(0x00);
                    OLED_WriteCommand(0x10);
                    OLED_Print("S2 Error");
                }
            }
            else
            {
                slave2_found = 0;
                OLED_WriteCommand(0xB2);
                OLED_WriteCommand(0x00);
                OLED_WriteCommand(0x10);
                OLED_Print("S2 Error");
                I2C_ClearBus();
            }
            /* status naar Bluetooth */
            char statusMsg[8];
            sprintf(statusMsg, "%d,%d\n", slave1_found, slave2_found);
            HAL_UART_Transmit(&huart1, (uint8_t*)statusMsg, strlen(statusMsg), 100);
        }
    }
}
/* ================= CLOCK ================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* HSI-klok configureren */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Systeemklokken configureren */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK    |
        RCC_CLOCKTYPE_SYSCLK  |
        RCC_CLOCKTYPE_PCLK1   |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ================= I2C1: OLED ================= */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.ClockSpeed       = 100000;
    hi2c1.Init.DutyCycle        = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ================= I2C2: SLAVES ================= */

static void MX_I2C2_Init(void)
{
    hi2c2.Instance              = I2C2;
    hi2c2.Init.ClockSpeed       = 100000;
    hi2c2.Init.DutyCycle        = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1      = 0;
    hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2      = 0;
    hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ================= USART1: HC-05 ================= */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ================= GPIO ================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO-klokken inschakelen */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /*
     * PB4 is standaard voor JTAG gereserveerd.
     * JTAG uitschakelen, maar SWD behouden.
     */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* ================= ENABLE-UITGANG ================= */

    HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = enable_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(enable_GPIO_Port, &GPIO_InitStruct);

    /* ================= KNOPPEN PB4 EN PB5 =================
     *
     * switch1 = PB5
     * switch2 = PB4
     *
     * Niet ingedrukt = HIGH
     * Ingedrukt      = LOW
     */

    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ================= HC-05 STATE PB15 =================
     *
     * LOW  = Bluetooth niet verbonden
     * HIGH = Bluetooth verbonden
     */

    GPIO_InitStruct.Pin   = GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ================= USART1 TX PA9 ================= */

    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ================= USART1 RX PA10 ================= */

    GPIO_InitStruct.Pin   = GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


/* ================= ERROR HANDLER ================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
        /* Programma blijft hier staan wanneer een fout optreedt. */
    }
}
