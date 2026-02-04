/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 42dB DSP Engine - I2S Audio Processor with BLE Control
 * @author         : Robin Kluit
 * @date           : 2026-02-04
 ******************************************************************************
 * @attention
 *
 * DSP audio processor: I2S input -> DSP processing -> I2S output
 * Control interface via ESP32 BLE GATT
 *
 * Peripherals:
 *   USART3 (Debug Console - ST-LINK VCP): PD8(TX), PD9(RX) @ 115200
 *   USART2 (ESP32 BLE Control): PD5(TX), PD6(RX) @ 115200
 *   I2S (Audio): TBD
 *
 ******************************************************************************
 */

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;  // ESP32 communication
UART_HandleTypeDef huart3;  // Debug console (VCP)

/* Ring buffer for received data */
#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;
uint8_t rx_byte;  // Single byte for UART interrupt

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void Error_Handler(void);

/* Debug printf redirection to USART3 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART3_UART_Init();  // Debug console first

    /* Boot message */
    const char *boot_msg = "\r\n\r\n*** 42dB DSP Engine ***\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)boot_msg, strlen(boot_msg), 1000);

    printf("[OK] HAL initialized\r\n");
    printf("[OK] Clock configured\r\n");
    printf("[OK] GPIO initialized\r\n");
    printf("[OK] USART3 (debug console) initialized\r\n");

    MX_USART2_UART_Init();  // ESP32 BLE control
    printf("[OK] USART2 (ESP32 BLE) initialized\r\n");

    /* Print startup banner */
    printf("\r\n");
    printf("=====================================\r\n");
    printf("42dB DSP Engine v0.1\r\n");
    printf("=====================================\r\n");
    printf("ESP32 BLE: PD5(TX), PD6(RX) @ 115200\r\n");
    printf("Debug: PD8(TX), PD9(RX) @ 115200\r\n");
    printf("I2S: Not configured\r\n");
    printf("=====================================\r\n\r\n");

    /* Start receiving data from ESP32 via interrupt */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    printf("[OK] USART2 RX interrupt enabled\r\n\r\n");

    /* Infinite loop */
    uint16_t byte_count = 0;
    uint8_t line_buffer[16];
    uint8_t line_pos = 0;

    while (1)
    {
        /* Check if we have received data */
        if (rx_head != rx_tail)
        {
            /* Get byte from ring buffer */
            uint8_t data = rx_buffer[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

            /* Add to line buffer */
            line_buffer[line_pos++] = data;

            /* Print hex dump in 16-byte rows with ASCII representation */
            if (line_pos == 1)
            {
                /* Start of new line - print offset */
                printf("%04X: ", byte_count);
            }

            /* Print hex byte */
            printf("%02X ", data);

            /* Every 16 bytes or if buffer empty, print ASCII and newline */
            if (line_pos == 16 || rx_head == rx_tail)
            {
                /* Pad if less than 16 bytes */
                for (int i = line_pos; i < 16; i++)
                {
                    printf("   ");
                }

                /* Print ASCII representation */
                printf(" | ");
                for (int i = 0; i < line_pos; i++)
                {
                    if (line_buffer[i] >= 32 && line_buffer[i] <= 126)
                    {
                        printf("%c", line_buffer[i]);
                    }
                    else
                    {
                        printf(".");
                    }
                }
                printf("\r\n");

                /* Reset line */
                byte_count += line_pos;
                line_pos = 0;
            }
        }

        /* Add a small delay to avoid tight polling */
        HAL_Delay(1);
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 120;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART2 Initialization Function (ESP32 Communication)
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART3 Initialization Function (Debug Console / VCP)
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    /* GPIO clocks are enabled in HAL_UART_MspInit() */
    /* UART GPIO pins are configured in HAL_UART_MspInit() */
}

/**
 * @brief  Rx Transfer completed callback
 * @param  huart UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Store byte in ring buffer */
        rx_buffer[rx_head] = rx_byte;
        rx_head = (rx_head + 1) % RX_BUFFER_SIZE;

        /* Continue receiving */
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/**
 * @brief  UART Error callback - required for continuous reception
 * @param  huart UART handle
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Abort resets the HAL state machine properly */
        HAL_UART_Abort(huart);

        /* Clear all error flags */
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                              UART_CLEAR_NEF | UART_CLEAR_OREF);

        /* Re-arm reception */
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
