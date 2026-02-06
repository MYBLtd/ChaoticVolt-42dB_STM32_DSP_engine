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
 *   SAI2_A (I2S Input from BM83): PD13(SCK), PD12(FS), PD11(SD) - Slave RX
 *   SAI2_B (I2S Output to DAC): PA0(SD) - Slave TX synced to SAI2_A
 *
 ******************************************************************************
 */

#include "stm32h7xx_hal.h"
#include "build_number.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Audio Configuration -------------------------------------------------------*/
#define AUDIO_SAMPLE_RATE       44100   /* Match ESP32 default */
#define AUDIO_BIT_DEPTH         16      /* 16-bit audio */
#define AUDIO_CHANNELS          2
#define AUDIO_BUFFER_SAMPLES    256     /* Samples per channel per half-buffer */
#define AUDIO_BUFFER_SIZE       (AUDIO_BUFFER_SAMPLES * AUDIO_CHANNELS)  /* Total samples per half */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;  // ESP32 communication
UART_HandleTypeDef huart3;  // Debug console (VCP)
SAI_HandleTypeDef hsai_rx;  // SAI2_A - I2S Input (Slave RX)
SAI_HandleTypeDef hsai_tx;  // SAI2_B - I2S Output (Slave TX)
DMA_HandleTypeDef hdma_sai_rx;  // DMA for SAI2_A RX
DMA_HandleTypeDef hdma_sai_tx;  // DMA for SAI2_B TX

/* Audio Buffers - placed in D2 SRAM for DMA1/DMA2 access --------------------*/
/* IMPORTANT: DMA1/DMA2 on STM32H7 can ONLY access D2 SRAM (0x30000000)! */
/* Double buffer: DMA fills one half while we process the other */
/* 16-bit samples to match ESP32 I2S format */
__attribute__((section(".RAM_D2"))) int16_t audio_rx_buffer[AUDIO_BUFFER_SIZE * 2];
__attribute__((section(".RAM_D2"))) int16_t audio_tx_buffer[AUDIO_BUFFER_SIZE * 2];

/* Audio state */
volatile uint8_t audio_rx_half_complete = 0;
volatile uint8_t audio_rx_full_complete = 0;
volatile uint32_t audio_buffer_count = 0;
volatile uint32_t audio_overrun_count = 0;

/* Ring buffer for received data */
#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;
uint8_t rx_byte;  // Single byte for UART interrupt

/* Line buffer for GATT commands from ESP32 */
#define LINE_BUFFER_SIZE 128
char line_buffer[LINE_BUFFER_SIZE];
uint16_t line_pos = 0;

/* GATT command handler - called when complete line received */
static void Process_GATT_Command(const char *line);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SAI2_Init(void);
void Error_Handler(void);

/* Audio processing */
static void Audio_Process(int16_t *rx_buf, int16_t *tx_buf, uint16_t samples);

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
    MX_DMA_Init();          // DMA must be initialized before peripherals that use it
    MX_USART3_UART_Init();  // Debug console first

    /* Boot message with build number */
    printf("\r\n\r\n*** 42dB DSP Engine (Build #%d) ***\r\n", BUILD_NUMBER);

    printf("[OK] HAL initialized\r\n");
    printf("[OK] Clock configured\r\n");
    printf("[OK] GPIO initialized\r\n");
    printf("[OK] DMA initialized\r\n");
    printf("[OK] USART3 (debug console) initialized\r\n");

    MX_USART2_UART_Init();  // ESP32 BLE control
    printf("[OK] USART2 (ESP32 BLE) initialized\r\n");

    MX_SAI2_Init();  // I2S Audio
    printf("[OK] SAI2 (I2S Audio) initialized\r\n");
    printf("[NOTE] Use 74HCT04 buffer for I2S signals (5V tolerant)\r\n");

    /* Print startup banner */
    printf("\r\n");
    printf("========================================\r\n");
    printf("42dB DSP Engine v0.2\r\n");
    printf("========================================\r\n");
    printf("ESP32 BLE: PD5(TX), PD6(RX) @ 115200\r\n");
    printf("Debug:     PD8(TX), PD9(RX) @ 115200\r\n");
    printf("I2S Input: PD13(SCK), PD12(FS), PD11(SD)\r\n");
    printf("I2S Output: PA0(SD)\r\n");
    printf("Audio:     %d Hz, %d ch, %d samples/buf\r\n",
           AUDIO_SAMPLE_RATE, AUDIO_CHANNELS, AUDIO_BUFFER_SAMPLES);
    printf("========================================\r\n\r\n");

    /* Start receiving data from ESP32 via interrupt */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    printf("[OK] USART2 RX interrupt enabled\r\n");

    /* Clear audio buffers */
    memset(audio_rx_buffer, 0, sizeof(audio_rx_buffer));
    memset(audio_tx_buffer, 0, sizeof(audio_tx_buffer));

    /* Clear any pending SAI errors and flush FIFOs before starting */
    __HAL_SAI_CLEAR_FLAG(&hsai_rx, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET | SAI_FLAG_WCKCFG);
    __HAL_SAI_CLEAR_FLAG(&hsai_tx, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET | SAI_FLAG_WCKCFG);

    /* Flush FIFOs */
    SAI2_Block_A->CR2 |= SAI_xCR2_FFLUSH;
    SAI2_Block_B->CR2 |= SAI_xCR2_FFLUSH;

    /* Pre-fill TX buffer with silence to avoid garbage at startup */
    memset(audio_tx_buffer, 0, sizeof(audio_tx_buffer));

    /* Start RX DMA first to receive audio data from ESP32/BM83 */
    HAL_StatusTypeDef rx_status = HAL_SAI_Receive_DMA(&hsai_rx, (uint8_t *)audio_rx_buffer,
                                                       AUDIO_BUFFER_SIZE * 2);

    /* Start TX DMA - synchronized to RX for same clock */
    HAL_StatusTypeDef tx_status = HAL_SAI_Transmit_DMA(&hsai_tx, (uint8_t *)audio_tx_buffer,
                                                        AUDIO_BUFFER_SIZE * 2);

    if (rx_status == HAL_OK && tx_status == HAL_OK)
    {
        printf("[OK] SAI2 RX+TX DMA started (passthrough mode)\r\n");
    }
    else
    {
        printf("[WARN] DMA start failed: RX=%d TX=%d\r\n", rx_status, tx_status);
    }

    printf("\r\n[INFO] PASSTHROUGH MODE - Audio from ESP32 to DAC\r\n");
    printf("       RX: BCLK->PD13, LRCK->PD12, SD->PD11\r\n");
    printf("       TX: SD->PA0 (shares BCLK/LRCK from ESP32)\r\n\r\n");

    /* Debug: print SAI and GPIO register status */
    printf("[DEBUG] SAI2_A CR1=0x%08lX SR=0x%08lX\r\n",
           SAI2_Block_A->CR1, SAI2_Block_A->SR);
    printf("[DEBUG] SAI2_B CR1=0x%08lX SR=0x%08lX\r\n",
           SAI2_Block_B->CR1, SAI2_Block_B->SR);
    /* GPIO mode and alternate function for PD11,12,13 */
    printf("[DEBUG] GPIOD MODER=0x%08lX AFR[1]=0x%08lX\r\n",
           GPIOD->MODER, GPIOD->AFR[1]);
    /* GPIO for PA0 (SAI2_B output) */
    printf("[DEBUG] GPIOA MODER=0x%08lX AFR[0]=0x%08lX\r\n",
           GPIOA->MODER, GPIOA->AFR[0]);
    /* Check SAI2 clock source */
    printf("[DEBUG] RCC D2CCIP1R=0x%08lX (SAI23SEL bits 8:6)\r\n",
           RCC->D2CCIP1R);
    /* Read GPIO input values to verify signals */
    printf("[DEBUG] GPIOD IDR=0x%04lX (PD11=%d PD12=%d PD13=%d)\r\n",
           GPIOD->IDR,
           (GPIOD->IDR >> 11) & 1,
           (GPIOD->IDR >> 12) & 1,
           (GPIOD->IDR >> 13) & 1);

    /* Variables for periodic status */
    uint32_t last_status_time = 0;
    uint32_t last_buffer_count = 0;
    uint32_t debug_counter = 0;

    /* Infinite loop */
    while (1)
    {
        /* Audio processing happens in RX DMA callbacks for lowest latency */

        /* Print status every second */
        uint32_t now = HAL_GetTick();
        if (now - last_status_time >= 1000)
        {
            uint32_t buffers_per_sec = audio_buffer_count - last_buffer_count;
            if (audio_buffer_count > 0)
            {
                printf("[AUDIO] Buffers/sec: %lu, Total: %lu, Overruns: %lu\r\n",
                       buffers_per_sec, audio_buffer_count, audio_overrun_count);
            }
            /* Debug: show SAI and DMA status every 5 seconds */
            debug_counter++;
            if ((debug_counter % 5) == 0)
            {
                printf("[DEBUG] SAI_A: SR=0x%lX CR1=0x%08lX\r\n",
                       SAI2_Block_A->SR, SAI2_Block_A->CR1);
                printf("[DEBUG] SAI_B: SR=0x%lX CR1=0x%08lX\r\n",
                       SAI2_Block_B->SR, SAI2_Block_B->CR1);
                printf("[DEBUG] DMA RX: CR=0x%lX NDTR=%lu\r\n",
                       DMA1_Stream0->CR, DMA1_Stream0->NDTR);
                printf("[DEBUG] DMA TX: CR=0x%lX NDTR=%lu\r\n",
                       DMA1_Stream1->CR, DMA1_Stream1->NDTR);
                extern volatile uint32_t audio_process_count;
                /* Print samples from start, middle and near DMA position */
                uint32_t dma_pos = 1024 - DMA1_Stream0->NDTR;

                /* Read SAI DR directly (may cause FIFO issues, just for debug) */
                uint32_t sai_dr = SAI2_Block_A->DR;

                printf("[DEBUG] RX[0..3]: %d %d %d %d\r\n",
                       audio_rx_buffer[0], audio_rx_buffer[1],
                       audio_rx_buffer[2], audio_rx_buffer[3]);
                printf("[DEBUG] TX[0..3]: %d %d %d %d\r\n",
                       audio_tx_buffer[0], audio_tx_buffer[1],
                       audio_tx_buffer[2], audio_tx_buffer[3]);
                printf("[DEBUG] Process=%lu, DMA_TX_NDTR=%lu\r\n",
                       audio_process_count, (uint32_t)DMA1_Stream1->NDTR);
            }
            last_buffer_count = audio_buffer_count;
            last_status_time = now;
        }

        /* Process ESP32 UART data - accumulate into line buffer */
        while (rx_head != rx_tail)
        {
            uint8_t data = rx_buffer[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

            if (data == '\n')
            {
                /* End of line - process command */
                line_buffer[line_pos] = '\0';
                if (line_pos > 0 && line_buffer[line_pos - 1] == '\r')
                {
                    line_buffer[line_pos - 1] = '\0';  /* Remove \r */
                }
                if (line_pos > 0)
                {
                    Process_GATT_Command(line_buffer);
                }
                line_pos = 0;
            }
            else if (line_pos < LINE_BUFFER_SIZE - 1)
            {
                line_buffer[line_pos++] = (char)data;
            }
        }

        /* Small delay to prevent tight loop when no audio */
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
 * @brief DMA Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn - SAI2_A RX */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    /* DMA1_Stream1_IRQn - SAI2_B TX */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/**
 * @brief SAI2 Initialization Function (I2S Audio Input/Output)
 *
 * SAI2_A: Slave Receiver - receives I2S from BM83
 *   - Pins: PD13 (SCK), PD12 (FS), PD11 (SD)
 *   - Mode: Slave, receives clock from BM83 (external master)
 *
 * SAI2_B: Slave Transmitter - outputs I2S to DAC
 *   - Pin: PA0 (SD)
 *   - Mode: Synchronous slave to SAI2_A (shares clock)
 *
 * @param None
 * @retval None
 */
static void MX_SAI2_Init(void)
{
    /* -------- SAI2 Block A - Slave Receiver (I2S Input) -------- */
    /* Using FREE protocol for full control over I2S timing */
    hsai_rx.Instance = SAI2_Block_A;
    hsai_rx.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai_rx.Init.AudioMode = SAI_MODESLAVE_RX;
    hsai_rx.Init.DataSize = SAI_DATASIZE_16;
    hsai_rx.Init.FirstBit = SAI_FIRSTBIT_MSB;
    hsai_rx.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
    hsai_rx.Init.Synchro = SAI_ASYNCHRONOUS;
    hsai_rx.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    hsai_rx.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
    hsai_rx.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
    hsai_rx.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
    hsai_rx.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_rx.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai_rx.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_rx.Init.TriState = SAI_OUTPUT_NOTRELEASED;

    /* Frame: 32 bits total (16 left + 16 right), I2S Philips style */
    hsai_rx.FrameInit.FrameLength = 32;
    hsai_rx.FrameInit.ActiveFrameLength = 16;
    hsai_rx.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
    hsai_rx.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
    hsai_rx.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

    /* Slots: 2 x 16-bit */
    hsai_rx.SlotInit.FirstBitOffset = 0;
    hsai_rx.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
    hsai_rx.SlotInit.SlotNumber = 2;
    hsai_rx.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

    if (HAL_SAI_Init(&hsai_rx) != HAL_OK)
    {
        Error_Handler();
    }

    /* Disable ALL error interrupts - in slave mode we trust external sync */
    __HAL_SAI_DISABLE_IT(&hsai_rx, SAI_IT_OVRUDR | SAI_IT_AFSDET | SAI_IT_LFSDET | SAI_IT_WCKCFG);
    __HAL_SAI_DISABLE_IT(&hsai_tx, SAI_IT_OVRUDR | SAI_IT_AFSDET | SAI_IT_LFSDET | SAI_IT_WCKCFG);

    /* -------- SAI2 Block B - Slave Transmitter (I2S Output) -------- */
    /* Synchronized to Block A */
    hsai_tx.Instance = SAI2_Block_B;
    hsai_tx.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai_tx.Init.AudioMode = SAI_MODESLAVE_TX;
    hsai_tx.Init.DataSize = SAI_DATASIZE_16;
    hsai_tx.Init.FirstBit = SAI_FIRSTBIT_MSB;
    hsai_tx.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
    hsai_tx.Init.Synchro = SAI_SYNCHRONOUS;
    hsai_tx.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
    hsai_tx.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
    hsai_tx.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
    hsai_tx.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
    hsai_tx.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_tx.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai_tx.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_tx.Init.TriState = SAI_OUTPUT_NOTRELEASED;

    /* Frame: same as RX (Philips) */
    hsai_tx.FrameInit.FrameLength = 32;
    hsai_tx.FrameInit.ActiveFrameLength = 16;
    hsai_tx.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
    hsai_tx.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
    hsai_tx.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

    /* Slots: same as RX */
    hsai_tx.SlotInit.FirstBitOffset = 0;
    hsai_tx.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
    hsai_tx.SlotInit.SlotNumber = 2;
    hsai_tx.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

    if (HAL_SAI_Init(&hsai_tx) != HAL_OK)
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
    /* GPIO clocks are enabled in HAL_xxx_MspInit() callbacks */
}

/**
 * @brief  Audio processing function - called for each buffer
 *         Currently just passes through (echo) for testing
 * @param  rx_buf: Pointer to received audio samples (16-bit)
 * @param  tx_buf: Pointer to transmit audio samples (16-bit)
 * @param  samples: Number of samples (L+R interleaved)
 * @retval None
 */
volatile uint32_t audio_process_count = 0;

/* Simple sine wave lookup table (256 entries, one full cycle) */
static const int16_t sine_table[256] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179,
    7962, 8739, 9512, 10278, 11039, 11793, 12539, 13279, 14010, 14732,
    15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403,
    22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571,
    30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521,
    32609, 32678, 32728, 32757, 32767, 32757, 32728, 32678, 32609, 32521,
    32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
    30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790,
    26319, 25832, 25329, 24811, 24279, 23731, 23170, 22594, 22005, 21403,
    20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151, 15446, 14732,
    14010, 13279, 12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179,
    6393, 5602, 4808, 4011, 3212, 2410, 1608, 804, 0, -804,
    -1608, -2410, -3212, -4011, -4808, -5602, -6393, -7179, -7962, -8739,
    -9512, -10278, -11039, -11793, -12539, -13279, -14010, -14732, -15446, -16151,
    -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683,
    -28105, -28510, -28898, -29268, -29621, -29956, -30273, -30571, -30852, -31113,
    -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32521, -32609, -32678,
    -32728, -32757, -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
    -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571, -30273, -29956,
    -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832,
    -25329, -24811, -24279, -23731, -23170, -22594, -22005, -21403, -20787, -20159,
    -19519, -18868, -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278, -9512, -8739, -7962, -7179, -6393, -5602,
    -4808, -4011, -3212, -2410, -1608, -804
};
static uint32_t sine_phase = 0;

/**
 * @brief  Parse hex string to bytes
 * @param  hex: Input hex string (e.g., "0164")
 * @param  out: Output byte array
 * @param  max_len: Maximum bytes to parse
 * @retval Number of bytes parsed
 */
static int hex_to_bytes(const char *hex, uint8_t *out, int max_len)
{
    int len = 0;
    while (*hex && *(hex + 1) && len < max_len)
    {
        char byte_str[3] = {hex[0], hex[1], '\0'};
        out[len++] = (uint8_t)strtol(byte_str, NULL, 16);
        hex += 2;
    }
    return len;
}

/**
 * @brief  Process GATT command received from ESP32
 *         Format: GATT:<characteristic>:<hex_data>
 *         Example: GATT:CTRL:0102 (Set preset to NIGHT)
 * @param  line: Complete line without \r\n
 */
static void Process_GATT_Command(const char *line)
{
    /* Check if it's a GATT command */
    if (strncmp(line, "GATT:", 5) != 0)
    {
        printf("[ESP32] %s\r\n", line);
        return;
    }

    /* Parse: GATT:<char>:<hex> */
    const char *char_start = line + 5;
    const char *colon = strchr(char_start, ':');
    if (!colon)
    {
        printf("[ESP32] Invalid GATT format: %s\r\n", line);
        return;
    }

    /* Extract characteristic name */
    int char_len = colon - char_start;
    char char_name[32];
    if (char_len >= (int)sizeof(char_name)) char_len = sizeof(char_name) - 1;
    strncpy(char_name, char_start, char_len);
    char_name[char_len] = '\0';

    /* Extract hex data */
    const char *hex_data = colon + 1;
    uint8_t cmd_bytes[16];
    int cmd_len = hex_to_bytes(hex_data, cmd_bytes, sizeof(cmd_bytes));

    /* Log the command */
    printf("[GATT] %s:", char_name);
    for (int i = 0; i < cmd_len; i++)
    {
        printf(" %02X", cmd_bytes[i]);
    }
    printf("\r\n");

    /* Handle specific commands */
    if (strcmp(char_name, "CTRL") == 0 && cmd_len >= 2)
    {
        uint8_t cmd_type = cmd_bytes[0];
        uint8_t cmd_value = cmd_bytes[1];

        switch (cmd_type)
        {
            case 0x01:  /* Set Preset */
                printf("[DSP] Set preset: %d\r\n", cmd_value);
                /* TODO: Apply preset to DSP */
                break;
            case 0x02:  /* Set Loudness */
                printf("[DSP] Loudness: %s\r\n", cmd_value ? "ON" : "OFF");
                /* TODO: Apply loudness */
                break;
            case 0x04:  /* Set Mute */
                printf("[DSP] Mute: %s\r\n", cmd_value ? "ON" : "OFF");
                /* TODO: Apply mute */
                break;
            case 0x07:  /* Set Volume */
                printf("[DSP] Volume: %d%%\r\n", cmd_value);
                /* TODO: Apply volume */
                break;
            case 0x08:  /* Set Bypass */
                printf("[DSP] Bypass: %s\r\n", cmd_value ? "ON" : "OFF");
                /* TODO: Apply bypass */
                break;
            case 0x09:  /* Set Bass Boost */
                printf("[DSP] Bass Boost: %s\r\n", cmd_value ? "ON" : "OFF");
                /* TODO: Apply bass boost */
                break;
            default:
                printf("[DSP] Unknown command: 0x%02X\r\n", cmd_type);
                break;
        }
    }
}

static void Audio_Process(int16_t *rx_buf, int16_t *tx_buf, uint16_t samples)
{
    audio_process_count++;

#if 1  /* Set to 1 for passthrough, 0 for test tone */
    /* Simple passthrough - copy RX to TX */
    memcpy(tx_buf, rx_buf, samples * sizeof(int16_t));
#else
    /* Generate 1kHz sine wave for testing TX path */
    /* At 44100Hz, 1kHz = 44.1 samples per cycle, use phase increment */
    for (uint16_t i = 0; i < samples; i += 2)
    {
        int16_t sample = sine_table[(sine_phase >> 8) & 0xFF];
        tx_buf[i] = sample;      /* Left */
        tx_buf[i+1] = sample;    /* Right */
        sine_phase += 1483;  /* ~1kHz at 44.1kHz: 256*65536/44100 ≈ 380, for 1kHz: 380*~4 */
    }
#endif
}

/**
 * @brief  SAI Rx Half Complete callback - copy to TX for passthrough
 * @param  hsai: SAI handle
 * @retval None
 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai->Instance == SAI2_Block_A)
    {
        Audio_Process(audio_rx_buffer, audio_tx_buffer, AUDIO_BUFFER_SIZE);
        audio_buffer_count++;
    }
}

/**
 * @brief  SAI Rx Complete callback - copy to TX for passthrough
 * @param  hsai: SAI handle
 * @retval None
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai->Instance == SAI2_Block_A)
    {
        Audio_Process(&audio_rx_buffer[AUDIO_BUFFER_SIZE],
                     &audio_tx_buffer[AUDIO_BUFFER_SIZE], AUDIO_BUFFER_SIZE);
        audio_buffer_count++;
    }
}

/**
 * @brief  SAI Tx Half Complete callback - RX callback handles passthrough
 * @param  hsai: SAI handle
 * @retval None
 */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    (void)hsai;
}

/**
 * @brief  SAI Tx Complete callback - RX callback handles passthrough
 * @param  hsai: SAI handle
 * @retval None
 */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    (void)hsai;
}

/**
 * @brief  SAI Error callback
 * @param  hsai: SAI handle
 * @retval None
 */
volatile uint32_t sai_error_count_a = 0;
volatile uint32_t sai_error_count_b = 0;
volatile uint32_t sai_last_error_a = 0;
volatile uint32_t sai_last_error_b = 0;

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai->Instance == SAI2_Block_A)
    {
        sai_error_count_a++;
        sai_last_error_a = hsai->ErrorCode;
        /* Just clear flags, don't restart DMA from interrupt */
        __HAL_SAI_CLEAR_FLAG(hsai, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET);
    }
    else if (hsai->Instance == SAI2_Block_B)
    {
        sai_error_count_b++;
        sai_last_error_b = hsai->ErrorCode;
        __HAL_SAI_CLEAR_FLAG(hsai, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET);
    }
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
