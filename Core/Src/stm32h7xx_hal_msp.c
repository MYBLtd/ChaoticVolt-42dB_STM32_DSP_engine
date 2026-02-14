/**
 ******************************************************************************
 * @file         stm32h7xx_hal_msp.c
 * @brief        HAL MSP module.
 ******************************************************************************
 */

#include "stm32h7xx_hal.h"
#include "main.h"  /* For Error_Handler() */

/* External DMA handles */
extern DMA_HandleTypeDef hdma_sai_rx;
extern DMA_HandleTypeDef hdma_sai_tx;

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(huart->Instance == USART2)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* USART2 GPIO Configuration
         * PD5 -----> USART2_TX
         * PD6 -----> USART2_RX
         */
        GPIO_InitStruct.Pin = GPIO_PIN_5;  /* TX */
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_6;  /* RX with pull-up */
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USART2 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else if(huart->Instance == USART3)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* USART3 GPIO Configuration
         * PD8 -----> USART3_TX
         * PD9 -----> USART3_RX
         */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance == USART2)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        /* USART2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
    else if(huart->Instance == USART3)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /* USART3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
}

/**
 * @brief DAC MSP Initialization
 * @param hdac: DAC handle pointer
 * @retval None
 */
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hdac->Instance == DAC1)
    {
        __HAL_RCC_DAC12_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA4 -----> DAC1_OUT1 (IN-13 VU meter) */
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/**
 * @brief DAC MSP De-Initialization
 * @param hdac: DAC handle pointer
 * @retval None
 */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
    if(hdac->Instance == DAC1)
    {
        __HAL_RCC_DAC12_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
    }
}

/**
 * @brief SAI MSP Initialization
 * This function configures the hardware resources for SAI
 *
 * SAI2_A (Slave RX from BM83):
 *   PD13 -----> SAI2_SCK_A (Bit Clock Input)
 *   PD12 -----> SAI2_FS_A  (Frame Sync / LRCK Input)
 *   PD11 -----> SAI2_SD_A  (Serial Data Input)
 *
 * SAI2_B (Slave TX to DAC, synced to SAI2_A):
 *   PA0  -----> SAI2_SD_B  (Serial Data Output)
 *
 * @param hsai: SAI handle pointer
 * @retval None
 */
void HAL_SAI_MspInit(SAI_HandleTypeDef* hsai)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hsai->Instance == SAI2_Block_A)
    {
        /* Peripheral clock enable */
        __HAL_RCC_SAI2_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* SAI2_A GPIO Configuration (I2S Input)
         * PD13 -----> SAI2_SCK_A (Bit Clock)  - use 1kΩ series resistor!
         * PD12 -----> SAI2_FS_A  (Frame Sync / Word Select)
         * PD11 -----> SAI2_SD_A  (Serial Data)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* Configure DMA for SAI2_A RX - 16-bit data */
        hdma_sai_rx.Instance = DMA1_Stream0;
        hdma_sai_rx.Init.Request = DMA_REQUEST_SAI2_A;
        hdma_sai_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_sai_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_sai_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_sai_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  /* 16-bit */
        hdma_sai_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    /* 16-bit */
        hdma_sai_rx.Init.Mode = DMA_CIRCULAR;
        hdma_sai_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_sai_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  /* Direct mode - no waiting */
        hdma_sai_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
        hdma_sai_rx.Init.MemBurst = DMA_MBURST_SINGLE;
        hdma_sai_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

        if (HAL_DMA_Init(&hdma_sai_rx) != HAL_OK)
        {
            Error_Handler();
        }

        /* Link DMA handle to SAI handle */
        __HAL_LINKDMA(hsai, hdmarx, hdma_sai_rx);

        /* SAI2 interrupt - DISABLED to avoid error abort, using DMA only */
        /* HAL_NVIC_SetPriority(SAI2_IRQn, 1, 0); */
        /* HAL_NVIC_EnableIRQ(SAI2_IRQn); */
    }
    else if(hsai->Instance == SAI2_Block_B)
    {
        /* SAI2 clock already enabled by Block A */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* SAI2_B GPIO Configuration (I2S Output)
         * PA0 -----> SAI2_SD_B (Serial Data Output)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Configure DMA for SAI2_B TX - 16-bit data */
        hdma_sai_tx.Instance = DMA1_Stream1;
        hdma_sai_tx.Init.Request = DMA_REQUEST_SAI2_B;
        hdma_sai_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_sai_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_sai_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_sai_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  /* 16-bit */
        hdma_sai_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    /* 16-bit */
        hdma_sai_tx.Init.Mode = DMA_CIRCULAR;
        hdma_sai_tx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_sai_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  /* Direct mode */
        hdma_sai_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
        hdma_sai_tx.Init.MemBurst = DMA_MBURST_SINGLE;
        hdma_sai_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;

        if (HAL_DMA_Init(&hdma_sai_tx) != HAL_OK)
        {
            Error_Handler();
        }

        /* Link DMA handle to SAI handle */
        __HAL_LINKDMA(hsai, hdmatx, hdma_sai_tx);
    }
}

/**
 * @brief SAI MSP De-Initialization
 * @param hsai: SAI handle pointer
 * @retval None
 */
void HAL_SAI_MspDeInit(SAI_HandleTypeDef* hsai)
{
    if(hsai->Instance == SAI2_Block_A)
    {
        /* Peripheral clock disable */
        __HAL_RCC_SAI2_CLK_DISABLE();

        /* SAI2_A GPIO DeInit */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13);

        /* DMA DeInit */
        HAL_DMA_DeInit(hsai->hdmarx);

        /* SAI2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(SAI2_IRQn);
    }
    else if(hsai->Instance == SAI2_Block_B)
    {
        /* SAI2_B GPIO DeInit */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

        /* DMA DeInit */
        HAL_DMA_DeInit(hsai->hdmatx);
    }
}
