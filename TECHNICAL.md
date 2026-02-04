# Technical Documentation

## Architecture

```
                    ┌─────────────────────────────────────────┐
                    │           STM32H753 DSP Engine          │
                    │                                         │
I2S Audio In ──────>│  I2S RX ──> DSP Processing ──> I2S TX  │──────> I2S Audio Out
                    │                   ▲                     │
                    │                   │                     │
                    │              Parameters                 │
                    │                   │                     │
ESP32 BLE ─────────>│  USART2 RX ──> Command Parser          │
                    │                                         │
                    │  Debug ──────────────────────> USART3  │──────> ST-LINK VCP
                    └─────────────────────────────────────────┘
```

## Peripheral Configuration

### USART2 (ESP32 BLE Control)

| Setting | Value | Notes |
|---------|-------|-------|
| Instance | USART2 | APB1 peripheral |
| TX Pin | PD5 (AF7) | CN9 pin 6 |
| RX Pin | PD6 (AF7) | CN9 pin 4, internal pull-up |
| Baud Rate | 115200 | |

### USART3 (Debug Console)

| Setting | Value | Notes |
|---------|-------|-------|
| Instance | USART3 | APB1 peripheral |
| TX Pin | PD8 (AF7) | ST-LINK VCP |
| RX Pin | PD9 (AF7) | ST-LINK VCP |
| Baud Rate | 115200 | |

### I2S (Audio) - TBD

| Setting | Value | Notes |
|---------|-------|-------|
| Instance | SPI/I2S2 or SAI | TBD |
| Sample Rate | 48000 Hz | TBD |
| Bit Depth | 24-bit | TBD |
| Channels | Stereo | TBD |

## Clock Configuration

- HSE: 8 MHz (bypass mode, from ST-LINK)
- PLL: 480 MHz system clock
- APB1: 120 MHz (USART2, USART3)
- I2S Clock: TBD (requires precise audio clock)

## HAL Initialization Order

GPIO pins for peripherals are configured in `HAL_xxx_MspInit()` callbacks:

```c
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    // 1. Enable peripheral clock
    __HAL_RCC_USARTx_CLK_ENABLE();
    __HAL_RCC_GPIOx_CLK_ENABLE();

    // 2. Configure GPIO with alternate function
    GPIO_InitStruct.Alternate = GPIO_AFx_USARTx;
    HAL_GPIO_Init(...);

    // 3. Enable NVIC interrupt
    HAL_NVIC_EnableIRQ(USARTx_IRQn);
}
```

## UART Reception

### Ring Buffer

```c
#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_head = 0;  // Write position (ISR)
volatile uint16_t rx_tail = 0;  // Read position (main loop)
```

### Error Recovery

The `HAL_UART_ErrorCallback()` handles errors and re-arms reception:

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Abort(huart);
    __HAL_UART_CLEAR_FLAG(huart, ...);
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}
```

## DSP Processing - TBD

### Audio Buffer Strategy

- Double buffering for DMA transfers
- Buffer size: TBD (balance latency vs. processing time)
- Processing in half-transfer and transfer-complete callbacks

### Planned DSP Features

- Volume control
- EQ (parametric/graphic)
- Dynamics (compressor/limiter)
- Delay/reverb
- Custom filters

## Memory Map

| Region | Address | Size | Usage |
|--------|---------|------|-------|
| Flash | 0x08000000 | 2 MB | Code |
| DTCM | 0x20000000 | 128 KB | Stack, fast variables |
| AXI SRAM | 0x24000000 | 512 KB | Audio buffers, heap |

## Troubleshooting

### UART Issues

1. **No data received:** Check wiring (ESP32 TX → STM32 PD6), verify GND connected
2. **Receives one byte then stops:** Missing error callback
3. **GPIO not working:** Ensure config is in `HAL_UART_MspInit()`

### I2S Issues - TBD

## References

- [STM32H753 Reference Manual (RM0433)](https://www.st.com/resource/en/reference_manual/rm0433)
- [Nucleo-H753ZI User Manual (UM2407)](https://www.st.com/resource/en/user_manual/um2407)
- [STM32H7 HAL Driver Documentation](https://www.st.com/resource/en/user_manual/um2217)
