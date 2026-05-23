# STM32 Configurations ⚙️

## Clock config ⏱️

RCC:
- HSE: Crystal Resonator

### Config for max frequency of 480Mhz
![clock_config_HS](imgs/Clock_config_HSE.png)

### Config for first bringup
![Clock_config_HSI.png](imgs/Clock_config_HSI.png)

### Config for Sensors tests

CPU at 80 MHz — SPI4/5 and I2C1/2/3 kernel clocks at 4 MHz.

**Clock source:** HSE — 16 MHz external crystal (no bypass)

**PLL source:** HSE, DIVM = 8 → 2 MHz reference input (all three PLLs)

| PLL | N | Output | Divisor | Frequency | Routed to |
|-----|---|--------|---------|-----------|-----------|
| PLL1 | 80 | P | /2 | **80 MHz** | SYSCLK |
| PLL1 | 80 | Q | /2 | 80 MHz | SDMMC (reserved) |
| PLL2 | 80 | Q | /40 | **4 MHz** | SPI4, SPI5 |
| PLL3 | 80 | Q | /2 | 80 MHz | — |
| PLL3 | 80 | R | /40 | **4 MHz** | I2C1, I2C2, I2C3 |

> All three PLLs share the same VCO: (16 MHz / 8) × 80 = **160 MHz**

**Bus clocks**

| Clock | Prescaler | Frequency |
|-------|-----------|-----------|
| SYSCLK | — | 80 MHz |
| CPUCLK / HCLK | /1 | 80 MHz |
| PCLK1–4 (APB1–4) | /2 | 40 MHz |

**Flash wait states:** 1 (VOS1, ACLK ≤ 140 MHz)

**NuttX register definitions**

```c
/* PLL1 — System clock */
#define STM32_PLLCFG_PLL1M  RCC_PLLCKSELR_DIVM1(8)
#define STM32_PLLCFG_PLL1N  RCC_PLL1DIVR_N1(80)
#define STM32_PLLCFG_PLL1P  RCC_PLL1DIVR_P1(2)   /* SYSCLK = 80 MHz */
#define STM32_PLLCFG_PLL1Q  RCC_PLL1DIVR_Q1(2)   /* 80 MHz, reserved for SDMMC */
#define STM32_PLLCFG_PLL1R  RCC_PLL1DIVR_R1(2)

/* PLL2 — SPI kernel clock */
#define STM32_PLLCFG_PLL2M  RCC_PLLCKSELR_DIVM2(8)
#define STM32_PLLCFG_PLL2N  RCC_PLL2DIVR_N2(80)
#define STM32_PLLCFG_PLL2P  RCC_PLL2DIVR_P2(2)
#define STM32_PLLCFG_PLL2Q  RCC_PLL2DIVR_Q2(40)  /* SPI4/5 = 4 MHz */
#define STM32_PLLCFG_PLL2R  RCC_PLL2DIVR_R2(2)

/* PLL3 — I2C kernel clock */
#define STM32_PLLCFG_PLL3M  RCC_PLLCKSELR_DIVM3(8)
#define STM32_PLLCFG_PLL3N  RCC_PLL3DIVR_N3(80)
#define STM32_PLLCFG_PLL3P  RCC_PLL3DIVR_P3(2)
#define STM32_PLLCFG_PLL3Q  RCC_PLL3DIVR_Q3(2)
#define STM32_PLLCFG_PLL3R  RCC_PLL3DIVR_R3(40)  /* I2C1/2/3 = 4 MHz */
```

## RGB Led config 💡

CubeMX Configuration — RGB LED (Common Anode) with TIM4

### Timer Setup

Navigate to **Timers → TIM4** in the `.ioc` file.

| Setting | Value |
|---|---|
| Clock Source | Internal Clock |
| Channel 1 | PWM Generation CH1 (Red) |
| Channel 2 | PWM Generation CH2 (Green) |
| Channel 3 | PWM Generation CH3 (Blue) |

### Parameter Settings

| Parameter | Value | Notes |
|---|---|---|
| Prescaler (PSC) | 79 | For 80 MHz clock → 1 MHz timer clock |
| Counter Mode | Up | |
| Counter Period (ARR) | 999 | Gives 1 kHz PWM frequency |
| PWM Mode (all channels) | **PWM Mode 2** | Inverts output for common anode |
| Pulse / CCR (all channels) | 0 | LED starts OFF with Mode 2 |

> **PWM frequency formula:** `PWM_freq = Timer_clock / ((PSC+1) × (ARR+1))`  
> Adjust PSC if your system clock differs from 80 MHz.

### Pin Assignment

Map TIM4_CH1, CH2, CH3 to the GPIO pins connected to your RGB LED in the **Pinout view**. Verify the alternate function (AF) mapping matches your PCB routing in the STM32 datasheet.

### Code — Start PWM

```c
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Red
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Green
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Blue
```

### Code — Set Color

With **PWM Mode 2**, the logic is intuitive: `0` = OFF, `999` = full brightness.

```c
void RGB_SetColor(uint16_t red, uint16_t green, uint16_t blue)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, red);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, green);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, blue);
}

// Examples
RGB_SetColor(999,   0,   0); // Red
RGB_SetColor(  0, 999,   0); // Green
RGB_SetColor(  0,   0, 999); // Blue
RGB_SetColor(999, 999, 999); // White
RGB_SetColor(  0,   0,   0); // Off
```

### Notes

- **Common Anode** → PWM Mode 2 is used to invert the signal (pin LOW = LED on).
- Ensure current-limiting resistors are fitted in series with each LED leg (typically 33–100 Ω).
- If TIM4 channels cannot be routed to your PCB pins, any timer with 3 channels (TIM2, TIM3…) can be used with the same configuration.