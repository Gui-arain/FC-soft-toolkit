/****************************************************************************
// Include file for the SHIRLEY FC DEV BOARD
// See config/pinout.yaml for the pinout of the board
// See Doc/STM32 Settings.md for the clock config
****************************************************************************/

#ifndef __BOARDS_ARM_STM32H7_SHIRLEY_FC_DEV_BOARD_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H7_SHIRLEY_FC_DEV_BOARD_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Shirley-FC-Dev-Board provides the following clock sources:
 *
 *   X: 16MHz external crystal oscillator
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 64 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 16 MHz from X
 *   CSI: 4 MHz RC
 */

#define STM32_BOARD_XTAL        16000000ul /* Crystal Oscillator*/

#define STM32_HSI_FREQUENCY     64000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
//#define STM32_LSE_FREQUENCY     32768 no external crystal for the low speed clock

/* Main PLL Configuration.
 *
 * PLL source is HSE = 16,000,000
 *
 *
 * When STM32_HSE_FREQUENCY / PLLM <= 2MHz VCOL must be selected.
 * VCOH otherwise.
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 63
 *     4 <= PLLN <= 512
 *   150 MHz <= PLL_VCOL <= 420MHz
 *   192 MHz <= PLL_VCOH <= 836MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * CPUCLK  = SYSCLK / D1CPRE
 * Subject to
 *
 *   PLLP1   = {2, 4, 6, 8, ..., 128}
 *   PLLP2,3 = {2, 3, 4, ..., 128}
 *   CPUCLK <= 400 MHz
 */

#define STM32_BOARD_USEHSE
#define STM32_HSEBYP_ENABLE

#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1, wide 2 - 4 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (16,000,000 / 8) * 80 = 160 MHz 
 *  
 *  Outputs of PPL1:
 *   PLL1P = PLL1_VCO/2  = 160 MHz / 2   = 80 MHz
 *   PLL1Q = PLL1_VCO/4  = 160 MHz / 2   = 80 MHz
 *   PLL1R = PLL1_VCO/8  = 160 MHz / 2   = 80 MHz
 */

#define STM32_PLLCFG_PLL1CFG     (RCC_PLLCFGR_PLL1VCOSEL_MEDIUM | \
                                  RCC_PLLCFGR_PLL1RGE_2_4_MHZ   | \
                                  RCC_PLLCFGR_DIVP1EN           | \
                                  RCC_PLLCFGR_DIVR1EN)
                                  /* DIVQ1EN omitted — PLL1Q unused */
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(8)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(80)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(2)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(2)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 8) * 80)
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)

/* PLL2, 16 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL2_VCO = (16,000,000 / 8) * 80 = 160 MHz
 *
 *  Outputs of PLL2:
 *   PLL2P = PLL2_VCO/2  = 160 MHz / 2  = 80 MHz
 *   PLL2Q = PLL2_VCO/40 = 160 MHz / 40 =  4 MHz  ← used for SPI4,5
 *   PLL2R = PLL2_VCO/2  = 160 MHz / 2  = 80 MHz
 */

#define STM32_PLLCFG_PLL2CFG     (RCC_PLLCFGR_PLL2VCOSEL_MEDIUM | \
                                  RCC_PLLCFGR_PLL2RGE_2_4_MHZ   | \
                                  RCC_PLLCFGR_DIVQ2EN)
                                  /* DIVP2EN, DIVR2EN omitted — unused */
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(8)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(80)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(40)
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(2)

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 8) * 80)
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 40)
#define STM32_PLL2R_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)

/* PLL3 */

/* PLL3, 16 MHz input, enable DIVQ, DIVR
 *
 *   PLL3_VCO = (16,000,000 / 8) * 80 = 160 MHz
 *
 *  Outputs of PLL3:
 *   PLL3P = PLL3_VCO/2  = 160 MHz / 2  = 80 MHz
 *   PLL3Q = PLL3_VCO/40 = 160 MHz / 2 =  80 MHz
 *   PLL3R = PLL3_VCO/2  = 160 MHz / 40  = 4 MHz  ← used for I2C1,2,3
 */

#define STM32_PLLCFG_PLL3CFG     (RCC_PLLCFGR_PLL3VCOSEL_MEDIUM | \
                                  RCC_PLLCFGR_PLL3RGE_2_4_MHZ   | \
                                  RCC_PLLCFGR_DIVQ3EN            | \
                                  RCC_PLLCFGR_DIVR3EN)
                                  /* DIVP3EN omitted — unused */
#define STM32_PLLCFG_PLL3M       RCC_PLLCKSELR_DIVM3(8)
#define STM32_PLLCFG_PLL3N       RCC_PLL3DIVR_N3(80)
#define STM32_PLLCFG_PLL3P       RCC_PLL3DIVR_P3(2)
#define STM32_PLLCFG_PLL3Q       RCC_PLL3DIVR_Q3(2)
#define STM32_PLLCFG_PLL3R       RCC_PLL3DIVR_R3(40)

#define STM32_VCO3_FREQUENCY     ((STM32_HSE_FREQUENCY / 8) * 80)
#define STM32_PLL3P_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL3Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL3R_FREQUENCY    (STM32_VCO2_FREQUENCY / 40)

/* SYSCLK = PLL1P = 80 MHz
 * CPUCLK = SYSCLK / 1 = 80 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1)

/* Configure Clock Assignments */

/* SYSCLK = PLL1P = 80 MHz
 * HPRE = /1  → HCLK = 80 MHz
 * All APB prescalers = /2 → PCLKx = 40 MHz
 */

#define STM32_RCC_D1CFGR_HPRE    RCC_D1CFGR_HPRE_SYSCLK         /* HCLK  = SYSCLK / 1 = 80 MHz */
#define STM32_ACLK_FREQUENCY     STM32_SYSCLK_FREQUENCY          /* ACLK  = 80 MHz */
#define STM32_HCLK_FREQUENCY     STM32_SYSCLK_FREQUENCY          /* HCLK  = 80 MHz */

/* APB1 (PCLK1) = HCLK/2 = 40 MHz */
#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY / 2)

/* APB2 (PCLK2) = HCLK/2 = 40 MHz */
#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd2
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY / 2)

/* APB3 (PCLK3) = HCLK/2 = 40 MHz */
#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd2
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY / 2)

/* APB4 (PCLK4) = HCLK/2 = 40 MHz */
#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd2
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY / 2)

/* Timer clock frequencies (×2 when APB prescaler != 1) */

/* Timers on APB1 */
#define STM32_APB1_TIM2_CLKIN   (2 * STM32_PCLK1_FREQUENCY)   /* 80 MHz */
#define STM32_APB1_TIM3_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2 * STM32_PCLK1_FREQUENCY)   /* RGB LED PWM */
#define STM32_APB1_TIM5_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2 * STM32_PCLK1_FREQUENCY)

/* Timers on APB2 */
#define STM32_APB2_TIM1_CLKIN   (2 * STM32_PCLK2_FREQUENCY)   /* Motor ESC PWM/DShot */
#define STM32_APB2_TIM8_CLKIN   (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (2 * STM32_PCLK2_FREQUENCY)

/* Kernel Clock Configuration
 *
 * Note: look at Table 54 in ST Manual
 */

/* I2C1,2,3 clock source - PLL3R (4 MHz) */
#define STM32_RCC_D2CCIP2R_I2C123SRC  RCC_D2CCIP2R_I2C123SEL_PLL3

/* I2C4 clock source - PLL3R (4 MHz) */
#define STM32_RCC_D3CCIPR_I2C4SRC     RCC_D3CCIPR_I2C4SEL_PLL3

/* SPI4,5 clock source - PLL2Q (80 MHz) */
#define STM32_RCC_D2CCIP1R_SPI45SRC   RCC_D2CCIP1R_SPI45SEL_PLL2

/* USART2,3 clock source - PCLK1 (40 MHz, D2 domain) */
#define STM32_RCC_D2CCIP2R_USART234578SRC  RCC_D2CCIP2R_USART234578SEL_PCLK

/* USART6 clock source - PCLK2 (40 MHz, D2 domain) */
#define STM32_RCC_D2CCIP2R_USART16910SRC   RCC_D2CCIP2R_USART16910SEL_PCLK

/* USB OTG FS clock source - HSI48 */
#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_HSI48

/* ADC1,2 clock source - PLL2P (80 MHz) */
#define STM32_RCC_D3CCIPR_ADCSRC     RCC_D3CCIPR_ADCSEL_PLL2

/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *  1.05-1.15 V     55 MHz    0
 *  (VOS2 level)   110 MHz    1
 *                 165 MHz    2
 *                 220 MHz    3
 *  0.95-1.05 V     45 MHz    0
 *  (VOS3 level)    90 MHz    1
 *                 135 MHz    2
 *                 180 MHz    3
 *                 225 MHz    4
 *  ------------ ---------- -----------
 * 
 * Vcore VOS1, ACLK=80 MHz → 1 wait state
 */

#define BOARD_FLASH_WAITSTATES 1

/* SDMMC1 clock source - PLL1Q
 * Note: PLL1Q must be re-enabled if SDMMC is used.
 * Init  400 kHz : PLL1Q / (2 * 100) = 80MHz / 200 = 400 kHz
 * Xfer  25  MHz : PLL1Q / (2 * 2)   = 80MHz / 4   = 20 MHz
 */
#define STM32_RCC_D1CCIPR_SDMMCSRC   RCC_D1CCIPR_SDMMCSEL_PLL1
#define STM32_SDMMC_INIT_CLKDIV      (100 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_MMCXFR_CLKDIV   (2   << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_SDXFR_CLKDIV    (2   << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_CLKCR_EDGE       STM32_SDMMC_CLKCR_NEGEDGE

/* LED definitions **********************************************************/

/* Single RGB LED driven by TIM4 CH1/2/3 (PD12/13/14)
 * plus system status LED on PD6 (GPIO output)
 */
#define BOARD_LED1          0   /* System status LED, PD6 */
#define BOARD_NLEDS         1

/* LED bits for use with board_userled_all() */


#define BOARD_LED1_BIT      (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *
 *   SYMBOL                     Meaning                LED state
 *                                                        Red   
 *   ----------------------  --------------------------  ------ 
 */

#define LED_STARTED        0 /* NuttX has been started   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       ON   */
#define LED_STACKCREATED   3 /* Idle stack created       ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C. */
#define LED_SIGNAL         5 /* In a signal handler      N/C  */
#define LED_ASSERTION      6 /* An assertion failed      N/C. */
#define LED_PANIC          7 /* The system has crashed   Blink*/
#define LED_IDLE           8 /* MCU is in sleep mode     OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button / GPIO definitions ************************************************/

#define BUTTON_USER         0   /* External switch PE3 */
#define NUM_BUTTONS         1
#define BUTTON_USER_BIT     (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* UART4 - Telemetry (AF8)
 * TX=PA0, RX=PA1, CTS=PB0, RTS=PB14
 */
#define GPIO_UART4_TX     (GPIO_UART4_TX_1   | GPIO_SPEED_100MHz)  /* PA0, AF8 */
#define GPIO_UART4_RX     (GPIO_UART4_RX_1   | GPIO_SPEED_100MHz)  /* PA1, AF8 */
#define GPIO_UART4_CTS    (GPIO_UART4_CTS_1  | GPIO_SPEED_100MHz)  /* PB0, AF8 */
#define GPIO_UART4_RTS    (GPIO_UART4_RTS_1  | GPIO_SPEED_100MHz)  /* PB14, AF8 */

/* UART5 - Mini Pad Out UART (AF14)
 * TX=PB13, RX=PB12
 */
#define GPIO_UART5_TX     (GPIO_UART5_TX_2   | GPIO_SPEED_100MHz)  /* PB13, AF14 */
#define GPIO_UART5_RX     (GPIO_UART5_RX_2   | GPIO_SPEED_100MHz)  /* PB12, AF14 */

/* UART8 - GPS (AF8)
 * TX=PE1, RX=PE0
 */
#define GPIO_UART8_TX     (GPIO_UART8_TX_1   | GPIO_SPEED_100MHz)  /* PE1, AF8 */
#define GPIO_UART8_RX     (GPIO_UART8_RX_1   | GPIO_SPEED_100MHz)  /* PE0, AF8 */

/* USART2 - Pad Out USART (AF7)
 * TX=PA2, RX=PA3
 */
#define GPIO_USART2_TX    (GPIO_USART2_TX_1  | GPIO_SPEED_100MHz)  /* PA2, AF7 */
#define GPIO_USART2_RX    (GPIO_USART2_RX_1  | GPIO_SPEED_100MHz)  /* PA3, AF7 */

/* USART6 - RC Receiver (AF7)
 * TX=PC6, RX=PC7
 */
#define GPIO_USART6_TX    (GPIO_USART6_TX_1  | GPIO_SPEED_100MHz)  /* PC6, AF7 */
#define GPIO_USART6_RX    (GPIO_USART6_RX_1  | GPIO_SPEED_100MHz)  /* PC7, AF7 */

/* SPI4 - Magnetometer (AF5)
 * SCK=PE2, MISO=PE5, MOSI=PE6
 * NSS handled by software (GPIO CS)
 */
#define GPIO_SPI4_SCK     (GPIO_SPI4_SCK_1   | GPIO_SPEED_50MHz)   /* PE2, AF5 */
#define GPIO_SPI4_MISO    (GPIO_SPI4_MISO_1  | GPIO_SPEED_50MHz)   /* PE5, AF5 */
#define GPIO_SPI4_MOSI    (GPIO_SPI4_MOSI_1  | GPIO_SPEED_50MHz)   /* PE6, AF5 */

/* SPI5 - IMU (AF5)
 * SCK=PF7, MISO=PF8, MOSI=PF9
 * NSS handled by software (GPIO CS)
 */
#define GPIO_SPI5_SCK     (GPIO_SPI5_SCK_1   | GPIO_SPEED_50MHz)   /* PF7, AF5 */
#define GPIO_SPI5_MISO    (GPIO_SPI5_MISO_1  | GPIO_SPEED_50MHz)   /* PF8, AF5 */
#define GPIO_SPI5_MOSI    (GPIO_SPI5_MOSI_1  | GPIO_SPEED_50MHz)   /* PF9, AF5 */

/* I2C1 - External Compass Connector (AF4)
 * SCL=PB6, SDA=PB7
 */
#define GPIO_I2C1_SCL     (GPIO_I2C1_SCL_1   | GPIO_SPEED_50MHz)   /* PB6, AF4 */
#define GPIO_I2C1_SDA     (GPIO_I2C1_SDA_1   | GPIO_SPEED_50MHz)   /* PB7, AF4 */

/* I2C2 - Barometer (AF4)
 * SCL=PF1, SDA=PF0
 */
#define GPIO_I2C2_SCL     (GPIO_I2C2_SCL_2   | GPIO_SPEED_50MHz)   /* PF1, AF4 */
#define GPIO_I2C2_SDA     (GPIO_I2C2_SDA_2   | GPIO_SPEED_50MHz)   /* PF0, AF4 */

/* I2C4 - Pad Out I2C (AF4)
 * SCL=PF14, SDA=PF15
 */
#define GPIO_I2C4_SCL     (GPIO_I2C4_SCL_1   | GPIO_SPEED_50MHz)   /* PF14, AF4 */
#define GPIO_I2C4_SDA     (GPIO_I2C4_SDA_1   | GPIO_SPEED_50MHz)   /* PF15, AF4 */

/* TIM1 - Motor ESC PWM/DShot (AF1)
 * CH1=PE9, CH2=PE11, CH3=PE13, CH4=PE14
 */
#define GPIO_TIM1_CH1OUT  (GPIO_TIM1_CH1OUT_2  | GPIO_SPEED_100MHz) /* PE9,  AF1 */
#define GPIO_TIM1_CH2OUT  (GPIO_TIM1_CH2OUT_2  | GPIO_SPEED_100MHz) /* PE11, AF1 */
#define GPIO_TIM1_CH3OUT  (GPIO_TIM1_CH3OUT_2  | GPIO_SPEED_100MHz) /* PE13, AF1 */
#define GPIO_TIM1_CH4OUT  (GPIO_TIM1_CH4OUT_2  | GPIO_SPEED_100MHz) /* PE14, AF1 */

/* TIM4 - RGB LED PWM (AF2)
 * CH1=PD12 (Red), CH2=PD13 (Green), CH3=PD14 (Blue)
 */
#define GPIO_TIM4_CH1OUT  (GPIO_TIM4_CH1OUT_2  | GPIO_SPEED_50MHz)  /* PD12, AF2 */
#define GPIO_TIM4_CH2OUT  (GPIO_TIM4_CH2OUT_2  | GPIO_SPEED_50MHz)  /* PD13, AF2 */
#define GPIO_TIM4_CH3OUT  (GPIO_TIM4_CH3OUT_2  | GPIO_SPEED_50MHz)  /* PD14, AF2 */

/* ADC1 - Battery Voltage/Current (no AF, analog mode)
 * INP2=PF11 (Voltage), INP3=PA6 (Current)
 */
#define GPIO_ADC1_INP2    GPIO_ADC1_INP2_0                          /* PF11 */
#define GPIO_ADC12_INP3   GPIO_ADC12_INP3_0                         /* PA6  */

/* ADC2 - Pad Out ADCs
 * INP2=PF13, INP4=PC4, INP5=PB1
 */
#define GPIO_ADC2_INP2    GPIO_ADC2_INP2_0                          /* PF13 */
#define GPIO_ADC12_INP4   GPIO_ADC12_INP4_0                         /* PC4  */
#define GPIO_ADC12_INP5   GPIO_ADC12_INP5_0                         /* PB1  */

/* USB OTG FS (AF10)
 * DM=PA11, DP=PA12, VBUS sense=PA9
 */
#define GPIO_OTGFS_DM     (GPIO_OTGFS_DM_0    | GPIO_SPEED_100MHz)  /* PA11, AF10 */
#define GPIO_OTGFS_DP     (GPIO_OTGFS_DP_0    | GPIO_SPEED_100MHz)  /* PA12, AF10 */
#define GPIO_OTGFS_ID     (GPIO_OTGFS_ID_0    | GPIO_SPEED_100MHz)  /* PA10, AF10 */

/* SDMMC1 - SD Card (AF12)
 * D0=PC8, D1=PC9, D2=PC10, D3=PC11, CK=PC12, CMD=PD2
 * CDS (card detect) = PD3, GPIO input
 */
#define GPIO_SDMMC1_D0    (GPIO_SDMMC1_D0_0   | GPIO_SPEED_100MHz)  /* PC8,  AF12 */
#define GPIO_SDMMC1_D1    (GPIO_SDMMC1_D1_0   | GPIO_SPEED_100MHz)  /* PC9,  AF12 */
#define GPIO_SDMMC1_D2    (GPIO_SDMMC1_D2_0   | GPIO_SPEED_100MHz)  /* PC10, AF12 */
#define GPIO_SDMMC1_D3    (GPIO_SDMMC1_D3_0   | GPIO_SPEED_100MHz)  /* PC11, AF12 */
#define GPIO_SDMMC1_CK    (GPIO_SDMMC1_CK_0   | GPIO_SPEED_100MHz)  /* PC12, AF12 */
#define GPIO_SDMMC1_CMD   (GPIO_SDMMC1_CMD_0  | GPIO_SPEED_100MHz)  /* PD2,  AF12 */

/* DMA *********************************************************************/

#define DMAMAP_UART4_RX    DMAMAP_DMA12_UART4RX_0
#define DMAMAP_UART4_TX    DMAMAP_DMA12_UART4TX_0
#define DMAMAP_UART5_RX    DMAMAP_DMA12_UART5RX_0
#define DMAMAP_UART5_TX    DMAMAP_DMA12_UART5TX_0
#define DMAMAP_UART8_RX    DMAMAP_DMA12_UART8RX_0
#define DMAMAP_UART8_TX    DMAMAP_DMA12_UART8TX_0
#define DMAMAP_USART2_RX   DMAMAP_DMA12_USART2RX_0
#define DMAMAP_USART2_TX   DMAMAP_DMA12_USART2TX_0
#define DMAMAP_USART6_RX   DMAMAP_DMA12_USART6RX_0
#define DMAMAP_USART6_TX   DMAMAP_DMA12_USART6TX_0
#define DMAMAP_SPI4_RX     DMAMAP_DMA12_SPI4RX_0
#define DMAMAP_SPI4_TX     DMAMAP_DMA12_SPI4TX_0
#define DMAMAP_SPI5_RX     DMAMAP_DMA12_SPI5RX_0
#define DMAMAP_SPI5_TX     DMAMAP_DMA12_SPI5TX_0
#define DMAMAP_SDMMC1      DMAMAP_DMA12_SDMMC1_0

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32H7_NUCLEO_H743ZI_INCLUDE_BOARD_H */
