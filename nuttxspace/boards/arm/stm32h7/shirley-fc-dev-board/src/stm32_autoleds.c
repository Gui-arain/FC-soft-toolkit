/****************************************************************************
 * boards/arm/stm32h7/shirley-fc-dev-board/src/stm32_autoleds.c
 *
 * board_autoled_on/off implementation for the Shirley FC Dev Board.
 * Uses the system status LED on PD6 (active-high GPIO output).
 *
 * LED state mapping (see board.h LED_* constants):
 *   LED_STACKCREATED  → on  (system up)
 *   LED_PANIC         → on  (crash indicator)
 *   LED_STARTED       → off (pre-boot)
 *   LED_IDLE          → off (CPU sleeping)
 *   everything else   → no change
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include "stm32_gpio.h"

#ifdef CONFIG_ARCH_LEDS

#define GPIO_LED_SYSTEM (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN6)

void board_autoled_initialize(void)
{
  stm32_configgpio(GPIO_LED_SYSTEM);
}

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
      case LED_PANIC:
        stm32_gpiowrite(GPIO_LED_SYSTEM, true);
        break;
      default:
        break;
    }
}

void board_autoled_off(int led)
{
  switch (led)
    {
      case LED_STARTED:
      case LED_IDLE:
        stm32_gpiowrite(GPIO_LED_SYSTEM, false);
        break;
      default:
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
