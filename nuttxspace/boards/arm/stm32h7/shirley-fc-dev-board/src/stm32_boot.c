/****************************************************************************
 * boards/arm/stm32h7/shirley-fc-dev-board/src/stm32_boot.c
 *
 * Board-level early initialization for the Slirley FC Dev Board(STM32H743, 16MHz HSE)
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "fc-dev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_board_initialize
 *
 * Description:
 *   All STM32H7 architectures must provide the following entry point.
 *   This is the earliest board-specific code to run — clocks and FPU only.
 *   Called before the OS scheduler starts.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
  /* The STM32H743 starts up on HSI (64 MHz internal oscillator).
   * stm32_clockconfig() switches to HSE + PLL1 as configured in board.h.
   * With 16 MHz HSE, DIVM1=8, DIVN1=80, DIVP1=2 → 80 MHz SYSCLK.
   */

  stm32_clockconfig();

  /* Enable the FPU. The H743 has a Cortex-M7 with double-precision FPU.
   * This must be done early — before any floating-point code runs,
   * including C++ constructors in fc-stack modules.
   */

  arm_fpuconfig();

  /* Configure GPIO clocks for the ports your board uses.
   * Add or remove ports to match your schematic.
   * At minimum enable the port(s) used by your debug UART.
   */

  stm32_gpiowrite(GPIO_PORTA, true);  /* USART2 TX/RX typically on PA2/PA3 */
  stm32_gpiowrite(GPIO_PORTB, true);
  stm32_gpiowrite(GPIO_PORTC, true);
  stm32_gpiowrite(GPIO_PORTD, true);
  stm32_gpiowrite(GPIO_PORTE, true);
  stm32_gpiowrite(GPIO_PORTF, true);

  /* Initialize the serial console UART early so boot messages
   * are visible from the very start.
   */

  arm_earlyserialinit();
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   Called after the OS scheduler starts, before the init task.
 *   Initializes buses, peripherals, and sensor drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  stm32_bringup();
}
#endif