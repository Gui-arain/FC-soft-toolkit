# DAKEFPV_H743 `estimator` hang — debug session

Board: DAKEFPV_H743 (dual ICM-42688-P), NuttX, `usbnsh` config (USB CDC/ACM
console, no UART converter available). Symptom: running `estimator` from
NSH reliably hangs partway through IMU init, with the console output cut
off mid-line and no recovery.

This log covers three confirmed, fixed bugs and one still-open issue.

## Bug 1 (fixed): EXTI4 pin-mux collision with leftover SD-card port code

`GPIO_IMU1_INT` (PC4, IMU1 data-ready) and `GPIO_SDIO_NCD` (PD4, SD card
detect) both map to STM32 EXTI line 4 — the SYSCFG_EXTICR mux only lets one
GPIO port own a given EXTI line number at a time. `stm32_sdio_initialize()`
ran after `stm32_spidev_initialize()` in bringup and silently stole EXTI4
away from IMU1, so `estimator` blocked forever on the first `read()` from
`/dev/imu0` — no interrupt could ever arrive.

Root cause: the DAKEFPV_H743 board port had inherited unused SD-card/NCD
board-port code from a WeAct-H743VIT reference port. **The DAKEFPV_H743
board has no SD card slot.**

Fix: removed the SD/SDIO code entirely —
`nuttxspace/boards/arm/stm32h7/dakefpv-h743/src/stm32_sdmmc.c` deleted,
`HAVE_SDIO`/`GPIO_SDIO_NCD`/`stm32_sdio_initialize()` removed from
`dakefpv-h743.h` and `stm32_bringup.c`, build-file references dropped from
`Makefile`/`CMakeLists.txt`, `CLAUDE.md` corrected.

## Bug 2 (fixed): HPWORK stack overflow in the FIFO worker

`icm_fifo_worker()` (runs on the HPWORK high-priority work queue) declared
`uint8_t raw[ICM_FIFO_MAX_BYTES]` (2080 bytes) as a stack local. The actual
build's `CONFIG_SCHED_HPWORKSTACKSIZE=2048` — smaller than the buffer
alone. Once bug 1 was fixed and IMU1 could actually deliver a real
watermark interrupt, the very first one scheduled this worker and blew its
stack immediately, with no fault dump (memory corruption, not necessarily
an immediate exception).

Fix: moved the burst-read buffer off the worker's stack into the
heap-allocated device struct (`dev->fifo_raw` in `struct icm_dev_s`,
`nuttxspace/drivers/sensors/icm42688p-fifo.c`). Safe because HPWORK runs
one job at a time (`CONFIG_SCHED_HPNTHREADS=1`) and the buffer is only
touched from within `icm_fifo_worker()`.

## Bug 3 (fixed): estimator's own `printf()` never appeared

`estimator_main()` runs an infinite loop calling `printf()` but never
`fflush(stdout)`/exits, so its output sat in the C library's stdio buffer
forever. The driver's `syslog()` calls appeared fine because
`CONFIG_SYSLOG_CONSOLE` writes straight to the console, bypassing stdio
buffering entirely — that difference is what made it look like only the
app's own output was broken.

Fix: `setvbuf(stdout, NULL, _IONBF, 0);` at the top of `estimator_main()`
(`nuttxspace/fc-stack/estimator/estimator_main.cpp`). Confirmed this holds
for `usbnsh` too — `CONFIG_CDCACM_CONSOLE=y` makes the USB CDC/ACM device
`/dev/console`, same as UART would be, so the fix is transport-agnostic.

## Open issue: still hangs, now consistently inside `icm_open()`'s last log line

After bugs 1–3, `estimator` gets much further — IRQ attaches, first
watermark interrupt is serviced, IRQ enables — then hangs **every single
run** partway through the final `syslog(LOG_INFO, "icm_open: streaming
armed\n")` call in `icm_open()`, before control ever returns to
`estimator_main()`'s own code.

### Diagnostic: independent LED heartbeat

Since no SWD/J-Link is currently connected (debug pins not wired up), added
a kernel thread heartbeat unrelated to console/USB/HPWORK, so a real
system-wide lockup can be told apart from a single stuck task by eye:

- `heartbeat_main()` in `stm32_bringup.c`, toggles `GPIO_LD1` (user LED,
  PD10) every 300 ms via `usleep()`, started with `kthread_create()` at the
  very top of `stm32_bringup()`.
- First run: priority `SCHED_PRIORITY_DEFAULT - 20` (below the estimator
  task). **Result: LED stopped blinking when the hang occurred** — could
  mean either a true global lockup, or just CPU starvation from something
  spinning at a priority above the heartbeat's.
- Bumped priority to `230` (above the estimator task's default 100, close
  to HPWORK's 224) to disambiguate. **Result: LED still stopped blinking**,
  and in the most recent run got stuck holding the pin high rather than
  mid-toggle. A thread at this priority failing to run at all is strong
  evidence of a genuine global lockup (most likely interrupts disabled and
  never restored, or a CPU-level halt) rather than plain priority
  starvation — normally the scheduler tick alone would still let a
  near-max-priority thread preempt anything.
- `CONFIG_ARCH_STACKDUMP=y` is enabled but no fault dump has appeared on
  any run, which argues against a straightforward CPU exception (hard
  fault) and toward either a genuine deadlock/lockup or a fault occurring
  somewhere the dump mechanism itself can't run.

**The heartbeat kthread is still in the code** (`stm32_bringup.c`) —
remove once this is root-caused, or ask to keep it if still debugging.

### Ruled out: simple USB CDC/ACM TX buffer exhaustion

`drivers/serial/serial.c` (`uart_putxmitchar()`) has a documented race
around blocking for TX buffer space, with a comment explicitly citing
upstream NuttX issue #14662 and calling out USB CDC/ACM by name as a
device where "the logic would hang below waiting for space in the TX
buffer." The board's CDC/ACM TX buffer defaults to only 193 bytes
(full-speed USB, `CONFIG_USBDEV_DUALSPEED` off), which seemed like a
plausible trigger given the volume of debug `syslog()` output right before
the hang.

Bumped `CONFIG_CDCACM_TXBUFSIZE` from 193 → 2048 (persisted via
`make savedefconfig` into
`nuttxspace/boards/arm/stm32h7/dakefpv-h743/configs/usbnsh/defconfig`,
along with several other Kconfig options — `CONFIG_SENSORS_ICM42688P`,
`CONFIG_FC_ESTIMATOR`, `CONFIG_SPECIFIC_DRIVERS`, `CONFIG_STM32H7_SPI4` —
that had only ever been set locally via menuconfig and were never saved,
discovered when an incidental `make distclean` reset `.config` to the
checked-in defconfig and dropped them).

**Result: no change at all** — same hang, same exact log line, every time.
A 10x larger buffer should have shifted or avoided a simple volume-based
buffer-full condition; it didn't. This rules out plain TX buffer
exhaustion as the trigger and points instead at something **timing-based**
rather than **data-volume-based**.

### Current working theory (untested — session ended mid-edit)

The IMU's default ODR is 1 kHz (register reset value 0x06, corrected from
an earlier miscalculation of 200 Hz) with a 50-sample watermark, so the
watermark interrupt fires roughly every 50 ms. The hang lands on the same
log line on every run, which is consistent with a fixed-timing collision
between that periodic interrupt and whatever `icm_open()`'s last console
write is doing — not consistent with a buffer simply filling up from
cumulative bytes.

Next step in progress: strip the debug `syslog()` calls out of the driver's
hot path entirely (`icm_fifo_start()`'s attach/enable messages,
`icm_fifo_worker()`'s first-interrupt message, `icm_open()`'s
streaming-armed message — keeping only error-path logging) and rely purely
on the LED heartbeat for pass/fail signal. If `estimator` gets past
`icm_open()` cleanly with the heartbeat still blinking, that confirms the
freeze is tied to console I/O landing near an active periodic interrupt,
not to the IMU/SPI logic itself — at which point `estimator_main()`'s own
`printf()` calls in its loop would need the same treatment (they write to
the same console and could hit the same window once the loop starts).

If it still hangs with zero console output anywhere near the interrupt,
the bug is unrelated to console I/O and lies elsewhere in the
interrupt/SPI/timing path — at that point reconnecting the J-Link for a
live GDB session (NuttX ships an RTOS-aware GDB extension at
`nuttxspace/nuttx/tools/pynuttx/` — `ps`, `deadlock`, `irqinfo` commands
work even with the target fully halted) becomes the most direct way
forward, since static analysis has been exhausted on the remaining
hypotheses.

## Files touched this session

- `nuttxspace/boards/arm/stm32h7/dakefpv-h743/src/stm32_sdmmc.c` — deleted
- `nuttxspace/boards/arm/stm32h7/dakefpv-h743/src/dakefpv-h743.h`
- `nuttxspace/boards/arm/stm32h7/dakefpv-h743/src/stm32_bringup.c`
- `nuttxspace/boards/arm/stm32h7/dakefpv-h743/src/Makefile`
- `nuttxspace/boards/arm/stm32h7/dakefpv-h743/src/CMakeLists.txt`
- `nuttxspace/boards/arm/stm32h7/dakefpv-h743/configs/usbnsh/defconfig`
- `nuttxspace/drivers/sensors/icm42688p-fifo.c`
- `nuttxspace/drivers/include/nuttx/sensors/icm42688p-fifo.h`
- `nuttxspace/fc-stack/estimator/estimator_main.cpp`
- `CLAUDE.md`
