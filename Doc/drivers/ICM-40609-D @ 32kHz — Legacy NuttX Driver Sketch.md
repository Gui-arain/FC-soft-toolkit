# ICM-40609-D @ 32kHz — Legacy NuttX Driver Sketch

Confirmed against TDK datasheet DS-000330 Rev 1.2 (FIFO section, pp.36-39).

**Sequencing**: build and verify the non-DMA version first (§1-8 below). Confirm no
overflow, sane timestamps, and a measured latency budget at 32kHz before adding DMA
(§9). DMA buffer/cache bugs are hard to debug blind — you want a known-good baseline
to diff against when something looks wrong.

## 1. FIFO packet format you'll actually use

You want accel + gyro + temp + timestamp → that's **Packet 3**:

```
Byte  Content
0x00  FIFO Header
0x01  Accel X[15:8]      0x02  Accel X[7:0]
0x03  Accel Y[15:8]      0x04  Accel Y[7:0]
0x05  Accel Z[15:8]      0x06  Accel Z[7:0]
0x07  Gyro  X[15:8]      0x08  Gyro  X[7:0]
0x09  Gyro  Y[15:8]      0x0A  Gyro  Y[7:0]
0x0B  Gyro  Z[15:8]      0x0C  Gyro  Z[7:0]
0x0D  Temperature[7:0]            (8-bit, NOT 16-bit — different scale than reg path)
0x0E  TimeStamp[15:8]    0x0F  TimeStamp[7:0]
```

**16 bytes/packet.** FIFO_ACCEL_EN=1, FIFO_GYRO_EN=1 → header pattern `0b0110_10xx` (or
`0b0110_1xxx` if `FIFO_TMST_FSYNC_EN`=1). This matches your existing `FIFO_CONFIG1` bit
choices — you already set `FIFO_TEMP_EN | FIFO_GYRO_EN | FIFO_ACCEL_EN` in your ioctl, but
note **FIFO_TEMP_EN isn't a real bit for this packet shape** — per the config table, temp
is *implied* once both accel+gyro are enabled (it rides along in the 16-byte packet "for
free"). Setting an undefined bit 2 of FIFO_CONFIG1 is harmless but not meaningful — drop it.

Read `TMST_CONFIG` and set `TMST_EN` (bit 0) — without it the 2 timestamp bytes are
present in the packet shape but not meaningfully populated.

## 2. FIFO storage ceiling — allocate 2080 bytes, not 2048

Per §6.3 of the datasheet: physical FIFO is 2048B, but due to the read-cache (2 packets
wide) sitting in front of it during active SPI traffic, **worst-case usable storage is
2048 + 1 packet-size = 2064B** — and the datasheet explicitly says **driver memory
allocation should always be 2080 bytes** to be safe. At 16B/packet that's 130 packets
max (2080/16), i.e. **~4.06ms of headroom at 32kHz** before overflow. That is your hard
real-time budget for IRQ → worker-thread → SPI-burst-drain.

## 3. Driver-side state machine

```
                 ┌──────────────────────────┐
 IRQ (FIFO_THS)  │  GPIO ISR (fast, no SPI) │  posts work to LP/HP worker queue,
 ───────────────►│  - clears nothing here   │  or gives a semaphore to a kthread
                 │  - just signals          │
                 └────────────┬─────────────┘
                              │
                              ▼
                 ┌──────────────────────────┐
                 │ Bottom half (worker      │  1. SPI: read FIFO_COUNTH/L  (2B)
                 │ thread / high-prio task) │  2. SPI: burst-read N*16 bytes from
                 │                          │     FIFO_DATA in ONE transaction
                 │                          │  3. Parse header per packet, validate
                 │                          │  4. Push parsed samples into circbuf
                 │                          │  5. Wake any blocked reader (poll/read)
                 └────────────┬─────────────┘
                              │
                              ▼
                 ┌─────────────────────────┐
       read()    │  circbuf (ring buffer)  │
 ◄────────────── │  of icm_sample_s        │
       poll()    │  guarded by g_lock      │
                 └─────────────────────────┘
```

This keeps the legacy `register_driver()`/`file_operations` shape: `icm_open`, `icm_close`,
`icm_read`, `icm_ioctl` all stay conceptually the same shape, but `icm_read()` now drains
the ring buffer instead of doing a synchronous SPI transaction, and there's a new
interrupt-context entry point plus a worker.

## 4. Concrete NuttX primitives to use

- **GPIO interrupt**: board-level `IMU_GPIO_IRQ` already routed to INT1 (per your existing
  `INT_CONFIG__INT1_*` bits in `icm_reset()`) — attach via `irq_attach()` /
  board-specific `xxx_gpiosetevent()`, called once at `icm40609d_register()` time, not per-open.
- **Bottom half**: NuttX `work_queue()` (`HPWORK` if available, else `LPWORK`) is the
  simplest legacy-compatible mechanism — avoids hand-rolling a kthread. At 32kHz with a
  ~4ms budget, HPWORK is strongly preferred if your board has a dedicated high-priority
  work queue thread (check `CONFIG_SCHED_HPWORK`).
- **Ring buffer**: NuttX has `include/nuttx/circbuf.h` (`circbuf_init/write/read`) — use
  it directly rather than reimplementing `bufpos`-style cursor logic.
- **Blocking reads / poll()**: add a `poll_s` waiter list + `nxsem_post()` from the worker
  so `icm_read()` can block (or return -EAGAIN for O_NONBLOCK) until data's available, and
  so userspace can `poll()`/`select()` on the fd instead of spin-calling `read()`.

## 5. Watermark sizing

`FIFO_CONFIG2`/`FIFO_CONFIG3` (12-bit watermark, bytes or records depending on
`INTF_CONFIG0__FIFO_COUNT_REC`). Pick watermark so that:

- Interrupt rate is sane (not 32kHz)
- Latency is acceptable for your application
- Worst case servicing time (IRQ latency + work-queue scheduling latency + SPI burst
  time) stays well under the 4.06ms hard ceiling

Example: watermark = 32 packets (512 bytes) → IRQ fires every 1ms → ~3ms of slack before
overflow even if one service cycle is missed. Tune based on your actual measured
IRQ-to-worker latency on your board.

## 6. SPI clock

Raise burst-read clock to the documented operating point — up to 24MHz per datasheet
§3.5 (`fSPC, SCLK Clock Frequency... 24 MHz`). Recommend running bursts at ~20MHz for
margin against board-level trace/connector parasitics, and keep a separate, slower
(~1–2MHz) clock only if you have flaky wiring for *config* writes — most designs can run
config writes at the same speed as bursts without issue. **2MHz, as currently hardcoded,
cannot sustain 32kHz × 16B/sample = 512kB/s payload** (2MHz SPI ≈ 250kB/s raw, and that's
before per-transaction overhead) — this must change regardless of anything else.

## 7. What stays the same from your existing driver

- Legacy `register_driver()` / `file_operations` table shape
- `icm_dev_s` struct as the per-device context (extended, not replaced)
- `__icm_read_reg_spi` / `__icm_write_reg_spi` as the low-level SPI primitives (just
  called with a higher frequency and, for bursts, a longer `len`)
- ioctl surface (`SNIOC_*`) — extend rather than replace; e.g. add
  `SNIOC_SET_WATERMARK`, keep `SNIOC_ENABLE_FIFO` but make it also arm the interrupt
  source registers (`INT_SOURCE0__FIFO_THS_INT1_EN`) instead of leaving them unset
- `icm_reset()` initialization flow — extend to also configure `TMST_CONFIG__TMST_EN`,
  watermark registers, and interrupt routing

## 8. What's structurally new

- `icm_dev_s` needs: `struct circbuf_s fifo_rb`, a `sem_t data_available`,
  `FAR struct pollfd *fds[CONFIG_xxx_NPOLLWAITERS]`, and an IRQ-context-safe flag
  (don't take `nxmutex_lock` from interrupt context — only the worker touches `dev->lock`)
- A new `icm_fifo_isr()` — minimal, just signals the worker, **no SPI calls inside the
  ISR itself** (SPI transactions can block / aren't IRQ-safe on most NuttX SPI drivers)
- A new `icm_fifo_worker()` — does the actual `FIFO_COUNTH/L` read, burst pop, header
  parse, circbuf push, poll-waiter wake
- A packet parser that walks the burst buffer in 16-byte strides, checks
  `HEADER_MSG` (bit 7 — if set, that "packet" is just empty-FIFO padding, stop), and
  rejects/logs on unexpected `HEADER_ACCEL`/`HEADER_GYRO` patterns (defensive — a
  corrupted SPI transaction would otherwise silently misalign every subsequent packet)

## 9. DMA — later optimization, not part of the first build

Once the byte-loop (`SPI_SEND()`-per-byte) version in §3-8 is working and verified,
the SPI burst transaction itself is the part worth offloading to DMA. Everything
above this section — ISR, work-queue bottom half, FIFO_COUNTH/L read, packet
parsing, circbuf, poll() — stays unchanged. DMA only replaces *how* the burst-read
in step 2 of the worker physically happens.

### 9.1 What DMA buys you

The byte-loop burst read blocks the worker thread's CPU for the entire transfer —
at a worst-case 2080-byte burst, that's ~2080 individual `SPI_SEND()` calls with the
CPU unavailable for anything else the whole time. DMA hands the transfer to the SPI
controller's DMA channel; the CPU sets up a descriptor, kicks it off, and is free
until a completion interrupt fires. This matters most if you're running other
time-critical work (flight control, fusion) on the same core — it doesn't change
your achievable SPI clock/throughput ceiling, which is still bounded by the bus
speed itself.

### 9.2 Platform dependency — check before designing around it

DMA support isn't something the IMU driver implements — it lives in the board's
NuttX SPI lower-half driver (e.g. `stm32_spi.c` or equivalent for your MCU). Check
whether your board's SPI driver has DMA enabled (e.g. `CONFIG_<CHIP>_SPI_DMA` or
similar). If it does, switching `__icm_read_reg_spi()`'s burst path from a manual
`SPI_SEND()` loop to `SPI_EXCHANGE()` (or `SPI_RECVBLOCK()`) is largely transparent —
the lower half uses DMA internally and your driver code barely changes.

### 9.3 Two-stage interrupt model

True DMA-triggered-by-IRQ means a second, separate completion interrupt — the DMA
controller's own IRQ — distinct from the IMU's `FIFO_THS` IRQ. The bottom half
becomes two stages instead of one:

```
 FIFO_THS IRQ  →  ISR kicks off DMA burst-read (does NOT block, does NOT parse)
 DMA-complete IRQ  →  ISR/worker parses packets, pushes to circbuf, poll_notify()
```

This is a real state-machine change to `icm_fifo_isr()`/`icm_fifo_worker()`, not a
one-line swap of the read call — the worker needs to track "DMA in flight" state and
the parse/circbuf-push logic moves to fire on DMA completion rather than immediately
after kicking the read.

### 9.4 Buffer requirements — the part most likely to bite

- **No stack buffers for DMA targets.** The `raw[ICM_FIFO_MAX_BYTES]` buffer in the
  worker (sketch §3 code) is stack-allocated per invocation — not safe for DMA on
  most NuttX-supported MCUs. Use `kmm_malloc()` once at init time (store the pointer
  in `icm_dev_s`) or a static buffer, not a per-call stack array.
- **Alignment** — DMA engines commonly require specific buffer alignment; check your
  MCU's DMA controller requirements.
- **Cache coherency** — on cache-coherent cores (Cortex-M7, A-class), the CPU can
  read stale cached data after a DMA write completes unless you call
  `up_invalidate_dcache()` on the buffer after DMA completion and before parsing, or
  place the buffer in a non-cached memory region.
- **Half-duplex framing** — your read needs a 1-byte register-address write before
  the burst clocks in data. Whether your controller's DMA path handles "write 1
  byte, then DMA-read N bytes" as one chained transaction or two separate ones is
  controller-specific; using `SPI_EXCHANGE()` with a prepared TX buffer
  (`[reg_addr, 0xff, 0xff, ...]`) handles this as one transaction on most platforms,
  but verify against your actual SPI lower-half implementation.

### Full thread process timeline

```
HPWORK THREAD                  USERSPACE APP TASK             IRQ CONTEXT
──────────────────────────────────────────────────────────────────────────

                               open("/dev/imu0")
                                  │
                               poll(fd, ...)  ← syscall
                                  │
                               [NuttX VFS calls icm_poll()]
                                  │  registers app's pollfd
                                  │  in dev->fds[]
                                  │  circbuf empty → nothing yet
                               [task suspended by scheduler]
                               [CPU free]

                                                          FIFO_THS fires
                                                             │
                                                          icm_fifo_isr()
                                                             │
                                                          work_queue(HPWORK,
                                                          icm_fifo_worker)
                                                             │
                                                          returns (ISR done)

icm_fifo_worker() runs
  __icm_read_reg(FIFO_COUNTH/L)
  __icm_read_reg(FIFO_DATA, N bytes)
  parse packets
  circbuf_write(samples)
  poll_notify(dev->fds)  ──────► [NuttX wakes the app task]
                                  [scheduler puts it back
                                   on the run queue]
(worker done)

                               [app task runs again]
                               poll() returns
                                  │
                               read(fd, buf, len)  ← syscall
                                  │
                               [NuttX VFS calls icm_read()]
                                  │  circbuf_read(samples)
                                  │  copies to userspace buf
                               [icm_read() returns]
                                  │
                               process samples...
                                  │
                               poll(fd, ...)  ← blocks again
```

### Calling the Driver from a user App

```cpp
/* Userspace application */ 
struct pollfd pfd; 
pfd.fd = open("/dev/imu0", O_RDONLY | O_NONBLOCK); 
pfd.events = POLLIN; /* I want to know when data is readable */ 
while (1) 
{ 
	poll(&pfd, 1, -1);  /* block indefinitely until IMU has data */ 
						/* CPU does NOTHING here — task is suspended */ 
	if (pfd.revents & POLLIN) 
	{ 
	read(pfd.fd, samples, sizeof(samples)); /* process samples */ 
	} 
}
```

### Enabling the driver

In menuconfig you need to select:
```
Board Specific drivers
FC Soft Toolkit Drivers -> IvenSense ICM-42688-P ...
fc-stack -> Estimator Main
```
