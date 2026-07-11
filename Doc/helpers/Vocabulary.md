## Embedded Systems Glossary (STM32 / Real-Time Context)

---

### 🧠 Data handling & timing

- **hardware FIFO**: A small memory buffer inside a peripheral (e.g., SPI, UART, IMU) that stores incoming/outgoing data in first-in-first-out order, reducing CPU intervention and smoothing burst transfers.

- **ring buffer**: A circular software buffer where read/write pointers wrap around; commonly used for continuous data streams without memory reallocation.

- **DMA (Direct Memory Access)**: A hardware engine that transfers data between memory and peripherals without CPU involvement, enabling high-throughput and low-latency data movement.

- **ODR (Output Data Rate)**: The frequency at which a sensor produces new data samples (e.g., 32 kHz for an IMU).

- **CPU overhead**: The portion of CPU time consumed by managing tasks (interrupts, copying data, protocol handling) rather than doing core computations.

- **downsampling**: Reducing data rate by keeping only a subset of samples (e.g., 32 kHz → 1 kHz), often to reduce processing load.

- **jitter**: Variability in timing (e.g., interrupt latency or sampling intervals), critical in real-time systems.

- **DRDY interrupt (Data Ready)**: A hardware signal from a peripheral (e.g., IMU) indicating new data is available.

- **watermark interrupt**: Trigger generated when a FIFO reaches a predefined fill level, allowing batch processing instead of per-sample interrupts.

---

### ⚙️ CPU, interrupts & execution

- **FPU (Floating Point Unit)**: Hardware accelerator for floating-point arithmetic (single/double precision), improving performance for control/estimation algorithms.

- **EXTI (External Interrupt/Event Controller)**: STM32 block that maps GPIO pins or internal signals to interrupt/event lines.

- **IRQ (Interrupt Request)**: Signal indicating an event requiring CPU attention.

- **ISR (Interrupt Service Routine)**: Function executed in response to an interrupt.

- **NVIC (Nested Vectored Interrupt Controller)**: ARM Cortex-M interrupt controller handling prioritization and nesting of interrupts.

- **SysTick timer**: Dedicated system timer (usually 1 ms tick) used for OS scheduling or time base.

---

### 💾 Memory & protection

- **MPU (Memory Protection Unit)**: Hardware unit enforcing memory access rules (regions, permissions), used in protected OS modes (e.g., NuttX protected mode).

- **SRAM (Static RAM)**: Volatile memory used for runtime data (stack, heap, buffers).

- **TCM RAM (Tightly Coupled Memory)**: High-speed memory directly connected to CPU (no bus arbitration), deterministic and ideal for real-time code/data.

- **ECC (Error Correction Code)**: Hardware mechanism to detect and correct memory bit errors.

- **CRC (Cyclic Redundancy Check)**: Hardware/software computation for data integrity verification.

---

### 🚌 Bus architecture (STM32H7 relevant)

- **APB buses (Advanced Peripheral Bus)**: Low-speed peripheral buses (timers, UART, etc.), simpler and lower power.

- **AHB buses (Advanced High-performance Bus)**: Higher-speed buses connecting memory and high-bandwidth peripherals.

- **AXI bus matrix**: High-performance interconnect allowing multiple masters (CPU, DMA) to access multiple slaves (memory/peripherals) in parallel.

---

### ⚡ Reset, power & clocks

- **POR (Power-On Reset)**: Reset triggered when power is first applied.

- **PDR (Power-Down Reset)**: Reset when supply voltage drops below a critical threshold.

- **BOR (Brown-Out Reset)**: Reset triggered when voltage drops below a configurable level to prevent malfunction.

- **RCC (Reset and Clock Controller)**: STM32 module managing clocks, PLLs, and peripheral resets.

- **HSI clock (High-Speed Internal)**: Internal RC oscillator (fast startup, lower accuracy).

- **HSE clock (High-Speed External)**: External crystal/oscillator (higher accuracy, used for precise timing).

- **PLLs (Phase-Locked Loops)**: Circuits that multiply/divide clock frequencies to generate system/peripheral clocks.

---

### 🔌 Communication interfaces

- **SPI (Serial Peripheral Interface)**: Full-duplex, synchronous, high-speed serial bus (master/slave).

- **QUADSPI**: Extended SPI using 4 data lines for high-throughput (commonly for external flash memory).

- **I2C (Inter-Integrated Circuit)**: Two-wire (SCL/SDA), half-duplex bus with addressing, slower but simple.

- **I2S (Inter-IC Sound)**: Serial interface optimized for digital audio streams.

- **SAI (Serial Audio Interface)**: Advanced audio interface supporting multiple protocols (I2S, TDM).

- **USART (Universal Synchronous/Asynchronous Receiver Transmitter)**: Flexible serial interface supporting UART, synchronous modes, LIN, etc.

- **SPDIFRX**: Receiver for Sony/Philips Digital Interface audio streams.

- **SWPMI (Single Wire Protocol Master Interface)**: Single-wire communication interface for low-pin-count devices.

- **MDIO (Management Data Input/Output)**: Interface for configuring Ethernet PHY devices.

---

### 💽 Storage & external interfaces

- **SD / SDIO / MMC**: Interfaces for SD cards and MultiMediaCards; SDIO is a parallel high-speed variant.

- **FMC (Flexible Memory Controller)**: Interface for external memories (SRAM, NOR, NAND, SDRAM).

---

### 🌐 Connectivity & advanced peripherals

- **FDCAN (Flexible Data-rate CAN)**: Modern CAN protocol supporting higher data rates and flexible payloads.

- **OTG_HS (USB On-The-Go High-Speed)**: USB interface supporting device/host roles at high speed (480 Mbps).

- **DCMI (Digital Camera Interface)**: Parallel interface for camera sensors.

- **DFSDM (Digital Filter for Sigma-Delta Modulators)**: Interface for sigma-delta ADC streams (e.g., audio, precision sensing).

- **RNG (Random Number Generator)**: Hardware generator producing true random numbers (for crypto, seeding).

---

### 🛠 Debug & development

- **JTAG (Joint Test Action Group)**: Standard interface for debugging, boundary scan, and programming.

- **J-Link**: A hardware debug probe (by SEGGER) that interfaces with JTAG/SWD to program and debug MCUs.

---

### ⏱ Reliability & safety

- **watchdogs**: Timers that reset the system if software fails to refresh them, ensuring recovery from hangs.

- **RTC (Real-Time Clock)**: Low-power clock keeping track of time/date, often running during low-power modes.

---

### 🔄 OS & scheduling

- **context switching**: The process where the CPU stops running one task, saves its entire state (registers, PC, stack pointer) into the task's TCB, and restores another task's saved state — allowing multiple tasks to share a single CPU core. Triggered by the scheduler on a timer tick, a blocking call, or a higher-priority task becoming ready.

- **round-robin scheduling**: A scheduling policy where the OS cycles through all ready tasks in order, giving each a fixed time slice (quantum) regardless of priority. Ensures fairness; commonly used as a tie-breaking rule between equal-priority tasks in preemptive schedulers like NuttX.

- **preemptive scheduling**: A scheduling mode where the OS can forcibly interrupt a running task at any point (e.g., on a timer interrupt) to run a higher-priority or time-sliced task. Contrasts with cooperative scheduling, where tasks voluntarily yield the CPU.

- **semantic**: The behavioral contract of an operation — what it guarantees about ordering, visibility, and correctness. Commonly appears in concurrency: *acquire semantics* (no reads/writes after this point may be reordered before it) and *release semantics* (no reads/writes before this point may be reordered after it). Critical for lock-free data structures and memory-barrier placement.

- **mutex (Mutual Exclusion lock)**: A synchronization primitive that allows only one task to hold it at a time. A task attempting to acquire a locked mutex is put to sleep until the holder releases it. Unlike a binary semaphore, a mutex has an owner — which enables priority inheritance to avoid priority inversion.

- **work queue**: A kernel mechanism for deferring work from an ISR or high-priority context to a lower-priority task context. Function pointers (with arguments) are enqueued and processed sequentially by a dedicated worker task, keeping interrupt handlers short and non-blocking.

- **callback function**: A function passed as an argument to another function, invoked by that outer function when a specific event or operation completes. Common pattern for deferred actions (e.g., completion handlers, event notifications).

- **Task Control Block (TCB)**: A dedicated data structure in RAM created for every task, serving as the system's record of that task's full state when it is not running:
  - **Context Storage**: saves CPU registers and the Program Counter (PC) during context switches.
  - **Stack Pointer**: points to the top of the task's private stack.
  - **Task State**: tracks whether the task is Running, Ready, Blocked, or Suspended.
  - **Priority Level**: integer representing the task's scheduling importance.
  - **Linking Pointers**: link the TCB into the scheduler's ready or blocked lists.

---

### 💻 NuttX / POSIX interfaces

- **syscall (system call)**: A controlled gateway from unprivileged userspace into the privileged OS kernel. The application invokes a well-defined entry point (e.g., `read`, `write`, `poll`) via a trap instruction (SVC on ARM Cortex-M); the kernel validates and executes the request, then returns the result. In NuttX flat-address mode, syscalls are direct function calls; in protected mode they cross a privilege boundary.

- **file descriptor (FD)**: A small non-negative integer returned by `open()` that acts as a handle to an open OS resource — a file, device driver, socket, or pipe. All subsequent operations (`read`, `write`, `ioctl`, `poll`, `close`) reference the resource through this integer. NuttX is POSIX-compliant: device drivers register in the VFS and are accessed via FDs like any file.

- **Virtual File System (VFS)**: An abstraction layer inside the OS kernel that presents a uniform file-like API (`open`/`read`/`write`/`ioctl`/`close`) over heterogeneous resources — files, character device drivers, network sockets, pipes, and more. In NuttX, drivers register as named nodes in the VFS tree (e.g., `/dev/imu0`), letting userspace access hardware through standard POSIX calls without knowing the underlying implementation.

- **polling (`poll()`)**: A POSIX syscall that lets a userspace task block on one or more file descriptors until at least one has data ready (readable, writable, or error). The task sleeps and consumes no CPU while waiting — the kernel wakes it when a driver signals readiness. Preferred over busy-waiting for event-driven I/O in multi-task systems.

---

- pointer casting: the process of changing the data type of a pointer without changing the underlying memory address it references. It instructs the compiler to interpret the raw bits at that memory location as a different data type

- **Zero-order-hold (ZOH) interpolation** is ==a mathematical method and practical reconstruction technique used in digital signal processing to convert a discrete-time signal (a series of samples) back into a continuous-time signal==. It holds the value of the most recent sample constant until the next sample arrives, producing a distinct staircase-like waveform

- **4-fold oversampling** (or 4x) refers to ==a technique in digital signal processing (DSP) and machine learning where a dataset or signal is evaluated, simulated, or sampled at **four times** the base rate==. Depending on the field, this technique is primarily used to prevent signal distortion or balance imbalanced datasets.

- Madgwick filter: