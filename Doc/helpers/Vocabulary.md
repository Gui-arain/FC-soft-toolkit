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
- Context switching: Context switching is the process where the CPU stops running one process, saves its current state, and loads the saved state of another process so that multiple processes can share the CPU effectively

- Round Robin scheduling: Round Robin Scheduling is a method used by operating systems to manage the execution time of multiple processes that are competing for CPU attention. It is called "round robin" because the system rotates through all the processes, allocating each of them a fixed time slice or "quantum", regardless of their priority. The primary goal of this scheduling method is to ensure that all processes are given an equal opportunity to execute, promoting fairness among tasks.

- preemptive scheduling: 