## Intro to RTOS

In this first part we aim at giving a first overview of why we uses RTOSs and what are the main concepts they introduce.

---
### The scheduler

When wanting to run multiple tasks on the same CPU core 

Every task is managed by the scheduler. The scheduler role is to allocate CPU time to each tasks so that every tasks can be completed at the same time.

The simplest scheduler that can be imagined is a **cooperative scheduler**. It is named "cooperrative" since everytask is run one after the other once there time to be executed has come. 
For example a 3 task cooperative sheduling would like this (in Baremetal):

```c

/* Our tasks definitions */

satic void task_led(void)
{
	led_toggle();
}

static void task_pot_servo(void)
{
	angle = pot_read() * 180/4095;
	servo_set(angle);
}

satic void task_lcd_display(void)
{
	display_update(angle);
}

/* the task formalised as a struct */

struct task {
	void (*run)(void);
	uint32_t interval_ms;
	uint32_t last_ms;
}

static struct task tasks[] = {
	{task_led, 500, 0},
	{task_pot_servo, 10, 0},
	{task_lcd_display, 100, 0},	
}

/* The main Loop */

int main()
{
	init();
	
	while(1).  // The Cooperative scheduler
	{
		uint32_t now = get_ms();
		for(int i=0; i <= ARRAY_SIZE(tasks); i++)
		{   // Checks if a task is due to run and run if so
			if(now - tasks[i].last_ms >= tasks[i].interval_ms)
			{
			tasks[i].last_ms = now;
			tasks[i].run();
			}
		}
	}
}

// Example from [1]
```

The issue with this scheduling architecture is the "cooperation" between each task that must be respected. In practical terms if we introduce an additional task that takes a long time to complete (relative to the others) `{task_slow_log, 3000, 0}` then the CPU will be stuck with finishing this task even if over tasks needs to run (thus miss their deadlines).

To prevent this we can use "tricks" like interrupts, DMA, or splitting the task in smaller ones but this can quickly lead to a non-uniform and messy architecture.

That's why modern RTOSs use **Premptive scheduling** instead. This means each task can be "preempted" (interrupted while running) to switch to a higher priority task. 
This task switching is often designated as *context switching* and is done by saving the entire state (registers, PC, stack pointer) of a task to later restore it cleanly.

The scheduler checks is usually implemented with a timer interrupt that regularly checks if another higher priority task is ready to execute. If multiple tasks are ready to run at the same time, then the higher priority task is executed first (and in the case of same priority, round-robing scheduling is usually used).

An implementation of the same 4 tasks would then look like this:

```c
// Implementation for FreeRTOS

// Logging handling function
static void log_write(const char *msg)
{
    uart2_send_string(msg);
}

static void task_led(void *arg)
{
    while (1) {
        led_toggle();
        log_write("[led_task]: toggled the LED state\r\n");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void task_pot_servo(void *arg)
{
    while (1) {
        angle = pot_read() * 180 / 4095;
        servo_set(angle);
        log_write("[pot_servo_task]: pot and set servo angle\r\n");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void task_lcd_display(void *arg)
{
    while (1) {
        display_update(angle, get_ms());
        log_write("[display_task]: updated angle and uptime\r\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void task_uart(void *arg)
{
    while (1) {
        uart_send_a_lot_of_data();
        log_write("[uart_task]: sent a lot of data\r\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

int main(void)
{
    init();
    
    log_mutex = xSemaphoreCreateMutex();

    xTaskCreate(task_led,       "led",       128, NULL, 1, NULL);
    xTaskCreate(task_pot_servo, "pot_servo", 128, NULL, 1, NULL);
    xTaskCreate(task_lcd_display,   "display",   128, NULL, 1, NULL);
    xTaskCreate(task_uart,      "uart",      128, NULL, 1, NULL);

    vTaskStartScheduler();
}
// Example from [1]
```

An additional advantage of using this kind of scheduler (so RTOS) is that each task can be imagined independently (as it's own program with loop) without worying about CPU time usage.

### Resource sharing

However independant tasks running simultenaously can lead to another problem when multiple tasks want to access the same resource. 

For example if several task wants to write logs to a UART, then some task logs might interfere with others if tasks are preempted at the wrong time.

That's why resource sharing mechanisms exist on RTOSs . One of them is called a mutex, and ensures only one task can access a specific resource. For example we can rewrite the `log_write()`function like so:

```c
// Implemented in FreeRTOS
static void log_write(const char *msg)
{
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    uart2_send_string(msg);
    xSemaphoreGive(log_mutex);
}
```

However this means each task has to wait that the locked log message is transmitted. Thus impacting the real time deadline of certain tasks (like the  `task_pot_servo()`) . 

### Queuing and Events 

To prevent this, we can create a separate task that exclusively handles logging messages. This allows to isolate the logging "subtasks" of each task and create a separate task/thread that will handle this job.

In order for this task to execute every other task's logging, we can use a queue. A queue let's other tasks "queue" pending work on another task/thread. This task when executing (scheduler gives CPU to the task) will then process each pending work in the order they where queued.

Here is the last example with the implemented logging task and queue mechanism

```c
// Implemented in FreeRTOS

static void log_write(const char *msg)
{
    xQueueSend(log_queue, &msg, 0);
}

static void task_log(void *arg)
{
    const char *msg;
    while (1) {
        xQueueReceive(log_queue, &msg, portMAX_DELAY);
        uart2_send_string(msg);
    }
}

static void task_led(void *arg)
{
    while (1) {
        led_toggle();
        log_write("led: toggle\r\n");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void task_pot_servo(void *arg)
{
    while (1) {
        angle = pot_read() * 180 / 4095;
        servo_set(angle);
        log_write("pot: update\r\n");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void task_lcd_display(void *arg)
{
    while (1) {
        display_update(angle, get_ms());
        log_write("display: update\r\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void task_uart(void *arg)
{
    while (1) {
        uart_send_a_lot_of_data();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

int main(void)
{
    init();

    log_queue = xQueueCreate(16, sizeof(const char *));

    xTaskCreate(task_log,       "log",       128, NULL, 1, NULL);
    xTaskCreate(task_led,       "led",       128, NULL, 1, NULL);
    xTaskCreate(task_pot_servo, "pot_servo", 128, NULL, 2, NULL);
    xTaskCreate(task_lcd_display,   "display",   128, NULL, 1, NULL);
    xTaskCreate(task_uart,      "uart",      128, NULL, 1, NULL);

    vTaskStartScheduler();
}
```

Moreover this `task_log` will be **event driven** meaning it will only be waken up by the scheduler once work has been added to the queue. It means our tasks that have requirements for running (for example some data needs to available) then it doesn't have to continuously check and can be "waken up" when the requirements are met.


---
## NuttX RTOS 

Let's switch back to NuttX to see how all these features can be used and what other specificities are available for this RTOS.

### Build modes & Memory space

NuttX specifically can be built in three different modes [2],[3].

#### Build modes
##### Flat build

In this mode NuttX and your custom code is compiled together as a single image and all components share the same address space. This means your code as well as the kernel can access directly any adrdess space the same way.

##### Protected build

In this mode two physically distinct address spaces have been assign to the kernel space and user space. This is implemented by using the **MPU** (Memory protection unit) that prevents user space code to directly access the kernel address space. This although still means there is a single user space Heap shared by all the tasks.

In this mode the build outputs two independant binaries one containing the privileged RTOS kernel and the other containing user apps.

`CONFIG_BUILD_PROTECTED=y`

The advantage of this build is to prevent user code to interfere with the kernel space by writing in an undesired memory space that could lead to a kernel crash.

On the other end it enforces the access to kernel level features through syscalls that adds small delays.
##### Kernel mode

The kernel mode allows each user process to have its own independant address space from where it executes. In practice, it means each task has its own independant heap managed by the MMU (memory management unit).

In addition, it also implements all the other features of the protected build.

`CONFIG_BUILD_KERNEL=y`

#### Heap allocation

In Apache NuttX, you allocate heap memory using standard POSIX/ANSI-C functions like `malloc()`, `calloc()`, `realloc()`, and `free()`

##### In user space:
```c
#include <stdlib.h>

void heap_example(void)
{
    // Allocate 100 bytes
    char *buffer = (char *)malloc(100);
    
    if (buffer == NULL) 
      {
        // Handle allocation failure
        return;
      }

    // Use memory...
    buffer[0] = 'A';

    // Release memory
    free(buffer);
}
```

##### In Kernel space:

Similar but using `kmm_malloc()`and `kmm_free()`
```c
struct icm_dev_s *priv;

priv = kmm_malloc(sizeof(struct icm_dev_s)); // Allocates the driver config struct on the kernel heap memory
```

##### Multiple heaps:

NuttX allows you to instantiate and manage entirely separate heap regions using its internal granular memory manager structures (`struct mm_heap_s`). 

To build a standalone custom heap instance out of a block of raw RAM, use mm_initialize() and its paired direct allocation APIs:

```c
#include <nuttx/mm/mm.h>

#define MY_HEAP_SIZE 1024
uint8_t my_memory_pool[MY_HEAP_SIZE];
struct mm_heap_s *g_myheap;

void setup_custom_heap(void)
{
    // Initialize the custom heap instance
    g_myheap = mm_initialize("custom_heap", my_memory_pool, MY_HEAP_SIZE);
}

void use_custom_heap(void)
{
    // Allocate specifically from your custom heap pool
    void *ptr = mm_malloc(g_myheap, 50);
    
    if (ptr != NULL)
      {
        // Free specifically to your custom heap pool
        mm_free(g_myheap, ptr);
      }
}
```
### Processes, Tasks and threads

The terminology used by NuttX differs slightly from that of a conventional desktop operating system.

A **thread** is an independently scheduled execution context. It has its own program counter, register state, stack and scheduling parameters.

NuttX distinguishes between **tasks** and **pthreads**:

- A task is created as an independently startable program entry point. (usually referred as the "user app")
- A task may create one or more POSIX threads using `pthread_create()`.
- The original task and all pthreads created from it form a **task group**.
- Threads belonging to the same task group share resources such as file descriptors, streams, sockets, environment variables and open message queues.

A task group is therefore the closest NuttX equivalent to a process. However, in Flat and Protected builds, it does not necessarily have an independent virtual address space. True process-like address spaces require a Kernel Build and an MMU.

A practical convention is:

- Use a **task** for an independently startable application or subsystem.
- Use a **pthread** for internal workers that belong to the same subsystem.
- Use a **kernel thread** only for logic that must execute with kernel privileges or access internal NuttX interfaces.

Kernel threads are created with `kthread_create()`. In Protected and Kernel builds, they execute with supervisor privileges and have access to internal OS resources. We can consider them as special “tasks” that reside within the OS.

For example:
```c
// Create a pthread within a task
ret = pthread_create(&thread1, NULL, thread_worker, &thread_arg);

// Create a Kernel thread
kthread_create("name", SCHED_PRIORITY, STACK_SIZE, fct_main, NULL);
```

#### Scheduling priorities

NuttX uses numerically increasing priorities:

```
0     Lowest priority
...
255   Highest priority
```

Priority zero is reserved for the idle task in normal use.

By default, NuttX applies strict fixed-priority scheduling. A ready higher-priority task prevents all lower-priority tasks from executing until it blocks, exits, or changes priority. Tasks at the same priority are scheduled using FIFO ordering. Round-robin time slicing can be enabled as an optional policy.

This means that assigning priorities is part of the system design. Priorities should represent timing urgency rather than the perceived importance of a software module.

---

### Task states and blocking

At any point, a task can conceptually be in one of the following states:

```
Running
    The task currently owns the CPU.

Ready
    The task can execute but another ready task currently has priority.

Blocked
    The task is waiting for an event, message, resource, I/O operation
    or timeout.

Inactive
    The task has not started or has terminated.
```

Only a ready task can be selected by the scheduler. A blocked task consumes no CPU time.

For example, a logging task can block while waiting for a message:

```c
for (;;)
{
  ssize_t size = mq_receive(log_queue,
                            (char *)&message,
                            sizeof(message),
                            NULL);

  if (size >= 0)
    {
      uart_write(message.data, message.length);
    }
}
```

While the queue is empty, `mq_receive()` blocks the logging task. When another task sends a message, the kernel moves the logging task back to the ready state. Whether it immediately starts running depends on its priority relative to the currently running task.

NuttX message queues copy messages into kernel-managed queue storage. They can block the sender when full and the receiver when empty, or return immediately when configured as non-blocking. Messages can also carry a priority, allowing higher-priority messages to be received first.

The queue must be bounded. The application should therefore define what happens during overload:

- block the producer;
- discard the newest message;
- discard the oldest message;
- store only a counter or compressed event;
- increase the queue capacity.

For a hard real-time control task, blocking indefinitely to produce a log is generally undesirable. Dropping low-priority diagnostic messages is often safer than delaying the control loop.

### Event driven & asynchronous tasks

A task is **event-driven** when it remains blocked until something relevant happens.

Examples of events include:

- a message being added to a queue;
- a semaphore being posted;
- data becoming readable from a device;
- a timer expiring;
- a DMA transfer completing;
- a signal being delivered.

This is preferable to continuously polling a condition:

```c
while (!data_available())
  {
    /* Busy wait: consumes CPU */
  }
```

An event-driven implementation instead allows the task to sleep:

```c
while (sem_wait(&data_ready_sem) < 0)
  {
    if (errno != EINTR)
      {
        /* Handle the error */
      }
  }
```

Another task or an interrupt handler can call `sem_post()` when the data becomes available. This moves the waiting task from the blocked state to the ready state. NuttX supports this event-signalling use of semaphores, including posting from an interrupt handler.

A semaphore used for signalling is conceptually different from one used for resource ownership:

```
Resource protection:
    The same task normally acquires and releases the semaphore.

Event signalling:
    One execution context waits and another context posts.
```

This distinction is important when considering priority inheritance and ownership.

### Interrupts and deferred work

Interrupt handlers should execute for as little time as possible. Their main purpose is generally to:

1. Identify and acknowledge the interrupt.
2. Capture time-critical state.
3. Start or complete a DMA transaction.
4. Notify another execution context.
5. Return.

Long operations should be deferred to a task or work queue.

NuttX provides work queues for this purpose. A work queue consists of one or more worker threads that execute queued callback functions. Work queues can be used for delayed processing, serialization and interrupt bottom halves.

A simplified driver pattern is:

```c
static struct work_s g_fifo_work;

static void imu_fifo_worker(void *arg)
{
  struct imu_dev_s *dev = arg;

  /* Perform the longer bottom-half processing here. */
  imu_process_fifo(dev);
}

static int imu_interrupt(int irq, void *context, void *arg)
{
  struct imu_dev_s *dev = arg;

  imu_acknowledge_interrupt(dev);

  if (work_available(&g_fifo_work))
    {
      work_queue(HPWORK, &g_fifo_work, imu_fifo_worker, dev, 0);
    }

  return OK;
}
```

`HPWORK` is intended for high-priority driver bottom halves. It is not a separate thread dedicated to each driver: several drivers may submit work to the same worker thread. Consequently, a long-running work item can delay every other item queued behind it.

The high-priority work queue should therefore contain only short, bounded operations. Longer algorithms, logging, filesystem operations and application processing should normally use a lower-priority work queue or a dedicated task.

For a high-rate IMU driver, a possible architecture is:

```
IMU data-ready interrupt
          ↓
Very short ISR
          ↓
Start SPI DMA or schedule HPWORK
          ↓
DMA-complete interrupt
          ↓
Publish sample / post semaphore
          ↓
Estimator task becomes ready
```

This separates interrupt latency, bus transfer, driver processing and application-level estimation.
### Semantics and resource sharing

### Booting process

The NuttX initialization sequence can be divided into three broad phases:

```
A. Architecture and hardware reset initialization
B. NuttX kernel initialization
C. Board and application initialization
```

The architecture startup code initializes the processor, stack and memory sections before calling `nx_start()`. `nx_start()` initializes the core kernel facilities, including task structures, synchronization primitives, memory management, clocks, drivers and scheduler-related state. It then brings up the configured work queues and starts the initial application.

NuttX provides several board-initialization hooks:

#### Low-level board initialization

```
<arch>_board_initialize()
```

This runs very early, before the operating system is available. It should only perform low-level operations such as pin configuration, power setup and essential memory initialization. It cannot block or depend on normal OS services.

#### Early board initialization

```
board_early_initialize()
```

This runs after more of the OS has been initialized and can initialize simple drivers. However, it still executes in a startup context that must not wait for events.

#### Late board initialization

```
board_late_initialize()
```

This runs shortly before the main application starts. It executes in a temporary kernel thread, so it may wait for events and use interfaces such as SPI and I²C. It is suitable for complex driver initialization and filesystem mounting.

Application-controlled board initialization can alternatively be requested through:

```
boardctl(BOARDIOC_INIT, 0);
```

This invokes the board-specific application initialization logic from application context. NSH can use this mechanism during startup.

A board port should clearly document which devices are initialized in each phase. Initializing everything in the earliest possible hook makes the boot process harder to understand and can introduce invalid blocking operations.

---

Proposed Plan





---

## References


[1] https://www.youtube.com/watch?v=i_eU16X67qU
[2] https://nuttx.apache.org/docs/latest/guides/protected_build.html
[3]https://nuttx.apache.org/docs/latest/implementation/memory_configurations.html