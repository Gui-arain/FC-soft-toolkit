
## Proto stack architecture

The goal is to design an architecture which combines both high rate controllers (8kHz) with a lower rate KF to estimate bias and noise.

```
IMU FIFO rawdata         Other sensors (MAG, GPS, VIO...)
|                          |           
| icm driver               |
v                          v
Ring buffer -------------> Estimator() 
|    |                     {
|    v                     convert_data();             (2)     attitude_target
|   (2)                    ESKF();                      |       |
|                          }                            |       |
v                          |                            v       v
Controller_rate()  <-------+---------------------> Controller_attitude()
{                      Error state estimate        {
convert_data();                                    convert_data();
Filter();                                          Error_state_corection();
Error_state_corection();    angular_rate_target    attitude_PID();
angular_rate_PID();   <--------------------------- }
}                                                  
|
|
v 
Mixer()
{
corection_motor_mapping();
send_dshot();
}
```

Two main paths:
- A high rate update (8kHz) path:
	- Read IMU data and apply the the estimated error state
	- Angular rate controller 
	- send motors commands
- A lower rate update path (500 Hz):
	- Read IMU data and convert
	- Error state Kalman Filter with additional sensors (MAG, GPS, VIO...)

Other controllers in a nested PID loop:
- Attitude controller
- position controller

## Final proposed architecture

![[agile_autonomy_fc_stack_ctbr_indi.png]]
*Figure 2: final architecture block diagram*

Note: We can also add other sensors like GPS and range finders by exposing them to the estimator through their respective drivers.

**collective-thrust-and-body-rates (CTBR)**: the companion's planner/MPC/learned policy outputs total thrust + desired body rates

## NuttX implementation

### Task list

We split in the task in two tasks with each one it's own independant priority and real time deadlines:

1. [Task 1] This task is composed of the sequence of functions that needs to be executed sequentially at the same rate:
	* IMU ring buffer read -> data convertion & filtering -> gyro bias correction (from estimated error state) -> angular rate controller -> mixer -> send motor commands (*Figure 2 orange blocks*)
2. [Task 2] Estimator task: this task runs at a lower rate and estimates the error state variable that is use by the different controllers to alleviate imu biases. (*Figure 2 blue blocks*)
	* this estimator can be a Mahony complementary filter or an ESKF, ESEKF

These two stacks creates the core of the flight stack but we can add others to add necessary/usefull features:

3. [Task 3] Logger task: This tasks receives all logging messages from other tasks through a message queue and use UART/USB/SD Card/Flash memory to send/write them.
4. [Task 4] Commander task: this task receiver command inputs from either the radio controller or the companion computer:
	1. Radio: receive packets from the RC receiver via UART/USART and expose them to other controller tasks
	2. CC: receives CTBR via USB and expose them to other controller tasks

### Task interfaces

**IMU data -> consumer tasks**
*1 writer / 1-3 consumers*

Several tasks need to consume data from the IMU. The IMU driver uses the FIFO/watermark to take advantage of the full 32kHz ODR of the icm while not overloading the CPU. Since the driver gathers several samples at a time and several consumers need to access those samples at different rates we use a ring buffer.

Two options:
- NuttX uORB style driver already includes an upper half with a ring buffer made for multiple consumers with each its own read cursor. It also handles `poll()` for each subscriber
- Re implement this uORB upper half feature by hand in our own driver

**Other sensors -> estimator**
*1 writer / 1 consumer*

one driver per other sensor with polling? 

**Estimator -> controllers**
*1 writer / 1-3 consumers*

Erros state is stored in the user heap and exposed through a **Seqlock** mechanism to provide **lock-free, high-speed reading** of shared data while providing exclusive write protection.

It uses a Seqcount to inform the reader if the data has been written mid read and retries if so. It is ideal for frequent read, rare writes  but the write operation must remain short.

[?] We might create multiple Seqlocks since some controllers don't need the full actualised error state. For example the angular rate controller only needs the updated angular rate error.

