
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/sensors/icm40609d.h>
#include <nuttx/sensors/ioctl.h>

// estimayor_main.cpp
extern "C" int estimator_main(int argc, char *argv[]);
// Private constructor
static inline void imu_read_registers(int fd, struct icm40609d_data_s* sample_ptr);
static void imu_read_FIFO();

// Public Variables
struct icm40609d_data_s sample;

int estimator_main(int argc, char *argv[])
{
    //Access IMU driver
    int fd = open("/dev/imu0", O_RDONLY);
    if (fd < 0) {
         printf("estimator: failed to open /dev/imu0: %d\n", errno);
         return -1;
    }
    // continuously reads the IMU output registers
    while(1)
    {
        //Read IMU
        imu_read_registers(fd, &sample);

        // Driver returns big-endian — swap on little-endian ARM
        float ax = (int16_t)__builtin_bswap16(sample.x_accel) / 2048.0f;
        float ay = (int16_t)__builtin_bswap16(sample.y_accel) / 2048.0f;
        float az = (int16_t)__builtin_bswap16(sample.z_accel) / 2048.0f;
        float gx = (int16_t)__builtin_bswap16(sample.x_gyro)  / 16.4f;
        float gy = (int16_t)__builtin_bswap16(sample.y_gyro)  / 16.4f;
        float gz = (int16_t)__builtin_bswap16(sample.z_gyro)  / 16.4f;
        float t  = (int16_t)__builtin_bswap16(sample.temp)    / 132.48f + 25.0f;
        
        printf("ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f t=%.1f\n", ax, ay, az, gx, gy, gz, t);
        
        //sleep for 1ms -> 1kHz read loop
        usleep(1000);
    }
     
    close(fd);
    return 0;
}

static inline void imu_read_registers(int fd, struct icm40609d_data_s* sample_ptr)
{

// Directly read the imu registers
ssize_t n_read = read(fd, sample_ptr, sizeof(*sample_ptr));    
// Check the correct number of bytes have been read
if (n_read != (ssize_t)sizeof(*sample_ptr)) {
      printf("estimator: read error %zd\n", n_read);
      return;
  }
    
}


static void imu_read_FIFO()
{
    //Enable FIFO continuous mode

}