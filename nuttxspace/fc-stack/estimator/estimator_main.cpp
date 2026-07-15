
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <poll.h>

#include <nuttx/sensors/icm42688p-fifo.h> // #include <nuttx/sensors/ioctl.h> included inside

// estimayor_main.cpp
extern "C" int estimator_main(int argc, char *argv[]);


int estimator_main(int argc, char *argv[])
{
    /* stdout is fully C-library buffered here (unlike the driver's syslog()
     * calls, which write straight to the console) — without this, printf()
     * output sits in the buffer forever since this loop never exits or
     * calls fflush().
     */
    setvbuf(stdout, NULL, _IONBF, 0);

    //Access IMU driver
    struct pollfd pfd;
    pfd.fd = open("/dev/imu0", O_RDONLY | O_NONBLOCK);

    if (pfd.fd < 0) {
         printf("estimator: failed to open /dev/imu0: %d\n", errno);
         return -1;
    }

    pfd.events = POLLIN; /* wake us when the ring buffer has data */

    // Create an array of samples to read from the ring buffer
    // The FIFO watermark is set to 50 by default (=sampling 641Hz)
    icm_fifo_sample_s sample_array[50];
    
    for(;;)
    {
      printf("estimator: Waiting for poll...\n");
      int ret = poll(&pfd, 1, -1);      /* block indefinitely for data */
      if (ret < 0)
        {
          printf("estimator: poll failed: %d\n", errno);
          break;
        }
 
      if (!(pfd.revents & POLLIN))
        {
          continue;
        }
      printf("estimator: Reading the samples...\n");
      ssize_t nread = read(pfd.fd, sample_array, sizeof(sample_array));
      if (nread < 0)
        {
          if (errno == EAGAIN)
            {
              continue;                 /* woke on a race; nothing to read yet */
            }
 
          printf("estimator: read failed: %d\n", errno);
          break;
        }
 
      int n = (int)(nread / (ssize_t)sizeof(icm_fifo_sample_s));
      if (n <= 0)
        {
          continue;
        }
 
      /* Convert the most recent sample to physical units.
       * Default full-scale after reset: accel +/-16 g -> 2048 LSB/g,
       * gyro +/-2000 dps -> 16.4 LSB/dps. FIFO temp is 8-bit.
       */
 
      const struct icm_fifo_sample_s *s = &sample_array[n - 1];
 
      float ax = s->accel_x / 2048.0f;
      float ay = s->accel_y / 2048.0f;
      float az = s->accel_z / 2048.0f;
      float gx = s->gyro_x  / 16.4f;
      float gy = s->gyro_y  / 16.4f;
      float gz = s->gyro_z  / 16.4f;
      float t  = s->temp    / 2.07f + 25.0f;
 
      printf("read %2d sample(s) | last: "
             "accel=[%7.3f %7.3f %7.3f] g  "
             "gyro=[%8.3f %8.3f %8.3f] dps  "
             "temp=%5.1f C  tmst=%u\n",
             n, ax, ay, az, gx, gy, gz, t, (unsigned)s->tmst);
      }
     
    close(pfd.fd);
    return 0;
}