
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>

#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ioctl.h>

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

    //Access IMU driver uORB topics
    struct pollfd pfd[2];

    pfd[0].fd = open("/dev/uorb/sensor_accel0", O_RDONLY | O_NONBLOCK);
    if (pfd[0].fd < 0) {
         printf("estimator: failed to open /dev/uorb/sensor_accel0: %d\n", errno);
         return -1;
    }

    pfd[1].fd = open("/dev/uorb/sensor_gyro0", O_RDONLY | O_NONBLOCK);
    if (pfd[1].fd < 0) {
         printf("estimator: failed to open /dev/uorb/sensor_gyro0: %d\n", errno);
         close(pfd[0].fd);
         return -1;
    }

    /* Request the same 1kHz cadence the chip defaults to after reset.
     * The driver may round up; it reports the actual achieved period
     * back into period_us, which we don't otherwise need here.
     */

    uint32_t period_us = 1000;
    ioctl(pfd[0].fd, SNIOC_SET_INTERVAL, (unsigned long)&period_us);
    period_us = 1000;
    ioctl(pfd[1].fd, SNIOC_SET_INTERVAL, (unsigned long)&period_us);

    pfd[0].events = POLLIN;
    pfd[1].events = POLLIN;

    // Create arrays of samples to read from each topic's uORB buffer.
    // The FIFO watermark is set to 50 by default (=sampling 641Hz)
    struct sensor_accel accel_array[50];
    struct sensor_gyro gyro_array[50];

    for(;;)
    {
      printf("estimator: Waiting for poll...\n");
      int ret = poll(pfd, 2, -1);      /* block indefinitely for data */
      if (ret < 0)
        {
          printf("estimator: poll failed: %d\n", errno);
          break;
        }

      if (pfd[0].revents & POLLIN)
        {
          ssize_t nread = read(pfd[0].fd, accel_array, sizeof(accel_array));
          if (nread < 0 && errno != EAGAIN)
            {
              printf("estimator: accel read failed: %d\n", errno);
              break;
            }

          int n = (int)(nread / (ssize_t)sizeof(struct sensor_accel));
          if (n > 0)
            {
              /* Only the most recent sample of the batch is used. */

              const struct sensor_accel *s = &accel_array[n - 1];

              printf("read %2d accel sample(s) | last: "
                     "accel=[%7.3f %7.3f %7.3f] m/s^2  "
                     "temp=%5.1f C  ts=%llu\n",
                     n, s->x, s->y, s->z, s->temperature,
                     (unsigned long long)s->timestamp);
            }
        }

      if (pfd[1].revents & POLLIN)
        {
          ssize_t nread = read(pfd[1].fd, gyro_array, sizeof(gyro_array));
          if (nread < 0 && errno != EAGAIN)
            {
              printf("estimator: gyro read failed: %d\n", errno);
              break;
            }

          int n = (int)(nread / (ssize_t)sizeof(struct sensor_gyro));
          if (n > 0)
            {
              /* Only the most recent sample of the batch is used. */

              const struct sensor_gyro *s = &gyro_array[n - 1];

              printf("read %2d gyro sample(s)  | last: "
                     "gyro=[%8.3f %8.3f %8.3f] rad/s  "
                     "temp=%5.1f C  ts=%llu\n",
                     n, s->x, s->y, s->z, s->temperature,
                     (unsigned long long)s->timestamp);
            }
        }
    }

    close(pfd[0].fd);
    close(pfd[1].fd);
    return 0;
}
