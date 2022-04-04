#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include "i2cbus.h"

#ifdef eprintf
#undef eprintf
#endif

#define eprintf(str, ...)                                                                       \
    {                                                                                           \
        fprintf(stderr, "[%s/%s():%d] " str "\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__); \
        fflush(stderr);                                                                         \
    }

static int i2clock_initd = 0; /// Indicate that the I2C bus has not been initialized

#ifndef I2CBUS_MAX_NUM
#define I2CBUS_MAX_NUM 2 /// Maximum 2 /dev/i2cX
#endif
/**
 * @brief Set of mutexes for the I2C bus
 *
 */
pthread_mutex_t i2cbus_locks[I2CBUS_MAX_NUM];

int i2cbus_open(i2cbus *dev, int id, int addr)
{
    int ret = 0;
    char fname[256];
    if (i2clock_initd++ == 0) // only do it when the lock init is zero
    {
        pthread_mutexattr_t attr;
        ret = pthread_mutexattr_init(&attr);
        if (ret)
        {
            eprintf("Could not initialize mutex attribute");
            ret = -1;
            goto err;
        }
        ret = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        if (ret)
        {
            eprintf("Could not initialize mutex attribute");
            pthread_mutexattr_destroy(&attr);
            ret = -1;
            goto err;
        }
        for (int i = 0; i < I2CBUS_MAX_NUM; i++)
        {
            ret = pthread_mutex_init(&(i2cbus_locks[i]), &attr);
            if (ret)
            {
                eprintf("Failed to init mutex %d, ", i);
                perror("mutex init");
                pthread_mutexattr_destroy(&attr);
                ret = -2;
                goto err;
            }
        }
        pthread_mutexattr_destroy(&attr);
    }
    // check 1: memory
    if (dev == NULL)
    {
        eprintf("Error: Device descriptor is NULL");
        ret = -3;
        goto err;
    }
    // check 2: id > I2CBUS_MAX_NUM - 1
    if (!(id < I2CBUS_MAX_NUM))
    {
        eprintf("System supports up to %d ID, you provided %d.", I2CBUS_MAX_NUM - 1, id);
        ret = -4;
        goto err;
    }
    // check 3: addr valid range
    if (addr < 8)
    {
        fprintf(stderr, "%s: Address 0x%02x is invalid\n", __func__, addr);
        ret = -5;
        goto err;
    }

    // Try to open the file descriptor
    // step 1: Create file name
    if (snprintf(fname, 256, "/dev/i2c-%d", id) < 0)
    {
        eprintf("Failed to generate device filename using snprintf. FATAL Error!");
        ret = -6;
        goto err;
    }
    if ((dev->fd = open(fname, O_RDWR)) < 0)
    {
        eprintf("Failed to open %s. Error %d\n", fname, errno);
        ret = -errno;
        goto err;
    }
    if (ioctl(dev->fd, I2C_SLAVE, addr) < 0)
    {
        eprintf("Failed to open I2C slave address 0x%02x on bus %s with error %d, returning...", addr, fname, errno);
        close(dev->fd);
        ret = -errno;
        goto err;
    }
    // if we are here, then everything was successful
    dev->id = id;                    // assign device id
    dev->lock = &(i2cbus_locks[id]); // assign lock
    return dev->fd;
err:
    i2clock_initd--;
    return -1;
}

int i2cbus_close(i2cbus *dev)
{
    if (--i2clock_initd == 0) // only do it when the lock init is zero
    {
        for (int i = 0; i < I2CBUS_MAX_NUM; i++)
        {
            int ret = pthread_mutex_destroy(&(i2cbus_locks[i]));
            if (ret != 0)
            {
                eprintf("Failed to destroy mutex %d, ", i);
                perror("mutex destroy");
                return -1;
            }
        }
    }
    if (dev != NULL)
    {
        if (dev->fd > 0)
            return close(dev->fd);
    }
    else
    {
        eprintf("Invalid device descriptor");
        return -3;
    }
    // will never reach this point
    return -1;
}

#ifdef __GNUC__
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#else
#define likely(x) (x)
#define unlikely(x) (x)
#endif

int i2cbus_write(i2cbus *dev, void *buf, int len)
{
    // usual checks
    if (unlikely(dev == NULL || dev->fd < 0))
    {
        eprintf("Invalid device pointer %p or file descriptor %d", dev, dev->fd);
        return -1;
    }
    if (unlikely(buf == NULL))
    {
        eprintf("Invalid write buffer pointer NULL");
        return -1;
    }
    int status = pthread_mutex_lock(dev->lock);
    if (status)
    {
        eprintf("Mutex lock returned %d, error", status);
        return -1;
    }
    status = write(dev->fd, buf, len);
    if (status != len)
    {
#ifdef I2C_DEBUG
        eprintf("Failed to write %d bytes, wrote %d bytes, errno %d", len, status, errno);
#endif
    }
    pthread_mutex_unlock(dev->lock);
    return status;
}

int i2cbus_read(i2cbus *dev, void *buf, int len)
{
    // usual checks
    if (unlikely(dev == NULL || dev->fd < 0))
    {
        eprintf("Invalid device pointer %p or file descriptor %d", dev, dev->fd);
        return -1;
    }
    if (unlikely(buf == NULL))
    {
        eprintf("Invalid write buffer pointer NULL");
        return -1;
    }
    int status = pthread_mutex_lock(dev->lock);
    if (status)
    {
        eprintf("Mutex lock returned %d, error", status);
        return -1;
    }
    status = read(dev->fd, buf, len);
    if (status != len)
    {
#ifdef I2C_DEBUG
        eprintf("Failed to read %d bytes, read %d bytes, errno %d", len, status, errno);
#endif
    }
    pthread_mutex_unlock(dev->lock);
    return status;
}

int i2cbus_xfer(i2cbus *dev,
                void *outbuf, int outlen,
                void *inbuf, int inlen,
                unsigned long timeout_usec)
{
    // usual checks
    if (unlikely(dev == NULL || dev->fd < 0))
    {
        eprintf("Invalid device pointer %p or file descriptor %d", dev, dev->fd);
        return -1;
    }
    if (unlikely(outbuf == NULL))
    {
        eprintf("Invalid write buffer pointer NULL");
        return -1;
    }
    if (unlikely(inbuf == NULL))
    {
        eprintf("Invalid read buffer pointer NULL");
        return -1;
    }
    int status = pthread_mutex_lock(dev->lock);
    if (status)
    {
        eprintf("Mutex lock returned %d, error", status);
        return -1;
    }
#ifdef I2C_DEBUG
    eprintf("Sending %d bytes ->", outlen);
    for (int i = 0; i < outlen; i++)
    {
        fprintf(stderr, " %02xh", ((unsigned char *)outbuf)[i]);
    }
    eprintf("\n");
#endif
    status = write(dev->fd, outbuf, outlen);
    if (status != outlen)
    {
#ifdef I2C_DEBUG
        eprintf("Failed to write %d bytes, wrote %d bytes, errno %d", outlen, status, errno);
#endif
        goto ret;
    }
    if (timeout_usec > 0)
    {
        usleep(timeout_usec);
    }
    status = read(dev->fd, inbuf, inlen);
    if (status != inlen)
    {
#ifdef I2C_DEBUG
        eprintf("Failed to read %d bytes, read %d bytes, errno %d", inlen, status, errno);
#endif
        goto ret;
    }
#ifdef I2C_DEBUG
    eprintf("Receiving %d bytes ->", inlen);
    for (int i = 0; i < inlen; i++)
    {
        fprintf(stderr, " %02xh", ((unsigned char *)inbuf)[i]);
    }
    eprintf("\n");
#endif
ret:
    pthread_mutex_unlock(dev->lock);
    return status;
}

int i2cbus_lock(unsigned int bus)
{
    if (unlikely(bus >= I2CBUS_MAX_NUM))
    {
        eprintf("Bus index %d not supported, maximum is %d", bus, I2CBUS_MAX_NUM - 1);
        return -100;
    }
    int ret = pthread_mutex_lock(&(i2cbus_locks[bus]));
    if (ret)
        return -ret;
    return 1;
}

int i2cbus_trylock(unsigned int bus)
{
    if (unlikely(bus >= I2CBUS_MAX_NUM))
    {
        eprintf("Bus index %d not supported, maximum is %d", bus, I2CBUS_MAX_NUM - 1);
        return -100;
    }
    int ret = pthread_mutex_trylock(&(i2cbus_locks[bus]));
    if (ret)
        return -ret;
    return 1;
}

int i2cbus_unlock(unsigned int bus)
{
    if (unlikely(bus >= I2CBUS_MAX_NUM))
    {
        eprintf("Bus index %d not supported, maximum is %d", bus, I2CBUS_MAX_NUM - 1);
        return -100;
    }
    int ret = pthread_mutex_unlock(&(i2cbus_locks[bus]));
    if (ret)
        return -ret;
    return 1;
}