/**
 * @file i2cbus.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-11-11
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __I2CBUS_H
#define __I2CBUS_H
#ifdef __cplusplus 
extern "C" {
#endif
#include <pthread.h>

/**
 * @brief Structure describing an I2C bus.
 * 
 */
typedef struct
{
    int fd;                ///< I2C device file descriptor
    int id;                ///< I2C device file id (X in /dev/i2c-X)
    pthread_mutex_t *lock; ///< Lock corresponding to the /dev/i2c-X file, assigned from the locks array indexed by id
} i2cbus;
/**
 * @brief Open an I2C bus file descriptor using the supplied parameters.
 * 
 * 
 * @param dev i2c device descriptor
 * @param id i2c device file ID (X in /dev/i2c-X)
 * @param addr i2c slave address
 * @return int fd, non-negative on success, negative on error. See open() for details.
 */
int i2cbus_open(i2cbus *dev, int id, int addr);
/**
 * @brief Close the file descriptor for the I2C device. 
 * 
 * @param dev i2c device descriptor.
 * @returns (int) Return code from close() or negative on error.
 */
int i2cbus_close(i2cbus *dev);
/**
 * @brief Write bytes to the i2c device.
 * Note: Bus access by this function is protected by a recursive
 * pthread mutex.
 * 
 * @param dev i2c device descriptor
 * @param buf Pointer to byte array to write (MSB first)
 * @param len Length of byte array
 * @return int Length of bytes written on success, -1 on failure
 */
int i2cbus_write(i2cbus *dev, void *buf, int len);
/**
 * @brief Read bytes from the i2c device.
 * Note: Bus access by this function is protected by a recursive
 * pthread mutex.
 * 
 * @param dev i2c device descriptor
 * @param buf Pointer to byte array to read to (MSB first)
 * @param len Length of byte array
 * @return int Length of bytes read on success, -1 on failure
 */
int i2cbus_read(i2cbus *dev, void *buf, int len);
/**
 * @brief Function to do a write, and get the reply in one operation
 * in order to avoid read/write mangling with multiple threads 
 * accessing the bus. Avoid using this function if you have read or
 * write buffer lengths zero.
 * 
 * Note: Bus access by this function is protected by a recursive
 * pthread mutex.
 * 
 * @param dev i2c device descriptor
 * @param outbuf Pointer to byte array to write (MSB first)
 * @param outlen Length of output byte array
 * @param inbuf Pointer to byte array to read to (MSB first), can be the same as outbuf
 * @param inlen Length of input byte array
 * @param timeout_usec Timeout between read and write (in microseconds)
 * @return int Length of bytes read on success, -1 on failure
 */
int i2cbus_xfer(i2cbus *dev,
                void *outbuf, int outlen,
                void *inbuf, int inlen,
                unsigned long timeout_usec);
/**
 * @brief Acquire lock on an i2c bus.
 * 
 * @param bus Bus index (X in /dev/i2c-X)
 * @return int Positive on success, negative on error (negative of error returned by pthread_mutex_lock)
 */
int i2cbus_lock(unsigned int bus);
/**
 * @brief Try to acquire lock on an i2c bus,
 * for timing jitter sensitive applications.
 * 
 * @param bus Bus index (X in /dev/i2c-X)
 * @return int Positive on success, negative on error (negative of error returned by pthread_mutex_trylock)
 */
int i2cbus_trylock(unsigned int bus);
/**
 * @brief Unlock an i2c bus.
 * 
 * @param bus Bus index (X in /dev/i2c-X)
 * @return int int Positive on success, negative on error (negative of error returned by pthread_mutex_lock)
 */
int i2cbus_unlock(unsigned int bus);
#ifdef __cplusplus 
}
#endif
#endif