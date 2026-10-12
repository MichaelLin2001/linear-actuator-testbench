#ifndef UTILS_H
#define UTILS_H
#include "stdint.h"
#include "stdio.h"
#include <sys/time.h>

#define JC_LOG(fmt, ...) \
    do { \
        printf("%s(line %d): " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
    } while(0)

#define CLAMP(var, min, max) \
    do { \
        if ((var) > (max)) { \
            (var) = (max); \
        } else if ((var) < (min)) { \
            (var) = (min); \
        } \
    } while(0)

#define HIGH_BYTE(x)  ((uint8_t)(((x) >> 8) & 0xFF))
#define LOW_BYTE(x)   ((uint8_t)((x) & 0xFF))

#define RAD2ANGLE(rad) (rad/3.1415926f * 180)
#define ANGLE2RAD(angle) (angle/180.0f * 3.1415926)

#define PI 3.1415926f
#define NSEC_PER_SEC 1000000000L

#define TIMESPEC2NS(T) ((uint64_t)((T).tv_sec * NSEC_PER_SEC + (T).tv_nsec))

static inline struct timespec timespec_add(const struct timespec *t1,
                                           const struct timespec *t2)
{
    struct timespec result;
    result.tv_sec  = t1->tv_sec + t2->tv_sec;
    result.tv_nsec = t1->tv_nsec + t2->tv_nsec;

    if (result.tv_nsec >= NSEC_PER_SEC) {
        result.tv_nsec -= NSEC_PER_SEC;
        result.tv_sec  += 1;
    }

    return result;
}

static inline struct timespec timespec_minus(const struct timespec *t1,
                                             const struct timespec *t2)
{
    struct timespec result;
    uint64_t time1_ns = TIMESPEC2NS(*t1);
    uint64_t time2_ns = TIMESPEC2NS(*t2);

    if (time1_ns < time2_ns) {
        result.tv_sec  = 0;
        result.tv_nsec = 0;
    } else {
        uint64_t diff_ns = time1_ns - time2_ns;
        result.tv_sec  = diff_ns / NSEC_PER_SEC;
        result.tv_nsec = diff_ns % NSEC_PER_SEC;
    }

    return result;
}

typedef union
{
    float to_float;
    int to_int;
    uint32_t to_uint;
    uint8_t buf[4];
} IntConvert;

typedef union
{
    int16_t to_int16;
    uint16_t to_uint16;
    uint8_t buf[2];
} ShortConvert;

void dump_buffer(const uint8_t* buffer, uint32_t length);
uint16_t modbus_crc16(const uint8_t *data, size_t length);
#endif //UTILS_H