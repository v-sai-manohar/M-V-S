/*
 * common.h  --  QNX Edge Analytics  PS#34  (v5 -- hardware-only / simulation)
 * =============================================================================
 * Shared types, constants, SPSC rings, and declarations.
 * Supports both real hardware and simulation mode (--simulate flag).
 * =============================================================================
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>

/* QNX-specific */
#include <sys/neutrino.h>   /* InterruptDisable/Enable, ThreadCtl, MsgSend etc. */
#include <sys/sched.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  THREAD PRIORITIES  (QNX SCHED_RR, range 1-63)
 *
 *  Sensors are HIGHEST: a DHT22 or MPU6050 read must never be preempted
 *  by analytics or server, which would corrupt microsecond-level timing.
 *
 *  Analytics below sensors: reads from ring after sensors have pushed.
 *  Transmission below analytics: runs after analytics sends IPC result.
 *  Server lowest: serves HTTP at human-perceived 500ms poll intervals.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define PRIORITY_SENSOR     22
#define PRIORITY_HIGH       20
#define PRIORITY_MEDIUM     15
#define PRIORITY_LOW        10

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  SIMULATION MODE FLAG
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
extern volatile int g_simulation_mode;  /* 1 = simulate sensors, 0 = real hardware */

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  TIMING
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define DHT22_SAMPLE_INTERVAL_MS     2000   /* DHT22: 1 read every 2 s       */
#define MPU6050_SAMPLE_INTERVAL_MS    100   /* MPU6050: 10 reads per second   */
#define ANALYTICS_WINDOW_MS          2000   /* Analytics window: 2 s          */

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  SPSC RING BUFFER SIZES  (must be powers of 2)
 *
 *  DHT22   : 1 sample/2s -> 32 slots = 64 seconds of backlog
 *  MPU6050 : 10 samples/s -> 128 slots = 12.8 seconds of backlog
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define DHT22_RING_SIZE      32
#define MPU6050_RING_SIZE   128

/* legacy alias used in a few places */
#define DHT22_BUFFER_SIZE    DHT22_RING_SIZE
#define MPU6050_BUFFER_SIZE  MPU6050_RING_SIZE

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  EDGE ANALYTICS THRESHOLDS
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define TEMP_HIGH_THRESHOLD        35.0f   /* deg C */
#define TEMP_LOW_THRESHOLD         15.0f   /* deg C */
#define HUMIDITY_HIGH_THRESHOLD    80.0f   /* %     */
#define ACCEL_SPIKE_THRESHOLD       2.0f   /* g     */
#define GYRO_SPIKE_THRESHOLD      100.0f   /* deg/s */
#define VIBRATION_FRACTION         0.20f   /* >20% of MPU samples in window  */
#define MOTION_FRACTION            0.10f   /* >10% of MPU samples in window  */

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  HTTP SERVER
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define SERVER_PORT          8080
#define SERVER_MAX_CLIENTS      5
#define SERVER_JSON_BUF      4096
#define SERVER_HTTP_BUF      4608

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  IPC MESSAGE TYPES
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define MSG_TYPE_ANALYTICS_RESULT   0x01
#define MSG_TYPE_SHUTDOWN           0xFF

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  SENSOR DATA STRUCTURES
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

/* One DHT22 reading */
typedef struct {
    float           temperature;    /* deg C  */
    float           humidity;       /* %      */
    struct timespec timestamp;      /* CLOCK_REALTIME at sample time */
    int             valid;          /* 1 = checksum passed + in range */
} dht22_data_t;

/* One MPU6050 reading (raw values already converted to engineering units) */
typedef struct {
    float           accel_x;        /* g      */
    float           accel_y;        /* g      */
    float           accel_z;        /* g      */
    float           gyro_x;         /* deg/s  */
    float           gyro_y;         /* deg/s  */
    float           gyro_z;         /* deg/s  */
    float           temp_onboard;   /* deg C  */
    struct timespec timestamp;
    int             valid;
} mpu6050_data_t;

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  LOCK-FREE SPSC RING BUFFERS
 *
 *  Single-Producer Single-Consumer ring buffer using C11 _Atomic indices.
 *
 *  Producer (sensor thread):
 *    - Reads tail (relaxed) to find write slot
 *    - Writes data into buf[tail]
 *    - Stores new tail (release) to publish the sample
 *    - NEVER touches head
 *
 *  Consumer (analytics thread):
 *    - Reads head (relaxed) to find read slot
 *    - Loads new tail (acquire) to check if data is available
 *    - Copies buf[head] out
 *    - Stores new head (release) to free the slot
 *    - NEVER touches tail
 *
 *  Cache-line padding (60 bytes) between head and tail prevents false
 *  sharing -- without it, each atomic access on one field bounces the
 *  entire 64-byte cache line between CPU cores.
 *
 *  No mutex. No semaphore. No blocking in the hot path.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

typedef struct {
    _Atomic uint32_t head;
    char             _pad_h[60];
    _Atomic uint32_t tail;
    char             _pad_t[60];
    dht22_data_t     buf[DHT22_RING_SIZE];
} dht22_spsc_t;

typedef struct {
    _Atomic uint32_t head;
    char             _pad_h[60];
    _Atomic uint32_t tail;
    char             _pad_t[60];
    mpu6050_data_t   buf[MPU6050_RING_SIZE];
} mpu6050_spsc_t;

/* Push -- called by producer (sensor thread). Returns 0 on success, -1 if full. */
static inline int dht22_spsc_push(dht22_spsc_t *r, const dht22_data_t *s)
{
    uint32_t t    = atomic_load_explicit(&r->tail, memory_order_relaxed);
    uint32_t next = (t + 1u) & (DHT22_RING_SIZE - 1u);
    if (next == atomic_load_explicit(&r->head, memory_order_acquire))
        return -1; /* full */
    r->buf[t] = *s;
    atomic_store_explicit(&r->tail, next, memory_order_release);
    return 0;
}

static inline int mpu6050_spsc_push(mpu6050_spsc_t *r, const mpu6050_data_t *s)
{
    uint32_t t    = atomic_load_explicit(&r->tail, memory_order_relaxed);
    uint32_t next = (t + 1u) & (MPU6050_RING_SIZE - 1u);
    if (next == atomic_load_explicit(&r->head, memory_order_acquire))
        return -1;
    r->buf[t] = *s;
    atomic_store_explicit(&r->tail, next, memory_order_release);
    return 0;
}

/* Pop -- called by consumer (analytics thread). Returns 0 on success, -1 if empty. */
static inline int dht22_spsc_pop(dht22_spsc_t *r, dht22_data_t *out)
{
    uint32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
    if (h == atomic_load_explicit(&r->tail, memory_order_acquire))
        return -1; /* empty */
    *out = r->buf[h];
    atomic_store_explicit(&r->head, (h + 1u) & (DHT22_RING_SIZE - 1u),
                           memory_order_release);
    return 0;
}

static inline int mpu6050_spsc_pop(mpu6050_spsc_t *r, mpu6050_data_t *out)
{
    uint32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
    if (h == atomic_load_explicit(&r->tail, memory_order_acquire))
        return -1;
    *out = r->buf[h];
    atomic_store_explicit(&r->head, (h + 1u) & (MPU6050_RING_SIZE - 1u),
                           memory_order_release);
    return 0;
}

/* Approximate count of readable entries (no lock, may be stale by 1) */
static inline uint32_t dht22_spsc_count(const dht22_spsc_t *r)
{
    uint32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
    uint32_t t = atomic_load_explicit(&r->tail, memory_order_relaxed);
    return (t - h) & (DHT22_RING_SIZE - 1u);
}
static inline uint32_t mpu6050_spsc_count(const mpu6050_spsc_t *r)
{
    uint32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
    uint32_t t = atomic_load_explicit(&r->tail, memory_order_relaxed);
    return (t - h) & (MPU6050_RING_SIZE - 1u);
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  SHARED SENSOR BUFFER  (two SPSC rings)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
typedef struct {
    dht22_spsc_t   dht22;
    mpu6050_spsc_t mpu6050;
} sensor_buffer_t;

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  ANALYTICS RESULT  (one per 2-second window)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
typedef struct {
    /* DHT22 window statistics */
    float    avg_temp;
    float    min_temp;
    float    max_temp;
    float    avg_humidity;
    float    min_humidity;
    float    max_humidity;
    int      dht22_samples;
    int      temp_alert;        /*  1=HIGH  -1=LOW  0=NORMAL */
    int      humidity_alert;    /*  1=HIGH   0=NORMAL        */

    /* MPU6050 window statistics */
    float    avg_accel_mag;
    float    max_accel_mag;
    float    avg_gyro_mag;
    float    max_gyro_mag;
    float    avg_mpu_temp;
    int      vibration_detected;
    int      motion_detected;
    int      mpu6050_samples;

    /* Window metadata */
    struct timespec window_start;
    struct timespec window_end;
    uint32_t        sequence;
} analytics_result_t;

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  QNX IPC MESSAGES  (Analytics -> Transmission via MsgSend/MsgReceive)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
typedef struct {
    uint8_t            type;
    analytics_result_t result;
} ipc_msg_t;

typedef struct {
    uint8_t status;
} ipc_reply_t;

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  SERVER SHARED STATE
 *
 *  Two separate update paths, two different protection mechanisms:
 *
 *  (A) Analytics result + TX stats  -- updated at 0.5 Hz
 *      Protected by g_server_mutex (mutex is fine at 0.5 Hz)
 *
 *  (B) Raw sensor values            -- updated at 10 Hz (MPU6050)
 *      Protected by _Atomic float   (single-instruction, never blocks)
 *
 *  Without (B): MPU6050 thread (P22) locks g_server_mutex at 10 Hz.
 *  Server thread (P10) also holds g_server_mutex during snprintf() (~200 us).
 *  P22 blocks waiting for P10 -- priority inversion 10x per second.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
typedef struct {
    /* (A) mutex-protected -- updated at 0.5 Hz */
    analytics_result_t  result;
    int                 ready;
    uint64_t            tx_sent;
    uint64_t            tx_failed;
    uint64_t            tx_bytes;
    uint32_t            tx_seq;

    /* (B) atomic -- updated at 10 Hz, zero blocking */
    _Atomic float       raw_temp;
    _Atomic float       raw_humidity;
    _Atomic float       raw_accel_mag;
    _Atomic float       raw_gyro_mag;
    _Atomic float       raw_mpu_temp;
} server_state_t;

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  GLOBALS  (defined in main.c / server.c)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
extern sensor_buffer_t  g_sensor_buf;
extern volatile int     g_running;
extern int              g_analytics_chid;
extern server_state_t   g_server_state;
extern pthread_mutex_t  g_server_mutex;

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  INLINE HELPERS
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

/* Sleep for ms milliseconds using nanosleep (releases CPU -- correct for long waits) */
static inline void sleep_ms(long ms)
{
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* Timestamp (wall clock) */
static inline void get_time(struct timespec *ts)
{
    clock_gettime(CLOCK_REALTIME, ts);
}

/* Vector magnitude */
static inline float vec3_mag(float x, float y, float z)
{
    return sqrtf(x*x + y*y + z*z);
}
/* alias for legacy callers */
static inline float vec3_magnitude(float x, float y, float z)
{
    return vec3_mag(x, y, z);
}

/* Clamp */
static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}


void *sensor_dht22_thread   (void *arg);
void *sensor_mpu6050_thread (void *arg);
void *analytics_thread      (void *arg);
void *transmission_thread   (void *arg);
void *server_thread         (void *arg);

void server_update_result  (const analytics_result_t *r);
void server_update_tx_stats(uint64_t sent, uint64_t failed,
                             uint64_t bytes, uint32_t seq);
/* Pass -1.0f for fields you do NOT want to overwrite */
void server_update_raw     (float temp, float hum,
                             float accel, float gyro, float mpu_temp);

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  LOGGING API  (thread-safe, defined in logger.c)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
void log_info  (const char *src, const char *fmt, ...);
void log_warn  (const char *src, const char *fmt, ...);
void log_error (const char *src, const char *fmt, ...);
void log_alert (const char *src, const char *fmt, ...);

#endif /* COMMON_H_ */

