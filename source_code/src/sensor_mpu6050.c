/*
 * sensor_mpu6050.c  --  QNX Edge Analytics  PS#34  (v5 -- hardware-only)
 * =============================================================================
 * REAL-TIME MPU6050 SENSOR DRIVER FOR QNX RTOS
 *
 * Hardware: Raspberry Pi 4, QNX SDP 8.0
 * Bus     : I2C1  (/dev/i2c1)  SDA=GPIO2 (pin 3), SCL=GPIO3 (pin 5)
 * Address : 0x68  (AD0 tied to GND)
 *
 * CONFIGURATION FOR REAL-TIME RESPONSE
 * -------------------------------------
 *  Clock     : PLL with gyro X reference (PWR_MGMT_1=0x01)
 *              More stable than internal 8 MHz RC oscillator.
 *
 *  DLPF      : 188 Hz bandwidth, 1 ms group delay (CONFIG=0x01)
 *              Removes high-frequency noise without adding lag.
 *              Does NOT smooth out real motion events.
 *
 *  Output rate: 100 Hz (SMPLRT_DIV=9 â†’ 1000/(9+1)=100 Hz)
 *              Matches our 100 ms poll interval exactly.
 *              MPU6050 data register always has fresh data when we read.
 *
 *  FS ranges : Â±2 g accel, Â±250 Â°/s gyro (highest resolution)
 *
 *  I2C speed : 400 kHz Fast Mode
 *              14-byte burst at 100 kHz â‰ˆ 1400 Âµs
 *              14-byte burst at 400 kHz â‰ˆ  350 Âµs
 *              Saves 1050 Âµs per read = 10.5 ms/sec at 10 Hz.
 *
 * RTOS DESIGN
 * -----------
 *  Priority       : PRIORITY_SENSOR (22) -- highest in system
 *  Scheduling     : SCHED_RR
 *  Timing         : clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)
 *  Shared buffer  : Lock-free SPSC ring (mpu6050_spsc_t)
 *  I2C            : Single combined WRITE+READ transaction per sample
 * =============================================================================
 */

#include "common.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <devctl.h>
#include <hw/i2c.h>

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  I2C / MPU6050 constants
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define I2C_BUS             "/dev/i2c1"
#define MPU6050_ADDR_PRIMARY   0x68
#define MPU6050_ADDR_ALT       0x69
#define I2C_SPEED_HZ        100000U     /* 100 kHz Fast Mode */

/* Register map */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B        /* first of 14 data bytes */
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75       /* typically 0x68 (or 0x69 with AD0 high) */

/* Scale factors for default FS ranges */
#define ACCEL_LSB_PER_G     16384.0f    /* Â±2 g  â†’ 16384 LSB/g   */
#define GYRO_LSB_PER_DPS      131.0f    /* Â±250Â°/s â†’ 131 LSB/(Â°/s) */

static int s_i2c_fd = -1;
static uint8_t s_mpu_addr = MPU6050_ADDR_PRIMARY;
static int s_recv_offset = 1;
#ifdef DCMD_I2C_SET_SLAVE_ADDR
static int s_addr_set_warned = 0;
#endif

/*
 * i2c_burst_read()
 * Performs a combined WRITE-then-READ in ONE I2C transaction:
 *   START  ADDR+W  REG  RESTART  ADDR+R  data[0..len-1]  STOP
 *
 * Two separate transactions (old approach) add an extra START/STOP
 * overhead and introduce a small gap during which the MPU6050 could
 * update its registers -- causing a torn read.
 */

static int i2c_burst_read(uint8_t reg, uint8_t *out, uint16_t len)
{
    struct {
        i2c_sendrecv_t hdr;
        uint8_t        data[64];
    } msg;

    if (len > 32u) return -1;

    memset(&msg, 0, sizeof(msg));
    msg.hdr.slave.addr = s_mpu_addr;
    msg.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg.hdr.send_len   = 1;
    msg.hdr.recv_len   = len;
    msg.hdr.stop       = 1;
    msg.data[0]        = reg;

    if (devctl(s_i2c_fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL) != EOK)
        return -1;

    memcpy(out, msg.data + s_recv_offset, len);
    return 0;
}

static void i2c_try_set_slave_addr(uint8_t addr)
{
#ifdef DCMD_I2C_SET_SLAVE_ADDR
    i2c_addr_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.addr = addr;
    cfg.fmt  = I2C_ADDRFMT_7BIT;

    if (devctl(s_i2c_fd, DCMD_I2C_SET_SLAVE_ADDR, &cfg, sizeof(cfg), NULL) != EOK) {
        if (!s_addr_set_warned) {
            log_warn("MPU6050",
                     "DCMD_I2C_SET_SLAVE_ADDR not accepted (%s) -- using per-message address",
                     strerror(errno));
            s_addr_set_warned = 1;
        }
    }
#else
    (void)addr;
#endif
}

static int i2c_write_reg(uint8_t reg, uint8_t val)
{
    struct {
        i2c_send_t hdr;
        uint8_t    data[2];
    } msg;

    memset(&msg, 0, sizeof(msg));
    msg.hdr.slave.addr = s_mpu_addr;
    msg.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg.hdr.len        = 2;
    msg.hdr.stop       = 1;
    msg.data[0]        = reg;
    msg.data[1]        = val;

    return (devctl(s_i2c_fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL) == EOK) ? 0 : -1;
}

/*
 * Some QNX BSP I2C resource managers place received bytes at data[0],
 * others at data[1]. Probe both offsets and both common MPU addresses.
 */
static int mpu_probe_addr_and_offset(uint8_t *who_out)
{
    const uint8_t addrs[2] = { MPU6050_ADDR_PRIMARY, MPU6050_ADDR_ALT };

    for (int a = 0; a < 2; a++) {
        struct {
            i2c_sendrecv_t hdr;
            uint8_t        data[16];
        } probe;

        memset(&probe, 0, sizeof(probe));
        i2c_try_set_slave_addr(addrs[a]);
        probe.hdr.slave.addr = addrs[a];
        probe.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
        probe.hdr.send_len   = 1;
        probe.hdr.recv_len   = 1;
        probe.hdr.stop       = 1;
        probe.data[0]        = REG_WHO_AM_I;

        if (devctl(s_i2c_fd, DCMD_I2C_SENDRECV, &probe, sizeof(probe), NULL) != EOK)
            continue;

        for (int i = 0; i < 16; i++) {
            if (probe.data[i] == 0x68 || probe.data[i] == 0x69) {
                s_mpu_addr = addrs[a];
                s_recv_offset = i;
                *who_out = probe.data[i];
                log_info("MPU6050",
                         "Probe OK  addr=0x%02X recv_offset=%d who=0x%02X",
                         s_mpu_addr, s_recv_offset, *who_out);
                return 0;
            }
        }
    }

    return -1;
}

/* Big-endian 16-bit signed integer from two bytes */
static inline int16_t be16s(const uint8_t *b)
{
    return (int16_t)((uint16_t)b[0] << 8 | b[1]);
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  mpu6050_init()
 *  Opens I2C bus, sets 400 kHz, wakes MPU6050, configures DLPF + sample rate.
 *  Returns 0 on success, -1 on failure. In simulation mode, always succeeds.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static int mpu6050_init(void)
{
    extern volatile int g_simulation_mode;  /* Simulation flag from main.c */
    
    /* In simulation mode, skip all hardware init */
    if (g_simulation_mode) {
        log_info("MPU6050", "Simulation mode: init skipped, will generate synthetic data");
        return 0;
    }

    s_i2c_fd = open(I2C_BUS, O_RDWR);
    if (s_i2c_fd < 0) {
        log_error("MPU6050", "open %s: %s", I2C_BUS, strerror(errno));
        return -1;
    }

    /* Set 400 kHz Fast Mode */
    uint32_t spd = I2C_SPEED_HZ;
    if (devctl(s_i2c_fd, DCMD_I2C_SET_BUS_SPEED, &spd, sizeof(spd), NULL) != EOK)
        log_warn("MPU6050", "400kHz failed (%s) -- using default 100kHz",
                 strerror(errno));
    else
        log_info("MPU6050", "I2C speed = 400 kHz");

    /* WHO_AM_I + BSP receive-offset auto-detection */
    uint8_t who = 0;
    if (mpu_probe_addr_and_offset(&who) != 0) {
        log_error("MPU6050",
                  "Probe failed: no valid WHO_AM_I on 0x68/0x69. Check wiring, AD0, power, pull-ups");
        close(s_i2c_fd);
        s_i2c_fd = -1;
        return -1;
    }

    /* Confirm via normal read path after offset detection */
    i2c_try_set_slave_addr(s_mpu_addr);
    if (i2c_burst_read(REG_WHO_AM_I, &who, 1) != 0 ||
        (who != 0x68 && who != 0x69)) {
        log_error("MPU6050", "WHO_AM_I confirm failed (0x%02X)", who);
        close(s_i2c_fd);
        s_i2c_fd = -1;
        return -1;
    }
    log_info("MPU6050", "WHO_AM_I confirm OK: 0x%02X", who);

    /* Wake device, select PLL+gyroX clock (stable) */
    if (i2c_write_reg(REG_PWR_MGMT_1, 0x01) != 0) {
        log_error("MPU6050", "Failed to write PWR_MGMT_1");
        close(s_i2c_fd);
        s_i2c_fd = -1;
        return -1;
    }
    sleep_ms(50);   /* PLL lock time */

    if (i2c_write_reg(REG_CONFIG,       0x01) != 0 ||
        i2c_write_reg(REG_SMPLRT_DIV,   9)    != 0 ||
        i2c_write_reg(REG_GYRO_CONFIG,  0x00) != 0 ||
        i2c_write_reg(REG_ACCEL_CONFIG, 0x00) != 0) {
        log_error("MPU6050", "Failed to configure MPU6050 registers");
        close(s_i2c_fd);
        s_i2c_fd = -1;
        return -1;
    }

    log_info("MPU6050",
              "Init OK  addr=0x%02X  DLPF=188Hz  rate=100Hz  "
              "accel=+/-2g  gyro=+/-250dps  i2c=400kHz",
              s_mpu_addr);
    return 0;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  mpu6050_sim_read_sample()
 *  Generates simulated MPU6050 data for testing without hardware.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static mpu6050_data_t mpu6050_sim_read_sample(void)
{
    static uint32_t sample_count = 0;
    mpu6050_data_t d = {0};

    sample_count++;

    /* Simulate sinusoidal acceleration (gentle motion) */
    float t = (float)(sample_count % 100) / 10.0f;  /* 0-10 Hz oscillation */
    d.accel_x = 0.5f * sinf(t);      /* Â±0.5g */
    d.accel_y = 0.3f * cosf(t * 1.5f);
    d.accel_z = 1.0f + 0.2f * sinf(t * 2.0f);  /* ~1g vertical + ripple */

    /* Simulate gyroscope (slight rotation) */
    d.gyro_x = 10.0f * sinf(t * 0.5f);    /* Â±10 deg/s */
    d.gyro_y = 5.0f * cosf(t);
    d.gyro_z = 2.0f * sinf(t * 1.2f);

    /* Simulate die temperature (stable, around 35Â°C) */
    d.temp_onboard = 35.0f + 0.5f * sinf(t * 0.3f);

    clock_gettime(CLOCK_REALTIME, &d.timestamp);
    d.valid = 1;

    return d;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  mpu6050_read_sample()
 *  Reads 14 bytes (accel XYZ + temp + gyro XYZ) in a single burst.
 *  Converts raw counts to engineering units.
 *  In simulation mode, generates synthetic data.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static mpu6050_data_t mpu6050_read_sample(void)
{
    extern volatile int g_simulation_mode;

    /* In simulation mode, return synthetic data */
    if (g_simulation_mode) {
        return mpu6050_sim_read_sample();
    }

    mpu6050_data_t d = {0};
    uint8_t raw[14];

    if (i2c_burst_read(REG_ACCEL_XOUT_H, raw, 14) != 0) {
        log_warn("MPU6050", "I2C burst read failed");
        return d;   /* d.valid = 0 */
    }

    /*
     * All-zero frame is a strong indicator of bus/configuration issue.
     * Treat as invalid instead of streaming fake-looking zeros.
     */
    int all_zero = 1;
    for (int i = 0; i < 14; i++) {
        if (raw[i] != 0) { all_zero = 0; break; }
    }
    if (all_zero) {
        log_warn("MPU6050", "All-zero frame received -- ignoring sample (check wiring/power/i2c driver)");
        return d;
    }

    /*
     * Register map at ACCEL_XOUT_H:
     *   [0-1]  ACCEL_XOUT  big-endian int16
     *   [2-3]  ACCEL_YOUT
     *   [4-5]  ACCEL_ZOUT
     *   [6-7]  TEMP_OUT
     *   [8-9]  GYRO_XOUT
     *   [10-11] GYRO_YOUT
     *   [12-13] GYRO_ZOUT
     */
    d.accel_x      = (float)be16s(&raw[0])  / ACCEL_LSB_PER_G;
    d.accel_y      = (float)be16s(&raw[2])  / ACCEL_LSB_PER_G;
    d.accel_z      = (float)be16s(&raw[4])  / ACCEL_LSB_PER_G;
    d.temp_onboard = (float)be16s(&raw[6])  / 340.0f + 36.53f;   /* MPU6050 formula */
    d.gyro_x       = (float)be16s(&raw[8])  / GYRO_LSB_PER_DPS;
    d.gyro_y       = (float)be16s(&raw[10]) / GYRO_LSB_PER_DPS;
    d.gyro_z       = (float)be16s(&raw[12]) / GYRO_LSB_PER_DPS;
    d.valid        = 1;
    get_time(&d.timestamp);
    return d;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  spsc_push_safe()
 *  Lock-free push. If ring is full, evict oldest (fresh data > stale data).
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static void spsc_push_safe(mpu6050_spsc_t *r, const mpu6050_data_t *s)
{
    if (mpu6050_spsc_push(r, s) == 0) return;

    uint32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
    atomic_store_explicit(&r->head,
                          (h + 1u) & (MPU6050_RING_SIZE - 1u),
                          memory_order_release);
    mpu6050_spsc_push(r, s);
    log_warn("MPU6050", "Ring full -- oldest sample evicted");
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  sensor_mpu6050_thread()
 *
 *  RTOS periodic task, 100 ms period.
 *  Reads one 14-byte MPU6050 sample per period via I2C burst.
 *  Pushes to lock-free SPSC ring for analytics.
 *  Updates HTTP server raw state via atomic store (non-blocking).
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
void *sensor_mpu6050_thread(void *arg)
{
    (void)arg;

    sigset_t mask;
    sigfillset(&mask);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);

    struct sched_param sp = { .sched_priority = PRIORITY_SENSOR };
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);

    if (mpu6050_init() != 0) {
        log_error("MPU6050", "Hardware init failed -- thread cannot start");
        return NULL;
    }

    log_info("MPU6050",
             "STARTED  prio=%d  period=%dms  "
             "DLPF=188Hz  rate=100Hz  i2c=400kHz  real_values_only",
             PRIORITY_SENSOR, MPU6050_SAMPLE_INTERVAL_MS);

    struct timespec next_wake;
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    int log_skip = 0;

    while (g_running) {

        /* Advance absolute deadline BEFORE reading */
        next_wake.tv_nsec += (long)MPU6050_SAMPLE_INTERVAL_MS * 1000000L;
        if (next_wake.tv_nsec >= 1000000000L) {
            next_wake.tv_sec  += next_wake.tv_nsec / 1000000000L;
            next_wake.tv_nsec  = next_wake.tv_nsec % 1000000000L;
        }

        /* Read real sensor data */
        mpu6050_data_t s = mpu6050_read_sample();

        if (s.valid) {
            float am = vec3_mag(s.accel_x, s.accel_y, s.accel_z);
            float gm = vec3_mag(s.gyro_x,  s.gyro_y,  s.gyro_z);

            /* Push to analytics pipeline (lock-free) */
            spsc_push_safe(&g_sensor_buf.mpu6050, &s);

            /* Update HTTP server raw state (atomic store, non-blocking) */
            server_update_raw(-1.0f, -1.0f, am, gm, s.temp_onboard);

            /* Log every 10th sample (1 s) -- avoids console flood */
            if ((log_skip++ % 10) == 0) {
                log_info("MPU6050",
                         "|a|=%6.3f g   |w|=%7.2f dps   die=%.1f C",
                         am, gm, s.temp_onboard);
            }
        }

        /* Sleep until absolute deadline (compensates for read time) */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, NULL);
    }

    if (s_i2c_fd >= 0) close(s_i2c_fd);
    log_info("MPU6050", "Thread exiting cleanly");
    return NULL;
}

