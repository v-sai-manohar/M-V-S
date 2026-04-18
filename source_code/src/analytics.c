/*
 * analytics.c  --  QNX Edge Analytics  PS#34  (v5 -- hardware-only)
 * =============================================================================
 * EDGE ANALYTICS TASK
 *
 * Runs at PRIORITY_HIGH (20) -- below sensors (22), above transmission (15).
 *
 * Every 2 seconds:
 *   1. Drains the DHT22 and MPU6050 SPSC rings (lock-free, non-blocking)
 *   2. Computes window statistics:
 *        DHT22  : avg/min/max temperature, avg/min/max humidity
 *        MPU6050: avg/max |accel| magnitude, avg/max |gyro| magnitude
 *   3. Applies edge detection rules:
 *        TEMP HIGH / LOW  -- threshold alerts
 *        HUM HIGH         -- threshold alert
 *        VIBRATION        -- >20% of MPU samples have |accel| > 2g
 *        MOTION           -- >10% of MPU samples have |gyro| > 100 dps
 *   4. Sends result to Transmission task via QNX MsgSend (IPC)
 *   5. Updates HTTP server shared state
 *
 * SPIN-POLL AFTER WAKE (jitter absorber):
 *   After clock_nanosleep returns, waits up to 10 ms for at least 1 DHT22
 *   and 10 MPU6050 samples to appear. DHT22 produces exactly 1 sample per
 *   2-second cycle. If the scheduler woke analytics a few ms early (normal
 *   QNX scheduling jitter), that sample might not be pushed yet.
 *   Without this wait, dht22_n=0 and the window is skipped entirely.
 * =============================================================================
 */

#include "common.h"

static uint32_t s_seq = 0;

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  Drain helpers (lock-free SPSC consumer side)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static int drain_dht22(dht22_data_t *out, int max)
{
    int n = 0;
    while (n < max && dht22_spsc_pop(&g_sensor_buf.dht22, &out[n]) == 0)
        n++;
    return n;
}

static int drain_mpu6050(mpu6050_data_t *out, int max)
{
    int n = 0;
    while (n < max && mpu6050_spsc_pop(&g_sensor_buf.mpu6050, &out[n]) == 0)
        n++;
    return n;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  compute_window()
 *  Computes all edge analytics over one 2-second window of real sensor data.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static analytics_result_t compute_window(
    const dht22_data_t   *dht22,    int nd,
    const mpu6050_data_t *mpu6050,  int nm)
{
    analytics_result_t r;
    memset(&r, 0, sizeof(r));
    r.sequence = s_seq++;

    /* â”€â”€ DHT22 statistics â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
    if (nd > 0) {
        float sum_t = 0.0f, sum_h = 0.0f;
        float mn_t = dht22[0].temperature;
        float mx_t = dht22[0].temperature;
        float mn_h = dht22[0].humidity;
        float mx_h = dht22[0].humidity;

        for (int i = 0; i < nd; i++) {
            float t = dht22[i].temperature;
            float h = dht22[i].humidity;
            sum_t += t;
            sum_h += h;
            if (t < mn_t) mn_t = t;
            if (t > mx_t) mx_t = t;
            if (h < mn_h) mn_h = h;
            if (h > mx_h) mx_h = h;
        }
        r.avg_temp     = sum_t / (float)nd;
        r.min_temp     = mn_t;
        r.max_temp     = mx_t;
        r.avg_humidity = sum_h / (float)nd;
        r.min_humidity = mn_h;
        r.max_humidity = mx_h;
        r.dht22_samples = nd;

        /* Temperature alert */
        if      (r.avg_temp > TEMP_HIGH_THRESHOLD) r.temp_alert =  1;
        else if (r.avg_temp < TEMP_LOW_THRESHOLD)  r.temp_alert = -1;
        else                                        r.temp_alert =  0;

        /* Humidity alert */
        r.humidity_alert = (r.avg_humidity > HUMIDITY_HIGH_THRESHOLD) ? 1 : 0;
    }

    /* â”€â”€ MPU6050 statistics â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
    if (nm > 0) {
        float sum_am = 0.0f, max_am = 0.0f;
        float sum_gm = 0.0f, max_gm = 0.0f;
        float sum_mt = 0.0f;
        int   vib_count = 0, mot_count = 0;

        for (int i = 0; i < nm; i++) {
            float am = vec3_mag(mpu6050[i].accel_x,
                                mpu6050[i].accel_y,
                                mpu6050[i].accel_z);
            float gm = vec3_mag(mpu6050[i].gyro_x,
                                mpu6050[i].gyro_y,
                                mpu6050[i].gyro_z);
            sum_am += am;
            sum_gm += gm;
            sum_mt += mpu6050[i].temp_onboard;
            if (am > max_am) max_am = am;
            if (gm > max_gm) max_gm = gm;
            if (am > ACCEL_SPIKE_THRESHOLD) vib_count++;
            if (gm > GYRO_SPIKE_THRESHOLD)  mot_count++;
        }

        r.avg_accel_mag   = sum_am / (float)nm;
        r.max_accel_mag   = max_am;
        r.avg_gyro_mag    = sum_gm / (float)nm;
        r.max_gyro_mag    = max_gm;
        r.avg_mpu_temp    = sum_mt / (float)nm;
        r.mpu6050_samples = nm;

        /* Vibration: >20% of samples exceed 2g threshold */
        r.vibration_detected =
            (vib_count > (int)((float)nm * VIBRATION_FRACTION)) ? 1 : 0;

        /* Motion: >10% of samples exceed 100 dps threshold */
        r.motion_detected =
            (mot_count > (int)((float)nm * MOTION_FRACTION)) ? 1 : 0;
    }

    return r;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  log_alerts()  -- prints alert messages for any triggered conditions
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static void log_alerts(const analytics_result_t *r)
{
    if (r->temp_alert == 1)
        log_alert("ANALYTICS", "HIGH TEMP    avg=%.1f C  [seq#%u]",
                  r->avg_temp, r->sequence);
    else if (r->temp_alert == -1)
        log_alert("ANALYTICS", "LOW TEMP     avg=%.1f C  [seq#%u]",
                  r->avg_temp, r->sequence);
    if (r->humidity_alert)
        log_alert("ANALYTICS", "HIGH HUMIDITY avg=%.1f %% [seq#%u]",
                  r->avg_humidity, r->sequence);
    if (r->vibration_detected)
        log_alert("ANALYTICS", "VIBRATION    max=%.3f g  [seq#%u]",
                  r->max_accel_mag, r->sequence);
    if (r->motion_detected)
        log_alert("ANALYTICS", "MOTION       max=%.2f dps [seq#%u]",
                  r->max_gyro_mag, r->sequence);
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  analytics_thread()
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
void *analytics_thread(void *arg)
{
    (void)arg;

    sigset_t mask;
    sigfillset(&mask);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);

    struct sched_param sp = { .sched_priority = PRIORITY_HIGH };
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);

    log_info("ANALYTICS", "STARTED  prio=%d  window=%dms",
             PRIORITY_HIGH, ANALYTICS_WINDOW_MS);

    /* Connect to Transmission task's QNX channel */
    int coid = ConnectAttach(0, 0, g_analytics_chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) {
        log_error("ANALYTICS", "ConnectAttach failed: %s", strerror(errno));
        return NULL;
    }
    log_info("ANALYTICS", "Connected to TX channel (coid=%d)", coid);

    /* Local drain buffers */
    dht22_data_t   local_d[DHT22_RING_SIZE];
    mpu6050_data_t local_m[MPU6050_RING_SIZE];

    /* TX accumulators */
    uint64_t tx_sent = 0, tx_fail = 0, tx_bytes = 0;

    /* Absolute-deadline loop */
    struct timespec next_wake;
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    while (g_running) {
        struct timespec wall_window_start;
        get_time(&wall_window_start);

        /* Advance absolute deadline by one window period */
        next_wake.tv_nsec += (long)ANALYTICS_WINDOW_MS * 1000000L;
        if (next_wake.tv_nsec >= 1000000000L) {
            next_wake.tv_sec  += next_wake.tv_nsec / 1000000000L;
            next_wake.tv_nsec  = next_wake.tv_nsec % 1000000000L;
        }

        /* Sleep to absolute deadline */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, NULL);

        /*
         * Spin-poll for up to 10 ms after waking.
         *
         * Rationale: DHT22 produces exactly 1 sample per 2-second cycle.
         * If the scheduler wakes analytics 2-5 ms early (normal jitter),
         * the DHT22 sample for this window may not yet be in the ring.
         * Without this wait, dht22_n=0 and the entire window is skipped.
         *
         * 10 ms is safe -- it is 0.5% of the 2-second window and well
         * within the sensor's next-sample deadline.
         */
        {
            struct timespec poll_end;
            clock_gettime(CLOCK_MONOTONIC, &poll_end);
            poll_end.tv_nsec += 10 * 1000000L;
            if (poll_end.tv_nsec >= 1000000000L) {
                poll_end.tv_sec++;
                poll_end.tv_nsec -= 1000000000L;
            }

            for (;;) {
                if (dht22_spsc_count(&g_sensor_buf.dht22) >= 1 &&
                    mpu6050_spsc_count(&g_sensor_buf.mpu6050) >= 10)
                    break;

                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                if (now.tv_sec > poll_end.tv_sec ||
                    (now.tv_sec == poll_end.tv_sec &&
                     now.tv_nsec >= poll_end.tv_nsec))
                    break;

                /* Yield 0.5 ms so sensor threads can push samples */
                struct timespec yt = {0, 500000L};
                nanosleep(&yt, NULL);
            }
        }

        /* Drain SPSC rings -- lock-free, never blocks sensor threads */
        int nd = drain_dht22 (local_d, DHT22_RING_SIZE);
        int nm = drain_mpu6050(local_m, MPU6050_RING_SIZE);

        if (nd == 0 && nm == 0) {
            log_warn("ANALYTICS",
                     "No sensor data in window (sensors may still be starting)");
            continue;
        }

        log_info("ANALYTICS", "Window #%u  DHT22=%d samples  MPU6050=%d samples",
                 s_seq, nd, nm);

        /* Compute edge analytics over real sensor data */
        analytics_result_t result = compute_window(local_d, nd, local_m, nm);
        result.window_start = wall_window_start;
        get_time(&result.window_end);

        /* Log any triggered alerts */
        log_alerts(&result);

        /* Push to HTTP server (mutex, 0.5 Hz -- fine) */
        server_update_result(&result);

        /* Forward to Transmission task via QNX IPC */
        ipc_msg_t   msg;
        ipc_reply_t reply;
        memset(&msg, 0, sizeof(msg));
        msg.type   = MSG_TYPE_ANALYTICS_RESULT;
        msg.result = result;

        int rc = MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply));
        if (rc == -1) {
            log_error("ANALYTICS", "MsgSend failed: %s [seq#%u]",
                      strerror(errno), result.sequence);
            tx_fail++;
        } else {
            tx_sent++;
            tx_bytes += (uint64_t)sizeof(analytics_result_t);
            log_info("ANALYTICS", "Result #%u sent to TX", result.sequence);
        }

        server_update_tx_stats(tx_sent, tx_fail, tx_bytes, s_seq);
    }

    /* Notify Transmission to exit */
    ipc_msg_t sd;
    memset(&sd, 0, sizeof(sd));
    sd.type = MSG_TYPE_SHUTDOWN;
    MsgSend(coid, &sd, sizeof(sd), NULL, 0);
    ConnectDetach(coid);

    log_info("ANALYTICS", "Thread exiting  (total windows=%u)", s_seq);
    return NULL;
}

