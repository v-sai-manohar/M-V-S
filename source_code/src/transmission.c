/*
 * transmission.c  --  QNX Edge Analytics  PS#34  (v5 -- hardware-only)
 * =============================================================================
 * Receives analytics_result_t from analytics_thread via QNX MsgReceive.
 * Replies IMMEDIATELY so analytics is never blocked.
 * Logs the result to console as a formatted report.
 * =============================================================================
 */

#include "common.h"

static uint64_t s_bytes    = 0;
static uint32_t s_sent     = 0;
static uint32_t s_failed   = 0;
static uint32_t s_received = 0;

static void print_report(const analytics_result_t *r)
{
    char ts[20];
    struct tm tm_info;
    localtime_r(&r->window_end.tv_sec, &tm_info);
    strftime(ts, sizeof(ts), "%H:%M:%S", &tm_info);

    long dur_ms = (long)(
        (r->window_end.tv_sec  - r->window_start.tv_sec)  * 1000L +
        (r->window_end.tv_nsec - r->window_start.tv_nsec) / 1000000L);

    int has_dht = (r->dht22_samples > 0);
    int has_mpu = (r->mpu6050_samples > 0);

    const char *tst = !has_dht ? "   N/A    " :
                      (r->temp_alert ==  1) ? "!! HIGH  !!" :
                      (r->temp_alert == -1) ? "!! LOW   !!" : "  NORMAL  ";
    const char *hst = !has_dht ? "   N/A    " :
                      (r->humidity_alert ? "!! HIGH  !!" : "  NORMAL  ");
    const char *vst = !has_mpu ? "   N/A    " :
                      (r->vibration_detected ? "VIBRATION!!" : "  STABLE  ");
    const char *mst = !has_mpu ? "   N/A    " :
                      (r->motion_detected ? "MOTION!!  " : "  STABLE  ");

    printf("\n+================================================================+\n");
    printf("| EDGE REPORT  seq#%-4u  %s  window=%ld ms              |\n",
           r->sequence, ts, dur_ms);
    printf("+================================+===============================+\n");
    printf("| DHT22  (%3d samples)           | MPU6050 (%3d samples)         |\n",
           r->dht22_samples, r->mpu6050_samples);
    printf("+--------------------------------+-------------------------------+\n");
    if (has_dht)
        printf("| Temp avg  : %6.1f C           | ", r->avg_temp);
    else
        printf("| Temp avg  :    N/A            | ");
    if (has_mpu)
        printf("Accel avg  : %8.4f g       |\n", r->avg_accel_mag);
    else
        printf("Accel avg  :    N/A            |\n");

    if (has_dht)
        printf("| Temp min  : %6.1f C           | ", r->min_temp);
    else
        printf("| Temp min  :    N/A            | ");
    if (has_mpu)
        printf("Accel max  : %8.4f g       |\n", r->max_accel_mag);
    else
        printf("Accel max  :    N/A            |\n");

    if (has_dht)
        printf("| Temp max  : %6.1f C           | ", r->max_temp);
    else
        printf("| Temp max  :    N/A            | ");
    if (has_mpu)
        printf("Gyro  avg  : %8.2f dps     |\n", r->avg_gyro_mag);
    else
        printf("Gyro  avg  :    N/A            |\n");

    if (has_dht)
        printf("| Hum  avg  : %6.1f %%          | ", r->avg_humidity);
    else
        printf("| Hum  avg  :    N/A            | ");
    if (has_mpu)
        printf("Gyro  max  : %8.2f dps     |\n", r->max_gyro_mag);
    else
        printf("Gyro  max  :    N/A            |\n");

    if (has_dht)
        printf("| Hum  min  : %6.1f %%          | ", r->min_humidity);
    else
        printf("| Hum  min  :    N/A            | ");
    if (has_mpu)
        printf("Die   temp : %8.2f C       |\n", r->avg_mpu_temp);
    else
        printf("Die   temp :    N/A            |\n");

    if (has_dht)
        printf("| Hum  max  : %6.1f %%          |                               |\n", r->max_humidity);
    else
        printf("| Hum  max  :    N/A            |                               |\n");
    printf("+--------------------------------+-------------------------------+\n");
    printf("| Temp   : %-12s         | Vibration  : %-12s        |\n", tst, vst);
    printf("| Humid  : %-12s         | Motion     : %-12s        |\n", hst, mst);
    printf("+================================+===============================+\n");
    printf("| Pkts sent=%-5u failed=%-5u  bytes=%-10llu                |\n",
           s_sent, s_failed, (unsigned long long)s_bytes);
    printf("+================================================================+\n");
    fflush(stdout);
}

void *transmission_thread(void *arg)
{
    (void)arg;
    sigset_t mask; sigfillset(&mask);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);

    struct sched_param sp = { .sched_priority = PRIORITY_MEDIUM };
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);

    log_info("TRANSMIT", "STARTED  prio=%d  chid=%d",
             PRIORITY_MEDIUM, g_analytics_chid);

    for (;;) {
        ipc_msg_t   msg;
        ipc_reply_t reply;
        int rcvid = MsgReceive(g_analytics_chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            log_error("TRANSMIT", "MsgReceive: %s", strerror(errno));
            break;
        }

        if (msg.type == MSG_TYPE_SHUTDOWN) {
            reply.status = 0;
            MsgReply(rcvid, 0, &reply, sizeof(reply));
            log_info("TRANSMIT", "Shutdown message received -- exiting");
            break;
        }

        if (msg.type == MSG_TYPE_ANALYTICS_RESULT) {
            s_received++;

            /* Reply immediately -- unblock analytics thread ASAP */
            reply.status = 0;
            MsgReply(rcvid, 0, &reply, sizeof(reply));

            /* Log formatted report */
            print_report(&msg.result);

            /* Count the transmission */
            s_sent++;
            s_bytes += sizeof(analytics_result_t);
            log_info("TRANSMIT", "Result #%u delivered  (total=%llu bytes)",
                     msg.result.sequence, (unsigned long long)s_bytes);
        } else {
            reply.status = 1;
            MsgReply(rcvid, 0, &reply, sizeof(reply));
            log_warn("TRANSMIT", "Unknown msg type 0x%02X", msg.type);
        }
    }

    log_info("TRANSMIT",
             "Exiting  rcvd=%u  sent=%u  failed=%u  bytes=%llu",
             s_received, s_sent, s_failed,
             (unsigned long long)s_bytes);
    return NULL;
}

