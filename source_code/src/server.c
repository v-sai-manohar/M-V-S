/*
 * server.c  --  QNX Edge Analytics  PS#34  (v5 -- hardware-only)
 * =============================================================================
 * LEGACY MODULE:
 * This file is retained for reference. Active build now uses
 * src/server_hw_dashboard.c (selected in Makefile SRCS).
 * =============================================================================
 * HTTP JSON server -- serves real sensor data to the dashboard.
 *
 * Two-tier state protection:
 *   (A) analytics_result + tx_stats  -> g_server_mutex   (0.5 Hz, mutex OK)
 *   (B) raw_temp, raw_humidity, etc. -> _Atomic float    (10 Hz, zero-block)
 *
 * Without (B), MPU6050 thread (P22) would lock g_server_mutex at 10 Hz.
 * Server thread (P10) holds it during snprintf (~200 Âµs) -> priority inversion.
 * =============================================================================
 */

#include "common.h"

server_state_t  g_server_state;
pthread_mutex_t g_server_mutex = PTHREAD_MUTEX_INITIALIZER;

/* â”€â”€â”€ API called by other threads â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

void server_update_result(const analytics_result_t *r)
{
    pthread_mutex_lock(&g_server_mutex);
    g_server_state.result = *r;
    g_server_state.ready  = 1;
    pthread_mutex_unlock(&g_server_mutex);
}

void server_update_tx_stats(uint64_t sent, uint64_t failed,
                              uint64_t bytes, uint32_t seq)
{
    pthread_mutex_lock(&g_server_mutex);
    g_server_state.tx_sent   = sent;
    g_server_state.tx_failed = failed;
    g_server_state.tx_bytes  = bytes;
    g_server_state.tx_seq    = seq;
    pthread_mutex_unlock(&g_server_mutex);
}

/*
 * server_update_raw()
 * Called by sensor threads at up to 10 Hz.
 * Uses atomic_store -- single instruction, zero blocking, no mutex.
 * Pass -1.0f for fields you do not want to change.
 */
void server_update_raw(float temp, float hum,
                        float accel, float gyro, float mpu_temp)
{
    if (temp     >= 0.0f)
        atomic_store_explicit(&g_server_state.raw_temp,      temp,
                               memory_order_relaxed);
    if (hum      >= 0.0f)
        atomic_store_explicit(&g_server_state.raw_humidity,  hum,
                               memory_order_relaxed);
    if (accel    >= 0.0f)
        atomic_store_explicit(&g_server_state.raw_accel_mag, accel,
                               memory_order_relaxed);
    if (gyro     >= 0.0f)
        atomic_store_explicit(&g_server_state.raw_gyro_mag,  gyro,
                               memory_order_relaxed);
    if (mpu_temp >= 0.0f)
        atomic_store_explicit(&g_server_state.raw_mpu_temp,  mpu_temp,
                               memory_order_relaxed);
}

/* â”€â”€â”€ JSON builder â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static void build_json(char *buf, int bufsz)
{
    /* Snapshot analytics struct + tx stats (mutex, 0.5 Hz path) */
    analytics_result_t r;
    int      ready;
    uint64_t tx_sent, tx_fail, tx_bytes;
    uint32_t tx_seq;

    pthread_mutex_lock(&g_server_mutex);
    r       = g_server_state.result;
    ready   = g_server_state.ready;
    tx_sent = g_server_state.tx_sent;
    tx_fail = g_server_state.tx_failed;
    tx_bytes= g_server_state.tx_bytes;
    tx_seq  = g_server_state.tx_seq;
    pthread_mutex_unlock(&g_server_mutex);

    /* Read live raw values (atomic load, no mutex) */
    float raw_t  = atomic_load_explicit(&g_server_state.raw_temp,
                                         memory_order_relaxed);
    float raw_h  = atomic_load_explicit(&g_server_state.raw_humidity,
                                         memory_order_relaxed);
    float raw_a  = atomic_load_explicit(&g_server_state.raw_accel_mag,
                                         memory_order_relaxed);
    float raw_g  = atomic_load_explicit(&g_server_state.raw_gyro_mag,
                                         memory_order_relaxed);
    float raw_mt = atomic_load_explicit(&g_server_state.raw_mpu_temp,
                                         memory_order_relaxed);

    snprintf(buf, (size_t)bufsz,
        "{"
          "\"seq\":%u,"
          "\"ts\":%ld,"
          "\"ready\":%s,"
          "\"dht22\":{"
            "\"temp_raw\":%.1f,"
            "\"hum_raw\":%.1f,"
            "\"temp_avg\":%.2f,"
            "\"temp_min\":%.1f,"
            "\"temp_max\":%.1f,"
            "\"hum_avg\":%.2f,"
            "\"hum_min\":%.1f,"
            "\"hum_max\":%.1f,"
            "\"samples\":%d,"
            "\"temp_alert\":%d,"
            "\"hum_alert\":%d"
          "},"
          "\"mpu6050\":{"
            "\"accel_raw\":%.4f,"
            "\"gyro_raw\":%.4f,"
            "\"die_temp\":%.2f,"
            "\"accel_avg\":%.4f,"
            "\"accel_max\":%.4f,"
            "\"gyro_avg\":%.4f,"
            "\"gyro_max\":%.4f,"
            "\"mpu_temp\":%.2f,"
            "\"vibration\":%s,"
            "\"motion\":%s,"
            "\"samples\":%d"
          "},"
          "\"tx\":{"
            "\"sent\":%llu,"
            "\"failed\":%llu,"
            "\"bytes\":%llu,"
            "\"seq\":%u"
          "}"
        "}",
        r.sequence,
        (long)time(NULL),
        ready ? "true" : "false",
        raw_t, raw_h,
        r.avg_temp,  r.min_temp,  r.max_temp,
        r.avg_humidity, r.min_humidity, r.max_humidity,
        r.dht22_samples, r.temp_alert, r.humidity_alert,
        raw_a, raw_g, raw_mt,
        r.avg_accel_mag, r.max_accel_mag,
        r.avg_gyro_mag,  r.max_gyro_mag,
        r.avg_mpu_temp,
        r.vibration_detected ? "true" : "false",
        r.motion_detected    ? "true" : "false",
        r.mpu6050_samples,
        (unsigned long long)tx_sent,
        (unsigned long long)tx_fail,
        (unsigned long long)tx_bytes,
        tx_seq
    );
}

/* â”€â”€â”€ HTTP request handler â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static void handle_client(int fd)
{
    char req[512];
    ssize_t n = recv(fd, req, sizeof(req) - 1, 0);
    if (n <= 0) { close(fd); return; }
    req[n] = '\0';

    /* CORS preflight */
    if (strncmp(req, "OPTIONS", 7) == 0) {
        const char *r =
            "HTTP/1.1 204 No Content\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, OPTIONS\r\n"
            "Access-Control-Allow-Headers: Content-Type\r\n"
            "Content-Length: 0\r\n"
            "Connection: close\r\n\r\n";
        send(fd, r, strlen(r), 0);
        close(fd);
        return;
    }

    /* Health check */
    if (strstr(req, "GET /health") != NULL) {
        const char *body = "{\"status\":\"ok\",\"mode\":\"hardware\"}";
        char hdr[256];
        snprintf(hdr, sizeof(hdr),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %zu\r\n"
            "Connection: close\r\n\r\n", strlen(body));
        send(fd, hdr,  strlen(hdr),  0);
        send(fd, body, strlen(body), 0);
        close(fd);
        return;
    }

    /* Full JSON payload -- stack buffers, never static (no re-entrancy issues) */
    char json[SERVER_JSON_BUF];
    char http[SERVER_HTTP_BUF];

    build_json(json, sizeof(json));
    snprintf(http, sizeof(http),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "Content-Length: %zu\r\n"
        "Connection: close\r\n\r\n%s",
        strlen(json), json);

    send(fd, http, strlen(http), 0);
    close(fd);
}

/* â”€â”€â”€ Thread entry point â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
void *server_thread(void *arg)
{
    (void)arg;
    sigset_t mask; sigfillset(&mask);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);

    struct sched_param sp = { .sched_priority = PRIORITY_LOW };
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { log_error("SERVER", "socket: %s", strerror(errno)); return NULL; }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(SERVER_PORT);

    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("SERVER", "bind port %d: %s", SERVER_PORT, strerror(errno));
        close(srv); return NULL;
    }
    if (listen(srv, SERVER_MAX_CLIENTS) < 0) {
        log_error("SERVER", "listen: %s", strerror(errno));
        close(srv); return NULL;
    }

    /* 1-second accept timeout to recheck g_running */
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    log_info("SERVER", "Listening  port=%d  prio=%d  "
             "Dashboard: http://<PI_IP>:%d",
             SERVER_PORT, PRIORITY_LOW, SERVER_PORT);

    while (g_running) {
        int cli = accept(srv, NULL, NULL);
        if (cli < 0) {
            if (errno==EAGAIN||errno==ETIMEDOUT||
                errno==EWOULDBLOCK||errno==EINTR)
                continue;
            log_error("SERVER", "accept: %s", strerror(errno));
            break;
        }
        handle_client(cli);
    }

    close(srv);
    log_info("SERVER", "Thread exiting");
    return NULL;
}

