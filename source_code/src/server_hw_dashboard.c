/*
 * server_hw_dashboard.c  --  QNX Edge Analytics  PS#34
 * =============================================================================
 * HTTP server module:
 *   GET /                -> animated dashboard (web/EdgeAnalyticsDashboard (2).html)
 *   GET /api/telemetry   -> real-time JSON payload (real formula values)
 *   GET /health          -> health endpoint
 *
 * This module is used in place of legacy server.c to ensure the dashboard
 * always displays real converted sensor values and no fake startup data.
 * =============================================================================
 */

#include "common.h"

#define DASHBOARD_FILE_PATH "web/EdgeAnalyticsDashboard (2).html"
#define RAW_UNSET_VALUE     (-1.0f)
#define MAX_DASHBOARD_SIZE  (512 * 1024)

server_state_t  g_server_state;
pthread_mutex_t g_server_mutex = PTHREAD_MUTEX_INITIALIZER;

static const char *k_dashboard_fallback_html =
"<!doctype html>"
"<html lang='en'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Edge Analytics</title>"
"<style>"
"body{font-family:Segoe UI,Tahoma,sans-serif;background:#f6f2e9;color:#1f2529;"
"margin:0;display:grid;place-items:center;min-height:100vh;padding:24px}"
".card{background:#fff;border-radius:14px;padding:20px;box-shadow:0 10px 28px "
"rgba(0,0,0,.08);max-width:760px}"
"h1{margin:.2rem 0 .8rem 0;font-size:1.4rem}code{background:#f1ece1;padding:2px 6px;"
"border-radius:6px}a{color:#0f6f6a}"
"</style></head><body><div class='card'>"
"<h1>Dashboard Asset Missing</h1>"
"<p>Deploy <code>web/EdgeAnalyticsDashboard (2).html</code> with the binary.</p>"
"<p>Telemetry API is live at <a href='/api/telemetry'>/api/telemetry</a>.</p>"
"</div></body></html>";

/* ------------------------------------------------------------------------- */
/* Outgoing HTTP helpers                                                     */
/* ------------------------------------------------------------------------- */

static int send_all(int fd, const char *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(fd, data + sent, len - sent, 0);
        if (n <= 0) return -1;
        sent += (size_t)n;
    }
    return 0;
}

static void send_response_buffer(int fd,
                                 const char *status,
                                 const char *content_type,
                                 const char *body,
                                 size_t body_len,
                                 int no_cache)
{
    char hdr[512];
    int n = snprintf(
        hdr, sizeof(hdr),
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Access-Control-Allow-Methods: GET, OPTIONS\r\n"
        "Access-Control-Allow-Headers: Content-Type\r\n"
        "Cache-Control: %s\r\n"
        "Content-Length: %zu\r\n"
        "Connection: close\r\n\r\n",
        status,
        content_type,
        no_cache ? "no-cache, no-store, must-revalidate" : "public, max-age=60",
        body_len
    );

    if (n > 0) {
        send_all(fd, hdr, (size_t)n);
    }
    if (body_len > 0 && body != NULL) {
        send_all(fd, body, body_len);
    }
}

static void send_response_text(int fd,
                               const char *status,
                               const char *content_type,
                               const char *body,
                               int no_cache)
{
    const size_t len = (body != NULL) ? strlen(body) : 0;
    send_response_buffer(fd, status, content_type, body, len, no_cache);
}

static void send_options_no_content(int fd)
{
    const char *hdr =
        "HTTP/1.1 204 No Content\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Access-Control-Allow-Methods: GET, OPTIONS\r\n"
        "Access-Control-Allow-Headers: Content-Type\r\n"
        "Content-Length: 0\r\n"
        "Connection: close\r\n\r\n";
    send_all(fd, hdr, strlen(hdr));
}

/* ------------------------------------------------------------------------- */
/* Shared state API                                                          */
/* ------------------------------------------------------------------------- */

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

void server_update_raw(float temp, float hum,
                       float accel, float gyro, float mpu_temp)
{
    if (temp >= 0.0f) {
        atomic_store_explicit(&g_server_state.raw_temp, temp, memory_order_relaxed);
    }
    if (hum >= 0.0f) {
        atomic_store_explicit(&g_server_state.raw_humidity, hum, memory_order_relaxed);
    }
    if (accel >= 0.0f) {
        atomic_store_explicit(&g_server_state.raw_accel_mag, accel, memory_order_relaxed);
    }
    if (gyro >= 0.0f) {
        atomic_store_explicit(&g_server_state.raw_gyro_mag, gyro, memory_order_relaxed);
    }
    if (mpu_temp >= 0.0f) {
        atomic_store_explicit(&g_server_state.raw_mpu_temp, mpu_temp, memory_order_relaxed);
    }
}

/* ------------------------------------------------------------------------- */
/* JSON builder                                                              */
/* ------------------------------------------------------------------------- */

static void format_float_or_null(char *dst, size_t dst_sz,
                                 float value, int decimals, int valid)
{
    if (!valid || !isfinite((double)value)) {
        snprintf(dst, dst_sz, "null");
        return;
    }

    if (decimals < 0) decimals = 0;
    if (decimals > 6) decimals = 6;

    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%.%df", decimals);
    snprintf(dst, dst_sz, fmt, value);
}

static void build_json_payload(char *buf, int bufsz)
{
    analytics_result_t r;
    int      ready;
    uint64_t tx_sent, tx_fail, tx_bytes;
    uint32_t tx_seq;

    pthread_mutex_lock(&g_server_mutex);
    r        = g_server_state.result;
    ready    = g_server_state.ready;
    tx_sent  = g_server_state.tx_sent;
    tx_fail  = g_server_state.tx_failed;
    tx_bytes = g_server_state.tx_bytes;
    tx_seq   = g_server_state.tx_seq;
    pthread_mutex_unlock(&g_server_mutex);

    float raw_t  = atomic_load_explicit(&g_server_state.raw_temp,      memory_order_relaxed);
    float raw_h  = atomic_load_explicit(&g_server_state.raw_humidity,  memory_order_relaxed);
    float raw_a  = atomic_load_explicit(&g_server_state.raw_accel_mag, memory_order_relaxed);
    float raw_g  = atomic_load_explicit(&g_server_state.raw_gyro_mag,  memory_order_relaxed);
    float raw_mt = atomic_load_explicit(&g_server_state.raw_mpu_temp,  memory_order_relaxed);

    const int has_dht = (r.dht22_samples > 0);
    const int has_mpu = (r.mpu6050_samples > 0);

    char temp_raw[24], hum_raw[24], accel_raw[24], gyro_raw[24], die_temp_raw[24];
    char temp_avg[24], temp_min[24], temp_max[24], hum_avg[24], hum_min[24], hum_max[24];
    char accel_avg[24], accel_max[24], gyro_avg[24], gyro_max[24], mpu_temp_avg[24];

    format_float_or_null(temp_raw,     sizeof(temp_raw),     raw_t,          1, raw_t  >= 0.0f);
    format_float_or_null(hum_raw,      sizeof(hum_raw),      raw_h,          1, raw_h  >= 0.0f);
    format_float_or_null(accel_raw,    sizeof(accel_raw),    raw_a,          4, raw_a  >= 0.0f);
    format_float_or_null(gyro_raw,     sizeof(gyro_raw),     raw_g,          4, raw_g  >= 0.0f);
    format_float_or_null(die_temp_raw, sizeof(die_temp_raw), raw_mt,         2, raw_mt >= 0.0f);

    format_float_or_null(temp_avg,     sizeof(temp_avg),     r.avg_temp,      2, has_dht);
    format_float_or_null(temp_min,     sizeof(temp_min),     r.min_temp,      1, has_dht);
    format_float_or_null(temp_max,     sizeof(temp_max),     r.max_temp,      1, has_dht);
    format_float_or_null(hum_avg,      sizeof(hum_avg),      r.avg_humidity,  2, has_dht);
    format_float_or_null(hum_min,      sizeof(hum_min),      r.min_humidity,  1, has_dht);
    format_float_or_null(hum_max,      sizeof(hum_max),      r.max_humidity,  1, has_dht);

    format_float_or_null(accel_avg,    sizeof(accel_avg),    r.avg_accel_mag, 4, has_mpu);
    format_float_or_null(accel_max,    sizeof(accel_max),    r.max_accel_mag, 4, has_mpu);
    format_float_or_null(gyro_avg,     sizeof(gyro_avg),     r.avg_gyro_mag,  4, has_mpu);
    format_float_or_null(gyro_max,     sizeof(gyro_max),     r.max_gyro_mag,  4, has_mpu);
    format_float_or_null(mpu_temp_avg, sizeof(mpu_temp_avg), r.avg_mpu_temp,  2, has_mpu);

    snprintf(buf, (size_t)bufsz,
        "{"
          "\"seq\":%u,"
          "\"ts\":%ld,"
          "\"ready\":%s,"
          "\"mode\":\"real_hardware\","
          "\"source\":\"formula_converted\","
          "\"dht22\":{"
            "\"temp_raw\":%s,"
            "\"hum_raw\":%s,"
            "\"temp_avg\":%s,"
            "\"temp_min\":%s,"
            "\"temp_max\":%s,"
            "\"hum_avg\":%s,"
            "\"hum_min\":%s,"
            "\"hum_max\":%s,"
            "\"samples\":%d,"
            "\"temp_alert\":%d,"
            "\"hum_alert\":%d"
          "},"
          "\"mpu6050\":{"
            "\"accel_raw\":%s,"
            "\"gyro_raw\":%s,"
            "\"die_temp\":%s,"
            "\"accel_avg\":%s,"
            "\"accel_max\":%s,"
            "\"gyro_avg\":%s,"
            "\"gyro_max\":%s,"
            "\"mpu_temp\":%s,"
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
        temp_raw, hum_raw,
        temp_avg, temp_min, temp_max,
        hum_avg, hum_min, hum_max,
        r.dht22_samples,
        has_dht ? r.temp_alert : 0,
        has_dht ? r.humidity_alert : 0,
        accel_raw, gyro_raw, die_temp_raw,
        accel_avg, accel_max, gyro_avg, gyro_max, mpu_temp_avg,
        (has_mpu && r.vibration_detected) ? "true" : "false",
        (has_mpu && r.motion_detected)    ? "true" : "false",
        r.mpu6050_samples,
        (unsigned long long)tx_sent,
        (unsigned long long)tx_fail,
        (unsigned long long)tx_bytes,
        tx_seq
    );
}

/* ------------------------------------------------------------------------- */
/* Dashboard file serving                                                    */
/* ------------------------------------------------------------------------- */

static int read_file_alloc(const char *path, char **out_data, size_t *out_len)
{
    *out_data = NULL;
    *out_len  = 0;

    FILE *fp = fopen(path, "rb");
    if (fp == NULL) return -1;

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return -1;
    }

    long sz = ftell(fp);
    if (sz < 0 || sz > MAX_DASHBOARD_SIZE) {
        fclose(fp);
        return -1;
    }
    rewind(fp);

    char *buf = (char *)malloc((size_t)sz + 1u);
    if (buf == NULL) {
        fclose(fp);
        return -1;
    }

    size_t got = fread(buf, 1, (size_t)sz, fp);
    fclose(fp);
    buf[got] = '\0';

    *out_data = buf;
    *out_len  = got;
    return 0;
}

static void send_dashboard_html(int fd)
{
    char   *html = NULL;
    size_t  len  = 0;

    if (read_file_alloc(DASHBOARD_FILE_PATH, &html, &len) == 0) {
        send_response_buffer(fd, "200 OK", "text/html; charset=utf-8",
                             html, len, 0);
        free(html);
        return;
    }

    send_response_text(fd, "200 OK", "text/html; charset=utf-8",
                       k_dashboard_fallback_html, 0);
}

/* ------------------------------------------------------------------------- */
/* Request handling                                                          */
/* ------------------------------------------------------------------------- */

static void handle_client(int fd)
{
    char req[1024];
    ssize_t n = recv(fd, req, sizeof(req) - 1, 0);
    if (n <= 0) {
        close(fd);
        return;
    }
    req[n] = '\0';

    char method[8] = {0};
    char path[256] = {0};
    if (sscanf(req, "%7s %255s", method, path) != 2) {
        send_response_text(fd, "400 Bad Request", "application/json",
                           "{\"error\":\"bad_request\"}", 1);
        close(fd);
        return;
    }

    if (strcmp(method, "OPTIONS") == 0) {
        send_options_no_content(fd);
        close(fd);
        return;
    }

    if (strcmp(method, "GET") != 0) {
        send_response_text(fd, "405 Method Not Allowed", "application/json",
                           "{\"error\":\"method_not_allowed\"}", 1);
        close(fd);
        return;
    }

    if (strcmp(path, "/health") == 0) {
        send_response_text(fd, "200 OK", "application/json",
                           "{\"status\":\"ok\",\"mode\":\"hardware\"}", 1);
        close(fd);
        return;
    }

    if (strcmp(path, "/api/telemetry") == 0 || strcmp(path, "/api/data") == 0) {
        char json[SERVER_JSON_BUF];
        build_json_payload(json, sizeof(json));
        send_response_text(fd, "200 OK", "application/json", json, 1);
        close(fd);
        return;
    }

    if (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0) {
        send_dashboard_html(fd);
        close(fd);
        return;
    }

    if (strcmp(path, "/favicon.ico") == 0) {
        send_options_no_content(fd);
        close(fd);
        return;
    }

    send_response_text(fd, "404 Not Found", "application/json",
                       "{\"error\":\"not_found\"}", 1);
    close(fd);
}

/* ------------------------------------------------------------------------- */
/* Thread entry point                                                        */
/* ------------------------------------------------------------------------- */

void *server_thread(void *arg)
{
    (void)arg;

    sigset_t mask;
    sigfillset(&mask);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);

    struct sched_param sp = { .sched_priority = PRIORITY_LOW };
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);

    /* Mark raw values as unavailable until real samples arrive. */
    atomic_store_explicit(&g_server_state.raw_temp,      RAW_UNSET_VALUE, memory_order_relaxed);
    atomic_store_explicit(&g_server_state.raw_humidity,  RAW_UNSET_VALUE, memory_order_relaxed);
    atomic_store_explicit(&g_server_state.raw_accel_mag, RAW_UNSET_VALUE, memory_order_relaxed);
    atomic_store_explicit(&g_server_state.raw_gyro_mag,  RAW_UNSET_VALUE, memory_order_relaxed);
    atomic_store_explicit(&g_server_state.raw_mpu_temp,  RAW_UNSET_VALUE, memory_order_relaxed);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) {
        log_error("SERVER", "socket: %s", strerror(errno));
        return NULL;
    }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(SERVER_PORT);

    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("SERVER", "bind port %d: %s", SERVER_PORT, strerror(errno));
        close(srv);
        return NULL;
    }
    if (listen(srv, SERVER_MAX_CLIENTS) < 0) {
        log_error("SERVER", "listen: %s", strerror(errno));
        close(srv);
        return NULL;
    }

    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    log_info("SERVER",
             "Listening port=%d prio=%d dashboard=http://<PI_IP>:%d/ api=/api/telemetry",
             SERVER_PORT, PRIORITY_LOW, SERVER_PORT);

    while (g_running) {
        int cli = accept(srv, NULL, NULL);
        if (cli < 0) {
            if (errno == EAGAIN || errno == ETIMEDOUT ||
                errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            log_error("SERVER", "accept: %s", strerror(errno));
            break;
        }
        handle_client(cli);
    }

    close(srv);
    log_info("SERVER", "Thread exiting");
    return NULL;
}


