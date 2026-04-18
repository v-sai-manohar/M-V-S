/*
 * logger.c
 * ─────────────────────────────────────────────────────────────────────────────
 * Thread-safe logger for the QNX Edge Analytics System.
 * All output is serialised through a single mutex so log lines from
 * different threads never interleave.
 *
 * Format:
 *   [HH:MM:SS.mmm] [LEVEL] [SOURCE           ] message
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "common.h"

static pthread_mutex_t s_log_mutex = PTHREAD_MUTEX_INITIALIZER;

static const char *level_colour(const char *level) {
    if (strcmp(level, "INFO")  == 0) return "\033[0;37m";
    if (strcmp(level, "WARN")  == 0) return "\033[0;33m";
    if (strcmp(level, "ERROR") == 0) return "\033[0;31m";
    if (strcmp(level, "ALERT") == 0) return "\033[1;35m";
    return "\033[0m";
}

static void log_generic(const char *level, const char *src,
                         const char *fmt, va_list ap)
{
    struct timeval  tv;
    struct tm       tm_info;
    char            time_buf[16];
    char            msg_buf[512];

    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm_info);
    strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &tm_info);
    vsnprintf(msg_buf, sizeof(msg_buf), fmt, ap);

    pthread_mutex_lock(&s_log_mutex);
    printf("%s[%s.%03ld] [%-5s] [%-18s] %s\033[0m\n",
           level_colour(level),
           time_buf,
           (long)(tv.tv_usec / 1000),
           level,
           src,
           msg_buf);
    fflush(stdout);
    pthread_mutex_unlock(&s_log_mutex);
}

void log_info(const char *src, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    log_generic("INFO", src, fmt, ap);
    va_end(ap);
}

void log_warn(const char *src, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    log_generic("WARN", src, fmt, ap);
    va_end(ap);
}

void log_error(const char *src, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    log_generic("ERROR", src, fmt, ap);
    va_end(ap);
}

void log_alert(const char *src, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    log_generic("ALERT", src, fmt, ap);
    va_end(ap);
}
