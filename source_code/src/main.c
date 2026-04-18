/*
 * main.c  --  QNX Edge Analytics  PS#34  (v5 -- hardware-only)
 * =============================================================================
 * Startup / shutdown controller.
 *
 * Thread spawn order (deliberate):
 *   1. server_thread      -- listening before any data arrives
 *   2. transmission_thread -- MsgReceive ready before analytics sends
 *   3. analytics_thread   -- connects to IPC channel before sensors push data
 *   4. sensor_dht22_thread
 *   5. sensor_mpu6050_thread
 *
 * Shutdown order on SIGINT/SIGTERM:
 *   g_running = 0 -> sensors exit their loops
 *   analytics sends MSG_SHUTDOWN to TX, then exits
 *   server exits on accept() timeout
 * =============================================================================
 */

#include "common.h"
#include <fcntl.h>

sensor_buffer_t  g_sensor_buf;
volatile int     g_running        = 1;
volatile int     g_simulation_mode = 0;  /* 0 = hardware, 1 = simulation */
int              g_analytics_chid = -1;

static void signal_handler(int sig)
{
    (void)sig;
    const char *msg = "\n[MAIN] Shutdown signal -- stopping...\n";
    write(STDOUT_FILENO, msg, strlen(msg));
    g_running = 0;
}

static pthread_t spawn_thread(void *(*fn)(void *), const char *name)
{
    pthread_t      tid;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate (&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    int rc = pthread_create(&tid, &attr, fn, NULL);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        log_error("MAIN", "pthread_create '%s': %s", name, strerror(rc));
        return (pthread_t)0;
    }
    log_info("MAIN", "Thread %-20s spawned", name);
    sleep_ms(80);   /* small gap: each thread completes init before next starts */
    return tid;
}

static int preflight_checks(void)
{
    int ok = 1;

    /* Skip hardware checks in simulation mode */
    if (g_simulation_mode) {
        log_info("MAIN", "SIMULATION MODE: hardware checks skipped");
        return 1;
    }

    int fd = open("/dev/i2c1", O_RDWR);
    if (fd < 0) {
        log_error("MAIN", "/dev/i2c1 not accessible: %s", strerror(errno));
        log_error("MAIN", "Start I2C resource manager, e.g. i2c-bcm2711 --paddr 0xFE804000 -d /dev/i2c1");
        ok = 0;
    } else {
        close(fd);
        log_info("MAIN", "/dev/i2c1 accessible");
    }

    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        log_error("MAIN", "ThreadCtl(_NTO_TCTL_IO) failed: %s (run as root / io_priv)",
                  strerror(errno));
        ok = 0;
    } else {
        log_info("MAIN", "I/O privilege check OK");
    }

    if (!ok) {
        log_warn("MAIN",
                 "Hardware preflight failed. Auto-switching to simulation mode");
        g_simulation_mode = 1;
        return 1;
    }

    return ok;
}

static void print_banner(void)
{
    printf("\n");
    printf("+====================================================================+\n");
    printf("|     QNX EDGE ANALYTICS  --  Problem Statement #34  [v5]          |\n");
    printf("|     DHT22 (Temp/Hum) + MPU6050 (Accel/Gyro)  --  Pi 4 / QNX    |\n");
    printf("+====================================================================+\n");
    printf("|  Pipeline:                                                         |\n");
    printf("|  [DHT22  P22]â”€â”€â”€â”                                                  |\n");
    printf("|                  â”œâ”€â”€>[Analytics P20]â”€â”€IPCâ”€â”€>[Transmit P15]         |\n");
    printf("|  [MPU6050 P22]â”€â”€â”€â”˜         â”‚                                       |\n");
    printf("|                            â””â”€â”€â”€â”€â”€â”€â”€â”€>[HTTP Server P10]:8080        |\n");
    printf("+====================================================================+\n");
    if (g_simulation_mode) {
        printf("|  Mode    : SIMULATION  (no real hardware required)                |\n");
        printf("|  DHT22   : Simulated temperature + humidity data                 |\n");
        printf("|  MPU6050 : Simulated accelerometer + gyroscope data              |\n");
    } else {
        printf("|  Mode    : REAL HARDWARE  (GPIO17=DHT22  /dev/i2c1=MPU6050)      |\n");
        printf("|  DHT22   : BCM GPIO17, physical pin 11, 10k pull-up to 3.3V      |\n");
        printf("|  MPU6050 : I2C1 (SDA=GPIO2/pin3, SCL=GPIO3/pin5), AD0->GND      |\n");
    }
    printf("|  Dashboard: http://<Pi_IP>:8080/  (API: /api/telemetry)          |\n");
    printf("|  Deploy web/index.html with binary for full animated dashboard    |\n");
    printf("|  Stop    : Ctrl+C                                                  |\n");
    printf("+====================================================================+\n\n");
    fflush(stdout);
}

int main(int argc, char *argv[])
{
    /* Parse command-line arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--simulate") == 0) {
            g_simulation_mode = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [--simulate]\n", argv[0]);
            printf("  --simulate : Run in simulation mode (no real hardware required)\n");
            return EXIT_SUCCESS;
        }
    }

    print_banner();

    /* Signal handling */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* Initialise SPSC sensor buffer (just needs memset -- no OS handles) */
    memset(&g_sensor_buf, 0, sizeof(g_sensor_buf));
    log_info("MAIN", "Sensor SPSC rings initialised (lock-free, no mutex)");

    /* Initialise server state */
    memset(&g_server_state, 0, sizeof(g_server_state));
    atomic_store_explicit(&g_server_state.raw_temp,      -1.0f, memory_order_relaxed);
    atomic_store_explicit(&g_server_state.raw_humidity,  -1.0f, memory_order_relaxed);
    atomic_store_explicit(&g_server_state.raw_accel_mag, -1.0f, memory_order_relaxed);
    atomic_store_explicit(&g_server_state.raw_gyro_mag,  -1.0f, memory_order_relaxed);
    atomic_store_explicit(&g_server_state.raw_mpu_temp,  -1.0f, memory_order_relaxed);

    if (!preflight_checks()) {
        log_error("MAIN", "Preflight failed -- cannot start sensors on this target");
        return EXIT_FAILURE;
    }

    /* Create QNX IPC channel for analytics -> transmission */
    g_analytics_chid = ChannelCreate(0);
    if (g_analytics_chid == -1) {
        log_error("MAIN", "ChannelCreate: %s", strerror(errno));
        return EXIT_FAILURE;
    }
    log_info("MAIN", "IPC channel created (chid=%d)", g_analytics_chid);

    /* Spawn all threads */
    pthread_t t_srv  = spawn_thread(server_thread,         "http_server");
    pthread_t t_tx   = spawn_thread(transmission_thread,   "transmission");
    pthread_t t_anal = spawn_thread(analytics_thread,      "analytics");
    pthread_t t_dht  = spawn_thread(sensor_dht22_thread,   "sensor_dht22");
    pthread_t t_mpu  = spawn_thread(sensor_mpu6050_thread, "sensor_mpu6050");

    if (!t_srv || !t_tx || !t_anal || !t_dht || !t_mpu) {
        log_error("MAIN", "A thread failed to start -- shutting down");
        g_running = 0;
    } else if (g_simulation_mode) {
        log_info("MAIN", "All threads running.  Reading simulated sensor data.");
    } else {
        log_info("MAIN", "All threads running.  Reading real sensor data.");
    }

    /* Main thread sleeps until signal */
    while (g_running) sleep_ms(500);

    log_info("MAIN", "Waiting for threads to join...");
    pthread_join(t_dht,  NULL);
    pthread_join(t_mpu,  NULL);
    pthread_join(t_anal, NULL);
    pthread_join(t_tx,   NULL);
    pthread_join(t_srv,  NULL);

    ChannelDestroy(g_analytics_chid);
    log_info("MAIN", "Shutdown complete.");
    return EXIT_SUCCESS;
}

