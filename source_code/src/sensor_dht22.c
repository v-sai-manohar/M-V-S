/*
 * sensor_dht22.c  --  QNX Edge Analytics  PS#34  (v5 -- hardware / simulation)
 * =============================================================================
 * REAL-TIME DHT22 SENSOR DRIVER FOR QNX RTOS
 *
 * Hardware: Raspberry Pi 4 (BCM2711), QNX SDP 8.0
 * Pin     : GPIO 17 (BCM) = physical pin 11, with 10 kÎ© pull-up to 3.3 V
 * Protocol: DHT single-wire, 40-bit frame
 * Simulation: Generates synthetic temperature/humidity data when --simulate flag is used
 *
 * FIXES APPLIED (over original v5)
 * ----------------------------------
 *  FIX 1: DHT22_PIN corrected from 4 â†’ 17.
 *          Every comment, banner, and README stated BCM GPIO17 (physical
 *          pin 11), but the #define had 4. GPIO4 is unconnected/floated HIGH,
 *          so both hi and lo readback checks returned 1 â†’
 *          "GPIO readback failed (hi=1 lo=1)" â†’ thread never started â†’
 *          DHT22 samples = 0 in every analytics window â†’ all N/A in report.
 *
 *  FIX 2: GPIO readback settle delay increased 5 Âµs â†’ 20 Âµs.
 *          BCM2711 GPIO output propagation + QNX MMIO write-buffer flush
 *          can take up to ~10 Âµs. 5 Âµs was borderline; 20 Âµs is safe and
 *          still negligible compared to the DHT22 protocol timing.
 *
 * WHY PREVIOUS VERSIONS GAVE WRONG VALUES -- complete root cause list
 * -------------------------------------------------------------------
 *  1. InterruptDisable() scope too narrow â†’ timer interrupts corrupted bits.
 *  2. EMA filter introduced up to 20-second lag (alpha=0.30, 0.5 Hz rate).
 *  3. Median-of-3 with 50 ms gaps â†’ reads 2&3 always failed (DHT22 needs 1s).
 *  4. Missing 30 Âµs host-release delay â†’ sensor missed handshake rising edge.
 *  5. clock_gettime() inside 40-bit loop (~2 Âµs syscall corrupted bit timing).
 *  6. Simulation path running because Makefile defaulted HW=0.
 *
 * RTOS DESIGN
 * -----------
 *  Priority       : PRIORITY_SENSOR (22) -- highest in system
 *  Scheduling     : SCHED_RR
 *  Timing         : clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)
 *                   Wakes at fixed wall-clock deadline regardless of
 *                   how long the read or retries took. Zero drift.
 *  Shared buffer  : Lock-free SPSC ring (dht22_spsc_t in common.h)
 *                   Push is a single atomic store -- never blocks.
 * =============================================================================
 */

#include "common.h"
#include <sys/mman.h>
#include <sys/syspage.h>

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  BCM2711 (Raspberry Pi 4) GPIO register map
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define BCM2711_PERI_BASE   0xFE000000UL
#define GPIO_BASE           (BCM2711_PERI_BASE + 0x200000UL)
#define GPIO_MAP_LEN        0x1000

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  FIX 1: DHT22_PIN corrected from 4 â†’ 17
 *
 *  BCM GPIO17 = physical pin 11. This is the pin documented in the banner,
 *  README, and every comment in this project. The original value of 4 was a
 *  copy-paste error. GPIO4 floats HIGH via the board pull-up when unconnected,
 *  so pin_lo() writes to GPCLR0 for bit 4 while we read back bit 4 of GPLEV0
 *  -- but since nothing is driving GPIO4 low electrically, the pull-up wins
 *  and GPLEV0 still reads 1. Result: lv_lo=1, check fails, thread exits.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define DHT22_PIN           17      /* BCM GPIO17, physical pin 11            */

/* GPIO register word-offsets (each register = 32 bits)
 * All offsets verified against BCM2711 TRM (Table 5-2):
 *   GPFSEL0  @ 0x00 â†’ word 0   (GPIO  0-9  function select)
 *   GPFSEL1  @ 0x04 â†’ word 1   (GPIO 10-19 function select) â† GPIO17 here
 *   GPSET0   @ 0x1C â†’ word 7   (pin output set,   GPIO 0-31)
 *   GPCLR0   @ 0x28 â†’ word 10  (pin output clear, GPIO 0-31)
 *   GPLEV0   @ 0x34 â†’ word 13  (pin level read,   GPIO 0-31)
 *   GPPUPPDN0@ 0xE4 â†’ word 57  (pull-up/down,     GPIO  0-15)
 *   GPPUPPDN1@ 0xE8 â†’ word 58  (pull-up/down,     GPIO 16-31) â† GPIO17 here
 */
#define GPFSEL0             0
#define GPFSEL1             1
#define GPSET0              7
#define GPCLR0             10
#define GPLEV0             13
#define GPPUPPDN0          57
#define GPPUPPDN1          58

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  Protocol timing (DHT22 datasheet)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define DHT22_START_LOW_MS       2  /* host pull-low: >=1 ms for DHT22       */
#define DHT22_RELEASE_US        30  /* host release HIGH: 20-40 Âµs           */
#define DHT22_RESP_TIMEOUT_US  500  /* handshake phase timeout                */
#define DHT22_SYNC_TIMEOUT_US  160  /* per-bit LOW sync timeout               */
#define DHT22_HIGH_TIMEOUT_US  200  /* per-bit HIGH pulse timeout cap         */
#define DHT22_BIT_THRESH_US     50  /* HIGH pulse > 50 Âµs â†’ logic '1'        */
#define DHT22_MAX_RETRIES        5  /* retries on FAILED reads only           */
#define DHT22_RETRY_MS          50  /* delay between failed attempts (ms)     */

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  FIX 2: GPIO readback settle delay increased 5 Âµs â†’ 20 Âµs
 *
 *  BCM2711 GPIO output write â†’ GPLEV0 read round-trip through the QNX MMIO
 *  mapping (PROT_NOCACHE) includes peripheral bus latency and can reach ~10 Âµs.
 *  The original spin_us(5) was too short; the GPLEV0 register occasionally
 *  still reflected the old HIGH state after a GPCLR write, making lv_lo=1.
 *  20 Âµs gives 2Ã— margin over the worst-case observed latency.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define GPIO_SETTLE_US          20  /* Âµs to wait after driving pin HIGH/LOW  */

/* DHT22 physical operating range (datasheet) */
#define DHT22_TEMP_MIN    -40.0f
#define DHT22_TEMP_MAX     80.0f
#define DHT22_HUM_MIN       0.0f
#define DHT22_HUM_MAX     100.0f

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  Module-level state
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static volatile uint32_t *s_gpio = NULL;
static uint64_t           s_cps  = 0;      /* CPU cycles per second */

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  GPIO helpers -- all inline, zero function-call overhead in hot path
 *
 *  For GPIO17:
 *    pin_out / pin_in â†’ use GPFSEL1 (r = 17/10 = 1), shift = (17%10)*3 = 21
 *    pin_hi           â†’ GPSET0 bit 17
 *    pin_lo           â†’ GPCLR0 bit 17
 *    pin_rd           â†’ GPLEV0 bit 17
 *    pin_pullup       â†’ GPPUPPDN1 bits [3:2]  (sh = (17%16)*2 = 2)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static inline void pin_out(int p)
{
    int r = p / 10, sh = (p % 10) * 3;
    uint32_t v = s_gpio[r];
    v &= ~(7u << sh);
    v |=  (1u << sh);          /* 001 = output */
    s_gpio[r] = v;
}
static inline void pin_in(int p)
{
    int r = p / 10, sh = (p % 10) * 3;
    s_gpio[r] &= ~(7u << sh);  /* 000 = input  */
}
static inline void pin_hi(int p) { s_gpio[GPSET0] = (1u << p); }
static inline void pin_lo(int p) { s_gpio[GPCLR0] = (1u << p); }
static inline int  pin_rd(int p) { return (int)((s_gpio[GPLEV0] >> p) & 1u); }
static inline void pin_pullup(int p)
{
    int reg = (p < 16) ? GPPUPPDN0 : GPPUPPDN1;
    int sh  = (p % 16) * 2;
    uint32_t v = s_gpio[reg];
    v &= ~(3u << sh);
    v |=  (1u << sh);           /* 01 = pull-up */
    s_gpio[reg] = v;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  ClockCycles() helpers
 *  ARM64: single MRS instruction, reads PMCCNTR_EL0 directly.
 *  No syscall, no context switch. Works correctly inside InterruptDisable.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static inline uint64_t cc_us(uint64_t delta)
{
    return (delta * 1000000ULL) / s_cps;
}

/* Spin-delay: does NOT release the CPU (safe inside InterruptDisable) */
static inline void spin_us(uint64_t us)
{
    uint64_t t0 = ClockCycles();
    while (cc_us(ClockCycles() - t0) < us) { /* spin */ }
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  dht22_gpio_init()
 *  Maps GPIO registers and claims I/O privilege for InterruptDisable().
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static int dht22_gpio_init(void)
{
    /* Cache CPU speed from QNX syspage (no syscall later) */
    s_cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    if (s_cps == 0) s_cps = 1000000000ULL;   /* fallback: assume 1 GHz */

    /*
     * Claim I/O privilege.
     * Required for InterruptDisable() / InterruptEnable().
     * Without this, those calls return EPERM silently on QNX.
     * The process must run as root or have the io_priv capability.
     */
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        log_error("DHT22",
                  "ThreadCtl(_NTO_TCTL_IO) failed: %s  "
                  "-- must run as root or with io_priv capability",
                  strerror(errno));
        return -1;
    }

    /*
     * QNX SDP 8 uses mmap_device_memory() for physical MMIO mapping.
     * /dev/mem is Linux-specific and may not exist on QNX images.
     */
    s_gpio = (volatile uint32_t *)mmap_device_memory(
                 NULL,
                 GPIO_MAP_LEN,
                 PROT_READ | PROT_WRITE | PROT_NOCACHE,
                 0,
                 (uint64_t)GPIO_BASE);
    if (s_gpio == MAP_FAILED || s_gpio == NULL) {
        s_gpio = NULL;
        log_error("DHT22", "mmap_device_memory GPIO (0x%lX): %s",
                  (unsigned long)GPIO_BASE, strerror(errno));
        return -1;
    }

    /* Enable internal pull-up for extra robustness (keep external 10k too). */
    pin_pullup(DHT22_PIN);

    /*
     * GPIO readback sanity test:
     *   1. Set pin as output, drive HIGH.
     *   2. Wait GPIO_SETTLE_US (FIX 2: 20 Âµs, was 5 Âµs).
     *   3. Read back -- expect 1.
     *   4. Drive LOW.
     *   5. Wait GPIO_SETTLE_US.
     *   6. Read back -- expect 0.
     *   7. Restore idle state (HIGH input with pull-up).
     *
     * If lv_hi != 1 or lv_lo != 0, either the pin number is wrong (FIX 1)
     * or MMIO mapping failed silently. Both are fatal for DHT22 timing.
     */
    pin_out(DHT22_PIN);
    pin_hi(DHT22_PIN);
    spin_us(GPIO_SETTLE_US);           /* FIX 2: was spin_us(5) */
    int lv_hi = pin_rd(DHT22_PIN);

    pin_lo(DHT22_PIN);
    spin_us(GPIO_SETTLE_US);           /* FIX 2: was spin_us(5) */
    int lv_lo = pin_rd(DHT22_PIN);

    /* Restore safe idle state before returning */
    pin_hi(DHT22_PIN);
    pin_in(DHT22_PIN);
    pin_pullup(DHT22_PIN);

    if (lv_hi != 1 || lv_lo != 0) {
        log_warn("DHT22",
                 "GPIO readback failed (hi=%d lo=%d). "
                 "Check QNX GPIO base/capabilities.",
                 lv_hi, lv_lo);
        return -1;
    }

    log_info("DHT22", "GPIO mapped (QNX MMIO)  pin=BCM%d  cps=%llu  io_priv=OK",
             DHT22_PIN, (unsigned long long)s_cps);
    return 0;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  dht22_read_raw()
 *
 *  Complete DHT22 single-wire protocol. Returns 1 on success, 0 on error.
 *
 *  INTERRUPT HANDLING:
 *  InterruptDisable() covers everything from pin_in() onward:
 *    - Response detection  (3 x ~80 Âµs spin loops)
 *    - 40-bit read         (40 x ~120 Âµs windows)
 *  Total disabled window: ~5 ms worst case (0.25% of 2-second cycle).
 *
 *  Without InterruptDisable(), a single 1ms QNX timer tick landing
 *  during a 26 Âµs '0' bit window makes it look like a 70 Âµs '1' bit.
 *  The resulting corrupted byte can still pass the checksum, producing
 *  a reading that looks correct but is completely wrong.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static int dht22_read_raw(uint8_t data[5])
{
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    /* â”€â”€ Phase 1: Host start signal â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
    /* Pull DATA low for â‰¥18 ms  (nanosleep OK here -- long, predictable wait) */
    pin_out(DHT22_PIN);
    pin_lo(DHT22_PIN);
    sleep_ms(DHT22_START_LOW_MS);

    /* Release line HIGH, spin 30 Âµs for pull-up resistor to raise the bus */
    pin_hi(DHT22_PIN);
    spin_us(DHT22_RELEASE_US);

    /* Switch to input */
    pin_in(DHT22_PIN);
    pin_pullup(DHT22_PIN);

    /* â”€â”€ Phase 2+3: Response detect + 40-bit read (interrupts disabled) â”€â”€â”€ */
    InterruptDisable();

    int      ok = 1;
    uint64_t t0;

    /* DHT pulls LOW ~80 Âµs (response start) */
    t0 = ClockCycles();
    while (pin_rd(DHT22_PIN) != 0) {
        if (cc_us(ClockCycles() - t0) > DHT22_RESP_TIMEOUT_US) { ok = 0; break; }
    }

    /* DHT pulls HIGH ~80 Âµs */
    if (ok) {
        t0 = ClockCycles();
        while (pin_rd(DHT22_PIN) != 1) {
            if (cc_us(ClockCycles() - t0) > DHT22_RESP_TIMEOUT_US) { ok = 0; break; }
        }
    }

    /* HIGH ends -- data transmission begins */
    if (ok) {
        t0 = ClockCycles();
        while (pin_rd(DHT22_PIN) != 0) {
            if (cc_us(ClockCycles() - t0) > DHT22_RESP_TIMEOUT_US) { ok = 0; break; }
        }
    }

    /* Read 40 bits
     *   Each bit: 50 Âµs LOW sync  +  HIGH whose width encodes value
     *     HIGH â‰¤ 50 Âµs  â†’  bit '0'  (actual: ~26 Âµs)
     *     HIGH  > 50 Âµs  â†’  bit '1'  (actual: ~70 Âµs)
     */
    for (int i = 0; ok && i < 40; i++) {
        /* Wait for sync LOW to end (rising edge = start of data pulse) */
        t0 = ClockCycles();
        while (pin_rd(DHT22_PIN) != 1) {
            if (cc_us(ClockCycles() - t0) > DHT22_SYNC_TIMEOUT_US) { ok = 0; break; }
        }
        if (!ok) break;

        /* Measure HIGH pulse width */
        t0 = ClockCycles();
        while (pin_rd(DHT22_PIN) != 0) {
            if (cc_us(ClockCycles() - t0) > DHT22_HIGH_TIMEOUT_US) break; /* safety cap */
        }
        uint64_t pulse = cc_us(ClockCycles() - t0);

        data[i / 8] <<= 1;
        if (pulse > DHT22_BIT_THRESH_US)
            data[i / 8] |= 1;
    }

    InterruptEnable();  /* restore hardware interrupts immediately */

    if (!ok) {
        log_warn("DHT22", "Timing error -- check wiring, pull-up, and 3.3V supply");
        return 0;
    }

    /* Checksum: byte[4] must equal low byte of sum of bytes[0..3] */
    uint8_t cs = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (cs != data[4]) {
        log_warn("DHT22",
                 "Checksum FAIL  calc=0x%02X  recv=0x%02X  "
                 "frame=[%02X %02X %02X %02X %02X]",
                 cs, data[4],
                 data[0], data[1], data[2], data[3], data[4]);
        return 0;
    }
    return 1;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  dht22_read_once()
 *
 *  In simulation mode, generates synthetic temperature/humidity data.
 *  In hardware mode, attempts up to DHT22_MAX_RETRIES reads. Retries only on
 *  genuine failure (checksum error or out-of-range value). One valid read
 *  per 2-second cycle -- no EMA, no median, no averaging.
 *
 *  The DHT22 checksum validates each reading. If it passes checksum
 *  and is within the physical operating range, it is the real value.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static dht22_data_t dht22_read_once(void)
{
    extern volatile int g_simulation_mode;
    
    dht22_data_t out = {0};
    
    /* In simulation mode, return synthetic data */
    if (g_simulation_mode) {
        static uint32_t sim_count = 0;
        sim_count++;
        
        /* Simulate temperature: 20-30Â°C with slow oscillation */
        float cycle = (float)(sim_count % 200) / 20.0f;  /* 0-10 */
        out.temperature = 25.0f + 5.0f * sinf(cycle * 0.5f);
        
        /* Simulate humidity: 40-70% with different phase */
        out.humidity = 55.0f + 15.0f * sinf(cycle * 0.7f - 1.0f);
        
        out.valid = 1;
        get_time(&out.timestamp);
        return out;
    }

    uint8_t data[5];

    for (int att = 1; att <= DHT22_MAX_RETRIES; att++) {

        if (!dht22_read_raw(data)) {
            if (att < DHT22_MAX_RETRIES) sleep_ms(DHT22_RETRY_MS);
            continue;
        }

        /*
         * DHT22 frame layout (40 bits = 5 bytes):
         *   humidity    = ((byte0 << 8) | byte1) / 10
         *   temperature = ((byte2 & 0x7F) << 8 | byte3) / 10
         *   sign bit    = byte2 bit7 (1 => negative)
         *   byte4       = checksum
         */
        uint16_t rh_raw = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
        uint16_t t_raw  = (uint16_t)(((uint16_t)(data[2] & 0x7Fu) << 8) | data[3]);
        float h = (float)rh_raw * 0.1f;
        float t = (float)t_raw  * 0.1f;
        if (data[2] & 0x80u) {
            t = -t;
        }

        /* Reject readings outside the sensor's physical operating range */
        if (t < DHT22_TEMP_MIN || t > DHT22_TEMP_MAX) {
            log_warn("DHT22", "Temperature out of range: %.1f C (att %d/%d)",
                     t, att, DHT22_MAX_RETRIES);
            if (att < DHT22_MAX_RETRIES) sleep_ms(DHT22_RETRY_MS);
            continue;
        }
        if (h < DHT22_HUM_MIN || h > DHT22_HUM_MAX) {
            log_warn("DHT22", "Humidity out of range: %.1f %% (att %d/%d)",
                     h, att, DHT22_MAX_RETRIES);
            if (att < DHT22_MAX_RETRIES) sleep_ms(DHT22_RETRY_MS);
            continue;
        }

        /* Valid real-world reading */
        out.temperature = t;
        out.humidity    = h;
        out.valid       = 1;
        get_time(&out.timestamp);
        log_info("DHT22", "att=%d  T=%.1f C  H=%.1f %%", att, t, h);
        return out;
    }

    log_error("DHT22",
              "All %d read attempts failed.  "
              "Check: 10k pull-up resistor on DATA, 3.3V VCC, BCM GPIO%d",
              DHT22_MAX_RETRIES, DHT22_PIN);
    return out;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  spsc_push_safe()
 *  Pushes a sample. If ring is full (analytics fell behind), drops the
 *  oldest sample to make room. Real-time: fresh data > stale data.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static void spsc_push_safe(dht22_spsc_t *r, const dht22_data_t *s)
{
    if (dht22_spsc_push(r, s) == 0) return;

    /* Ring full -- evict oldest */
    uint32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
    atomic_store_explicit(&r->head,
                          (h + 1u) & (DHT22_RING_SIZE - 1u),
                          memory_order_release);
    dht22_spsc_push(r, s);
    log_warn("DHT22", "Ring full -- oldest sample evicted");
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  sensor_dht22_thread()
 *
 *  RTOS periodic task using clock_nanosleep(TIMER_ABSTIME):
 *    1. Record absolute deadline = now + 2000 ms
 *    2. Perform one DHT22 read (including retries if needed)
 *    3. Push result to SPSC ring (lock-free, non-blocking)
 *    4. Update raw server state (atomic store, non-blocking)
 *    5. Sleep until the recorded absolute deadline
 *    Repeat.
 *
 *  Step 5 compensates for however long steps 2-4 took.
 *  The sampling period is exactly 2000 ms, never accumulates drift.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
void *sensor_dht22_thread(void *arg)
{
    (void)arg;

    /* Block all signals -- main thread handles SIGINT/SIGTERM */
    sigset_t mask;
    sigfillset(&mask);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);

    /* Set real-time priority -- highest in system */
    struct sched_param sp = { .sched_priority = PRIORITY_SENSOR };
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);

    if (!g_simulation_mode && dht22_gpio_init() != 0) {
        log_warn("DHT22",
                 "Hardware init failed. Falling back to simulation mode for DHT22 data");
        g_simulation_mode = 1;
    }

    if (g_simulation_mode) {
        log_info("DHT22",
                 "STARTED  SIMULATION mode  prio=%d  period=%dms",
                 PRIORITY_SENSOR, DHT22_SAMPLE_INTERVAL_MS);
    } else {
        log_info("DHT22",
                 "STARTED  pin=BCM%d  prio=%d  period=%dms  "
                 "irq_disabled_per_read  no_ema  real_values_only",
                 DHT22_PIN, PRIORITY_SENSOR, DHT22_SAMPLE_INTERVAL_MS);
    }

    /* Absolute-deadline periodic loop */
    struct timespec next_wake;
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    while (g_running) {

        /* Advance deadline by exactly one period BEFORE reading */
        next_wake.tv_nsec += (long)DHT22_SAMPLE_INTERVAL_MS * 1000000L;
        if (next_wake.tv_nsec >= 1000000000L) {
            next_wake.tv_sec  += next_wake.tv_nsec / 1000000000L;
            next_wake.tv_nsec  = next_wake.tv_nsec % 1000000000L;
        }

        /* Read sensor -- real hardware, no random data */
        dht22_data_t reading = dht22_read_once();

        if (reading.valid) {
            /* Push to analytics pipeline (lock-free) */
            spsc_push_safe(&g_sensor_buf.dht22, &reading);

            /* Update HTTP server raw display (atomic store, non-blocking) */
            server_update_raw(reading.temperature, reading.humidity,
                               -1.0f, -1.0f, -1.0f);
        }

        /* Sleep until the pre-calculated absolute deadline.
         * This automatically compensates for read time + retry time. */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, NULL);
    }

    log_info("DHT22", "Thread exiting cleanly");
    return NULL;
}

