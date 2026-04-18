# =============================================================================
# Makefile  --  QNX Edge Analytics  PS#34  (v5 -- hardware-only build)
# Target   : Raspberry Pi 4  (BCM2711, QNX SDP 8.0, aarch64le)
# Compiler : QNX qcc cross-compiler inside Momentics IDE on Windows
# =============================================================================
#
# HARDWARE-ONLY BUILD
# -------------------
# This project reads REAL sensor data from physical hardware:
#   DHT22   -- temperature + humidity via GPIO17 (BCM), physical pin 11
#   MPU6050 -- accelerometer + gyroscope via I2C1 (/dev/i2c1)
#
# -DUSE_REAL_HARDWARE is ALWAYS defined for aarch64le.
# There is NO simulation/random-data path in this build.
#
# HOW MOMENTICS IDE USES THIS FILE
# ---------------------------------
# The IDE sets CPUVARDIR automatically per Build Configuration:
#   aarch64le-debug   -> CPUVARDIR=aarch64le -> qcc -Vgcc_ntoaarch64le
#   x86_64-debug      -> CPUVARDIR=x86_64    -> qcc -Vgcc_ntox86_64
#
# Object files go into objs/<CPUVARDIR>/ so the two configs never mix.
# =============================================================================

ifndef CPUVARDIR
    CPUVARDIR = aarch64le
endif

ifeq ($(CPUVARDIR),aarch64le)
    QCC_TARGET = gcc_ntoaarch64le
    TARGET     = edge_analytics_aarch64le
else ifeq ($(CPUVARDIR),x86_64)
    QCC_TARGET = gcc_ntox86_64
    TARGET     = edge_analytics_x86_64
else
    $(error Unknown CPUVARDIR=$(CPUVARDIR))
endif

$(info [Makefile] CPUVARDIR  = $(CPUVARDIR))
$(info [Makefile] QCC_TARGET = $(QCC_TARGET))
$(info [Makefile] TARGET     = $(TARGET))
$(info [Makefile] Mode       = REAL HARDWARE -- DHT22 GPIO17 + MPU6050 I2C1)

CC     = qcc
SRCDIR = src
OBJDIR = objs/$(CPUVARDIR)

SRCS = main.c           \
       logger.c         \
       sensor_dht22.c   \
       sensor_mpu6050.c \
       analytics.c      \
       transmission.c   \
       server_hw_dashboard.c

OBJS = $(addprefix $(OBJDIR)/,$(SRCS:.c=.o))

# -DUSE_REAL_HARDWARE always on -- no simulation, no random data
CFLAGS = -V$(QCC_TARGET)      \
         -Wall                 \
         -Wextra               \
         -Wno-comment          \
         -O2                   \
         -g                    \
         -I$(SRCDIR)           \
         -DUSE_REAL_HARDWARE

# QNX SDP 8.0: pthread is inside libc -- no -lpthread
LDFLAGS = -V$(QCC_TARGET) \
          -lm              \
          -lsocket

.PHONY: all clean

all: $(OBJDIR) $(TARGET)
	@echo ""
	@echo "BUILD OK: $(TARGET)  [$(CPUVARDIR)]  HARDWARE MODE"
	@echo ""
	@echo "Deploy to Pi 4:"
	@echo "  scp $(TARGET) root@<PI_IP>:/tmp/edge_analytics/"
	@echo "  scp -r web root@<PI_IP>:/tmp/edge_analytics/"
	@echo "  ssh root@<PI_IP> 'cd /tmp/edge_analytics && chmod +x $(TARGET) && ./$(TARGET)'"
	@echo ""

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

$(OBJDIR)/%.o: $(SRCDIR)/%.c $(SRCDIR)/common.h | $(OBJDIR)
	
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	
	
	
	@echo "Cleaned."


