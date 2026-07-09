#include "k4w_internal.h"
#include <libusb.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

/* ═══ K4W Motor Protocol (via audio device 02bb) ═══════════ */
#define MOTOR_MAGIC_CMD   0x06022009
#define MOTOR_MAGIC_REPLY 0x0a6fe000
#define CMD_SET_TILT      0x803b
#define CMD_SET_LED       0x10
#define CMD_GET_ACCEL     0x8032

#define LED_OFF        1
#define LED_BLINK      2
#define LED_GREEN      3
#define LED_RED        4

typedef struct {
    uint32_t magic;
    uint32_t tag;
    uint32_t arg1;
    uint32_t cmd;
    uint32_t arg2;
} motor_cmd_t;

typedef struct {
    uint32_t magic;
    uint32_t tag;
    uint32_t status;
} motor_reply_t;

static uint32_t g_tag_seq = 0;

/* ═══ Motor State ══════════════════════════════════════════ */
static freenect_device *g_freenect_dev = NULL;
static libusb_context *g_usb_ctx = NULL;
static pthread_mutex_t g_motor_lock = PTHREAD_MUTEX_INITIALIZER;
static double g_current_tilt = 0;
static double g_target_tilt = 0;
static int g_motor_busy = 0;

/* ═══ Init ══════════════════════════════════════════════════ */
int k4w_motor_init(freenect_context *ctx) {
    (void)ctx;
    libusb_init(&g_usb_ctx);
    K4W_LOG("Motor: libusb context initialized\n");
    g_current_tilt = 0;
    g_target_tilt = 0;
    return 0;
}

void k4w_motor_set_device(freenect_device *dev) {
    g_freenect_dev = dev;
}

/* ═══ Open/close audio device per command ════════════════ */
static libusb_device_handle *motor_open_device(void) {
    libusb_device_handle *dev = libusb_open_device_with_vid_pid(g_usb_ctx, 0x045e, 0x02bb);
    if (!dev) {
        K4W_LOG("Motor: open failed\n");
        return NULL;
    }
    /* Aggressively detach ALL kernel drivers from ALL interfaces */
    for (int i = 0; i < 4; i++) {
        int r = libusb_kernel_driver_active(dev, i);
        if (r == 1) {
            K4W_LOG("Motor: detaching kernel driver from IF%d\n", i);
            libusb_detach_kernel_driver(dev, i);
        }
        /* Even if not active, try to detach to be sure */
        libusb_detach_kernel_driver(dev, i);
    }
    usleep(100000); /* 100ms for kernel to settle */
    /* Retry loop for claiming interfaces */
    int r0 = -1, r1 = -1;
    for (int attempt = 0; attempt < 5; attempt++) {
        r0 = libusb_claim_interface(dev, 0);
        r1 = libusb_claim_interface(dev, 1);
        if (r0 == 0 && r1 == 0) break;
        K4W_LOG("Motor: claim attempt %d failed IF0=%d IF1=%d\n", attempt + 1, r0, r1);
        /* Release any partially claimed interfaces */
        if (r0 == 0) libusb_release_interface(dev, 0);
        if (r1 == 0) libusb_release_interface(dev, 1);
        usleep(1000000); /* 1s between retries */
    }
    if (r0 != 0 || r1 != 0) {
        K4W_LOG("Motor: claim failed after 5 attempts IF0=%d IF1=%d\n", r0, r1);
        libusb_close(dev);
        return NULL;
    }
    return dev;
}

static void motor_close_device(libusb_device_handle *dev) {
    if (!dev) return;
    libusb_release_interface(dev, 1);
    libusb_release_interface(dev, 0);
    libusb_close(dev);
    usleep(100000); /* 100ms for kernel to re-attach */
}

/* ═══ LED Control ══════════════════════════════════════════ */
int k4w_motor_set_led(freenect_device *dev, int color) {
    (void)dev;

    int led;
    switch (color) {
    case K4W_LED_OFF:          led = LED_OFF; break;
    case K4W_LED_GREEN:        led = LED_GREEN; break;
    case K4W_LED_RED:          led = LED_RED; break;
    case K4W_LED_YELLOW:       led = LED_GREEN; break;  /* K4W has no yellow */
    case K4W_LED_BLINK_GREEN:  led = LED_BLINK; break;
    case K4W_LED_BLINK_RED:    led = LED_RED; break;
    case K4W_LED_BLINK_YELLOW: led = LED_BLINK; break;
    default:                   led = LED_GREEN; break;
    }

    pthread_mutex_lock(&g_motor_lock);
    libusb_device_handle *usbdev = motor_open_device();
    if (!usbdev) { pthread_mutex_unlock(&g_motor_lock); return -1; }

    motor_cmd_t cmd_pkt;
    cmd_pkt.magic = MOTOR_MAGIC_CMD;
    cmd_pkt.tag   = g_tag_seq++;
    cmd_pkt.arg1  = 0;
    cmd_pkt.cmd   = CMD_SET_LED;
    cmd_pkt.arg2  = led;

    int transferred = 0;
    libusb_bulk_transfer(usbdev, 0x01, (unsigned char *)&cmd_pkt, 20, &transferred, 2000);

    unsigned char buf[512] = {0};
    libusb_bulk_transfer(usbdev, 0x81, buf, 512, &transferred, 2000);

    motor_close_device(usbdev);
    pthread_mutex_unlock(&g_motor_lock);

    K4W_LOG("Motor: LED set to %d\n", led);
    return 0;
}

/* ═══ Tilt Control ══════════════════════════════════════════ */
int k4w_motor_set_tilt(freenect_device *dev, double angle) {
    (void)dev;

    if (angle > 31) angle = 31;
    if (angle < -31) angle = -31;

    pthread_mutex_lock(&g_motor_lock);
    g_target_tilt = angle;
    g_motor_busy = 1;

    K4W_LOG("Motor: tilt %.0f°\n", angle);

    libusb_device_handle *usbdev = motor_open_device();
    if (!usbdev) {
        K4W_LOG("Motor: cannot open device for tilt\n");
        g_motor_busy = 0;
        pthread_mutex_unlock(&g_motor_lock);
        return -1;
    }

    /* Send tilt command */
    motor_cmd_t cmd_pkt;
    cmd_pkt.magic = MOTOR_MAGIC_CMD;
    cmd_pkt.tag   = g_tag_seq++;
    cmd_pkt.arg1  = 0;
    cmd_pkt.cmd   = CMD_SET_TILT;
    cmd_pkt.arg2  = (uint32_t)(int32_t)(angle * 2);

    int transferred = 0;
    libusb_bulk_transfer(usbdev, 0x01, (unsigned char *)&cmd_pkt, 20, &transferred, 2000);

    /* Read tilt reply */
    unsigned char buf[512] = {0};
    libusb_bulk_transfer(usbdev, 0x81, buf, 512, &transferred, 2000);
    motor_reply_t reply;
    if (transferred >= 12) {
        memcpy(&reply, buf, 12);
        K4W_LOG("Motor: tilt reply status=%u\n", reply.status);
    }

    /* Wait for motor to move */
    usleep(3000000);  /* 3s fixed wait */

    /* Read final tilt position */
    motor_cmd_t accel_cmd;
    accel_cmd.magic = MOTOR_MAGIC_CMD;
    accel_cmd.tag   = g_tag_seq++;
    accel_cmd.arg1  = 0x68;
    accel_cmd.cmd   = CMD_GET_ACCEL;
    accel_cmd.arg2  = 0;

    unsigned char abuf[512] = {0};
    int at = 0;
    libusb_bulk_transfer(usbdev, 0x01, (unsigned char *)&accel_cmd, 16, &at, 2000);
    libusb_bulk_transfer(usbdev, 0x81, abuf, 512, &at, 2000);

    if (at >= 32) {
        int32_t tilt_raw;
        memcpy(&tilt_raw, abuf + 28, 4);
        g_current_tilt = (double)tilt_raw / 2.0;
        K4W_LOG("Motor: final tilt %.1f° (target %.1f°)\n", g_current_tilt, g_target_tilt);
    }
    /* Trailing reply */
    unsigned char rbuf[512] = {0};
    libusb_bulk_transfer(usbdev, 0x81, rbuf, 512, &at, 200);

    motor_close_device(usbdev);
    g_motor_busy = 0;
    pthread_mutex_unlock(&g_motor_lock);
    return 0;
}

/* ═══ Tilt State ═══════════════════════════════════════════ */
int k4w_motor_get_tilt(freenect_device *dev, double *angle) {
    (void)dev;
    if (!angle) return -1;

    pthread_mutex_lock(&g_motor_lock);

    libusb_device_handle *usbdev = motor_open_device();
    if (!usbdev) { pthread_mutex_unlock(&g_motor_lock); return -1; }

    motor_cmd_t cmd;
    cmd.magic = MOTOR_MAGIC_CMD;
    cmd.tag   = g_tag_seq++;
    cmd.arg1  = 0x68;
    cmd.cmd   = CMD_GET_ACCEL;
    cmd.arg2  = 0;

    unsigned char buf[512] = {0};
    int transferred = 0;

    int r = libusb_bulk_transfer(usbdev, 0x01, (unsigned char *)&cmd, 16, &transferred, 2000);
    if (r != 0) { motor_close_device(usbdev); pthread_mutex_unlock(&g_motor_lock); return r; }

    r = libusb_bulk_transfer(usbdev, 0x81, buf, 512, &transferred, 2000);
    if (r != 0) { motor_close_device(usbdev); pthread_mutex_unlock(&g_motor_lock); return r; }

    if (transferred >= 32) {
        int32_t tilt_raw;
        memcpy(&tilt_raw, buf + 28, 4);
        *angle = (double)tilt_raw / 2.0;
        g_current_tilt = *angle;
    } else {
        *angle = g_current_tilt;
    }

    /* Read trailing reply */
    unsigned char reply_buf[512] = {0};
    libusb_bulk_transfer(usbdev, 0x81, reply_buf, 512, &transferred, 1000);
    motor_close_device(usbdev);

    pthread_mutex_unlock(&g_motor_lock);
    return 0;
}

/* ═══ Accelerometer ════════════════════════════════════════ */
int k4w_motor_get_accel(freenect_device *dev, double *x, double *y, double *z) {
    (void)dev;
    if (!x || !y || !z) return -1;

    pthread_mutex_lock(&g_motor_lock);

    libusb_device_handle *usbdev = motor_open_device();
    if (!usbdev) { pthread_mutex_unlock(&g_motor_lock); return -1; }

    motor_cmd_t cmd;
    cmd.magic = MOTOR_MAGIC_CMD;
    cmd.tag   = g_tag_seq++;
    cmd.arg1  = 0x68;
    cmd.cmd   = CMD_GET_ACCEL;
    cmd.arg2  = 0;

    unsigned char buf[512] = {0};
    int transferred = 0;

    int r = libusb_bulk_transfer(usbdev, 0x01, (unsigned char *)&cmd, 16, &transferred, 500);
    if (r != 0) { motor_close_device(usbdev); pthread_mutex_unlock(&g_motor_lock); return r; }

    r = libusb_bulk_transfer(usbdev, 0x81, buf, 512, &transferred, 500);
    if (r != 0) { motor_close_device(usbdev); pthread_mutex_unlock(&g_motor_lock); return r; }

    if (transferred >= 28) {
        int32_t ax, ay, az;
        memcpy(&ax, buf + 16, 4);
        memcpy(&ay, buf + 20, 4);
        memcpy(&az, buf + 24, 4);
        *x = (double)ax / 819.0 * 9.80665;
        *y = (double)ay / 819.0 * 9.80665;
        *z = (double)az / 819.0 * 9.80665;
    }

    /* Read trailing reply */
    unsigned char reply_buf[512] = {0};
    libusb_bulk_transfer(usbdev, 0x81, reply_buf, 512, &transferred, 200);
    motor_close_device(usbdev);

    pthread_mutex_unlock(&g_motor_lock);
    return 0;
}

/* ═══ Query motor busy state ═══════════════════════════════ */
int k4w_motor_is_busy(void) { return g_motor_busy; }
double k4w_motor_get_current_tilt(void) { return g_current_tilt; }
