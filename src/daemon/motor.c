#include "k4w_internal.h"
#include <libusb.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

/* ═══ K4W Motor Protocol (via audio device 02bb) ═══════════ */
/*
 * Commands go through audio device endpoint 0x01 (OUT) / 0x81 (IN).
 * Command struct (20 bytes): magic(4) tag(4) arg1(4) cmd(4) arg2(4)
 * Reply struct (12 bytes):   magic(4) tag(4) status(4)
 *
 * magic = 0x06022009 (cmd) / 0x0a6fe000 (reply)
 * cmd 0x803b = set tilt, arg2 = angle (-31..+31)
 * cmd 0x10   = set LED,   arg2 = led_state (1=off,2=blink,3=green,4=red)
 * cmd 0x8032 = get accel, arg1 = 0x68
 */

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

/* ═══ Motor State ══════════════════════════════════════════ */
static libusb_context *g_usb_ctx = NULL;
static uint32_t g_tag_seq = 0;
static pthread_mutex_t g_motor_lock = PTHREAD_MUTEX_INITIALIZER;
static double g_current_tilt = 0;
static double g_target_tilt = 0;
static int g_motor_busy = 0;

/* ═══ Open/close audio device per command ════════════════ */
static libusb_device_handle *motor_open_device(void) {
    libusb_device_handle *dev = libusb_open_device_with_vid_pid(g_usb_ctx, 0x045e, 0x02bb);
    if (!dev) {
        K4W_LOG("Motor: open failed\n");
        return NULL;
    }
    /* Detach ALL kernel drivers — ALSA needs IF 2/3 but we need IF 0/1.
     * Audio is already paused, so it's safe to detach. */
    for (int i = 0; i < 4; i++) {
        if (libusb_kernel_driver_active(dev, i) == 1) {
            libusb_detach_kernel_driver(dev, i);
        }
    }
    int r0 = libusb_claim_interface(dev, 0);
    int r1 = libusb_claim_interface(dev, 1);
    if (r0 != 0 || r1 != 0) {
        K4W_LOG("Motor: claim failed IF0=%d IF1=%d\n", r0, r1);
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
    /* Let kernel re-attach snd-usb-audio to IF 2/3 */
    usleep(100000);
}

/* ═══ Low-level USB I/O ═══════════════════════════════════ */
static int motor_send_cmd(uint32_t cmd, uint32_t arg2, uint32_t arg1,
                           motor_reply_t *reply_out) {
    libusb_device_handle *dev = motor_open_device();
    if (!dev) return -1;

    motor_cmd_t cmd_pkt;
    cmd_pkt.magic = MOTOR_MAGIC_CMD;
    cmd_pkt.tag   = g_tag_seq++;
    cmd_pkt.arg1  = arg1;
    cmd_pkt.cmd   = cmd;
    cmd_pkt.arg2  = arg2;

    int transferred = 0;
    /* Bulk OUT may timeout (-7) but device still processes the command */
    libusb_bulk_transfer(dev, 0x01,
                          (unsigned char *)&cmd_pkt, 20,
                          &transferred, 2000);

    /* Read reply */
    unsigned char buf[512] = {0};
    int r = libusb_bulk_transfer(dev, 0x81, buf, 512, &transferred, 2000);
    if (r != 0) {
        K4W_LOG("Motor: bulk IN failed: %s\n", libusb_strerror(r));
        motor_close_device(dev);
        return r;
    }

    if (reply_out && transferred >= 12) {
        memcpy(reply_out, buf, 12);
        if (reply_out->magic != MOTOR_MAGIC_REPLY) {
            K4W_LOG("Motor: bad reply magic 0x%08x\n", reply_out->magic);
            motor_close_device(dev);
            return -1;
        }
    }
    motor_close_device(dev);
    return 0;
}

/* ═══ Open audio device for motor control ═════════════════ */
int k4w_motor_init(freenect_context *ctx) {
    (void)ctx;
    libusb_init(&g_usb_ctx);
    K4W_LOG("Motor: libusb context initialized\n");
    g_current_tilt = 0;
    g_target_tilt = 0;
    return 0;
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
    motor_reply_t reply;
    int r = motor_send_cmd(CMD_SET_LED, led, 0, &reply);
    pthread_mutex_unlock(&g_motor_lock);

    if (r == 0)
        K4W_LOG("Motor: LED set to %d\n", led);
    return r;
}

/* ═══ Tilt Control (with movement wait) ════════════════════ */
int k4w_motor_set_tilt(freenect_device *dev, double angle) {
    (void)dev;

    if (angle > 31) angle = 31;
    if (angle < -31) angle = -31;

    pthread_mutex_lock(&g_motor_lock);
    g_target_tilt = angle;
    g_motor_busy = 1;

    K4W_LOG("Motor: tilt %.0f°\n", angle);

    /* Open device and keep it open for the entire operation */
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
        g_current_tilt = (double)tilt_raw;
        K4W_LOG("Motor: final tilt %.0f° (target %.0f°)\n", g_current_tilt, g_target_tilt);
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

    int r = libusb_bulk_transfer(usbdev, 0x01,
                                  (unsigned char *)&cmd, 16,
                                  &transferred, 2000);
    if (r != 0) { motor_close_device(usbdev); pthread_mutex_unlock(&g_motor_lock); return r; }

    /* Read 104-byte accel data (magic=0x00000001, tilt at offset 28) */
    r = libusb_bulk_transfer(usbdev, 0x81, buf, 512, &transferred, 2000);
    if (r != 0) { motor_close_device(usbdev); pthread_mutex_unlock(&g_motor_lock); return r; }

    if (transferred >= 32) {
        int32_t tilt_raw;
        memcpy(&tilt_raw, buf + 28, 4);
        *angle = (double)tilt_raw;
        g_current_tilt = *angle;
    } else {
        *angle = g_current_tilt;
    }

    /* Read trailing 12-byte reply */
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

    int r = libusb_bulk_transfer(usbdev, 0x01,
                                  (unsigned char *)&cmd, 16,
                                  &transferred, 500);
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
