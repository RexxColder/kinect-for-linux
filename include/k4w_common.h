#ifndef K4W_COMMON_H
#define K4W_COMMON_H

#include <stdint.h>
#include <stdbool.h>

/* ─── SHM Ring Buffer Header ──────────────────────── */
#define K4W_SHM_DEPTH  "/k4w_depth"
#define K4W_SHM_VIDEO  "/k4w_video"
#define K4W_SHM_AUDIO  "/k4w_audio"

#define K4W_RING_SIZE   3
#define K4W_DEPTH_W     640
#define K4W_DEPTH_H     480
#define K4W_DEPTH_BPP   2   /* 11-bit stored as uint16 */
#define K4W_VIDEO_W     640
#define K4W_VIDEO_H     480
#define K4W_VIDEO_BPP   3   /* RGB */
#define K4W_AUDIO_RATE  16000
#define K4W_AUDIO_CH    4
#define K4W_AUDIO_SAMPLES 512

typedef struct {
    uint32_t magic;        /* 0x4B345750 = "K4WP" */
    uint32_t frame_size;   /* bytes per frame */
    uint32_t w;
    uint32_t h;
    uint32_t bpp;
    uint32_t write_idx;    /* next frame to write */
    uint32_t read_idx;     /* next frame to read */
    uint32_t seq;          /* monotonic frame counter */
    uint64_t timestamp_ms; /* ms since epoch */
    uint8_t  data[];       /* ring buffer: frame_size * K4W_RING_SIZE */
} k4w_shm_header_t;

#define K4W_SHM_MAGIC 0x4B345750

/* ─── Socket Commands ─────────────────────────────── */
#define K4W_SOCKET_PATH "/tmp/k4w.sock"
#define K4W_BUF_SIZE    4096

typedef enum {
    K4W_CMD_TILT,
    K4W_CMD_LED,
    K4W_CMD_STATUS,
    K4W_CMD_ACCEL,
    K4W_CMD_MOTOR_INFO,
} k4w_cmd_type_t;

typedef struct {
    k4w_cmd_type_t type;
    union {
        int32_t  tilt_angle;
        int32_t  led_color;
        char     reserved[252];
    };
} k4w_cmd_t;

typedef struct {
    bool     ok;
    float    tilt_deg;
    float    accel_x, accel_y, accel_z;
    char     v4l2_dev[64];
    char     audio_src[64];
    uint32_t depth_seq;
    uint32_t video_seq;
    uint32_t audio_seq;
} k4w_status_t;

/* ─── LED Colors (freenect_led_options) ───────────── */
#define K4W_LED_OFF          0
#define K4W_LED_GREEN        1
#define K4W_LED_RED          2
#define K4W_LED_YELLOW       3
#define K4W_LED_BLINK_GREEN  4
#define K4W_LED_BLINK_RED    5
#define K4W_LED_BLINK_YELLOW 6

#endif /* K4W_COMMON_H */
