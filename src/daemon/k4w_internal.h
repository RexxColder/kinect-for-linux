#ifndef K4W_INTERNAL_H
#define K4W_INTERNAL_H

#include "k4w_common.h"
#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <libfreenect/libfreenect.h>

#define K4W_LOG(fmt, ...) fprintf(stderr, "[k4wd] " fmt, ##__VA_ARGS__)

/* ─── SHM ─────────────────────────────────────────── */
int  k4w_shm_create(k4w_shm_header_t **hdr, const char *name, uint32_t frame_size);
void k4w_shm_destroy(const char *name);
void k4w_shm_write_frame(k4w_shm_header_t *hdr, const void *data, uint32_t size);
bool k4w_shm_read_frame(k4w_shm_header_t *hdr, void *out, uint32_t size, uint32_t *seq);

/* ─── V4L2 Loopback ───────────────────────────────── */
int  k4w_v4l2_open(const char *device, int width, int height);
int  k4w_v4l2_write_frame(int fd, const void *rgb_data, int width, int height);
void k4w_v4l2_close(int fd);

/* ─── PulseAudio ──────────────────────────────────── */
void *k4w_audio_pulse_open(const char *source_name);
int  k4w_audio_pulse_write(void *sink, const int16_t *samples, int count);
void k4w_audio_pulse_close(void *sink);

/* ─── Motor ───────────────────────────────────────── */
int  k4w_motor_init(freenect_context *ctx);
void k4w_motor_set_device(freenect_device *dev);
int  k4w_motor_set_led(freenect_device *dev, int color);
int  k4w_motor_set_tilt(freenect_device *dev, double angle);
int  k4w_motor_get_tilt(freenect_device *dev, double *angle);
int  k4w_motor_get_accel(freenect_device *dev, double *x, double *y, double *z);
int  k4w_motor_is_busy(void);
double k4w_motor_get_current_tilt(void);

/* ─── PipeWire Source ─────────────────────────────── */
int  k4w_pw_source_init(int width, int height, int fps);
int  k4w_pw_source_push_frame(const uint8_t *rgb, int width, int height);
void k4w_pw_source_stop(void);

/* ─── Socket Server ───────────────────────────────── */
int  k4w_socket_init(void);
int  k4w_socket_accept(int server_fd);
void k4w_socket_handle(int client_fd);

/* ─── Firmware ────────────────────────────────────── */
int  k4w_firmware_setup(void);

/* ─── Config ──────────────────────────────────────── */
typedef struct {
    int  v4l2_width;
    int  v4l2_height;
    bool enable_audio;
    bool enable_v4l2;
    char v4l2_device[128];
    char audio_source[128];
} k4w_config_t;

int  k4w_config_load(k4w_config_t *cfg);

/* ─── Kinect Event Loop ───────────────────────────── */
typedef enum { KINECT_STATE_RUNNING = 0, KINECT_STATE_WAITING } kinect_state_t;

typedef struct {
    freenect_context  *ctx;
    freenect_device   *kinect_dev;
    k4w_shm_header_t  *depth_shm;
    k4w_shm_header_t  *video_shm;
    k4w_shm_header_t  *audio_shm;
    int               v4l2_fd;
    void              *pulse_sink;  /* pa_simple* — void* to avoid 64-bit truncation */
    kinect_state_t    kinect_state;
    char              v4l2_device[128];
    char              audio_source[128];
    volatile bool     running;
    volatile bool     motor_paused;     /* true while motor commands are in progress */
    pthread_mutex_t   lock;
    char              usb_if2_path[64];  /* sysfs path for audio IF2 (dynamic) */
    char              usb_if3_path[64];  /* sysfs path for audio IF3 (dynamic) */
} k4w_state_t;

void k4w_socket_set_state(k4w_state_t *state);
void k4w_socket_init_motor(void);
int  k4w_kinect_init(k4w_state_t *state, const k4w_config_t *cfg);
int  k4w_kinect_run(k4w_state_t *state);
int  k4w_kinect_start_audio(k4w_state_t *state);
void k4w_kinect_set_state(k4w_state_t *state);
void k4w_kinect_audio_pause(void);
void k4w_kinect_audio_resume(void);
void k4w_kinect_stop(k4w_state_t *state);

#endif /* K4W_INTERNAL_H */
