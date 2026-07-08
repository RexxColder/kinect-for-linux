#include "k4w_internal.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>

static k4w_state_t *g_state = NULL;

void k4w_socket_set_state(k4w_state_t *state) { g_state = state; }

int k4w_socket_init(void) {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, K4W_SOCKET_PATH, sizeof(addr.sun_path) - 1);
    unlink(K4W_SOCKET_PATH);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(fd); return -1; }
    if (listen(fd, 5) < 0) { close(fd); unlink(K4W_SOCKET_PATH); return -1; }
    chmod(K4W_SOCKET_PATH, 0666);
    K4W_LOG("Socket on %s\n", K4W_SOCKET_PATH);
    return fd;
}

int k4w_socket_accept(int fd) { return accept(fd, NULL, NULL); }

void k4w_socket_handle(int client_fd) {
    if (!g_state) { close(client_fd); return; }

    int cmd_buf[2];
    memset(cmd_buf, 0, sizeof(cmd_buf));
    ssize_t n = recv(client_fd, cmd_buf, sizeof(cmd_buf), 0);
    K4W_LOG("Socket: handle client fd=%d n=%zd cmd=%d arg=%d\n",
            client_fd, n, cmd_buf[0], cmd_buf[1]);
    if (n != sizeof(cmd_buf)) { close(client_fd); return; }
    int cmd_type = cmd_buf[0];
    int cmd_arg  = cmd_buf[1];

    k4w_status_t resp;
    memset(&resp, 0, sizeof(resp));
    resp.ok = true;

    /* ── Fill status fields (always) ── */
    strncpy(resp.v4l2_dev, g_state->v4l2_device, sizeof(resp.v4l2_dev) - 1);
    strncpy(resp.audio_src, g_state->audio_source, sizeof(resp.audio_src) - 1);
    if (g_state->depth_shm) resp.depth_seq = g_state->depth_shm->seq;
    if (g_state->video_shm) resp.video_seq = g_state->video_shm->seq;
    if (g_state->audio_shm) resp.audio_seq = g_state->audio_shm->seq;

    /* ── Process command ── */
    switch (cmd_type) {
    case K4W_CMD_TILT: {
        K4W_LOG("Socket: TILT command -> %d°\n", cmd_arg);
        k4w_kinect_audio_pause();
        /* K4W motor needs reset to 0° before descending */
        if (cmd_arg < (int)k4w_motor_get_current_tilt()) {
            K4W_LOG("Motor: resetting to 0° before descending\n");
            k4w_motor_set_tilt(NULL, 0.0);
            usleep(3000000);  /* 3s for motor to settle */
        }
        int r = k4w_motor_set_tilt(NULL, (double)cmd_arg);
        resp.tilt_deg = (float)k4w_motor_get_current_tilt();
        resp.ok = (r == 0);
        usleep(2000000);
        k4w_kinect_audio_resume();
        break;
    }
    case K4W_CMD_LED: {
        K4W_LOG("Socket: LED command -> %d\n", cmd_arg);
        k4w_kinect_audio_pause();
        int r = k4w_motor_set_led(NULL, cmd_arg);
        usleep(500000);
        k4w_kinect_audio_resume();
        resp.ok = (r == 0);
        break;
    }
    case K4W_CMD_ACCEL: {
        /* ACCEL shares USB endpoints with audio — must pause/resume */
        k4w_kinect_audio_pause();
        double ax, ay, az;
        int ar = k4w_motor_get_accel(NULL, &ax, &ay, &az);
        resp.tilt_deg = (float)k4w_motor_get_current_tilt();
        K4W_LOG("Socket: ACCEL result=%d ax=%.2f ay=%.2f az=%.2f tilt=%.1f\n", ar, ax, ay, az, resp.tilt_deg);
        if (ar == 0) {
            resp.accel_x = (float)ax;
            resp.accel_y = (float)ay;
            resp.accel_z = (float)az;
        }
        usleep(200000);
        k4w_kinect_audio_resume();
        break;
    }
    case K4W_CMD_MOTOR_INFO: {
        k4w_kinect_audio_pause();
        double tilt;
        k4w_motor_get_tilt(NULL, &tilt);
        resp.tilt_deg = (float)tilt;
        resp.ok = !k4w_motor_is_busy();
        usleep(500000);
        k4w_kinect_audio_resume();
        break;
    }
    case K4W_CMD_STATUS:
    default:
        break;
    }

    send(client_fd, &resp, sizeof(resp), 0);
    close(client_fd);
}
