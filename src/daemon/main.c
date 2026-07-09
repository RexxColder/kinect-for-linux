#include "k4w_internal.h"
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>

static volatile sig_atomic_t g_running = 1;
static k4w_state_t g_state;
static int g_pid_fd = -1;

static void sig_handler(int sig) {
    if (sig == SIGSEGV) {
        K4W_LOG("SEGFAULT! signal=%d\n", sig);
        _exit(1);
    }
    g_running = 0;
    g_state.running = false;
}

static void *socket_thread(void *arg) {
    int sock_fd = *(int *)arg;
    K4W_LOG("Socket thread: listening on fd=%d\n", sock_fd);
    while (g_running) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock_fd, &fds);
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        int r = select(sock_fd + 1, &fds, NULL, NULL, &tv);
        if (r > 0 && FD_ISSET(sock_fd, &fds)) {
            K4W_LOG("Socket thread: connection incoming\n");
            int client = k4w_socket_accept(sock_fd);
            if (client >= 0) {
                K4W_LOG("Socket thread: accepted client fd=%d\n", client);
                k4w_socket_handle(client);
            }
        }
    }
    return NULL;
}

static int acquire_pid_lock(void) {
    g_pid_fd = open("/tmp/k4w.pid", O_CREAT | O_EXCL | O_WRONLY, 0666);
    if (g_pid_fd < 0) {
        if (errno == EEXIST) {
            FILE *f = fopen("/tmp/k4w.pid", "r");
            if (f) {
                int old_pid = 0;
                fscanf(f, "%d", &old_pid);
                fclose(f);
                if (old_pid > 0 && kill(old_pid, 0) == 0) {
                    K4W_LOG("Another instance running (PID %d)\n", old_pid);
                    return -1;
                }
                K4W_LOG("Stale PID (dead %d), removing\n", old_pid);
                unlink("/tmp/k4w.pid");
            } else {
                unlink("/tmp/k4w.pid");
            }
            g_pid_fd = open("/tmp/k4w.pid", O_CREAT | O_EXCL | O_WRONLY, 0666);
            if (g_pid_fd < 0) return -1;
        } else {
            K4W_LOG("PID file error: %s\n", strerror(errno));
            return -1;
        }
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d\n", getpid());
    write(g_pid_fd, buf, strlen(buf));
    return 0;
}

static void release_pid_lock(void) {
    if (g_pid_fd >= 0) { close(g_pid_fd); g_pid_fd = -1; }
    unlink("/tmp/k4w.pid");
}

int main(int argc, char **argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--setup-only") == 0) return k4w_firmware_setup();
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGSEGV, sig_handler);

    K4W_LOG("k4wd v1.0.0\n");

    if (acquire_pid_lock() < 0) return 1;

    k4w_firmware_setup();
    k4w_config_t cfg;
    k4w_config_load(&cfg);

    if (k4w_kinect_init(&g_state, &cfg) < 0) {
        K4W_LOG("Init failed\n");
        release_pid_lock();
        return 1;
    }

    /* Initialize motor control via audio device */
    k4w_motor_init(NULL);

    /* Set state for kinect module (needed for motor pause/resume) */
    k4w_kinect_set_state(&g_state);

    /* Initialize PipeWire video source for camera detection */
    k4w_pw_source_init(cfg.v4l2_width, cfg.v4l2_height, 30);

    /* Start audio thread if enabled */
    if (cfg.enable_audio) {
        k4w_kinect_start_audio(&g_state);
    }

    int sock_fd = k4w_socket_init();
    if (sock_fd < 0) K4W_LOG("Socket init failed\n");
    k4w_socket_set_state(&g_state);

    pthread_t sock_tid;
    int sock_ok = (sock_fd >= 0);
    if (sock_ok) {
        K4W_LOG("Creating socket thread (fd=%d)\n", sock_fd);
        int r = pthread_create(&sock_tid, NULL, socket_thread, &sock_fd);
        K4W_LOG("Socket thread created: %d\n", r);
    }

    K4W_LOG("Running\n");
    k4w_kinect_run(&g_state);

    g_running = 0;
    k4w_pw_source_stop();
    k4w_kinect_stop(&g_state);

    if (sock_ok) {
        close(sock_fd);
        unlink(K4W_SOCKET_PATH);
        pthread_join(sock_tid, NULL);
    }

    release_pid_lock();
    K4W_LOG("Stopped\n");
    return 0;
}
