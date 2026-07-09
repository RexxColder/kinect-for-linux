#include "k4w_internal.h"
#include <libfreenect/libfreenect_sync.h>
#include <alsa/asoundlib.h>
#include <libusb.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>

static k4w_state_t *g_state = NULL;
static pthread_t g_audio_tid;
static volatile bool g_audio_running = false;
static volatile bool g_audio_paused = false;
static volatile bool g_audio_dropped = false;

static void depth_cb(freenect_device *dev, void *data, uint32_t timestamp) {
    (void)dev; (void)timestamp;
    if (g_state && g_state->depth_shm) {
        k4w_shm_write_frame(g_state->depth_shm, data,
                            K4W_DEPTH_W * K4W_DEPTH_H * K4W_DEPTH_BPP);
    }
}

static void video_cb(freenect_device *dev, void *data, uint32_t timestamp) {
    (void)dev; (void)timestamp;
    if (!g_state) return;

    if (g_state->video_shm) {
        k4w_shm_write_frame(g_state->video_shm, data,
                            K4W_VIDEO_W * K4W_VIDEO_H * K4W_VIDEO_BPP);
    }

    if (g_state->v4l2_fd >= 0) {
        k4w_v4l2_write_frame(g_state->v4l2_fd, data,
                             K4W_VIDEO_W, K4W_VIDEO_H);
    }

    k4w_pw_source_push_frame(data, K4W_VIDEO_W, K4W_VIDEO_H);
}

static void audio_cb(freenect_device *dev, int num_samples,
                     int32_t *mic1, int32_t *mic2,
                     int32_t *mic3, int32_t *mic4,
                     int16_t *cancelled, void *unknown) {
    (void)dev; (void)mic1; (void)mic2; (void)mic3; (void)mic4;
    (void)unknown; (void)num_samples;

    if (!g_state) return;

    if (g_state->audio_shm) {
        k4w_shm_write_frame(g_state->audio_shm, cancelled,
                            num_samples * sizeof(int16_t));
    }

    if (g_state->pulse_sink) {
        k4w_audio_pulse_write(g_state->pulse_sink, cancelled, num_samples);
    }
}

/* ─── Audio Thread (ALSA capture) ────────────────────── */
static snd_pcm_t *g_pcm = NULL;
static char g_alsa_device[32] = {0};
static char g_usb_if2_path[64] = {0};
static char g_usb_if3_path[64] = {0};

void k4w_kinect_set_state(k4w_state_t *state) { g_state = state; }

static void *audio_thread_func(void *arg) {
    k4w_state_t *state = (k4w_state_t *)arg;
    K4W_LOG("Audio thread starting (ALSA capture)\n");

    /* Discover Kinect audio USB paths dynamically */
    if (g_usb_if2_path[0] == '\0') {
        /* Scan /sys/bus/usb/devices for Kinect composite device */
       DIR *dir = opendir("/sys/bus/usb/devices");
        if (dir) {
            struct dirent *ent;
            while ((ent = readdir(dir)) != NULL) {
                if (ent->d_name[0] == '.') continue;
                char vid_path[256], pid_path[256];
                char vid[8] = {0}, pid[8] = {0};
                snprintf(vid_path, sizeof(vid_path), "/sys/bus/usb/devices/%s/idVendor", ent->d_name);
                snprintf(pid_path, sizeof(pid_path), "/sys/bus/usb/devices/%s/idProduct", ent->d_name);
                FILE *fv = fopen(vid_path, "r");
                if (fv) { fgets(vid, sizeof(vid), fv); fclose(fv); }
                FILE *fp = fopen(pid_path, "r");
                if (fp) { fgets(pid, sizeof(pid), fp); fclose(fp); }
                /* Match Kinect audio device 045e:02bb */
                if (strstr(vid, "045e") && strstr(pid, "02bb")) {
                    snprintf(g_usb_if2_path, sizeof(g_usb_if2_path),
                             "/sys/bus/usb/drivers/snd-usb-audio/bind");
                    snprintf(g_usb_if3_path, sizeof(g_usb_if3_path),
                             "/sys/bus/usb/drivers/snd-usb-audio/bind");
                    /* Store the actual USB interface paths for bind */
                    /* Format: bus-dev:interface */
                    char if2[64], if3[64];
                    snprintf(if2, sizeof(if2), "%s:1.2", ent->d_name);
                    snprintf(if3, sizeof(if3), "%s:1.3", ent->d_name);
                    /* Store for later use by audio_resume */
                    strncpy(g_usb_if2_path, if2, sizeof(g_usb_if2_path) - 1);
                    strncpy(g_usb_if3_path, if3, sizeof(g_usb_if3_path) - 1);
                    K4W_LOG("Audio: discovered USB paths: %s, %s\n", g_usb_if2_path, g_usb_if3_path);
                    break;
                }
            }
            closedir(dir);
        }
        /* Fallback to hardcoded if discovery failed */
        if (g_usb_if2_path[0] == '\0') {
            strncpy(g_usb_if2_path, "1-6.1:1.2", sizeof(g_usb_if2_path) - 1);
            strncpy(g_usb_if3_path, "1-6.1:1.3", sizeof(g_usb_if3_path) - 1);
            K4W_LOG("Audio: using fallback USB paths: %s, %s\n", g_usb_if2_path, g_usb_if3_path);
        }
    }

    /* Rebind snd-usb-audio to Kinect audio IFs if not already bound */
    {
        bool needs_rebind = true;
        for (int card = 0; card < 10; card++) {
            char path[128];
            snprintf(path, sizeof(path), "/proc/asound/card%d/usbid", card);
            FILE *f = fopen(path, "r");
            if (!f) continue;
            char usbid[128] = {0};
            fgets(usbid, sizeof(usbid), f);
            fclose(f);
            if (strstr(usbid, "045e") && strstr(usbid, "02bb")) {
                needs_rebind = false;
                break;
            }
        }
        if (needs_rebind) {
            K4W_LOG("Audio: rebinding snd-usb-audio to Kinect IFs...\n");
            FILE *f1 = fopen("/sys/bus/usb/drivers/snd-usb-audio/bind", "w");
            if (f1) { fprintf(f1, "%s\n", g_usb_if2_path); fclose(f1); }
            FILE *f2 = fopen("/sys/bus/usb/drivers/snd-usb-audio/bind", "w");
            if (f2) { fprintf(f2, "%s\n", g_usb_if3_path); fclose(f2); }
            usleep(3000000); /* 3s for driver to settle */
        }
    }

    /* Find Kinect audio device by scanning cards via USB ID */
    bool found = false;
    for (int card = 0; card < 10; card++) {
        char path[128];
        snprintf(path, sizeof(path), "/proc/asound/card%d/usbid", card);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        char usbid[128] = {0};
        fgets(usbid, sizeof(usbid), f);
        fclose(f);
        if (strstr(usbid, "045e") && strstr(usbid, "02bb")) {
            snprintf(g_alsa_device, sizeof(g_alsa_device), "hw:%d,0", card);
            K4W_LOG("Audio: found Kinect on card %d\n", card);
            found = true;
            break;
        }
    }
    if (!found) {
        K4W_LOG("Audio: no Kinect audio device found\n");
        return NULL;
    }

    /* Open ALSA device ONCE - we keep it open for the lifetime of the thread */
    snd_pcm_hw_params_t *params;
    int err = snd_pcm_open(&g_pcm, g_alsa_device, SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        K4W_LOG("Audio: open '%s' failed: %s\n", g_alsa_device, snd_strerror(err));
        return NULL;
    }
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(g_pcm, params);
    snd_pcm_hw_params_set_access(g_pcm, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(g_pcm, params, SND_PCM_FORMAT_S32_LE);
    snd_pcm_hw_params_set_channels(g_pcm, params, 4);
    unsigned int rate = K4W_AUDIO_RATE;
    snd_pcm_hw_params_set_rate_near(g_pcm, params, &rate, 0);
    snd_pcm_hw_params_set_period_size(g_pcm, params, K4W_AUDIO_SAMPLES, 0);
    snd_pcm_hw_params(g_pcm, params);
    snd_pcm_prepare(g_pcm);

    K4W_LOG("Audio thread running (ALSA %dHz 4ch S32 → mono mixdown)\n", K4W_AUDIO_RATE);
    K4W_LOG("Audio: pulse_sink=%p audio_shm=%p\n", (void*)state->pulse_sink, (void*)state->audio_shm);
    g_audio_running = true;

    int32_t raw_buf[K4W_AUDIO_SAMPLES * 4];
    int16_t mono_buf[K4W_AUDIO_SAMPLES];
    int consecutive_errs = 0;
    bool was_paused = false;

    while (g_audio_running) {
        /* Pause: drop capture to free USB device for motor */
        if (g_audio_paused) {
            if (!was_paused && g_pcm) {
                snd_pcm_drop(g_pcm);
                g_audio_dropped = true;
                K4W_LOG("Audio: capture dropped\n");
            }
            was_paused = true;
            usleep(50000);
            continue;
        }

        /* Resume: reopen ALSA after motor detached IF 2/3 */
        if (was_paused) {
            if (g_pcm) {
                snd_pcm_close(g_pcm);
                g_pcm = NULL;
            }
            /* Reopen ALSA with retry (rebind may take time) */
            snd_pcm_hw_params_t *params;
            int err = -1;
            for (int retry = 0; retry < 10 && err != 0; retry++) {
                usleep(500000);  /* 500ms between retries */
                /* Rescan for device in case card number changed */
                for (int card = 0; card < 10; card++) {
                    char path[128];
                    snprintf(path, sizeof(path), "/proc/asound/card%d/usbid", card);
                    FILE *f = fopen(path, "r");
                    if (!f) continue;
                    char usbid[128] = {0};
                    fgets(usbid, sizeof(usbid), f);
                    fclose(f);
                    if (strstr(usbid, "045e") && strstr(usbid, "02bb")) {
                        snprintf(g_alsa_device, sizeof(g_alsa_device), "hw:%d,0", card);
                        break;
                    }
                }
                err = snd_pcm_open(&g_pcm, g_alsa_device, SND_PCM_STREAM_CAPTURE, 0);
                if (err == 0) break;
                K4W_LOG("Audio: reopen attempt %d failed: %s\n", retry+1, snd_strerror(err));
            }
            if (err == 0) {
                snd_pcm_hw_params_alloca(&params);
                snd_pcm_hw_params_any(g_pcm, params);
                snd_pcm_hw_params_set_access(g_pcm, params, SND_PCM_ACCESS_RW_INTERLEAVED);
                snd_pcm_hw_params_set_format(g_pcm, params, SND_PCM_FORMAT_S32_LE);
                snd_pcm_hw_params_set_channels(g_pcm, params, 4);
                unsigned int rate = K4W_AUDIO_RATE;
                snd_pcm_hw_params_set_rate_near(g_pcm, params, &rate, 0);
                snd_pcm_hw_params_set_period_size(g_pcm, params, K4W_AUDIO_SAMPLES, 0);
                snd_pcm_hw_params(g_pcm, params);
                snd_pcm_prepare(g_pcm);
                K4W_LOG("Audio: reopened %s after motor\n", g_alsa_device);
            } else {
                K4W_LOG("Audio: reopen failed: %s\n", snd_strerror(err));
            }
            was_paused = false;
            consecutive_errs = 0;
            continue;
        }

        if (!g_pcm) { usleep(100000); continue; }

        int frames = snd_pcm_readi(g_pcm, raw_buf, K4W_AUDIO_SAMPLES);
        if (frames < 0) {
            consecutive_errs++;
            if (consecutive_errs <= 3) {
                K4W_LOG("Audio: read error %d: %s\n", frames, snd_strerror(frames));
            }
            snd_pcm_recover(g_pcm, frames, 0);
            consecutive_errs = 0;
            continue;
        }
        if (consecutive_errs > 3) {
            K4W_LOG("Audio: recovered after %d errors\n", consecutive_errs);
        }
        consecutive_errs = 0;
        /* Convert S32 4ch to S16 mono: audio in upper 16 bits [31:16] */
        for (int i = 0; i < frames; i++) {
            int32_t s0 = raw_buf[i*4]   >> 16;
            int32_t s1 = raw_buf[i*4+1] >> 16;
            int32_t s2 = raw_buf[i*4+2] >> 16;
            int32_t s3 = raw_buf[i*4+3] >> 16;
            mono_buf[i] = (int16_t)((s0 + s1 + s2 + s3) / 4);
        }
        if (state->audio_shm) {
            k4w_shm_write_frame(state->audio_shm, mono_buf,
                                frames * sizeof(int16_t));
        }
        if (state->pulse_sink) {
            k4w_audio_pulse_write(state->pulse_sink, mono_buf, frames);
        }
    }

    if (g_pcm) { snd_pcm_close(g_pcm); g_pcm = NULL; }
    K4W_LOG("Audio thread stopped\n");
    return NULL;
}

int k4w_kinect_start_audio(k4w_state_t *state) {
    if (!state->audio_shm && !state->pulse_sink) {
        K4W_LOG("Audio: no SHM or PulseAudio configured\n");
        return -1;
    }
    return pthread_create(&g_audio_tid, NULL, audio_thread_func, state);
}

void k4w_kinect_audio_pause(void) {
    g_audio_paused = true;
    g_audio_dropped = false;
    /* Set motor_paused flag to stop main loop from re-opening devices */
    if (g_state) g_state->motor_paused = true;
    K4W_LOG("Audio: pause requested, waiting for drop...\n");
    for (int i = 0; i < 60 && !g_audio_dropped; i++) usleep(50000); /* 3s timeout */
    /* Fully close ALSA to release USB interface for motor */
    if (g_pcm && g_audio_dropped) {
        snd_pcm_close(g_pcm);
        g_pcm = NULL;
        K4W_LOG("Audio: ALSA closed for motor access\n");
    }
    /* Stop freenect_sync to release ALL USB interfaces on 02bb (motor IF0/IF1 included) */
    K4W_LOG("Audio: stopping freenect_sync for motor access...\n");
    freenect_sync_stop();
    usleep(1000000); /* 1s for libusb to fully release */
    K4W_LOG("Audio: paused (dropped=%d)\n", g_audio_dropped);
}

void k4w_kinect_audio_resume(void) {
    /* Clear motor_paused flag so main loop can re-open devices */
    if (g_state) g_state->motor_paused = false;
    /* Rebind snd-usb-audio to Kinect audio IFs using discovered paths */
    FILE *f1 = fopen("/sys/bus/usb/drivers/snd-usb-audio/bind", "w");
    if (f1) { fprintf(f1, "%s\n", g_usb_if2_path); fclose(f1); }
    FILE *f2 = fopen("/sys/bus/usb/drivers/snd-usb-audio/bind", "w");
    if (f2) { fprintf(f2, "%s\n", g_usb_if3_path); fclose(f2); }
    /* Wait for driver to rebind and ALSA card to appear */
    usleep(3000000); /* 3s for driver to settle */
    g_audio_paused = false;
    K4W_LOG("Audio: resumed after motor (rebind sent)\n");
}

int k4w_kinect_init(k4w_state_t *state, const k4w_config_t *cfg) {
    memset(state, 0, sizeof(*state));
    state->v4l2_fd = -1;
    state->pulse_sink = NULL;
    pthread_mutex_init(&state->lock, NULL);

    /* Check Kinect first — skip USB reset if already working */
    K4W_LOG("Checking Kinect via sync API...\n");
    void *test_depth = NULL;
    uint32_t test_ts = 0;
    int r = freenect_sync_get_depth(&test_depth, &test_ts, 0, FREENECT_DEPTH_11BIT);
    if (r != 0) {
        /* Only reset USB if sync API fails */
        K4W_LOG("Sync failed, resetting camera USB device...\n");
        libusb_context *lctx;
        libusb_init(&lctx);
        libusb_device_handle *cam = libusb_open_device_with_vid_pid(lctx, 0x045e, 0x02ae);
        if (cam) {
            libusb_reset_device(cam);
            libusb_close(cam);
            K4W_LOG("Camera reset, waiting for re-enumeration...\n");
            sleep(3);
        }
        libusb_exit(lctx);

        /* Retry sync after reset */
        for (int attempt = 0; attempt < 5; attempt++) {
            r = freenect_sync_get_depth(&test_depth, &test_ts, 0, FREENECT_DEPTH_11BIT);
            if (r == 0) break;
            K4W_LOG("Sync attempt %d failed (error %d), retrying...\n", attempt + 1, r);
            freenect_sync_stop();
            sleep(2);
        }
    }
    if (r != 0) {
        K4W_LOG("Cannot access Kinect (sync error %d)\n", r);
        return -1;
    }
    K4W_LOG("Kinect accessible (depth test OK)\n");

    /* Create SHM segments — check each return */
    if (k4w_shm_create(&state->depth_shm, K4W_SHM_DEPTH,
                    K4W_DEPTH_W * K4W_DEPTH_H * K4W_DEPTH_BPP) < 0) {
        K4W_LOG("Failed to create depth SHM\n");
        return -1;
    }
    state->depth_shm->w = K4W_DEPTH_W;
    state->depth_shm->h = K4W_DEPTH_H;
    state->depth_shm->bpp = K4W_DEPTH_BPP;

    if (k4w_shm_create(&state->video_shm, K4W_SHM_VIDEO,
                    K4W_VIDEO_W * K4W_VIDEO_H * K4W_VIDEO_BPP) < 0) {
        K4W_LOG("Failed to create video SHM\n");
        k4w_shm_destroy(K4W_SHM_DEPTH);
        return -1;
    }
    state->video_shm->w = K4W_VIDEO_W;
    state->video_shm->h = K4W_VIDEO_H;
    state->video_shm->bpp = K4W_VIDEO_BPP;

    if (cfg->enable_audio) {
        if (k4w_shm_create(&state->audio_shm, K4W_SHM_AUDIO,
                        K4W_AUDIO_SAMPLES * sizeof(int16_t)) == 0) {
            state->pulse_sink = k4w_audio_pulse_open(cfg->audio_source);
        }
    }

    if (cfg->enable_v4l2) {
        state->v4l2_fd = k4w_v4l2_open(cfg->v4l2_device,
                                         cfg->v4l2_width, cfg->v4l2_height);
    }

    state->running = true;
    strncpy(state->v4l2_device, cfg->v4l2_device, sizeof(state->v4l2_device) - 1);
    strncpy(state->audio_source, cfg->audio_source, sizeof(state->audio_source) - 1);
    K4W_LOG("Kinect initialized\n");
    return 0;
}

int k4w_kinect_run(k4w_state_t *state) {
    K4W_LOG("Starting sync polling loop\n");

    uint32_t depth_seq = 0, video_seq = 0;
    int iter = 0;
    int consecutive_errors = 0;
    bool device_ok = true;
    bool was_motor_paused = false;

    while (state->running) {
        /* Skip sync polling while motor commands are in progress */
        if (state->motor_paused) {
            was_motor_paused = true;
            usleep(100000);
            continue;
        }
        /* After motor resumes, let freenect_sync re-initialize */
        if (was_motor_paused) {
            was_motor_paused = false;
            consecutive_errors = 0;
            K4W_LOG("Motor resumed, waiting for sync re-init...\n");
            usleep(2000000); /* 2s for USB + freenect_sync to stabilize */
        }
        /* Poll depth */
        void *depth_data = NULL;
        uint32_t depth_ts = 0;
        int dr = freenect_sync_get_depth(&depth_data, &depth_ts, 0, FREENECT_DEPTH_11BIT);

        if (dr == 0) {
            if (depth_data && state->depth_shm) {
                k4w_shm_write_frame(state->depth_shm, depth_data,
                    K4W_DEPTH_W * K4W_DEPTH_H * K4W_DEPTH_BPP);
            }
            consecutive_errors = 0;
            device_ok = true;
        } else {
            consecutive_errors++;
            if (consecutive_errors > 30) { /* ~1 second of failures */
                if (device_ok) {
                    K4W_LOG("USB device lost (error %d), stopping streams...\n", dr);
                    device_ok = false;
                    freenect_sync_stop();
                }
                /* Reconnection backoff: 2 seconds */
                usleep(2000000);
                K4W_LOG("Attempting device reconnection...\n");
                consecutive_errors = 0;
                continue;
            }
        }

        /* Poll video */
        void *video_data = NULL;
        uint32_t video_ts = 0;
        if (freenect_sync_get_video(&video_data, &video_ts, 0, FREENECT_VIDEO_RGB) == 0) {
            if (video_data && state->video_shm) {
                k4w_shm_write_frame(state->video_shm, video_data,
                    K4W_VIDEO_W * K4W_VIDEO_H * K4W_VIDEO_BPP);
            }
            if (video_data && state->v4l2_fd >= 0) {
                k4w_v4l2_write_frame(state->v4l2_fd, video_data,
                    K4W_VIDEO_W, K4W_VIDEO_H);
            }
            if (video_data) {
                k4w_pw_source_push_frame(video_data, K4W_VIDEO_W, K4W_VIDEO_H);
            }

            /* After first frame written, restart WirePlumber for camera detection */
            static int wp_restarted = 0;
            if (!wp_restarted && video_data && state->v4l2_fd >= 0) {
                wp_restarted = 1;
                K4W_LOG("Restarting WirePlumber after first frame...\n");
                const char *xdg = getenv("XDG_RUNTIME_DIR");
                char cmd[256];
                if (xdg)
                    snprintf(cmd, sizeof(cmd),
                        "XDG_RUNTIME_DIR=%s systemctl --user restart wireplumber 2>/dev/null", xdg);
                else
                    snprintf(cmd, sizeof(cmd),
                        "systemctl --user restart wireplumber 2>/dev/null");
                system(cmd);
            }
        }

        if (++iter % 30 == 0) {
            if (state->depth_shm && state->video_shm) {
                K4W_LOG("iter=%d depth_seq=%u video_seq=%u\n", iter,
                    state->depth_shm->seq, state->video_shm->seq);
            }
        }

        usleep(30000); /* ~30fps */
    }

    freenect_sync_stop();
    return 0;
}

void k4w_kinect_stop(k4w_state_t *state) {
    state->running = false;
    g_audio_running = false;

    /* Stop audio thread first */
    if (g_audio_tid) {
        pthread_join(g_audio_tid, NULL);
        g_audio_tid = 0;
    }

    pthread_mutex_lock(&state->lock);

    /* Sync API handles cleanup via freenect_sync_stop() */
    freenect_sync_stop();

    k4w_v4l2_close(state->v4l2_fd);
    k4w_audio_pulse_close(state->pulse_sink);

    k4w_shm_destroy(K4W_SHM_DEPTH);
    k4w_shm_destroy(K4W_SHM_VIDEO);
    k4w_shm_destroy(K4W_SHM_AUDIO);

    pthread_mutex_unlock(&state->lock);
    pthread_mutex_destroy(&state->lock);

    K4W_LOG("Kinect stopped\n");
}
