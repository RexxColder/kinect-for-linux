#include "k4w_internal.h"
#ifdef HAVE_PULSEAUDIO

#include <pulse/simple.h>
#include <pulse/error.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_null_sink_module = -1;
static int g_remap_module = -1;

static int ensure_null_sink(const char *sink_name) {
    /* Create a PulseAudio null-sink so consumers can read from its monitor.
     * Also create a remap source so Discord/browsers can detect it.
     * Returns the module index for later cleanup, or -1 on error. */
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
        "pactl load-module module-null-sink "
        "sink_name=%s sink_properties=device.description=\"Kinect+Microphone\" "
        "rate=%d channels=1",
        sink_name, K4W_AUDIO_RATE);
    FILE *p = popen(cmd, "r");
    if (!p) return -1;

    int module_idx = -1;
    char line[64] = {0};
    if (fgets(line, sizeof(line), p)) {
        module_idx = atoi(line);
    }
    int rc = pclose(p);
    if (rc != 0 || module_idx < 0) {
        K4W_LOG("Audio: failed to create null-sink '%s'\n", sink_name);
        return -1;
    }

    /* Create remap source so Discord/browsers detect it as input */
    snprintf(cmd, sizeof(cmd),
        "pactl load-module module-remap-source "
        "master=%s.monitor "
        "source_name=Kinect-Microphone "
        "source_properties=device.description=\"Kinect Microphone\"",
        sink_name);
    p = popen(cmd, "r");
    if (p) {
        if (fgets(line, sizeof(line), p))
            g_remap_module = atoi(line);
        pclose(p);
    }

    return module_idx;
}

static void unload_null_sink(int module_idx) {
    if (module_idx >= 0) {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "pactl unload-module %d", module_idx);
        system(cmd);
    }
    if (g_remap_module >= 0) {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "pactl unload-module %d", g_remap_module);
        system(cmd);
        g_remap_module = -1;
    }
}

void *k4w_audio_pulse_open(const char *source_name) {
    /* source_name is the sink name — create null-sink first */
    g_null_sink_module = ensure_null_sink(source_name);
    if (g_null_sink_module < 0) {
        K4W_LOG("Audio: could not create null-sink, PulseAudio capture may not work\n");
        /* Continue anyway — try to open PulseAudio in case sink already exists */
    }

    pa_sample_spec ss = {
        .format = PA_SAMPLE_S16LE,
        .rate = K4W_AUDIO_RATE,
        .channels = 1
    };

    int error;
    /* PA_STREAM_PLAYBACK: write audio INTO the sink.
     * Consumers read from "Monitor of <sink_name>" source. */
    pa_simple *s = pa_simple_new(NULL, source_name, PA_STREAM_PLAYBACK,
                                 source_name, "Kinect Microphone",
                                 &ss, NULL, NULL, &error);
    if (!s) {
        K4W_LOG("PulseAudio error: %s\n", pa_strerror(error));
        unload_null_sink(g_null_sink_module);
        g_null_sink_module = -1;
        return NULL;
    }

    K4W_LOG("PulseAudio sink '%s' opened, module %d (consumers use 'Monitor of %s')\n",
            source_name, g_null_sink_module, source_name);
    return (void *)s;
}

int k4w_audio_pulse_write(void *sink, const int16_t *samples, int count) {
    if (!sink) return -1;
    pa_simple *s = (pa_simple *)sink;
    int error;
    if (pa_simple_write(s, samples, count * sizeof(int16_t), &error) < 0) {
        K4W_LOG("PulseAudio write error: %s\n", pa_strerror(error));
        return -1;
    }
    return 0;
}

void k4w_audio_pulse_close(void *sink) {
    if (!sink) {
        unload_null_sink(g_null_sink_module);
        g_null_sink_module = -1;
        return;
    }
    pa_simple *s = (pa_simple *)sink;
    pa_simple_free(s);
    unload_null_sink(g_null_sink_module);
    g_null_sink_module = -1;
}

#else

void *k4w_audio_pulse_open(const char *name) {
    (void)name;
    K4W_LOG("PulseAudio not compiled in\n");
    return NULL;
}

int k4w_audio_pulse_write(void *sink, const int16_t *s, int c) {
    (void)sink; (void)s; (void)c;
    return -1;
}

void k4w_audio_pulse_close(void *sink) { (void)sink; }

#endif
