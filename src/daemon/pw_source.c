#include "k4w_internal.h"
#ifdef HAVE_PIPEWIRE

#include <pipewire/pipewire.h>
#include <spa/param/video/raw.h>
#include <spa/param/video/raw-utils.h>
#include <spa/param/video/format-utils.h>
#include <spa/param/format-utils.h>
#include <spa/pod/builder.h>
#include <spa/utils/result.h>
#include <spa/buffer/meta.h>
#include <string.h>
#include <stdlib.h>

static struct pw_thread_loop *s_loop;
static struct pw_context *s_context;
static struct pw_stream *s_stream;
static struct spa_hook s_stream_listener;
static int s_width, s_height, s_stride;
static uint8_t *s_frame_buf;
static int s_frame_size;

static void on_process(void *userdata) {
    (void)userdata;
    struct pw_buffer *buf = pw_stream_dequeue_buffer(s_stream);
    if (!buf) return;

    struct spa_data *d = &buf->buffer->datas[0];
    uint8_t *dst = d->data;
    if (!dst || !s_frame_buf) {
        pw_stream_queue_buffer(s_stream, buf);
        return;
    }

    memcpy(dst, s_frame_buf, s_frame_size);

    struct spa_chunk *chunk = d->chunk;
    chunk->offset = 0;
    chunk->size = s_frame_size;
    chunk->stride = s_stride;
    chunk->flags = SPA_CHUNK_FLAG_NONE;

    pw_stream_queue_buffer(s_stream, buf);
}

static void on_param_changed(void *userdata, uint32_t id, const struct spa_pod *param) {
    (void)userdata;
    if (!param || id != SPA_PARAM_Format) return;

    struct spa_video_info_raw info;
    spa_format_video_raw_parse(param, &info);
    s_width = info.size.width;
    s_height = info.size.height;
    s_stride = s_width * 4;
    s_frame_size = s_stride * s_height;
    K4W_LOG("PipeWire source: format changed %dx%d\n", s_width, s_height);
}

static void on_state_changed(void *userdata, enum pw_stream_state old,
                             enum pw_stream_state state, const char *error) {
    (void)userdata; (void)error;
    K4W_LOG("PipeWire source: state %s -> %s\n",
            pw_stream_state_as_string(old), pw_stream_state_as_string(state));
}

static const struct pw_stream_events stream_events = {
    PW_VERSION_STREAM_EVENTS,
    .process = on_process,
    .param_changed = on_param_changed,
    .state_changed = on_state_changed,
};

int k4w_pw_source_init(int width, int height, int fps) {
    s_width = width;
    s_height = height;
    s_stride = width * 4;
    s_frame_size = s_stride * s_height;
    s_frame_buf = calloc(1, s_frame_size);
    s_loop = NULL;
    s_context = NULL;
    s_stream = NULL;

    pw_init(NULL, NULL);

    s_loop = pw_thread_loop_new("k4w-pw-source", NULL);
    if (!s_loop) {
        K4W_LOG("PipeWire source: failed to create thread loop\n");
        return -1;
    }

    struct pw_loop *loop = pw_thread_loop_get_loop(s_loop);
    s_context = pw_context_new(loop, NULL, 0);
    if (!s_context) {
        K4W_LOG("PipeWire source: failed to create context\n");
        pw_thread_loop_destroy(s_loop);
        s_loop = NULL;
        return -1;
    }

    struct pw_core *core = pw_context_connect(s_context, NULL, 0);
    if (!core) {
        K4W_LOG("PipeWire source: failed to connect core\n");
        pw_context_destroy(s_context);
        s_context = NULL;
        pw_thread_loop_destroy(s_loop);
        s_loop = NULL;
        return -1;
    }

    s_stream = pw_stream_new(
        core,
        "Kinect Camera",
        pw_properties_new(
            PW_KEY_MEDIA_TYPE, "Video",
            PW_KEY_MEDIA_CATEGORY, "Source",
            PW_KEY_MEDIA_ROLE, "Camera",
            PW_KEY_NODE_NAME, "Kinect Camera",
            PW_KEY_NODE_DESCRIPTION, "Kinect Camera",
            PW_KEY_NODE_VIRTUAL, "true",
            NULL));

    if (!s_stream) {
        K4W_LOG("PipeWire source: failed to create stream\n");
        pw_core_disconnect(core);
        pw_context_destroy(s_context);
        s_context = NULL;
        pw_thread_loop_destroy(s_loop);
        s_loop = NULL;
        return -1;
    }

    pw_stream_add_listener(s_stream, &s_stream_listener, &stream_events, NULL);

    uint8_t buffer[1024];
    struct spa_pod_builder b;
    b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
    const struct spa_pod *params[1];

    struct spa_video_info_raw info;
    memset(&info, 0, sizeof(info));
    info.format = SPA_VIDEO_FORMAT_BGRx;
    info.size.width = width;
    info.size.height = height;
    info.framerate.num = fps;
    info.framerate.denom = 1;
    info.max_framerate.num = fps;
    info.max_framerate.denom = 1;

    params[0] = spa_format_video_raw_build(&b, SPA_FORMAT_VIDEO_format, &info);

    pw_thread_loop_start(s_loop);

    int ret = pw_stream_connect(s_stream,
                                PW_DIRECTION_OUTPUT,
                                PW_ID_ANY,
                                PW_STREAM_FLAG_AUTOCONNECT | PW_STREAM_FLAG_MAP_BUFFERS,
                                params, 1);
    if (ret < 0) {
        K4W_LOG("PipeWire source: connect failed: %s\n", spa_strerror(ret));
        pw_stream_destroy(s_stream);
        s_stream = NULL;
        pw_thread_loop_stop(s_loop);
        pw_context_destroy(s_context);
        s_context = NULL;
        pw_thread_loop_destroy(s_loop);
        s_loop = NULL;
        return -1;
    }

    K4W_LOG("PipeWire source: initialized %dx%d@%dfps\n", width, height, fps);
    return 0;
}

int k4w_pw_source_push_frame(const uint8_t *rgb, int width, int height) {
    if (!s_stream || !s_frame_buf) return -1;

    int src_stride = width * 3;
    int cols = width < s_width ? width : s_width;
    int rows = height < s_height ? height : s_height;

    for (int y = 0; y < rows; y++) {
        const uint8_t *src_row = rgb + y * src_stride;
        uint8_t *dst_row = s_frame_buf + y * s_stride;
        for (int x = 0; x < cols; x++) {
            dst_row[x * 4 + 0] = src_row[x * 3 + 2]; /* B */
            dst_row[x * 4 + 1] = src_row[x * 3 + 1]; /* G */
            dst_row[x * 4 + 2] = src_row[x * 3 + 0]; /* R */
            dst_row[x * 4 + 3] = 0;                   /* X */
        }
    }

    return 0;
}

void k4w_pw_source_stop(void) {
    if (s_stream) {
        pw_stream_set_active(s_stream, false);
        pw_stream_destroy(s_stream);
        s_stream = NULL;
    }
    if (s_context) {
        pw_context_destroy(s_context);
        s_context = NULL;
    }
    if (s_loop) {
        pw_thread_loop_stop(s_loop);
        pw_thread_loop_destroy(s_loop);
        s_loop = NULL;
    }
    free(s_frame_buf);
    s_frame_buf = NULL;
    K4W_LOG("PipeWire source: stopped\n");
}

#else

int k4w_pw_source_init(int width, int height, int fps) {
    (void)width; (void)height; (void)fps;
    K4W_LOG("PipeWire source not compiled in\n");
    return -1;
}

int k4w_pw_source_push_frame(const uint8_t *rgb, int width, int height) {
    (void)rgb; (void)width; (void)height;
    return -1;
}

void k4w_pw_source_stop(void) {}

#endif
