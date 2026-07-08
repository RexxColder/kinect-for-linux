#include "k4w_internal.h"
#include <string.h>
#include <stdlib.h>

int k4w_config_load(k4w_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->v4l2_width = K4W_VIDEO_W;
    cfg->v4l2_height = K4W_VIDEO_H;
    cfg->enable_audio = true; /* audio via separate async thread */
    cfg->enable_v4l2 = true;
    strncpy(cfg->v4l2_device, "/dev/video100", sizeof(cfg->v4l2_device) - 1);
    strncpy(cfg->audio_source, "k4w-mic", sizeof(cfg->audio_source) - 1);

    const char *v = getenv("K4W_V4L2_DEVICE");
    if (v) strncpy(cfg->v4l2_device, v, sizeof(cfg->v4l2_device) - 1);

    v = getenv("K4W_AUDIO_SOURCE");
    if (v) strncpy(cfg->audio_source, v, sizeof(cfg->audio_source) - 1);

    v = getenv("K4W_NO_V4L2");
    if (v && strcmp(v, "1") == 0) cfg->enable_v4l2 = false;

    v = getenv("K4W_NO_AUDIO");
    if (v && strcmp(v, "1") == 0) cfg->enable_audio = false;

    return 0;
}
