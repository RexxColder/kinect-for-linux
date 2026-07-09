#ifndef K4W_PW_SOURCE_H
#define K4W_PW_SOURCE_H

#include <stdint.h>

/* Initialize PipeWire source node. Returns 0 on success. */
int k4w_pw_source_init(int width, int height, int fps);

/* Push an RGB24 frame to the PipeWire source. Thread-safe. */
int k4w_pw_source_push_frame(const uint8_t *rgb, int width, int height);

/* Stop and destroy the PipeWire source. */
void k4w_pw_source_stop(void);

#endif
