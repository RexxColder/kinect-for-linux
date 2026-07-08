#include "k4w_internal.h"
#ifdef HAVE_V4L2

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

/* Convert RGB24 to YUYV (YUV 4:2:2) for maximum compatibility */
static void rgb_to_yuyv(const uint8_t *rgb, uint8_t *yuyv, int width, int height) {
    int size = width * height;
    for (int i = 0; i < size; i += 2) {
        int r1 = rgb[i*3+0], g1 = rgb[i*3+1], b1 = rgb[i*3+2];
        int r2 = rgb[(i+1)*3+0], g2 = rgb[(i+1)*3+1], b2 = rgb[(i+1)*3+2];

        int y1 = ((66*r1 + 129*g1 + 25*b1 + 128) >> 8) + 16;
        int u1 = ((-38*r1 - 74*g1 + 112*b1 + 128) >> 8) + 128;
        int v1 = ((112*r1 - 94*g1 - 18*b1 + 128) >> 8) + 128;
        int y2 = ((66*r2 + 129*g2 + 25*b2 + 128) >> 8) + 16;

        yuyv[i*2+0] = (uint8_t)(y1 < 0 ? 0 : y1 > 255 ? 255 : y1);
        yuyv[i*2+1] = (uint8_t)(u1 < 0 ? 0 : u1 > 255 ? 255 : u1);
        yuyv[i*2+2] = (uint8_t)(y2 < 0 ? 0 : y2 > 255 ? 255 : y2);
        yuyv[i*2+3] = (uint8_t)(v1 < 0 ? 0 : v1 > 255 ? 255 : v1);
    }
}

int k4w_v4l2_open(const char *device, int width, int height) {
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        K4W_LOG("V4L2: cannot open %s: ", device);
        perror("");
        return -1;
    }

    /* Try YUYV first (most compatible), fall back to RGB24 */
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.bytesperline = width * 2;
    fmt.fmt.pix.sizeimage = width * height * 2;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        /* Fall back to RGB24 */
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
        fmt.fmt.pix.bytesperline = width * 3;
        fmt.fmt.pix.sizeimage = width * height * 3;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            K4W_LOG("V4L2: cannot set format on %s\n", device);
            close(fd);
            return -1;
        }
        K4W_LOG("V4L2: opened %s (%dx%d RGB24)\n", device, width, height);
    } else {
        K4W_LOG("V4L2: opened %s (%dx%d YUYV)\n", device, width, height);
    }

    return fd;
}

int k4w_v4l2_write_frame(int fd, const void *rgb_data, int width, int height) {
    if (fd < 0) return -1;

    /* Query current format */
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) return -1;

    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
        /* Convert RGB to YUYV */
        uint8_t yuyv[width * height * 2];
        rgb_to_yuyv((const uint8_t *)rgb_data, yuyv, width, height);
        return write(fd, yuyv, width * height * 2);
    } else {
        /* Direct RGB write */
        return write(fd, rgb_data, width * height * 3);
    }
}

void k4w_v4l2_close(int fd) {
    if (fd >= 0) close(fd);
}

#else

int k4w_v4l2_open(const char *device, int width, int height) {
    (void)device; (void)width; (void)height;
    K4W_LOG("V4L2 not compiled in\n");
    return -1;
}

int k4w_v4l2_write_frame(int fd, const void *data, int w, int h) {
    (void)fd; (void)data; (void)w; (void)h;
    return -1;
}

void k4w_v4l2_close(int fd) { (void)fd; }

#endif
