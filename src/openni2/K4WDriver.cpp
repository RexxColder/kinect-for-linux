/*
 * K4W OpenNI2 Driver Plugin
 * Kinect v1 (K4W 1473) → OpenNI2 bridge
 * Reads depth/video from k4wd daemon SHM ring buffers
 */
#include "Driver/OniDriverAPI.h"
#include "XnLib.h"
#include "XnHash.h"
#include "XnEvent.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

/* ─── SHM Header (must match k4w_common.h) ──────── */
#define K4W_MAGIC       0x4B345750
#define K4W_RING_SIZE   3
#define K4W_DEPTH_W     640
#define K4W_DEPTH_H     480
#define K4W_DEPTH_BPP   2
#define K4W_VIDEO_W     640
#define K4W_VIDEO_H     480
#define K4W_VIDEO_BPP   3

typedef struct {
    uint32_t magic;
    uint32_t frame_size;
    uint32_t w;
    uint32_t h;
    uint32_t bpp;
    uint32_t write_idx;
    uint32_t read_idx;
    uint32_t seq;
    uint64_t timestamp_ms;
    uint8_t  data[];
} K4W_ShmHeader;

static void *shm_open_ro(const char *name, size_t *out_size) {
    char path[256];
    snprintf(path, sizeof(path), "/dev/shm/%s", name);
    int fd = open(path, O_RDONLY);
    if (fd < 0) return NULL;
    struct stat st;
    fstat(fd, &st);
    void *p = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) return NULL;
    *out_size = st.st_size;
    return p;
}

/* ─── Depth Stream ──────────────────────────────── */
class K4WDepthStream : public oni::driver::StreamBase {
public:
    OniStatus start() {
        m_running = true;
        xnOSCreateThread(threadFunc, this, &m_thread);
        return ONI_STATUS_OK;
    }
    void stop() { m_running = false; xnOSWaitForThreadExit(m_thread, 2000); }

    OniStatus getProperty(int id, void *data, int *sz) {
        if (id == ONI_STREAM_PROPERTY_VIDEO_MODE && *sz == sizeof(OniVideoMode)) {
            OniVideoMode *m = (OniVideoMode *)data;
            m->pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
            m->fps = 30;
            m->resolutionX = K4W_DEPTH_W;
            m->resolutionY = K4W_DEPTH_H;
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_NOT_IMPLEMENTED;
    }

    OniStatus setProperty(int, const void*, int) { return ONI_STATUS_NOT_IMPLEMENTED; }

private:
    static XN_THREAD_PROC threadFunc(XN_THREAD_PARAM p) {
        ((K4WDepthStream *)p)->mainloop();
        XN_THREAD_PROC_RETURN(XN_STATUS_OK);
    }

    void mainloop() {
        int last_seq = -1;
        int frameId = 0;
        while (m_running) {
            size_t sz;
            void *p = shm_open_ro("k4w_depth", &sz);
            if (!p) { xnOSSleep(100); continue; }

            K4W_ShmHeader *hdr = (K4W_ShmHeader *)p;
            if (hdr->magic != K4W_MAGIC || sz < sizeof(K4W_ShmHeader)) {
                munmap(p, sz); xnOSSleep(50); continue;
            }

            int seq = hdr->seq;
            if (seq == last_seq || seq < 1) {
                munmap(p, sz); xnOSSleep(10); continue;
            }
            last_seq = seq;

            int wi = hdr->write_idx;
            int idx = (wi + K4W_RING_SIZE - 1) % K4W_RING_SIZE;
            uint16_t *raw = (uint16_t *)(hdr->data + idx * hdr->frame_size);

            int w = hdr->w, h = hdr->h;
            if (w == 0 || h == 0) { munmap(p, sz); xnOSSleep(50); continue; }

            OniFrame *frame = getServices().acquireFrame();
            if (!frame) { munmap(p, sz); continue; }

            /* Convert uint16 raw depth to mm */
            OniDepthPixel *dst = (OniDepthPixel *)frame->data;
            for (int i = 0; i < w * h; i++)
                dst[i] = raw[i];

            frame->frameIndex = frameId++;
            frame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
            frame->videoMode.resolutionX = w;
            frame->videoMode.resolutionY = h;
            frame->videoMode.fps = 30;
            frame->width = w;
            frame->height = h;
            frame->cropOriginX = frame->cropOriginY = 0;
            frame->croppingEnabled = FALSE;
            frame->sensorType = ONI_SENSOR_DEPTH;
            frame->stride = w * sizeof(OniDepthPixel);
            frame->timestamp = hdr->timestamp_ms * 1000;

            raiseNewFrame(frame);
            getServices().releaseFrame(frame);
            munmap(p, sz);

            xnOSSleep(10); /* ~30fps pacing */
        }
    }

    bool m_running = false;
    XN_THREAD_HANDLE m_thread;
};

/* ─── Color Stream ──────────────────────────────── */
class K4WColorStream : public oni::driver::StreamBase {
public:
    OniStatus start() {
        m_running = true;
        xnOSCreateThread(threadFunc, this, &m_thread);
        return ONI_STATUS_OK;
    }
    void stop() { m_running = false; xnOSWaitForThreadExit(m_thread, 2000); }

    OniStatus getProperty(int id, void *data, int *sz) {
        if (id == ONI_STREAM_PROPERTY_VIDEO_MODE && *sz == sizeof(OniVideoMode)) {
            OniVideoMode *m = (OniVideoMode *)data;
            m->pixelFormat = ONI_PIXEL_FORMAT_RGB888;
            m->fps = 30;
            m->resolutionX = K4W_VIDEO_W;
            m->resolutionY = K4W_VIDEO_H;
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_NOT_IMPLEMENTED;
    }

    OniStatus setProperty(int, const void*, int) { return ONI_STATUS_NOT_IMPLEMENTED; }

private:
    static XN_THREAD_PROC threadFunc(XN_THREAD_PARAM p) {
        ((K4WColorStream *)p)->mainloop();
        XN_THREAD_PROC_RETURN(XN_STATUS_OK);
    }

    void mainloop() {
        int last_seq = -1;
        int frameId = 0;
        while (m_running) {
            size_t sz;
            void *p = shm_open_ro("k4w_video", &sz);
            if (!p) { xnOSSleep(100); continue; }

            K4W_ShmHeader *hdr = (K4W_ShmHeader *)p;
            if (hdr->magic != K4W_MAGIC || sz < sizeof(K4W_ShmHeader)) {
                munmap(p, sz); xnOSSleep(50); continue;
            }

            int seq = hdr->seq;
            if (seq == last_seq || seq < 1) {
                munmap(p, sz); xnOSSleep(10); continue;
            }
            last_seq = seq;

            int wi = hdr->write_idx;
            int idx = (wi + K4W_RING_SIZE - 1) % K4W_RING_SIZE;
            uint8_t *rgb = hdr->data + idx * hdr->frame_size;

            int w = hdr->w, h = hdr->h;
            if (w == 0 || h == 0) { munmap(p, sz); xnOSSleep(50); continue; }

            OniFrame *frame = getServices().acquireFrame();
            if (!frame) { munmap(p, sz); continue; }

            memcpy(frame->data, rgb, w * h * 3);

            frame->frameIndex = frameId++;
            frame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_RGB888;
            frame->videoMode.resolutionX = w;
            frame->videoMode.resolutionY = h;
            frame->videoMode.fps = 30;
            frame->width = w;
            frame->height = h;
            frame->cropOriginX = frame->cropOriginY = 0;
            frame->croppingEnabled = FALSE;
            frame->sensorType = ONI_SENSOR_COLOR;
            frame->stride = w * 3;
            frame->timestamp = hdr->timestamp_ms * 1000;

            raiseNewFrame(frame);
            getServices().releaseFrame(frame);
            munmap(p, sz);

            xnOSSleep(10);
        }
    }

    bool m_running = false;
    XN_THREAD_HANDLE m_thread;
};

/* ─── Device ────────────────────────────────────── */
class K4WDevice : public oni::driver::DeviceBase {
public:
    K4WDevice(OniDeviceInfo *info, oni::driver::DriverServices &svc)
        : m_info(info), m_svc(svc) {
        /* Depth sensor */
        m_sensors[0].sensorType = ONI_SENSOR_DEPTH;
        m_sensors[0].numSupportedVideoModes = 1;
        m_sensors[0].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, 1);
        m_sensors[0].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
        m_sensors[0].pSupportedVideoModes[0].fps = 30;
        m_sensors[0].pSupportedVideoModes[0].resolutionX = K4W_DEPTH_W;
        m_sensors[0].pSupportedVideoModes[0].resolutionY = K4W_DEPTH_H;

        /* Color sensor */
        m_sensors[1].sensorType = ONI_SENSOR_COLOR;
        m_sensors[1].numSupportedVideoModes = 1;
        m_sensors[1].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, 1);
        m_sensors[1].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_RGB888;
        m_sensors[1].pSupportedVideoModes[0].fps = 30;
        m_sensors[1].pSupportedVideoModes[0].resolutionX = K4W_VIDEO_W;
        m_sensors[1].pSupportedVideoModes[0].resolutionY = K4W_VIDEO_H;
    }

    OniDeviceInfo *GetInfo() { return m_info; }

    OniStatus getSensorInfoList(OniSensorInfo **p, int *n) {
        *p = m_sensors;
        *n = 2;
        return ONI_STATUS_OK;
    }

    oni::driver::StreamBase *createStream(OniSensorType type) {
        if (type == ONI_SENSOR_DEPTH) return XN_NEW(K4WDepthStream);
        if (type == ONI_SENSOR_COLOR) return XN_NEW(K4WColorStream);
        return NULL;
    }

    void destroyStream(oni::driver::StreamBase *s) { XN_DELETE(s); }

    OniStatus getProperty(int id, void *data, int *sz) {
        if (id == ONI_DEVICE_PROPERTY_DRIVER_VERSION && *sz == sizeof(OniVersion)) {
            OniVersion *v = (OniVersion *)data;
            v->major = v->minor = v->maintenance = v->build = 2;
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

private:
    OniDeviceInfo *m_info;
    oni::driver::DriverServices &m_svc;
    OniSensorInfo m_sensors[2];
};

/* ─── Driver ────────────────────────────────────── */
class K4WDriver : public oni::driver::DriverBase {
public:
    K4WDriver(OniDriverServices *p) : DriverBase(p) {
        /* Defer device registration — can't call deviceConnected from ctor */
        xnOSCreateThread(registerThreadFunc, this, &m_registerThread);
    }

    static XN_THREAD_PROC registerThreadFunc(XN_THREAD_PARAM p) {
        K4WDriver *self = (K4WDriver *)p;
        /* Wait for OpenNI2 runtime to finish loading all drivers */
        xnOSSleep(500);
        struct stat st;
        if (stat("/dev/shm/k4w_video", &st) == 0) {
            OniDeviceInfo *info = XN_NEW(OniDeviceInfo);
            xnOSStrCopy(info->uri, "k4w", ONI_MAX_STR);
            xnOSStrCopy(info->vendor, "Microsoft", ONI_MAX_STR);
            xnOSStrCopy(info->name, "Kinect v1 (K4W)", ONI_MAX_STR);
            info->usbVendorId = 0x045e;
            info->usbProductId = 0x02ae;
            self->m_devices[info] = NULL;
            self->deviceConnected(info);
        }
        XN_THREAD_PROC_RETURN(XN_STATUS_OK);
    }

    virtual oni::driver::DeviceBase *deviceOpen(const char *uri, const char *) {
        for (auto it = m_devices.Begin(); it != m_devices.End(); ++it) {
            if (xnOSStrCmp(it->Key()->uri, uri) == 0) {
                if (it->Value()) return it->Value();
                K4WDevice *dev = XN_NEW(K4WDevice, it->Key(), getServices());
                it->Value() = dev;
                return dev;
            }
        }
        return NULL;
    }

    virtual void deviceClose(oni::driver::DeviceBase *dev) {
        for (auto it = m_devices.Begin(); it != m_devices.End(); ++it) {
            if (it->Value() == dev) {
                it->Value() = NULL;
                XN_DELETE(dev);
                return;
            }
        }
    }

    virtual OniStatus tryDevice(const char *uri) {
        if (xnOSStrCmp(uri, "k4w") != 0)
            return ONI_STATUS_ERROR;

        /* Check if daemon SHM is available */
        struct stat st;
        if (stat("/dev/shm/k4w_video", &st) != 0)
            return ONI_STATUS_ERROR;

        OniDeviceInfo *info = XN_NEW(OniDeviceInfo);
        xnOSStrCopy(info->uri, uri, ONI_MAX_STR);
        xnOSStrCopy(info->vendor, "Microsoft", ONI_MAX_STR);
        xnOSStrCopy(info->name, "Kinect v1 (K4W)", ONI_MAX_STR);
        info->usbVendorId = 0x045e;
        info->usbProductId = 0x02ae;
        m_devices[info] = NULL;
        deviceConnected(info);
        return ONI_STATUS_OK;
    }

    void shutdown() {}

private:
    XN_THREAD_HANDLE m_registerThread;
    xnl::Hash<OniDeviceInfo *, oni::driver::DeviceBase *> m_devices;
};

ONI_EXPORT_DRIVER(K4WDriver);
