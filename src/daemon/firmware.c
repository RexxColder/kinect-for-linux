#include "k4w_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <libusb-1.0/libusb.h>

#define KINECT_VID       0x045e
#define KINECT_PID_BOOT  0x02ad
#define KINECT_PID_UAC   0x02bb
#define KINECT_EP_IN     0x81
#define KINECT_EP_OUT    0x01
#define BL_MAGIC         0x06022009
#define BL_REPLY_MAGIC   0x0a6fe000

#pragma pack(push, 1)
typedef struct { uint32_t magic, seq, bytes, cmd, addr, unk; } bl_cmd_t;
typedef struct { uint32_t magic, seq, status; } bl_reply_t;
#pragma pack(pop)

static int bl_get_reply(libusb_device_handle *dev) {
    unsigned char buf[512];
    int xferred = 0;
    int r = libusb_bulk_transfer(dev, KINECT_EP_IN, buf, 512, &xferred, 10000);
    if (r != 0 || xferred < 12) return -1;
    bl_reply_t *rep = (bl_reply_t *)buf;
    if (rep->magic != BL_REPLY_MAGIC) return -1;
    return 0;
}

static int bl_get_first_reply(libusb_device_handle *dev) {
    unsigned char buf[512];
    int xferred = 0;
    return libusb_bulk_transfer(dev, KINECT_EP_IN, buf, 512, &xferred, 10000);
}

static int upload_uac_firmware(const char *fw_path) {
    FILE *f = fopen(fw_path, "rb");
    if (!f) {
        K4W_LOG("Cannot open firmware: %s\n", fw_path);
        return -1;
    }

    libusb_context *ctx;
    libusb_init(&ctx);

    libusb_device_handle *dev = libusb_open_device_with_vid_pid(ctx, KINECT_VID, KINECT_PID_BOOT);
    if (!dev) {
        K4W_LOG("No Kinect audio bootloader found (0x%04x:0x%04x)\n", KINECT_VID, KINECT_PID_BOOT);
        fclose(f);
        libusb_exit(ctx);
        return -1;
    }

    if (libusb_kernel_driver_active(dev, 0) == 1)
        libusb_detach_kernel_driver(dev, 0);

    libusb_set_auto_detach_kernel_driver(dev, 1);
    int r = libusb_claim_interface(dev, 0);
    if (r != 0) {
        K4W_LOG("Cannot claim audio interface: %s\n", libusb_error_name(r));
        libusb_close(dev);
        libusb_exit(ctx);
        fclose(f);
        return -1;
    }

    K4W_LOG("Uploading UAC firmware...\n");

    uint32_t seq = 1;
    bl_cmd_t cmd = { BL_MAGIC, seq, 0x60, 0, 0x15, 0 };
    int xferred = 0;
    libusb_bulk_transfer(dev, KINECT_EP_OUT, (unsigned char *)&cmd, sizeof(cmd), &xferred, 10000);
    bl_get_first_reply(dev);
    bl_get_reply(dev);
    seq++;

    uint32_t addr = 0x00080000;
    unsigned char page[0x4000];
    int total = 0;
    int read_bytes;

    while ((read_bytes = fread(page, 1, 0x4000, f)) > 0) {
        cmd.seq = seq;
        cmd.bytes = read_bytes;
        cmd.cmd = 0x03;
        cmd.addr = addr;
        cmd.unk = 0;

        r = libusb_bulk_transfer(dev, KINECT_EP_OUT, (unsigned char *)&cmd, sizeof(cmd), &xferred, 10000);
        if (r != 0) { K4W_LOG("FW cmd error\n"); break; }

        int sent = 0;
        while (sent < read_bytes) {
            int ts = (read_bytes - sent > 512) ? 512 : (read_bytes - sent);
            r = libusb_bulk_transfer(dev, KINECT_EP_OUT, page + sent, ts, &xferred, 10000);
            if (r != 0) { K4W_LOG("FW data error\n"); goto out; }
            sent += ts;
        }

        bl_get_reply(dev);
        total += read_bytes;
        addr += read_bytes;
        seq++;
    }

    K4W_LOG("FW uploaded %d bytes, launching...\n", total);
    cmd.seq = seq;
    cmd.bytes = 0;
    cmd.cmd = 0x04;
    cmd.addr = 0x00080030;
    cmd.unk = 0;
    libusb_bulk_transfer(dev, KINECT_EP_OUT, (unsigned char *)&cmd, sizeof(cmd), &xferred, 10000);
    bl_get_reply(dev);

out:
    libusb_release_interface(dev, 0);
    libusb_close(dev);
    libusb_exit(ctx);
    fclose(f);
    return 0;
}

static void unbind_snd_usb_audio(void) {
    DIR *bus = opendir("/sys/bus/usb/devices");
    if (!bus) return;

    struct dirent *de;
    while ((de = readdir(bus)) != NULL) {
        if (de->d_name[0] == '.') continue;

        char path[512];
        snprintf(path, sizeof(path), "/sys/bus/usb/devices/%s/idVendor", de->d_name);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        char vid[8] = {0};
        fgets(vid, sizeof(vid), f);
        fclose(f);

        snprintf(path, sizeof(path), "/sys/bus/usb/devices/%s/idProduct", de->d_name);
        f = fopen(path, "r");
        if (!f) continue;
        char pid[8] = {0};
        fgets(pid, sizeof(pid), f);
        fclose(f);

        if (strcmp(vid, "045e\n") == 0 && strcmp(pid, "02bb\n") == 0) {
            char intf_path[512];
            snprintf(intf_path, sizeof(intf_path), "/sys/bus/usb/devices/%s", de->d_name);
            DIR *intf_dir = opendir(intf_path);
            if (!intf_dir) continue;

            struct dirent *ie;
            while ((ie = readdir(intf_dir)) != NULL) {
                if (ie->d_name[0] != '1' && ie->d_name[0] != '2' &&
                    ie->d_name[0] != '3' && ie->d_name[0] != '4') continue;

                char drv_path[512];
                snprintf(drv_path, sizeof(drv_path), "%s/%s/driver", intf_path, ie->d_name);
                char link[256] = {0};
                ssize_t len = readlink(drv_path, link, sizeof(link) - 1);
                if (len <= 0) continue;
                link[len] = '\0';

                if (strstr(link, "snd-usb-audio")) {
                    char unbind_path[512];
                    snprintf(unbind_path, sizeof(unbind_path),
                             "/sys/bus/usb/drivers/snd-usb-audio/unbind");
                    FILE *uf = fopen(unbind_path, "w");
                    if (uf) {
                        fprintf(uf, "%s", ie->d_name);
                        fclose(uf);
                        K4W_LOG("Unbound snd-usb-audio IF %s\n", ie->d_name);
                    }
                }
            }
            closedir(intf_dir);
        }
    }
    closedir(bus);
}

int k4w_firmware_setup(void) {
    /* Check if already in UAC mode */
    libusb_device_handle *dev = NULL;
    libusb_context *ctx;
    libusb_init(&ctx);

    /* Check if audio bootloader (02ad) is present */
    dev = libusb_open_device_with_vid_pid(ctx, KINECT_VID, KINECT_PID_BOOT);
    if (dev) {
        libusb_close(dev);
        libusb_exit(ctx);
        K4W_LOG("Audio bootloader found, uploading firmware...\n");
        if (upload_uac_firmware(KINECT_FW_PATH) == 0) {
            sleep(3);
            /* DO NOT unbind snd-usb-audio - kernel driver needed for audio */
            return 0;
        }
        libusb_init(&ctx);
    }

    /* Check if already in UAC mode (02bb) */
    dev = libusb_open_device_with_vid_pid(ctx, KINECT_VID, KINECT_PID_UAC);
    if (dev) {
        libusb_close(dev);
        libusb_exit(ctx);
        K4W_LOG("Audio already in UAC mode\n");
        /* DO NOT unbind snd-usb-audio - kernel driver needed for audio */
        return 0;
    }

    libusb_exit(ctx);
    K4W_LOG("No Kinect audio device found\n");
    return -1;
}
