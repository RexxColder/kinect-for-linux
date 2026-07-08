#include "k4w_internal.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

int k4w_shm_create(k4w_shm_header_t **hdr, const char *name, uint32_t frame_size) {
    size_t total = sizeof(k4w_shm_header_t) + frame_size * K4W_RING_SIZE;

    int fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (fd < 0) return -1;

    if (ftruncate(fd, total) < 0) {
        close(fd);
        shm_unlink(name);
        return -1;
    }

    void *ptr = mmap(NULL, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (ptr == MAP_FAILED) {
        shm_unlink(name);
        return -1;
    }

    *hdr = (k4w_shm_header_t *)ptr;
    memset(*hdr, 0, total);
    (*hdr)->magic = K4W_SHM_MAGIC;
    (*hdr)->frame_size = frame_size;

    return 0;
}

void k4w_shm_destroy(const char *name) {
    shm_unlink(name);
}

void k4w_shm_write_frame(k4w_shm_header_t *hdr, const void *data, uint32_t size) {
    if (!hdr || size > hdr->frame_size) size = hdr->frame_size;
    if (!hdr) return;
    uint32_t wi = hdr->write_idx;
    uint32_t idx = wi % K4W_RING_SIZE;

    memcpy(hdr->data + idx * hdr->frame_size, data, size);
    hdr->timestamp_ms = now_ms();
    hdr->write_idx = (wi + 1) % K4W_RING_SIZE;  /* advance first */
    hdr->seq++;                                     /* then publish */
}

bool k4w_shm_read_frame(k4w_shm_header_t *hdr, void *out, uint32_t size, uint32_t *seq) {
    uint32_t cur = hdr->seq;
    if (cur == *seq) return false;

    uint32_t wi = hdr->write_idx;
    uint32_t idx = (wi + K4W_RING_SIZE - 1) % K4W_RING_SIZE;
    if (size > hdr->frame_size) size = hdr->frame_size;
    memcpy(out, hdr->data + idx * hdr->frame_size, size);
    *seq = cur;
    return true;
}
