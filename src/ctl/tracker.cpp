#include "tracker.h"
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* MediaPipe Pose 33-landmark connections */
const int SkeletonTracker::MP_CONNECTIONS[][2] = {
    {0,1},{1,2},{2,3},{3,7},      /* left face */
    {0,4},{4,5},{5,6},{6,8},      /* right face */
    {9,10},                        /* mouth */
    {11,12},{11,13},{13,15},      /* left arm */
    {12,14},{14,16},               /* right arm */
    {11,23},{12,24},{23,24},       /* torso */
    {23,25},{25,27},{27,29},{27,31}, /* left leg */
    {24,26},{26,28},{28,30},{28,32}, /* right leg */
    {15,17},{15,19},{15,21},      /* left hand */
    {16,18},{16,20},{16,22},      /* right hand */
    {29,31},{30,32}               /* feet */
};
const int SkeletonTracker::NUM_CONNECTIONS = 31;

SkeletonTracker::SkeletonTracker() : m_enabled(true) {}
SkeletonTracker::~SkeletonTracker() {}

static void *openSHM(const char *name, size_t *out_size) {
    char path[256];
    snprintf(path, sizeof(path), "/dev/shm/%s", name);
    int fd = open(path, O_RDONLY);
    if (fd < 0) return nullptr;
    struct stat st;
    fstat(fd, &st);
    void *p = mmap(nullptr, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) return nullptr;
    *out_size = st.st_size;
    return p;
}

void SkeletonTracker::detectPersons(const QImage&, const QImage&) {
    m_skeletons.clear();
    if (!m_enabled) return;

    size_t sz;
    void *p = openSHM("k4w_skeleton", &sz);
    if (!p) return;

    uint32_t *hdr = (uint32_t *)p;
    if (hdr[0] != 0x4B345750) { munmap(p, sz); return; }

    int n_skeletons = (int)hdr[3];
    if (n_skeletons <= 0) { munmap(p, sz); return; }

    /* Each skeleton: bbox(16) + conf(4) + 33 keypoints * (x:i32 + y:i32 + c:f32) = 416 bytes */
    const uint8_t *data = (const uint8_t *)(hdr + 8); /* Skip 32-byte header */

    for (int i = 0; i < n_skeletons && i < 10; i++) {
        const uint8_t *sk = data + i * 416;

        Skeleton s;
        float bx, by, bw, bh;
        memcpy(&bx, sk, 4);
        memcpy(&by, sk + 4, 4);
        memcpy(&bw, sk + 8, 4);
        memcpy(&bh, sk + 12, 4);
        s.bbox = QRect((int)bx, (int)by, (int)bw, (int)bh);

        memcpy(&s.confidence, sk + 16, 4);

        const uint8_t *kps = sk + 20;
        for (int j = 0; j < NUM_LANDMARKS; j++) {
            int32_t kx, ky;
            float kc;
            memcpy(&kx, kps + j * 12, 4);
            memcpy(&ky, kps + j * 12 + 4, 4);
            memcpy(&kc, kps + j * 12 + 8, 4);
            s.keypoints.append({kx, ky, kc});
        }

        m_skeletons.append(s);
    }

    munmap(p, sz);
}

void SkeletonTracker::drawSkeleton(QImage &frame) {
    QPainter p(&frame);
    p.setRenderHint(QPainter::Antialiasing);

    for (const Skeleton &sk : m_skeletons) {
        if (sk.confidence < 0.4f) continue;

        /* Draw bounding box */
        p.setPen(QPen(QColor(0, 255, 0, 100), 1));
        p.setBrush(Qt::NoBrush);
        p.drawRect(sk.bbox);

        /* Draw skeleton connections — only high-confidence lines */
        for (int c = 0; c < NUM_CONNECTIONS; c++) {
            int a = MP_CONNECTIONS[c][0], b = MP_CONNECTIONS[c][1];
            if (a >= sk.keypoints.size() || b >= sk.keypoints.size()) continue;

            const SkeletonPoint &pa = sk.keypoints[a];
            const SkeletonPoint &pb = sk.keypoints[b];

            /* Skip if either point is below confidence threshold */
            if (pa.confidence < 0.4f || pb.confidence < 0.4f) continue;

            /* Check if points are within image bounds */
            bool a_in = (pa.x >= 0 && pa.x < frame.width() && pa.y >= 0 && pa.y < frame.height());
            bool b_in = (pb.x >= 0 && pb.x < frame.width() && pb.y >= 0 && pb.y < frame.height());
            if (!a_in || !b_in) continue;

            /* Color by confidence: green=high, yellow=medium, red=low */
            float avg = (pa.confidence + pb.confidence) * 0.5f;
            int r = avg < 0.6f ? 255 : (int)(255 * (1.0f - avg));
            int g = avg < 0.6f ? (int)(200 * avg / 0.6f) : 200;
            QColor lineColor(r, g, 0, 220);

            p.setPen(QPen(lineColor, 3));
            p.drawLine(pa.x, pa.y, pb.x, pb.y);
        }

        /* Draw keypoints (only visible ones) */
        for (int j = 0; j < sk.keypoints.size(); j++) {
            const SkeletonPoint &kp = sk.keypoints[j];
            if (kp.confidence < 0.4f) continue;
            bool on_screen = (kp.x >= 0 && kp.x < frame.width() && kp.y >= 0 && kp.y < frame.height());
            if (!on_screen) continue;

            int r = (int)(kp.confidence * 4) + 2;
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(255, 120, 0, 200));
            p.drawEllipse(kp.x - r, kp.y - r, r * 2, r * 2);
            p.setBrush(QColor(255, 220, 50, 255));
            p.drawEllipse(kp.x - 2, kp.y - 2, 4, 4);
        }
    }
    p.end();
}

void SkeletonTracker::updateSkeletons() {
    detectPersons(QImage(), QImage());
}

QImage SkeletonTracker::process(const QImage &rgb, const QImage &) {
    QImage frame = rgb.copy();
    if (!m_enabled) return frame;

    detectPersons(QImage(), QImage());
    drawSkeleton(frame);

    if (!m_skeletons.isEmpty()) {
        QPainter p(&frame);
        p.setPen(QColor(0, 255, 0));
        p.setFont(QFont("monospace", 10));
        p.drawText(10, 20, QString("Skeletons: %1").arg(m_skeletons.size()));
        p.end();
    }

    return frame;
}
