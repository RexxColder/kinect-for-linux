#include "tracker.h"
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QFileInfo>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <opencv2/imgproc.hpp>

/* MediaPipe Pose 33-landmark connections */
const int SkeletonTracker::MP_CONNECTIONS[][2] = {
    {0,1},{1,2},{2,3},{3,7},
    {0,4},{4,5},{5,6},{6,8},
    {9,10},
    {11,12},{11,13},{13,15},
    {12,14},{14,16},
    {11,23},{12,24},{23,24},
    {23,25},{25,27},{27,29},{27,31},
    {24,26},{26,28},{28,30},{28,32},
    {15,17},{15,19},{15,21},
    {16,18},{16,20},{16,22},
    {29,31},{30,32}
};
const int SkeletonTracker::NUM_CONNECTIONS = 31;

SkeletonTracker::SkeletonTracker(QObject *parent)
    : QObject(parent)
    , m_enabled(true)
    , m_modelsLoaded(false)
    , m_newFrame(false)
    , m_worker(nullptr)
{
}

SkeletonTracker::~SkeletonTracker() {
    if (m_worker) {
        m_worker->quit();
        m_worker->wait();
    }
}

bool SkeletonTracker::loadModels(const QString &detectModelPath, const QString &poseModelPath) {
    if (!QFileInfo::exists(detectModelPath)) {
        emit error(QString("Detector model not found: %1").arg(detectModelPath));
        return false;
    }
    if (!QFileInfo::exists(poseModelPath)) {
        emit error(QString("Pose model not found: %1").arg(poseModelPath));
        return false;
    }

    try {
        m_detectNet = cv::dnn::readNetFromONNX(detectModelPath.toStdString());
        m_poseNet = cv::dnn::readNetFromONNX(poseModelPath.toStdString());

        /* Use CUDA if compiled with CUDA support, fallback to CPU */
#ifdef HAVE_OPENCV_CUDA
        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            m_detectNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            m_detectNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            m_poseNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            m_poseNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        } else
#endif
        {
            m_detectNet.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            m_detectNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            m_poseNet.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            m_poseNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }

        m_modelsLoaded = true;
        return true;
    } catch (const cv::Exception &e) {
        emit error(QString("Failed to load models: %1").arg(e.what()));
        return false;
    }
}

void SkeletonTracker::setEnabled(bool enabled) {
    m_enabled = enabled;
    if (!enabled) {
        QMutexLocker lock(&m_mutex);
        m_skeletons.clear();
    }
}

/* ─── SHM Reading ─────────────────────────────────── */
void *SkeletonTracker::openSHM(const char *name, size_t *out_size) {
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

void SkeletonTracker::updateFromSHM() {
    if (!m_enabled) return;

    size_t sz;
    void *p = openSHM("k4w_skeleton", &sz);
    if (!p) return;

    uint32_t *hdr = (uint32_t *)p;
    if (hdr[0] != 0x4B345750) { munmap(p, sz); return; }

    int n_skeletons = (int)hdr[3];
    if (n_skeletons <= 0) { munmap(p, sz); return; }

    const uint8_t *data = (const uint8_t *)(hdr + 8);

    QVector<Skeleton> skels;
    for (int i = 0; i < n_skeletons && i < SKELETON_MAX_PERSONS; i++) {
        const uint8_t *sk = data + i * 416;

        Skeleton s;
        float bx, by, bw, bh;
        memcpy(&bx, sk, 4);
        memcpy(&by, sk + 4, 4);
        memcpy(&bw, sk + 8, 4);
        memcpy(&bh, sk + 12, 4);
        s.bbox_x = (int)bx;
        s.bbox_y = (int)by;
        s.bbox_w = (int)bw;
        s.bbox_h = (int)bh;
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
        skels.append(s);
    }
    munmap(p, sz);

    QMutexLocker lock(&m_mutex);
    m_skeletons = skels;
}

/* ─── Detection Postprocessing ─────────────────────── */
void SkeletonTracker::postprocessDetection(const cv::Mat &output, int imgW, int imgH,
                                           QVector<cv::Rect> &outBoxes, QVector<float> &outScores) {
    outBoxes.clear();
    outScores.clear();

    /* MediaPipe detection output: [1, N, 6] -> [batch, num_detections, (y1, x1, y2, x2, score, class)] */
    const int numDets = output.size[1];
    const float *data = (const float *)output.data;

    for (int i = 0; i < numDets; i++) {
        float score = data[i * 6 + 4];
        int classId = (int)data[i * 6 + 5];
        if (score < 0.5f || classId != 0) continue;  /* class 0 = person */

        float y1 = data[i * 6 + 0] * imgH;
        float x1 = data[i * 6 + 1] * imgW;
        float y2 = data[i * 6 + 2] * imgH;
        float x2 = data[i * 6 + 3] * imgW;

        int bx = std::max(0, (int)x1);
        int by = std::max(0, (int)y1);
        int bw = std::min(imgW - bx, (int)(x2 - x1));
        int bh = std::min(imgH - by, (int)(y2 - y1));

        if (bw > 10 && bh > 10) {
            outBoxes.append(cv::Rect(bx, by, bw, bh));
            outScores.append(score);
        }
    }
}

/* ─── Pose Postprocessing ──────────────────────────── */
void SkeletonTracker::postprocessPose(const cv::Mat &heatmap, const cv::Mat &ldmk,
                                      int cropW, int cropH, int ox, int oy, Skeleton &sk) {
    sk.keypoints.clear();
    sk.keypoints.resize(NUM_LANDMARKS);

    const int hW = heatmap.size[3];  /* heatmap width */
    const int hH = heatmap.size[2];  /* heatmap height */
    const int lD = ldmk.size[2];     /* landmark dim (xy * num landmarks) */

    const float *heatData = (const float *)heatmap.data;
    const float *ldmkData = (const float *)ldmk.data;

    for (int k = 0; k < NUM_LANDMARKS; k++) {
        /* Find max activation for this landmark */
        float maxVal = -1e9f;
        int maxX = 0, maxY = 0;
        for (int y = 0; y < hH; y++) {
            for (int x = 0; x < hW; x++) {
                float v = heatData[k * hH * hW + y * hW + x];
                if (v > maxVal) {
                    maxVal = v;
                    maxX = x;
                    maxY = y;
                }
            }
        }

        /* Refine with offset from landmarks */
        float offX = ldmkData[k * 2 * hH * hW + maxY * hW + maxX];
        float offY = ldmkData[(k * 2 + 1) * hH * hW + maxY * hW + maxX];

        /* Map to crop coordinates */
        float cx = ((float)maxX + offX) / (float)hW * cropW + ox;
        float cy = ((float)maxY + offY) / (float)hH * cropH + oy;

        float conf = 1.0f / (1.0f + std::exp(-maxVal));  /* sigmoid */
        sk.keypoints[k] = {(int)cx, (int)cy, conf};
    }
}

/* ─── Main Detection + Pose Pipeline ───────────────── */
void SkeletonTracker::detectAndTrack(const cv::Mat &bgr) {
    m_skeletons.clear();
    if (!m_enabled || !m_modelsLoaded) return;

    int imgW = bgr.cols;
    int imgH = bgr.rows;

    /* Step 1: Person Detection */
    cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0 / 127.5, cv::Size(192, 192),
                                          cv::Scalar(127.5, 127.5, 127.5), true, false);
    m_detectNet.setInput(blob);
    cv::Mat detOutput = m_detectNet.forward();

    QVector<cv::Rect> boxes;
    QVector<float> scores;
    postprocessDetection(detOutput, imgW, imgH, boxes, scores);

    /* Step 2: Pose estimation for each detected person (max 2) */
    for (int i = 0; i < boxes.size() && i < SKELETON_MAX_PERSONS; i++) {
        cv::Rect roi = boxes[i];

        /* Expand ROI for better pose estimation */
        int expand = std::max(roi.width, roi.height) / 4;
        int ox = std::max(0, roi.x - expand);
        int oy = std::max(0, roi.y - expand);
        int ow = std::min(imgW - ox, roi.width + expand * 2);
        int oh = std::min(imgH - oy, roi.height + expand * 2);

        cv::Mat crop = bgr(cv::Rect(ox, oy, ow, oh)).clone();

        /* Pose estimation input: 256x256 */
        cv::Mat poseBlob = cv::dnn::blobFromImage(crop, 1.0 / 255.0, cv::Size(256, 256),
                                                  cv::Scalar(0, 0, 0), false, false);
        m_poseNet.setInput(poseBlob);

        /* Forward pass — get heatmap + landmarks */
        std::vector<cv::String> outNames = m_poseNet.getUnconnectedOutLayersNames();
        std::vector<cv::Mat> poseOutputs;
        m_poseNet.forward(poseOutputs, outNames);

        Skeleton sk;
        sk.confidence = scores[i];
        sk.bbox_x = roi.x;
        sk.bbox_y = roi.y;
        sk.bbox_w = roi.width;
        sk.bbox_h = roi.height;

        if (poseOutputs.size() >= 2) {
            postprocessPose(poseOutputs[0], poseOutputs[1], ow, oh, ox, oy, sk);
        }

        m_skeletons.append(sk);
    }
}

/* ─── Slot: Process Frame ──────────────────────────── */
void SkeletonTracker::processFrame(const QImage &rgb) {
    if (!m_enabled || rgb.isNull()) return;

    m_lastFrame = rgb;

    /* QImage -> cv::Mat */
    QImage img = rgb.convertToFormat(QImage::Format_RGB888);
    cv::Mat bgr(img.height(), img.width(), CV_8UC3, (void*)img.constBits(), img.bytesPerLine());
    cv::cvtColor(bgr, bgr, cv::COLOR_RGB2BGR);

    detectAndTrack(bgr);

    QImage result = drawSkeleton(rgb);
    emit frameProcessed(result);
}

/* ─── Draw Skeleton Overlay ────────────────────────── */
QImage SkeletonTracker::drawSkeleton(const QImage &frame) {
    QImage result = frame.copy();
    QPainter p(&result);
    p.setRenderHint(QPainter::Antialiasing);

    QMutexLocker lock(&m_mutex);

    for (const Skeleton &sk : m_skeletons) {
        if (sk.confidence < 0.4f) continue;

        /* Draw bounding box */
        p.setPen(QPen(QColor(0, 255, 0, 100), 1));
        p.setBrush(Qt::NoBrush);
        p.drawRect(sk.bbox_x, sk.bbox_y, sk.bbox_w, sk.bbox_h);

        /* Draw skeleton connections */
        for (int c = 0; c < NUM_CONNECTIONS; c++) {
            int a = MP_CONNECTIONS[c][0], b = MP_CONNECTIONS[c][1];
            if (a >= sk.keypoints.size() || b >= sk.keypoints.size()) continue;

            const SkeletonPoint &pa = sk.keypoints[a];
            const SkeletonPoint &pb = sk.keypoints[b];
            if (pa.confidence < 0.4f || pb.confidence < 0.4f) continue;

            bool a_in = (pa.x >= 0 && pa.x < frame.width() && pa.y >= 0 && pa.y < frame.height());
            bool b_in = (pb.x >= 0 && pb.x < frame.width() && pb.y >= 0 && pb.y < frame.height());
            if (!a_in || !b_in) continue;

            float avg = (pa.confidence + pb.confidence) * 0.5f;
            int r = avg < 0.6f ? 255 : (int)(255 * (1.0f - avg));
            int g = avg < 0.6f ? (int)(200 * avg / 0.6f) : 200;

            p.setPen(QPen(QColor(r, g, 0, 220), 3));
            p.drawLine(pa.x, pa.y, pb.x, pb.y);
        }

        /* Draw keypoints */
        for (int j = 0; j < sk.keypoints.size(); j++) {
            const SkeletonPoint &kp = sk.keypoints[j];
            if (kp.confidence < 0.4f) continue;
            bool on_screen = (kp.x >= 0 && kp.x < frame.width() && kp.y >= 0 && kp.y < frame.height());
            if (!on_screen) continue;

            int rad = (int)(kp.confidence * 4) + 2;
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(255, 120, 0, 200));
            p.drawEllipse(kp.x - rad, kp.y - rad, rad * 2, rad * 2);
            p.setBrush(QColor(255, 220, 50, 255));
            p.drawEllipse(kp.x - 2, kp.y - 2, 4, 4);
        }

        /* Label */
        p.setPen(QColor(0, 255, 0));
        p.setFont(QFont("monospace", 10));
        p.drawText(sk.bbox_x, sk.bbox_y - 5,
                   QString("Person %1 (%2%)").arg(sk.confidence * 100, 0, 'f', 0));
    }

    p.end();
    return result;
}
