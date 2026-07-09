#ifndef TRACKER_H
#define TRACKER_H

#include <QObject>
#include <QImage>
#include <QVector>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <opencv2/dnn.hpp>

#define NUM_LANDMARKS 33
#define SKELETON_MAX_PERSONS 2

struct SkeletonPoint {
    int x, y;
    float confidence;
};

struct Skeleton {
    QVector<SkeletonPoint> keypoints;
    int bbox_x, bbox_y, bbox_w, bbox_h;
    float confidence;
};

class SkeletonTracker : public QObject {
    Q_OBJECT
public:
    explicit SkeletonTracker(QObject *parent = nullptr);
    ~SkeletonTracker();

    bool loadModels(const QString &detectModelPath, const QString &poseModelPath);
    void setEnabled(bool enabled);
    bool isEnabled() const { return m_enabled; }
    QImage lastFrame() const { return m_lastFrame; }

    /* Synchronous draw (called from main thread) */
    QImage drawSkeleton(const QImage &frame);

    /* SHM update */
    void updateFromSHM();
    const QVector<Skeleton>& skeletons() const { return m_skeletons; }

public slots:
    void processFrame(const QImage &rgb);

signals:
    void frameProcessed(const QImage &frame);
    void error(const QString &msg);

private:
    void detectAndTrack(const cv::Mat &bgr);
    void poseEstimate(const cv::Mat &personCrop, int personIdx, int ox, int oy, int ow, int oh);
    void postprocessDetection(const cv::Mat &output, int imgW, int imgH,
                              QVector<cv::Rect> &outBoxes, QVector<float> &outScores);
    void postprocessPose(const cv::Mat &heatmap, const cv::Mat &ldmk,
                         int cropW, int cropH, int ox, int oy, Skeleton &sk);

    bool m_enabled;
    bool m_modelsLoaded;

    cv::dnn::Net m_detectNet;
    cv::dnn::Net m_poseNet;

    QVector<Skeleton> m_skeletons;
    QImage m_lastFrame;

    /* SHM reading */
    void *openSHM(const char *name, size_t *out_size);

    /* MediaPipe Pose 33-landmark connections */
    static const int MP_CONNECTIONS[][2];
    static const int NUM_CONNECTIONS;

    /* Thread-safe frame queue */
    QMutex m_mutex;
    QWaitCondition m_cond;
    bool m_newFrame;
    QImage m_inputFrame;

    QThread *m_worker;
};

#endif
