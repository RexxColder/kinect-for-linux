#ifndef TRACKER_H
#define TRACKER_H

#include <QImage>
#include <QRect>
#include <QVector>

#define NUM_LANDMARKS 33

struct SkeletonPoint {
    int x, y;
    float confidence;
};

struct Skeleton {
    QVector<SkeletonPoint> keypoints; /* 33 MediaPipe landmarks */
    QRect bbox;
    float confidence;
};

class SkeletonTracker {
public:
    SkeletonTracker();
    ~SkeletonTracker();

    QImage process(const QImage &rgb, const QImage &depth);
    void updateSkeletons();    /* Read from SHM only */
    void drawSkeleton(QImage &frame);  /* Draw last known skeletons */
    const QVector<Skeleton>& skeletons() const { return m_skeletons; }
    void setEnabled(bool enabled) { m_enabled = enabled; }
    bool isEnabled() const { return m_enabled; }

private:
    void detectPersons(const QImage &rgb, const QImage &depth);

    bool m_enabled;
    QVector<Skeleton> m_skeletons;

    /* MediaPipe Pose 33-landmark connections */
    static const int MP_CONNECTIONS[][2];
    static const int NUM_CONNECTIONS;
};

#endif
