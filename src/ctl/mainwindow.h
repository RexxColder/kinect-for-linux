#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QComboBox>
#include <QSlider>
#include <QPushButton>
#include <QCheckBox>
#include <QTableWidget>
#include <QLocalSocket>
#include <QElapsedTimer>
#include <QSystemTrayIcon>
#include <QMenu>
#include <QDir>
#include <QThread>
#include <QMutex>
#include <QProcess>
#include "tracker.h"

/* ─── Microphone Monitor Thread ─────────────────────────── */
class MicMonitorThread : public QThread {
    Q_OBJECT
public:
    explicit MicMonitorThread(QObject *parent = nullptr);
    void setVolume(int vol);   /* 0..100 */
    void setEnabled(bool on);
    bool isRunning() const { return m_running; }
    /* Latest peak levels for 8 bands, updated by the thread */
    float peaks[8];

signals:
    void levelsUpdated();

protected:
    void run() override;

private:
    bool m_running = false;
    bool m_enabled = false;
    int  m_volume = 80;
    QMutex m_mtx;
};

/* ─── Sensor Poll Thread ────────────────────────────────── */
class SensorPollThread : public QThread {
    Q_OBJECT
public:
    explicit SensorPollThread(QObject *parent = nullptr);
    void setEnabled(bool on);

signals:
    void dataReady(float ax, float ay, float az, float tiltDeg, bool motorOk);
    void daemonStatus(bool online);

protected:
    void run() override;

private:
    bool m_enabled = false;
    QMutex m_mtx;
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onRefreshFrame();
    void onReconnectTick();
    void onModeChanged(int index);
    void onMotorTilt(int angle);
    void onCapturePhoto();
    void onRecordToggle();
    void onTrayActivated(QSystemTrayIcon::ActivationReason reason);
    void onTiltDebounce();
    void onTiltApply();
    void onTiltReset();
    void onStartDaemon();
    void onResetDaemon();
    void onShutdownDaemon();
    void onStatusBarClicked();

private:
    void setupUI();
    void connectDaemon();
    bool isConnected() const;
    void updateTrayIcon();
    void updateStatusDots();
    QImage readRGB();
    QImage readDepth();
    QImage readIR();
    QImage readOverlay();
    bool eventFilter(QObject *obj, QEvent *event) override;

    /* UI */
    QLabel *m_statusDot;
    QLabel *m_statusText;
    QComboBox *m_modeCombo;
    QLabel *m_mainView;
    QSlider *m_tiltSlider;
    QLabel *m_tiltValue;
    QPushButton *m_tiltApplyBtn;
    QPushButton *m_tiltResetBtn;
    QLabel *m_tiltCooldownLabel;
    QLabel *m_kinectUp;
    QLabel *m_kinectDown;
    QTableWidget *m_sensorTable;
    QPushButton *m_captureBtn;
    QPushButton *m_recordBtn;
    QPushButton *m_stopBtn;
    QLabel *m_timerLabel;
    QLabel *m_statusBar;
    QPushButton *m_startDaemonBtn;
    QPushButton *m_resetDaemonBtn;
    QPushButton *m_shutdownDaemonBtn;

    /* Sensor values */
    QLabel *m_accelValue;
    QLabel *m_tiltValueSensor;
    QLabel *m_sensorLedLabel;
    QLabel *m_sensorMotorLabel;
    QLabel *m_sensorAudioLabel;

    /* Status dots */
    QLabel *m_motorStatusDot;
    QLabel *m_audioStatusDot;

    /* Motor debounce + cooldown */
    QTimer *m_tiltDebounce;
    QTimer *m_tiltCooldown;
    QTimer *m_tiltCountdownTimer;
    bool m_tiltBusy;
    int m_tiltTarget;
    int m_tiltCountdownSec;

    /* Skeleton frame skip */
    int m_skeletonFrameSkip;
    int m_skeletonFrameCounter;

    /* System Tray */
    QSystemTrayIcon *m_trayIcon;
    QMenu *m_trayMenu;

    /* Skeleton Tracker */
    SkeletonTracker *m_tracker;
    QProcess *m_skeletonProcess;

    /* State */
    QTimer *m_frameTimer;
    QTimer *m_reconnectTimer;
    QTimer *m_recordTimerTick;
    QLocalSocket *m_socket;
    QElapsedTimer m_recordElapsed;
    bool m_recording;
    QString m_recordDir;
    int m_recordFrameCount;
    int m_cameraMode;
    bool m_isStarting;
    bool m_daemonOnline;

    /* Microphone */
    MicMonitorThread *m_micThread;

    /* Sensor Poll */
    SensorPollThread *m_sensorPollThread;
    QSlider *m_volSlider;
    QLabel *m_volLabel;
    QLabel *m_waveform;
    QCheckBox *m_micTestCheck;
    QLabel *m_micStatusLabel;
    QPushButton *m_micBindBtn;
    QTimer *m_waveTimer;
};

#endif
