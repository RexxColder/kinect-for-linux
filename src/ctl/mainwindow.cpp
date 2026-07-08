#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QApplication>
#include <QDateTime>
#include <QPainter>
#include <QFileDialog>
#include <QProcess>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <cstring>
#include <cmath>
#include <pulse/simple.h>
#include <pulse/error.h>

#include "k4w_common.h"

/* ═══ MicMonitorThread Implementation ═══════════════════ */
MicMonitorThread::MicMonitorThread(QObject *parent)
    : QThread(parent) {
    memset(peaks, 0, sizeof(peaks));
}

void MicMonitorThread::setVolume(int vol) {
    QMutexLocker lk(&m_mtx);
    m_volume = vol;
}

void MicMonitorThread::setEnabled(bool on) {
    QMutexLocker lk(&m_mtx);
    m_enabled = on;
}

void MicMonitorThread::run() {
    m_running = true;

    /* Read audio directly from daemon's SHM ring buffer */
    int last_seq = -1;
    int16_t buf[K4W_AUDIO_SAMPLES];

    while (m_running) {
        bool enabled;
        int volume;
        {
            QMutexLocker lk(&m_mtx);
            enabled = m_enabled;
            volume = m_volume;
        }
        if (!enabled) { usleep(100000); continue; }

        /* Read from SHM */
        char path[256];
        snprintf(path, sizeof(path), "/dev/shm/k4w_audio");
        int fd = open(path, O_RDONLY);
        if (fd < 0) { usleep(100000); continue; }
        struct stat st;
        fstat(fd, &st);
        void *p = mmap(nullptr, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
        ::close(fd);
        if (p == MAP_FAILED || !p) { usleep(100000); continue; }

        k4w_shm_header_t *hdr = (k4w_shm_header_t *)p;
        if (hdr->magic != K4W_SHM_MAGIC) { munmap(p, st.st_size); usleep(100000); continue; }

        int seq = hdr->seq;
        if (seq == last_seq || seq < 1) { munmap(p, st.st_size); usleep(10000); continue; }
        last_seq = seq;

        /* Read latest frame from ring buffer */
        int wi = hdr->write_idx;
        int idx = (wi + K4W_RING_SIZE - 1) % K4W_RING_SIZE;
        memcpy(buf, hdr->data + idx * K4W_AUDIO_SAMPLES * 2, K4W_AUDIO_SAMPLES * 2);
        munmap(p, st.st_size);

        /* Apply volume */
        if (volume < 100) {
            float gain = volume / 100.0f;
            for (int i = 0; i < K4W_AUDIO_SAMPLES; i++)
                buf[i] = (int16_t)(buf[i] * gain);
        }

        /* Compute peak levels for 8 bands */
        float bandPeaks[8] = {};
        int bandSize = K4W_AUDIO_SAMPLES / 8;
        for (int b = 0; b < 8; b++) {
            float peak = 0;
            for (int i = b * bandSize; i < (b + 1) * bandSize; i++) {
                float v = fabsf((float)buf[i]) / 32768.0f;
                if (v > peak) peak = v;
            }
            bandPeaks[b] = peak;
        }
        {
            QMutexLocker lk(&m_mtx);
            memcpy(peaks, bandPeaks, sizeof(peaks));
        }
        emit levelsUpdated();
    }
}

/* ─── Sensor Poll Thread ───────────────────────────────── */
SensorPollThread::SensorPollThread(QObject *parent) : QThread(parent) {}

void SensorPollThread::setEnabled(bool on) {
    QMutexLocker lk(&m_mtx);
    m_enabled = on;
}

void SensorPollThread::run() {
    fprintf(stderr, "[SensorPoll] Thread started\n");
    while (true) {
        bool enabled;
        {
            QMutexLocker lk(&m_mtx);
            enabled = m_enabled;
        }
        if (!enabled) { usleep(200000); continue; }

        fprintf(stderr, "[SensorPoll] Connecting...\n");
        QLocalSocket sock;
        sock.connectToServer("/tmp/k4w.sock");
        if (!sock.waitForConnected(500)) {
            fprintf(stderr, "[SensorPoll] Connect FAILED\n");
            emit daemonStatus(false);
            usleep(1000000);
            continue;
        }
        fprintf(stderr, "[SensorPoll] Connected, sending STATUS\n");

        /* Send STATUS command — lightweight, no USB motor access */
        int cmd[2] = { 2, 0 }; /* K4W_CMD_STATUS = 2 */
        sock.write((const char *)cmd, sizeof(cmd));
        if (!sock.waitForBytesWritten(500)) { sock.disconnectFromServer(); usleep(500000); continue; }

        if (sock.waitForReadyRead(2000)) {
            char buf[256] = {0};
            qint64 n = sock.read(buf, sizeof(buf));
            fprintf(stderr, "[SensorPoll] Got %zd bytes\n", n);
            if (n >= (int)sizeof(k4w_status_t)) {
                k4w_status_t *resp = (k4w_status_t *)buf;
                fprintf(stderr, "[SensorPoll] tilt=%.1f ax=%.2f ay=%.2f az=%.2f\n",
                        resp->tilt_deg, resp->accel_x, resp->accel_y, resp->accel_z);
                emit dataReady(resp->accel_x, resp->accel_y, resp->accel_z,
                               resp->tilt_deg, resp->ok);
                emit daemonStatus(true);
            }
        } else {
            fprintf(stderr, "[SensorPoll] Read TIMEOUT\n");
        }
        sock.disconnectFromServer();
        usleep(5000000); /* Poll every 5s — ACCEL shares USB with audio */
    }
}

/* ─── SHM Helper ──────────────────────────────────────── */
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

static QImage depthToJET(const uint16_t *raw, int w, int h) {
    QImage img(w, h, QImage::Format_RGB888);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint8_t v = (uint8_t)(raw[y * w + x] * 255 / 2048);
            uint8_t r = (v < 128) ? 0 : (v - 128) * 2;
            uint8_t g = (v < 64) ? v * 4 : (v < 192) ? 255 : (255 - v) * 4;
            uint8_t b = (v < 128) ? (128 - v) * 2 : 0;
            img.setPixel(x, y, qRgb(r, g, b));
        }
    return img;
}

/* ─── Constructor ─────────────────────────────────────── */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), m_recording(false), m_cameraMode(0) {
    setupUI();
    setWindowTitle("Kinect for Linux v1.0");
    {
        QIcon icon;
        for (int sz : {16, 22, 24, 32, 48, 64, 128, 256, 512}) {
            QString path = QString("/home/rexx/Proyectos/k4w-suite/src/ctl/icons/kinection.png");
            QPixmap pm(path);
            if (!pm.isNull())
                icon.addPixmap(pm.scaled(sz, sz, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
        setWindowIcon(icon);
        qApp->setWindowIcon(icon);
    }
    resize(1020, 780);

    m_isStarting = false;
    m_daemonOnline = false;

    m_frameTimer = new QTimer(this);
    connect(m_frameTimer, &QTimer::timeout, this, &MainWindow::onRefreshFrame);
    m_frameTimer->start(33);

    m_reconnectTimer = new QTimer(this);
    connect(m_reconnectTimer, &QTimer::timeout, this, &MainWindow::onReconnectTick);
    m_reconnectTimer->start(2000);

    /* Motor tilt debounce + cooldown */
    m_tiltBusy = false;
    m_tiltDebounce = new QTimer(this);
    m_tiltDebounce->setSingleShot(true);
    connect(m_tiltDebounce, &QTimer::timeout, this, &MainWindow::onTiltDebounce);

    m_tiltCooldown = new QTimer(this);
    m_tiltCooldown->setSingleShot(true);
    m_tiltCooldown->setInterval(2000);  /* 2s cooldown between movements */

    /* Countdown timer for visual feedback */
    m_tiltCountdownSec = 0;
    m_tiltCountdownTimer = new QTimer(this);
    m_tiltCountdownTimer->setInterval(1000);
    connect(m_tiltCountdownTimer, &QTimer::timeout, this, [this]() {
        m_tiltCountdownSec--;
        if (m_tiltCountdownSec <= 0) {
            m_tiltCountdownTimer->stop();
            m_tiltCooldownLabel->setText("");
        } else {
            m_tiltCooldownLabel->setText(QString("Cooldown: %1s").arg(m_tiltCountdownSec));
        }
    });

    /* Check audio device status on startup */
    QTimer::singleShot(500, this, [this]() {
        bool found = false;
        for (int c = 0; c < 10; c++) {
            QFile f(QString("/proc/asound/card%1/usbid").arg(c));
            if (f.open(QIODevice::ReadOnly)) {
                QString id = f.readAll();
                if (id.contains("045e") && id.contains("02bb")) { found = true; break; }
            }
        }
        if (m_micStatusLabel) {
            if (found) {
                m_micStatusLabel->setText("Estado: Kinect Audio OK");
                m_micStatusLabel->setStyleSheet("font-size:10px; color:#a6e3a1;");
            } else {
                m_micStatusLabel->setText("Estado: NO encontrado — usar Bind");
                m_micStatusLabel->setStyleSheet("font-size:10px; color:#f38ba8;");
            }
        }
    });

    /* Skeleton: update from SHM every 3rd frame (~100ms at 30fps) */
    m_skeletonFrameSkip = 3;
    m_skeletonFrameCounter = 0;

    m_recordTimerTick = new QTimer(this);
    connect(m_recordTimerTick, &QTimer::timeout, this, [this]() {
        qint64 ms = m_recordElapsed.elapsed();
        QTime t(0, 0, 0);
        t = t.addMSecs(ms);
        m_timerLabel->setText(t.toString("HH:mm:ss"));
    });

    m_socket = new QLocalSocket(this);

    /* Skeleton Tracker */
    m_tracker = new SkeletonTracker;
    m_skeletonProcess = nullptr;

    /* Sensor Poll Thread — only for daemon detection, no sensor data */
    m_sensorPollThread = new SensorPollThread(this);
    m_sensorPollThread->setEnabled(true);
    m_sensorPollThread->start();
    connect(m_sensorPollThread, &SensorPollThread::daemonStatus, this, [this](bool online) {
        if (online && !m_daemonOnline) {
            fprintf(stderr, "[App] Daemon came ONLINE\n");
            m_daemonOnline = true;
            m_statusText->setText("Daemon Corriendo - Todo OK");
            m_statusText->setStyleSheet("color:#a6e3a1; font-size:11px; font-weight:bold;");
            m_statusDot->setStyleSheet("background:#a6e3a1; border:2px solid #40a02b; border-radius:7px;");
            m_statusBar->setText("  Daemon Corriendo (Todo OK)");
            m_statusBar->setStyleSheet("background:#1e1e2e; border:1px solid #a6e3a1; border-radius:4px; padding:5px; font-size:11px; color:#a6e3a1;");
            m_startDaemonBtn->setEnabled(false);
            m_startDaemonBtn->setText("Conectado");
            m_startDaemonBtn->setStyleSheet(
                "QPushButton { background:#a6e3a1; color:#1e1e2e; border:1px solid #40a02b; "
                "border-radius:6px; padding:6px 14px; font-size:11px; font-weight:bold; }");
        } else if (!online && m_daemonOnline) {
            fprintf(stderr, "[App] Daemon went OFFLINE\n");
            m_daemonOnline = false;
            m_statusText->setText("Daemon Detenido");
            m_statusText->setStyleSheet("color:#f38ba8; font-size:11px; font-weight:bold;");
            m_statusDot->setStyleSheet("background:#f38ba8; border:2px solid #e64553; border-radius:7px;");
            m_statusBar->setText("  Daemon Detenido (Clic para iniciar)");
            m_statusBar->setStyleSheet("background:#1e1e2e; border:1px solid #f38ba8; border-radius:4px; padding:5px; font-size:11px; color:#f38ba8;");
            m_startDaemonBtn->setEnabled(true);
            m_startDaemonBtn->setText("Conectar a Daemon");
            m_startDaemonBtn->setStyleSheet(
                "QPushButton { background:#a6e3a1; color:#1e1e2e; border:1px solid #40a02b; "
                "border-radius:6px; padding:6px 14px; font-size:11px; font-weight:bold; }"
                "QPushButton:hover { background:#94d67a; }"
                "QPushButton:pressed { background:#74c76a; }");
            m_accelValue->setText("—");
            m_tiltValueSensor->setText("—");
        }
    });

    connectDaemon();

    /* System Tray */
    m_trayMenu = new QMenu(this);
    m_trayMenu->addAction("Mostrar", this, &QMainWindow::show);
    m_trayMenu->addSeparator();
    m_trayMenu->addAction("Salir", qApp, &QApplication::quit);
    m_trayIcon = new QSystemTrayIcon(this);
    m_trayIcon->setContextMenu(m_trayMenu);
    connect(m_trayIcon, &QSystemTrayIcon::activated, this, &MainWindow::onTrayActivated);
    updateTrayIcon();
    m_trayIcon->show();
}

MainWindow::~MainWindow() {
    m_sensorPollThread->setEnabled(false);
    m_sensorPollThread->wait(2000);
    m_micThread->setEnabled(false);
    m_micThread->quit();
    m_micThread->wait(2000);
    m_frameTimer->stop();
    m_reconnectTimer->stop();
    m_recordTimerTick->stop();
    if (m_skeletonProcess && m_skeletonProcess->state() != QProcess::NotRunning) {
        m_skeletonProcess->terminate();
        m_skeletonProcess->waitForFinished(3000);
        if (m_skeletonProcess->state() != QProcess::NotRunning)
            m_skeletonProcess->kill();
    }
    delete m_skeletonProcess;
}

/* ─── Connection ──────────────────────────────────────── */
bool MainWindow::isConnected() const {
    return m_daemonOnline;
}

void MainWindow::connectDaemon() {
    /* Daemon detection is handled by SensorPollThread.
     * This function just updates UI based on m_daemonOnline flag. */
}

void MainWindow::onReconnectTick() {
    updateTrayIcon();
    updateStatusDots();
}

void MainWindow::updateTrayIcon() {
    QString basePrefix;
    QString tooltip;
    if (isConnected()) {
        basePrefix = "/home/rexx/Proyectos/k4w-suite/src/ctl/icons/kinection-tray-green";
        tooltip = "k4w-ctl: Daemon corriendo";
        m_isStarting = false;
    } else if (m_isStarting) {
        basePrefix = "/home/rexx/Proyectos/k4w-suite/src/ctl/icons/kinection-tray-yellow";
        tooltip = "k4w-ctl: Conectando...";
    } else {
        basePrefix = "/home/rexx/Proyectos/k4w-suite/src/ctl/icons/kinection-tray-red";
        tooltip = "k4w-ctl: Daemon detenido";
    }
    QIcon icon;
    for (int sz : {16, 22, 24, 32, 48, 64}) {
        QPixmap pm(QString("%1-%2.png").arg(basePrefix).arg(sz));
        if (!pm.isNull())
            icon.addPixmap(pm);
    }
    m_trayIcon->setIcon(icon);
    m_trayIcon->setToolTip(tooltip);
}

void MainWindow::updateStatusDots() {
    bool connected = isConnected();

    /* Motor: red=disconnected, green outline=idle, green filled=moving */
    if (!connected) {
        m_motorStatusDot->setStyleSheet("background:transparent; border:2px solid #f38ba8; border-radius:7px;");
    } else if (m_tiltBusy) {
        m_motorStatusDot->setStyleSheet("background:#a6e3a1; border:2px solid #40a02b; border-radius:7px;");
    } else {
        m_motorStatusDot->setStyleSheet("background:transparent; border:2px solid #a6e3a1; border-radius:7px;");
    }

    /* Audio: red=not found, green outline=idle, green filled=testing, yellow=bindling */
    bool audioOk = false;
    for (int c = 0; c < 10; c++) {
        QFile f(QString("/proc/asound/card%1/usbid").arg(c));
        if (f.open(QIODevice::ReadOnly)) {
            QString id = f.readAll();
            if (id.contains("045e") && id.contains("02bb")) { audioOk = true; break; }
        }
    }
    if (!connected || !audioOk) {
        m_audioStatusDot->setStyleSheet("background:transparent; border:2px solid #f38ba8; border-radius:6px;");
    } else if (m_micTestCheck && m_micTestCheck->isChecked()) {
        m_audioStatusDot->setStyleSheet("background:#a6e3a1; border:2px solid #40a02b; border-radius:6px;");
    } else {
        m_audioStatusDot->setStyleSheet("background:transparent; border:2px solid #a6e3a1; border-radius:6px;");
    }
}

void MainWindow::onTrayActivated(QSystemTrayIcon::ActivationReason reason) {
    if (reason == QSystemTrayIcon::DoubleClick) {
        show();
        raise();
        activateWindow();
    }
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
    if (obj == m_statusBar && event->type() == QEvent::MouseButtonPress) {
        onStatusBarClicked();
        return true;
    }
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::onStatusBarClicked() {
    onStartDaemon();
}

void MainWindow::onStartDaemon() {
    if (isConnected()) return;
    m_isStarting = true;

    /* Launch daemon with sudo for proper USB/ALSA permissions */
    QFile::remove("/tmp/k4w.pid");
    QFile::remove("/tmp/k4w.sock");

    /* Rebind audio driver first (needs root) */
    QProcess::execute("pkexec", {"bash", "-c",
        "echo 1-6.1:1.2 > /sys/bus/usb/drivers/snd-usb-audio/bind 2>/dev/null; "
        "echo 1-6.1:1.3 > /sys/bus/usb/drivers/snd-usb-audio/bind 2>/dev/null"});

    /* Start daemon with pkexec for graphical sudo */
    QProcess::startDetached("pkexec", {"/home/rexx/Proyectos/k4w-suite/build/k4w"});

    m_startDaemonBtn->setEnabled(false);
    m_startDaemonBtn->setText("Iniciando...");
    m_startDaemonBtn->setStyleSheet(
        "QPushButton { background:#f9e2af; color:#1e1e2e; border:1px solid #df8e1d; "
        "border-radius:6px; padding:6px 14px; font-size:11px; font-weight:bold; }");
    m_statusBar->setText("  Iniciando daemon...");
    m_statusBar->setStyleSheet("background:#1e1e2e; border:1px solid #f9e2af; border-radius:4px; padding:5px; font-size:11px; color:#f9e2af;");

    m_reconnectTimer->start(500);
}

void MainWindow::onResetDaemon() {
    m_isStarting = true;
    m_resetDaemonBtn->setEnabled(false);
    m_resetDaemonBtn->setText("Reiniciando...");
    m_resetDaemonBtn->setStyleSheet(
        "QPushButton { background:#f9e2af; color:#1e1e2e; border:1px solid #df8e1d; "
        "border-radius:6px; padding:6px 14px; font-size:11px; font-weight:bold; }");
    m_statusBar->setText("  Reiniciando daemon...");
    m_statusBar->setStyleSheet("background:#1e1e2e; border:1px solid #f9e2af; border-radius:4px; padding:5px; font-size:11px; color:#f9e2af;");

    /* Stop sensor poll during reset */
    m_sensorPollThread->setEnabled(false);

    /* Kill running daemon by reading PID file */
    QFile pidFile("/tmp/k4w.pid");
    if (pidFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QByteArray pidLine = pidFile.readLine().trimmed();
        pidFile.close();
        if (!pidLine.isEmpty()) {
            pid_t pid = pidLine.toInt();
            if (pid > 0) {
                kill(pid, SIGTERM);
                /* Wait up to 2s for graceful shutdown */
                for (int i = 0; i < 20 && kill(pid, 0) == 0; i++) {
                    usleep(100000);
                }
                /* Force kill if still alive */
                if (kill(pid, 0) == 0) {
                    kill(pid, SIGKILL);
                    usleep(200000);
                }
            }
        }
    }

    /* Clean stale files */
    QFile::remove("/tmp/k4w.pid");
    QFile::remove("/tmp/k4w.sock");

    /* Rebind audio driver (needs root) */
    QProcess::execute("pkexec", {"bash", "-c",
        "echo 1-6.1:1.2 > /sys/bus/usb/drivers/snd-usb-audio/bind 2>/dev/null; "
        "echo 1-6.1:1.3 > /sys/bus/usb/drivers/snd-usb-audio/bind 2>/dev/null"});

    /* Start daemon fresh */
    QProcess::startDetached("pkexec", {"/home/rexx/Proyectos/k4w-suite/build/k4w"});

    m_startDaemonBtn->setEnabled(false);
    m_startDaemonBtn->setText("Iniciando...");
    m_startDaemonBtn->setStyleSheet(
        "QPushButton { background:#f9e2af; color:#1e1e2e; border:1px solid #df8e1d; "
        "border-radius:6px; padding:6px 14px; font-size:11px; font-weight:bold; }");

    m_reconnectTimer->start(500);
}

/* ─── Frame Reading ───────────────────────────────────── */
QImage MainWindow::readRGB() {
    size_t sz;
    k4w_shm_header_t *h = (k4w_shm_header_t *)openSHM("k4w_video", &sz);
    if (!h || h->magic != K4W_SHM_MAGIC) { if (h) munmap(h, sz); return QImage(); }
    uint32_t idx = (h->write_idx + K4W_RING_SIZE - 1) % K4W_RING_SIZE;
    QImage img(h->data + idx * h->frame_size, h->w, h->h, h->w * 3, QImage::Format_RGB888);
    QImage result = img.copy();
    munmap(h, sz);
    return result;
}

QImage MainWindow::readDepth() {
    size_t sz;
    k4w_shm_header_t *h = (k4w_shm_header_t *)openSHM("k4w_depth", &sz);
    if (!h || h->magic != K4W_SHM_MAGIC) { if (h) munmap(h, sz); return QImage(); }
    uint32_t idx = (h->write_idx + K4W_RING_SIZE - 1) % K4W_RING_SIZE;
    const uint16_t *raw = (const uint16_t *)(h->data + idx * h->frame_size);
    QImage img = depthToJET(raw, h->w, h->h);
    munmap(h, sz);
    return img;
}

QImage MainWindow::readIR() {
    size_t sz;
    k4w_shm_header_t *h = (k4w_shm_header_t *)openSHM("k4w_depth", &sz);
    if (!h || h->magic != K4W_SHM_MAGIC) { if (h) munmap(h, sz); return QImage(); }
    uint32_t idx = (h->write_idx + K4W_RING_SIZE - 1) % K4W_RING_SIZE;
    const uint16_t *raw = (const uint16_t *)(h->data + idx * h->frame_size);
    QImage img(h->w, h->h, QImage::Format_RGB888);
    for (int y = 0; y < (int)h->h; y++)
        for (int x = 0; x < (int)h->w; x++) {
            uint8_t v = (uint8_t)(raw[y * h->w + x] * 255 / 2048);
            img.setPixel(x, y, qRgb(v, v, v));
        }
    munmap(h, sz);
    return img;
}

QImage MainWindow::readOverlay() {
    /* Picture-in-Picture: RGB fullscreen + depth in top-right */
    QImage rgb = readRGB();
    QImage depth = readDepth();
    if (rgb.isNull() && depth.isNull()) return QImage();
    if (rgb.isNull()) return depth;
    if (depth.isNull()) return rgb;

    QImage result = rgb;

    /* Draw depth PiP in top-right corner */
    QPainter p(&result);
    int pipW = result.width() / 3;
    int pipH = result.height() / 3;
    QImage depthScaled = depth.scaled(pipW, pipH, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    int x = result.width() - pipW - 10;
    int y = 10;
    p.setPen(QPen(QColor(255, 255, 255, 200), 2));
    p.drawRect(x - 2, y - 2, depthScaled.width() + 4, depthScaled.height() + 4);
    p.drawImage(x, y, depthScaled);
    p.end();
    return result;
}

/* ─── Frame Refresh ───────────────────────────────────── */
void MainWindow::onRefreshFrame() {
    QImage frame;
    switch (m_cameraMode) {
    case 0: frame = readRGB(); break;      /* Ver Cámara (RGB) */
    case 1: frame = readIR(); break;       /* Cámara IR */
    case 2: frame = readDepth(); break;    /* Cámara Depth */
    case 3: frame = readOverlay(); break;  /* Supercam (Overlay) */
    }
    if (!frame.isNull()) {
        /* Skeleton only in Supercam mode: update from SHM every N frames, draw every frame */
        if (m_cameraMode == 3) {
            m_skeletonFrameCounter++;
            if (m_skeletonFrameCounter >= m_skeletonFrameSkip) {
                m_skeletonFrameCounter = 0;
                m_tracker->updateSkeletons();
            }
            m_tracker->drawSkeleton(frame);
        }
        m_mainView->setPixmap(QPixmap::fromImage(frame).scaled(
            m_mainView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

        /* Record frames to disk */
        if (m_recording && !m_recordDir.isEmpty()) {
            QString path = QString("%1/frame_%2.png").arg(m_recordDir)
                .arg(m_recordFrameCount++, 6, 10, QChar('0'));
            frame.save(path);
        }
    }
}

/* ─── Camera Mode ─────────────────────────────────────── */
void MainWindow::onModeChanged(int index) {
    m_cameraMode = index;
    if (index == 3) {
        /* Start skeleton tracker process if not running */
        if (!m_skeletonProcess || m_skeletonProcess->state() == QProcess::NotRunning) {
            delete m_skeletonProcess;
            m_skeletonProcess = new QProcess(this);
            m_skeletonProcess->setProgram("/tmp/mediapipe-venv/bin/python3");
            m_skeletonProcess->setArguments({"/home/rexx/Proyectos/k4w-suite/scripts/skeleton_tracker.py"});
            m_skeletonProcess->setProcessChannelMode(QProcess::ForwardedChannels);
            m_skeletonProcess->start();
        }
    } else {
        /* Stop skeleton tracker when leaving Supercam */
        if (m_skeletonProcess && m_skeletonProcess->state() != QProcess::NotRunning) {
            m_skeletonProcess->terminate();
            m_skeletonProcess->waitForFinished(3000);
            if (m_skeletonProcess->state() != QProcess::NotRunning)
                m_skeletonProcess->kill();
        }
    }
}

/* ─── Motor ───────────────────────────────────────────── */
void MainWindow::onMotorTilt(int angle) {
    m_tiltValue->setText(QString("%1°").arg(angle));
    int clamped = qBound(-31, angle, 31);
    m_kinectUp->setStyleSheet(QString("background:transparent; transform: rotate(%1deg);").arg(-clamped));
    if (!isConnected()) return;

    /* Store target and restart debounce timer (800ms cooldown) */
    m_tiltTarget = clamped;
    m_tiltDebounce->start(800);
}

void MainWindow::onTiltDebounce() {
    /* Only auto-send if user is NOT clicking Apply */
    if (m_tiltBusy || m_tiltCooldown->isActive()) return;
}

static bool sendDaemonCmd(int cmd_type, int cmd_arg, k4w_status_t *resp) {
    QLocalSocket sock;
    sock.connectToServer("/tmp/k4w.sock");
    if (!sock.waitForConnected(1000)) return false;
    int cmd[2] = { cmd_type, cmd_arg };
    sock.write((const char *)cmd, sizeof(cmd));
    if (!sock.waitForBytesWritten(500)) { sock.disconnectFromServer(); return false; }
    if (!sock.waitForReadyRead(8000)) { sock.disconnectFromServer(); return false; }
    char buf[256] = {0};
    qint64 n = sock.read(buf, sizeof(buf));
    sock.disconnectFromServer();
    if (n >= (int)sizeof(k4w_status_t)) {
        memcpy(resp, buf, sizeof(k4w_status_t));
        return true;
    }
    return false;
}

void MainWindow::onTiltApply() {
    if (m_tiltBusy || m_tiltCooldown->isActive()) return;
    if (!isConnected()) return;

    m_tiltBusy = true;
    updateStatusDots();
    m_tiltApplyBtn->setEnabled(false);
    m_tiltApplyBtn->setText("Moviendo...");
    m_tiltResetBtn->setEnabled(false);
    m_tiltSlider->setEnabled(false);

    int angle = qBound(-15, m_tiltSlider->value(), 15);
    k4w_status_t resp;
    if (sendDaemonCmd(K4W_CMD_TILT, angle, &resp)) {
        m_tiltValue->setText(QString("%1°").arg((int)resp.tilt_deg));
        m_tiltValueSensor->setText(QString("%1°").arg((int)resp.tilt_deg));
        m_accelValue->setText(QString("X:%1 Y:%2 Z:%3").arg(resp.accel_x, 0, 'f', 2).arg(resp.accel_y, 0, 'f', 2).arg(resp.accel_z, 0, 'f', 2));
        int actual = qBound(-31, (int)resp.tilt_deg, 31);
        m_kinectUp->setStyleSheet(
            QString("background:transparent; transform: rotate(%1deg);").arg(-actual));
        m_tiltSlider->setValue(actual);
    }

    m_tiltSlider->setEnabled(true);
    m_tiltApplyBtn->setEnabled(true);
    m_tiltApplyBtn->setText("Aplicar");
    m_tiltResetBtn->setEnabled(true);
    m_tiltBusy = false;
    updateStatusDots();
    m_tiltCooldown->start();
    m_tiltCountdownSec = 2;
    m_tiltCooldownLabel->setText("Cooldown: 2s");
    m_tiltCountdownTimer->start();
}

void MainWindow::onTiltReset() {
    if (m_tiltBusy || m_tiltCooldown->isActive()) return;
    if (!isConnected()) return;

    m_tiltBusy = true;
    updateStatusDots();
    m_tiltApplyBtn->setEnabled(false);
    m_tiltResetBtn->setEnabled(false);
    m_tiltResetBtn->setText("Reseteando...");
    m_tiltSlider->setEnabled(false);

    k4w_status_t resp;
    if (sendDaemonCmd(K4W_CMD_TILT, 0, &resp)) {
        m_tiltValue->setText(QString("%1°").arg((int)resp.tilt_deg));
        m_tiltValueSensor->setText(QString("%1°").arg((int)resp.tilt_deg));
        m_accelValue->setText(QString("X:%1 Y:%2 Z:%3").arg(resp.accel_x, 0, 'f', 2).arg(resp.accel_y, 0, 'f', 2).arg(resp.accel_z, 0, 'f', 2));
        m_kinectUp->setStyleSheet(
            QString("background:transparent; transform: rotate(0deg);"));
        m_tiltSlider->setValue(0);
    }

    m_tiltSlider->setEnabled(true);
    m_tiltApplyBtn->setEnabled(true);
    m_tiltResetBtn->setEnabled(true);
    m_tiltResetBtn->setText("Reset 0°");
    m_tiltBusy = false;
    updateStatusDots();
    m_tiltCooldown->start();
    m_tiltCountdownSec = 2;
    m_tiltCooldownLabel->setText("Cooldown: 2s");
    m_tiltCountdownTimer->start();
}

/* ─── Capture / Record ────────────────────────────────── */
void MainWindow::onCapturePhoto() {
    QString path = QFileDialog::getSaveFileName(this, "Guardar Foto",
        QString("kinect_%1.png").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")),
        "PNG (*.png)");
    if (path.isEmpty()) return;
    QImage frame = readRGB();
    if (!frame.isNull()) {
        frame.save(path);
        m_captureBtn->setText("  Guardado!");
        QTimer::singleShot(1500, m_captureBtn, [this]{ m_captureBtn->setText("  Capturar Foto"); });
    }
}

void MainWindow::onRecordToggle() {
    if (!m_recording) {
        /* Start recording: create directory and begin saving frames */
        m_recordDir = QString("kinect_rec_%1").arg(
            QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
        QDir().mkpath(m_recordDir);
        m_recordFrameCount = 0;
        m_recording = true;
        m_recordElapsed.start();
        m_recordTimerTick->start(100);
        m_recordBtn->setVisible(false);
        m_stopBtn->setVisible(true);
    } else {
        /* Stop recording */
        m_recording = false;
        m_recordTimerTick->stop();
        m_recordBtn->setVisible(true);
        m_stopBtn->setVisible(false);
        m_timerLabel->setText("00:00:00");
    }
}

/* ─── UI Setup (matching design mockup) ───────────────── */
void MainWindow::setupUI() {
    QWidget *central = new QWidget;
    setCentralWidget(central);
    QVBoxLayout *root = new QVBoxLayout(central);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(6);

    /* ═══ Title Bar ═══ */
    QHBoxLayout *titleRow = new QHBoxLayout;
    titleRow->setSpacing(8);

    /* Green/Red status dot */
    m_statusDot = new QLabel;
    m_statusDot->setFixedSize(14, 14);
    m_statusDot->setStyleSheet("background:#a6e3a1; border:2px solid #40a02b; border-radius:8px;");

    m_statusText = new QLabel("Daemon Corriendo - Todo OK");
    m_statusText->setStyleSheet("color:#a6e3a1; font-size:11px; font-weight:bold;");

    titleRow->addWidget(m_statusDot);
    titleRow->addWidget(m_statusText);
    titleRow->addStretch();

    /* Iniciar Daemon button */
    m_startDaemonBtn = new QPushButton("Conectar a Daemon");
    m_startDaemonBtn->setStyleSheet(
        "QPushButton { background:#a6e3a1; color:#1e1e2e; border:1px solid #40a02b; "
        "border-radius:6px; padding:6px 14px; font-size:11px; font-weight:bold; }"
        "QPushButton:hover { background:#94d67a; }"
        "QPushButton:pressed { background:#74c76a; }");
    connect(m_startDaemonBtn, &QPushButton::clicked, this, &MainWindow::onStartDaemon);
    titleRow->addWidget(m_startDaemonBtn);

    /* Reset Daemon button */
    m_resetDaemonBtn = new QPushButton("Reset Daemon");
    m_resetDaemonBtn->setStyleSheet(
        "QPushButton { background:#f38ba8; color:#1e1e2e; border:1px solid #e64553; "
        "border-radius:6px; padding:6px 14px; font-size:11px; font-weight:bold; }"
        "QPushButton:hover { background:#e06a8a; }"
        "QPushButton:pressed { background:#d05a7a; }");
    connect(m_resetDaemonBtn, &QPushButton::clicked, this, &MainWindow::onResetDaemon);
    titleRow->addWidget(m_resetDaemonBtn);

    root->addLayout(titleRow);

    /* ═══ Main 3-Column Layout ═══ */
    QHBoxLayout *mainRow = new QHBoxLayout;
    mainRow->setSpacing(10);

    /* ── Left Column: Camera Mode Dropdown ── */
    QVBoxLayout *leftCol = new QVBoxLayout;
    leftCol->setSpacing(6);

    QLabel *modeLabel = new QLabel("Lista Desplegable");
    modeLabel->setStyleSheet("font-weight:bold; font-size:11px; color:#cdd6f4;");
    leftCol->addWidget(modeLabel);

    m_modeCombo = new QComboBox;
    m_modeCombo->addItems({
        "Ver Cámara (RGB, Seleccionada)",
        "Cámara IR",
        "Cámara Depth",
        "Supercam (Overlay)"
    });
    m_modeCombo->setMinimumWidth(180);
    m_modeCombo->setStyleSheet(
        "QComboBox { background:#313244; color:#cdd6f4; border:1px solid #585b70; "
        "border-radius:4px; padding:6px 10px; font-size:11px; }"
        "QComboBox::drop-down { border:none; width:20px; }"
        "QComboBox QAbstractItemView { background:#313244; color:#cdd6f4; "
        "selection-background-color:#585b70; border:1px solid #585b70; }");
    connect(m_modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onModeChanged);
    leftCol->addWidget(m_modeCombo);

    /* ═══ Microphone Section ═══ */
    QGroupBox *micBox = new QGroupBox("Micrófono");
    micBox->setStyleSheet(
        "QGroupBox { color:#cdd6f4; border:1px solid #45475a; border-radius:6px; "
        "margin-top:8px; padding-top:14px; font-weight:bold; font-size:11px; }"
        "QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; }");

    m_audioStatusDot = new QLabel;
    m_audioStatusDot->setFixedSize(12, 12);
    m_audioStatusDot->setStyleSheet("background:transparent; border:2px solid #a6e3a1; border-radius:6px;");
    QVBoxLayout *micLayout = new QVBoxLayout;

    /* Audio status row */
    QHBoxLayout *audioStatusRow = new QHBoxLayout;
    QLabel *audioStatusLabel = new QLabel("Estado");
    audioStatusLabel->setStyleSheet("font-size:10px; color:#a6adc8;");
    audioStatusRow->addWidget(audioStatusLabel);
    audioStatusRow->addStretch();
    audioStatusRow->addWidget(m_audioStatusDot);
    micLayout->addLayout(audioStatusRow);
    micLayout->setSpacing(6);

    /* Waveform visualization */
    m_waveform = new QLabel;
    m_waveform->setFixedHeight(60);
    m_waveform->setMinimumWidth(160);
    m_waveform->setStyleSheet("background:#181825; border:1px solid #45475a; border-radius:4px;");
    m_waveform->setPixmap(QPixmap(160, 60));
    micLayout->addWidget(m_waveform);

    /* Volume slider */
    QHBoxLayout *volRow = new QHBoxLayout;
    QLabel *volIcon = new QLabel("Vol");
    volIcon->setStyleSheet("font-size:10px; color:#a6adc8;");
    volRow->addWidget(volIcon);

    m_volSlider = new QSlider(Qt::Horizontal);
    m_volSlider->setRange(0, 100);
    m_volSlider->setValue(80);
    m_volSlider->setStyleSheet(
        "QSlider::groove:horizontal { background:#45475a; height:6px; border-radius:3px; }"
        "QSlider::handle:horizontal { background:#a6e3a1; width:16px; height:16px; "
        "margin:-5px; border-radius:8px; border:2px solid #40a02b; }"
        "QSlider::sub-page:horizontal { background:#a6e3a1; border-radius:3px; }");
    volRow->addWidget(m_volSlider, 1);

    m_volLabel = new QLabel("80%");
    m_volLabel->setFixedWidth(36);
    m_volLabel->setStyleSheet("font-size:10px; color:#cdd6f4; font-family:monospace;");
    volRow->addWidget(m_volLabel);
    micLayout->addLayout(volRow);

    /* Test microphone checkbox */
    m_micTestCheck = new QCheckBox("Probar Micrófono");
    m_micTestCheck->setStyleSheet(
        "QCheckBox { color:#cdd6f4; font-size:11px; }"
        "QCheckBox::indicator { width:16px; height:16px; }"
        "QCheckBox::indicator:unchecked { background:#313244; border:1px solid #585b70; border-radius:3px; }"
        "QCheckBox::indicator:checked { background:#a6e3a1; border:1px solid #40a02b; border-radius:3px; }");
    micLayout->addWidget(m_micTestCheck);

    /* Status label */
    m_micStatusLabel = new QLabel("Estado: verificando...");
    m_micStatusLabel->setStyleSheet("font-size:10px; color:#a6adc8;");
    micLayout->addWidget(m_micStatusLabel);

    /* Bind button */
    m_micBindBtn = new QPushButton("Bind Audio Driver");
    m_micBindBtn->setStyleSheet(
        "QPushButton { background:#89b4fa; color:#1e1e2e; border:1px solid #74c7ec; "
        "border-radius:6px; padding:6px 12px; font-size:11px; font-weight:bold; }"
        "QPushButton:hover { background:#74c7ec; }"
        "QPushButton:disabled { background:#45475a; color:#585b70; }");
    connect(m_micBindBtn, &QPushButton::clicked, this, [this]() {
        m_micBindBtn->setEnabled(false);
        m_micBindBtn->setText("Bindiendo...");
        m_micStatusLabel->setText("Re-bindando driver...");
        m_micStatusLabel->setStyleSheet("font-size:10px; color:#f9e2af;");
        /* Yellow dot while binding */
        m_audioStatusDot->setStyleSheet("background:#f9e2af; border:2px solid #df8e1d; border-radius:6px;");
        QProcess::execute("pkexec", {"bash", "-c",
            "echo 1-6.1:1.2 > /sys/bus/usb/drivers/snd-usb-audio/bind 2>/dev/null; "
            "echo 1-6.1:1.3 > /sys/bus/usb/drivers/snd-usb-audio/bind 2>/dev/null"});
        /* Cooldown 3s */
        QTimer::singleShot(3000, this, [this]() {
            m_micBindBtn->setEnabled(true);
            m_micBindBtn->setText("Bind Audio Driver");
            /* Check if card appeared */
            bool found = false;
            for (int c = 0; c < 10; c++) {
                QFile f(QString("/proc/asound/card%1/usbid").arg(c));
                if (f.open(QIODevice::ReadOnly)) {
                    QString id = f.readAll();
                    if (id.contains("045e") && id.contains("02bb")) { found = true; break; }
                }
            }
            if (found) {
                m_micStatusLabel->setText("Estado: Kinect Audio OK");
                m_micStatusLabel->setStyleSheet("font-size:10px; color:#a6e3a1;");
            } else {
                m_micStatusLabel->setText("Estado: NO encontrado");
                m_micStatusLabel->setStyleSheet("font-size:10px; color:#f38ba8;");
            }
            updateStatusDots();
        });
    });
    micLayout->addWidget(m_micBindBtn);

    micBox->setLayout(micLayout);
    leftCol->addWidget(micBox);

    /* Connect mic signals */
    m_micThread = new MicMonitorThread(this);
    connect(m_volSlider, &QSlider::valueChanged, this, [this](int v) {
        m_volLabel->setText(QString("%1%").arg(v));
        m_micThread->setVolume(v);
    });
    connect(m_micTestCheck, &QCheckBox::toggled, this, [this](bool on) {
        if (on) {
            m_micThread->setEnabled(true);
            if (!m_micThread->isRunning()) m_micThread->start();
        } else {
            m_micThread->setEnabled(false);
        }
        updateStatusDots();
    });
    /* Waveform repaint timer */
    m_waveTimer = new QTimer(this);
    m_waveTimer->start(33);
    connect(m_waveTimer, &QTimer::timeout, this, [this]() {
        QPixmap pm(m_waveform->size());
        pm.fill(QColor("#181825"));
        QPainter p(&pm);
        int w = pm.width();
        int h = pm.height();
        int barW = w / 8 - 2;
        p.setPen(Qt::NoPen);
        for (int b = 0; b < 8; b++) {
            float peak = m_micThread->peaks[b];
            int barH = (int)(peak * (h - 4));
            if (barH < 2) barH = 2;
            int x = b * (barW + 2) + 1;
            int y = h - barH - 2;
            QColor c = peak > 0.8f ? QColor("#f38ba8") :
                       peak > 0.5f ? QColor("#f9e2af") : QColor("#a6e3a1");
            p.setBrush(c);
            p.drawRoundedRect(x, y, barW, barH, 2, 2);
        }
        p.end();
        m_waveform->setPixmap(pm);
    });
    connect(m_micThread, &MicMonitorThread::levelsUpdated,
            m_waveTimer, [this](){}); /* just trigger repaint via timer */

    leftCol->addStretch();
    mainRow->addLayout(leftCol, 0);

    /* ── Center Column: Camera View + Sensors ── */
    QVBoxLayout *centerCol = new QVBoxLayout;
    centerCol->setSpacing(6);

    QLabel *viewTitle = new QLabel("Vista de Cámara Principal (Cámara 1473)");
    viewTitle->setStyleSheet("font-weight:bold; font-size:12px; color:#cdd6f4;");
    centerCol->addWidget(viewTitle);

    m_mainView = new QLabel;
    m_mainView->setMinimumSize(500, 380);
    m_mainView->setAlignment(Qt::AlignCenter);
    m_mainView->setStyleSheet(
        "background:#181825; border:2px solid #45475a; border-radius:8px;");
    m_mainView->setText("RGB webcam stream");
    m_mainView->setStyleSheet(m_mainView->styleSheet() + " color:#585b70; font-size:14px;");
    centerCol->addWidget(m_mainView, 1);

    /* Buttons Row */
    QHBoxLayout *btnRow = new QHBoxLayout;
    btnRow->setSpacing(8);

    m_captureBtn = new QPushButton("  Capturar Foto");
    m_captureBtn->setStyleSheet(
        "QPushButton { background:#313244; color:#cdd6f4; border:1px solid #585b70; "
        "border-radius:6px; padding:8px 14px; font-size:11px; }"
        "QPushButton:hover { background:#45475a; }");
    connect(m_captureBtn, &QPushButton::clicked, this, &MainWindow::onCapturePhoto);
    btnRow->addWidget(m_captureBtn);

    m_recordBtn = new QPushButton("  Grabar Video");
    m_recordBtn->setStyleSheet(
        "QPushButton { background:#45475a; color:#cdd6f4; border:1px solid #585b70; "
        "border-radius:6px; padding:8px 14px; font-size:11px; }"
        "QPushButton:hover { background:#585b70; }");
    connect(m_recordBtn, &QPushButton::clicked, this, &MainWindow::onRecordToggle);
    btnRow->addWidget(m_recordBtn);

    m_stopBtn = new QPushButton("  Detener");
    m_stopBtn->setStyleSheet(
        "QPushButton { background:#f38ba8; color:#1e1e2e; border:1px solid #e64553; "
        "border-radius:6px; padding:8px 14px; font-size:11px; font-weight:bold; }");
    m_stopBtn->setVisible(false);
    connect(m_stopBtn, &QPushButton::clicked, this, &MainWindow::onRecordToggle);
    btnRow->addWidget(m_stopBtn);

    m_timerLabel = new QLabel("00:00:00");
    m_timerLabel->setStyleSheet("font-size:13px; font-family:monospace; color:#cdd6f4; padding:6px;");
    btnRow->addWidget(m_timerLabel);
    btnRow->addStretch();
    centerCol->addLayout(btnRow);

    mainRow->addLayout(centerCol, 1);

    /* ── Right Column: Motor Control ── */
    QVBoxLayout *rightCol = new QVBoxLayout;
    rightCol->setSpacing(6);

    QLabel *motorTitle = new QLabel("Control de Movimiento\n(Kinect Motor)");
    motorTitle->setAlignment(Qt::AlignCenter);
    motorTitle->setStyleSheet("font-weight:bold; font-size:12px; color:#cdd6f4;");

    m_motorStatusDot = new QLabel;
    m_motorStatusDot->setFixedSize(14, 14);
    m_motorStatusDot->setStyleSheet("background:transparent; border:2px solid #a6e3a1; border-radius:7px;");

    QHBoxLayout *motorTitleRow = new QHBoxLayout;
    motorTitleRow->addStretch();
    motorTitleRow->addWidget(motorTitle);
    motorTitleRow->addWidget(m_motorStatusDot);
    motorTitleRow->addStretch();
    rightCol->addLayout(motorTitleRow);

    /* Motor slider area */
    QHBoxLayout *motorArea = new QHBoxLayout;
    motorArea->setAlignment(Qt::AlignTop);

    /* Labels + Slider column */
    QVBoxLayout *sliderCol = new QVBoxLayout;
    sliderCol->setAlignment(Qt::AlignTop);
    sliderCol->setSpacing(4);

    QLabel *plusLabel = new QLabel("+15");
    plusLabel->setAlignment(Qt::AlignCenter);
    plusLabel->setStyleSheet("font-size:10px; color:#a6adc8;");
    sliderCol->addWidget(plusLabel);

    QLabel *zoneLabel = new QLabel("Zona\nzona");
    zoneLabel->setAlignment(Qt::AlignCenter);
    zoneLabel->setStyleSheet("font-size:9px; color:#585b70;");
    sliderCol->addWidget(zoneLabel);

    m_tiltSlider = new QSlider(Qt::Vertical);
    m_tiltSlider->setRange(-15, 15);
    m_tiltSlider->setValue(0);
    m_tiltSlider->setTickPosition(QSlider::TicksBothSides);
    m_tiltSlider->setTickInterval(5);
    m_tiltSlider->setMinimumHeight(140);
    m_tiltSlider->setStyleSheet(
        "QSlider::groove:vertical { background:#45475a; width:10px; border-radius:5px; }"
        "QSlider::handle:vertical { background:#89b4fa; height:20px; width:20px; "
        "margin:-5px; border-radius:10px; border:2px solid #74c7ec; }"
        "QSlider::sub-page:vertical { background:#89b4fa; border-radius:5px; }");
    connect(m_tiltSlider, &QSlider::valueChanged, this, [this](int angle) {
        m_tiltValue->setText(QString("%1°").arg(angle));
        int clamped = qBound(-31, angle, 31);
        m_kinectUp->setStyleSheet(QString("background:transparent; transform: rotate(%1deg);").arg(-clamped));
    });
    sliderCol->addWidget(m_tiltSlider);

    QLabel *minusLabel = new QLabel("-15");
    minusLabel->setAlignment(Qt::AlignCenter);
    minusLabel->setStyleSheet("font-size:10px; color:#a6adc8;");
    sliderCol->addWidget(minusLabel);

    /* Aplicar button */
    m_tiltApplyBtn = new QPushButton("Aplicar");
    m_tiltApplyBtn->setMinimumWidth(80);
    m_tiltApplyBtn->setStyleSheet(
        "QPushButton { background:#89b4fa; color:#1e1e2e; border:1px solid #74c7ec; "
        "border-radius:6px; padding:8px 16px; font-size:12px; font-weight:bold; }"
        "QPushButton:hover { background:#74c7ec; }"
        "QPushButton:disabled { background:#45475a; color:#585b70; }");
    connect(m_tiltApplyBtn, &QPushButton::clicked, this, &MainWindow::onTiltApply);
    sliderCol->addWidget(m_tiltApplyBtn);

    /* Reset button */
    m_tiltResetBtn = new QPushButton("Reset 0°");
    m_tiltResetBtn->setMinimumWidth(80);
    m_tiltResetBtn->setStyleSheet(
        "QPushButton { background:#f38ba8; color:#1e1e2e; border:1px solid #e64553; "
        "border-radius:6px; padding:6px 12px; font-size:11px; font-weight:bold; }"
        "QPushButton:hover { background:#e64553; }"
        "QPushButton:disabled { background:#45475a; color:#585b70; }");
    connect(m_tiltResetBtn, &QPushButton::clicked, this, &MainWindow::onTiltReset);
    sliderCol->addWidget(m_tiltResetBtn);

    /* Cooldown countdown label */
    m_tiltCooldownLabel = new QLabel("");
    m_tiltCooldownLabel->setAlignment(Qt::AlignCenter);
    m_tiltCooldownLabel->setStyleSheet("font-size:11px; color:#f9e2af; font-weight:bold;");
    sliderCol->addWidget(m_tiltCooldownLabel);

    /* Sensor data in motor section */
    QLabel *sensorTitle = new QLabel("Sensores");
    sensorTitle->setAlignment(Qt::AlignCenter);
    sensorTitle->setStyleSheet("font-weight:bold; font-size:10px; color:#a6adc8; margin-top:8px;");
    sliderCol->addWidget(sensorTitle);

    m_accelValue = new QLabel("—");
    m_accelValue->setAlignment(Qt::AlignCenter);
    m_accelValue->setStyleSheet("font-size:10px; color:#cdd6f4; font-family:monospace; padding:2px;");
    sliderCol->addWidget(m_accelValue);

    m_tiltValueSensor = new QLabel("—");
    m_tiltValueSensor->setAlignment(Qt::AlignCenter);
    m_tiltValueSensor->setStyleSheet("font-size:10px; color:#cdd6f4; font-family:monospace; padding:2px;");
    sliderCol->addWidget(m_tiltValueSensor);

    QPushButton *sensorUpdateBtn = new QPushButton("Actualizar Sensores");
    sensorUpdateBtn->setMinimumWidth(80);
    sensorUpdateBtn->setStyleSheet(
        "QPushButton { background:#45475a; color:#cdd6f4; border:1px solid #585b70; "
        "border-radius:6px; padding:6px 12px; font-size:10px; }"
        "QPushButton:hover { background:#585b70; }");
    connect(sensorUpdateBtn, &QPushButton::clicked, this, [this, sensorUpdateBtn]() {
        if (!isConnected()) return;
        sensorUpdateBtn->setEnabled(false);
        sensorUpdateBtn->setText("Leyendo...");
        /* Block briefly on ACCEL — user explicitly requested it */
        k4w_status_t resp;
        bool ok = sendDaemonCmd(K4W_CMD_ACCEL, 0, &resp);
        if (ok) {
            m_accelValue->setText(QString("X:%1 Y:%2 Z:%3").arg(resp.accel_x, 0, 'f', 2).arg(resp.accel_y, 0, 'f', 2).arg(resp.accel_z, 0, 'f', 2));
            m_tiltValueSensor->setText(QString("Ángulo: %1°").arg((int)resp.tilt_deg));
        } else {
            m_accelValue->setText("Error");
            m_tiltValueSensor->setText("Error");
        }
        sensorUpdateBtn->setEnabled(true);
        sensorUpdateBtn->setText("Actualizar Sensores");
    });
    sliderCol->addWidget(sensorUpdateBtn);

    motorArea->addLayout(sliderCol);

    /* Kinect device icons */
    QVBoxLayout *iconsCol = new QVBoxLayout;
    iconsCol->setAlignment(Qt::AlignTop);
    iconsCol->setSpacing(4);

    m_kinectUp = new QLabel;
    m_kinectUp->setPixmap(QPixmap("/home/rexx/Proyectos/k4w-suite/src/ctl/icons/kinection-enabled.png")
        .scaled(180, 140, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    m_kinectUp->setAlignment(Qt::AlignCenter);
    iconsCol->addWidget(m_kinectUp);

    m_tiltValue = new QLabel("0°");
    m_tiltValue->setAlignment(Qt::AlignCenter);
    m_tiltValue->setStyleSheet("font-size:16px; font-weight:bold; color:#89b4fa;");
    iconsCol->addWidget(m_tiltValue);

    m_kinectDown = new QLabel;
    m_kinectDown->setPixmap(QPixmap("/home/rexx/Proyectos/k4w-suite/src/ctl/icons/kinection-tilt.png")
        .scaled(180, 140, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    m_kinectDown->setAlignment(Qt::AlignCenter);
    iconsCol->addWidget(m_kinectDown);

    motorArea->addLayout(iconsCol);
    rightCol->addLayout(motorArea);
    rightCol->addStretch();

    mainRow->addLayout(rightCol, 0);

    root->addLayout(mainRow, 1);

    /* ═══ Status Bar (Bottom) ═══ */
    m_statusBar = new QLabel("  Daemon Corriendo (Todo OK)");
    m_statusBar->setStyleSheet(
        "background:#1e1e2e; border:1px solid #a6e3a1; border-radius:4px; "
        "padding:6px; font-size:11px; color:#a6e3a1;");
    m_statusBar->installEventFilter(this);
    m_statusBar->setCursor(Qt::PointingHandCursor);
    root->addWidget(m_statusBar);
}
