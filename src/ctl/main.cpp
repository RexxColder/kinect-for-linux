#include <QApplication>
#include <QLocalServer>
#include <QLocalSocket>
#include <QFile>
#include <QCoreApplication>
#include <signal.h>
#include "mainwindow.h"

#define SOCKET_PATH "/tmp/kinect-for-linux.sock"
#define PID_PATH "/tmp/kinect-for-linux.pid"

static bool isProcessAlive(pid_t pid) {
    return kill(pid, 0) == 0;
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("Kinect for Linux");
    app.setOrganizationName("KinectForLinux");

    /* Single instance check */
    QFile pidFile(PID_PATH);
    if (pidFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        pid_t oldPid = pidFile.readLine().trimmed().toInt();
        pidFile.close();
        if (oldPid > 0 && isProcessAlive(oldPid)) {
            /* Another instance is running — try to raise it */
            QLocalSocket sock;
            sock.connectToServer(SOCKET_PATH);
            if (sock.waitForConnected(500)) {
                sock.write("show");
                sock.waitForBytesWritten(500);
                sock.disconnectFromServer();
                return 0;
            }
        }
        /* Stale PID — clean up */
        QFile::remove(PID_PATH);
        QFile::remove(SOCKET_PATH);
    }

    /* We are the first instance — create server */
    QLocalServer::removeServer(SOCKET_PATH);
    QLocalServer server;
    server.listen(SOCKET_PATH);

    /* Write our PID */
    if (pidFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        pidFile.write(QByteArray::number(QCoreApplication::applicationPid()));
        pidFile.close();
    }

    MainWindow w;

    /* When收到 "show" signal, raise window */
    QObject::connect(&server, &QLocalServer::newConnection, [&w, &server]() {
        QLocalSocket *client = server.nextPendingConnection();
        if (client) {
            client->readAll();
            client->deleteLater();
        }
        w.show();
        w.raise();
        w.activateWindow();
    });

    w.show();
    int ret = app.exec();

    /* Cleanup */
    QFile::remove(PID_PATH);
    QFile::remove(SOCKET_PATH);
    return ret;
}
