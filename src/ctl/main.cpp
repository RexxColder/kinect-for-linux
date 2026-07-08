#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("Kinect for Linux");
    app.setOrganizationName("KinectForLinux");

    MainWindow w;
    w.show();
    return app.exec();
}
