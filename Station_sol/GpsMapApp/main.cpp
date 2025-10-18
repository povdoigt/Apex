#include "mainwindow.h"
#include <QCoreApplication>
#include <QApplication>
#include <QWebEngineView>

int main(int argc, char *argv[]) {
    QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);  // Force Software OpenGL
    QApplication app(argc, argv);

    QWebEngineView view;
    view.setUrl(QUrl("https://www.openstreetmap.org"));
    view.show();

    return app.exec();
}
