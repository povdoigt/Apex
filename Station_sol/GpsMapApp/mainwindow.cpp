#include "mainwindow.h"
#include <QCoreApplication>
#include <QVBoxLayout>
#include <QFile>
#include <QTextStream>
#include <QUrl>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    // Create WebEngineView for OpenStreetMap
    webView = new QWebEngineView(this);
    setCentralWidget(webView);

    // Load the HTML file (map.html) with OpenStreetMap
    webView->setUrl(QUrl::fromLocalFile(QCoreApplication::applicationDirPath() + "/map.html"));

    // Timer to update GPS points every 3 seconds
    gpsTimer = new QTimer(this);
    connect(gpsTimer, &QTimer::timeout, this, &MainWindow::updateGpsPoints);
    gpsTimer->start(3000);
}

MainWindow::~MainWindow() {
    delete webView;
    delete gpsTimer;
}

// Read GPS points from file and send to JavaScript
void MainWindow::updateGpsPoints() {
    QFile file(QCoreApplication::applicationDirPath() + "/gps.txt");

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Could not open gps.txt";
        return;
    }

    QTextStream in(&file);
    QString jsCommand = "clearMarkers();\n";  // Clear old markers

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        QStringList coords = line.split(",");
        if (coords.size() == 2) {
            QString lat = coords[0].trimmed();
            QString lon = coords[1].trimmed();
            jsCommand += QString("addMarker(%1, %2);\n").arg(lat).arg(lon);
        }
    }

    file.close();

    // Inject JavaScript to update the map with new markers
    webView->page()->runJavaScript(jsCommand);
}
