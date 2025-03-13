#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // Create and set up WebEngineView
    webView = new QWebEngineView(this);
    setCentralWidget(webView);
    webView->load(QUrl::fromLocalFile(QCoreApplication::applicationDirPath() + "/map.html"));

    // Timer to update GPS data every 3 seconds
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::readGpsFile);
    timer->start(3000);  // 3-second interval
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::readGpsFile() {
    QFile file(QCoreApplication::applicationDirPath() + "/gps.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    QTextStream in(&file);
    QStringList coordinates;
    while (!in.atEnd()) {
        coordinates.append(in.readLine());
    }
    file.close();

    // Send GPS points to JavaScript
    QString jsCode = "updateTrail([";
    for (const QString &coord : coordinates) {
        QStringList latLon = coord.split(",");
        if (latLon.size() == 2) {
            jsCode += QString("[%1, %2],").arg(latLon[0].trimmed()).arg(latLon[1].trimmed());
        }
    }
    jsCode.chop(1);  // Remove last comma
    jsCode += "]);";

    webView->page()->runJavaScript(jsCode);
}
