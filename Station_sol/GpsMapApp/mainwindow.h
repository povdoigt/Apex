#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWebEngineView>
#include <QTimer>

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateGpsPoints();

private:
    QWebEngineView *webView;
    QTimer *gpsTimer;
};

#endif // MAINWINDOW_H
