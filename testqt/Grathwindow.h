#include <QApplication>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <iostream>

using namespace QtCharts;

class Grathwindow : public QChartView {
    Q_OBJECT

public:
    Grathwindow(QWidget *parent = nullptr) ;

public slots:
    void updateData() ;

private:
    QChart *chart;
    QLineSeries *series;
};

