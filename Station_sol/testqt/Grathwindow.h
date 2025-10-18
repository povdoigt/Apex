/*#include <QApplication>
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
*/
#pragma once
#include <QApplication>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QVector>

using namespace QtCharts;

class Grathwindow : public QChartView {
    Q_OBJECT

public:
    explicit Grathwindow(QWidget *parent = nullptr);

    int addCurve(const QString &name = QString());
    void addDataPoint(int curveIndex, qreal x, qreal y);
    void resetGraph();

private:
    QChart *chart;
    QVector<QLineSeries*> seriesList;
    QValueAxis *axisX;
    QValueAxis *axisY;

    qreal minX, maxX;
    qreal minY, maxY;

    qreal xOffset = 0;
    qreal lastX = 0;

    void updateAxisRanges(qreal x, qreal y);
};
