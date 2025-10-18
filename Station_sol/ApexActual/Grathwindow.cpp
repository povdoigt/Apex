/*#include "Grathwindow.h"

Grathwindow::Grathwindow(QWidget *parent) : QChartView(parent) {
        series = new QLineSeries();
        chart = new QChart();
        chart->addSeries(series);
        chart->setTitle("Real-Time Data Plot");

        // Create and configure X and Y axes
        QValueAxis *axisX = new QValueAxis();
        QValueAxis *axisY = new QValueAxis();
        axisX->setRange(0, 100);
        axisY->setRange(0, 100);

        // Add axes to the chart
        chart->addAxis(axisX, Qt::AlignBottom);
        chart->addAxis(axisY, Qt::AlignLeft);

        // Attach series to the axes
        series->attachAxis(axisX);
        series->attachAxis(axisY);

        setChart(chart);

        // Set up a timer to update the chart periodically
        QTimer *timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &Grathwindow::updateData);
        timer->start(1000);  // Update every 1 second
    }

void Grathwindow::updateData() {
        QFile dataFile("data.txt");
        if (!dataFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            std::cerr << "Error: Unable to open data file." << std::endl;
            return;
        }

        series->clear();
        QTextStream in(&dataFile);
        int x, y;
        while (!in.atEnd()) {
            in >> x >> y;
            series->append(x, y);
        }
        dataFile.close();

        chart->update();
    }*/
#include "Grathwindow.h"

Grathwindow::Grathwindow(QWidget *parent) : QChartView(parent) {
    chart = new QChart();
    chart->setTitle("Real-Time Data Plot");

    axisX = new QValueAxis();
    axisY = new QValueAxis();

    minX = 0;
    maxX = 100;
    minY = 0;
    maxY = 100;

    axisX->setRange(minX, maxX);
    axisY->setRange(minY, maxY);

    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);

    setChart(chart);
    setRenderHint(QPainter::Antialiasing);
}

int Grathwindow::addCurve(const QString &name) {
    QLineSeries *newSeries = new QLineSeries();
    if (!name.isEmpty()) newSeries->setName(name);

    chart->addSeries(newSeries);
    newSeries->attachAxis(axisX);
    newSeries->attachAxis(axisY);

    seriesList.append(newSeries);
    return seriesList.size() - 1;
}

void Grathwindow::addDataPoint(int curveIndex, qreal x, qreal y) {
    if (curveIndex < 0 || curveIndex >= seriesList.size()) return;

    lastX = x;  // Save raw X

    qreal shiftedX = x - xOffset;  // Shift origin
    seriesList[curveIndex]->append(shiftedX, y);
    updateAxisRanges(shiftedX, y);
    chart->update();
}

void Grathwindow::updateAxisRanges(qreal x, qreal y) {
    bool changed = false;
    if (x < minX) { minX = x; changed = true; }
    if (x > maxX) { maxX = x; changed = true; }
    if (y < minY) { minY = y; changed = true; }
    if (y > maxY) { maxY = y; changed = true; }

    if (changed) {
        axisX->setRange(minX, maxX);
        axisY->setRange(minY, maxY);
    }
}

void Grathwindow::resetGraph() {
    for (QLineSeries* s : seriesList) {
        s->clear();
    }

    xOffset = lastX;  // Set new origin from last raw X

    minX = 0;
    maxX = 100;
    minY = 0;
    maxY = 100;
    axisX->setRange(minX, maxX);
    axisY->setRange(minY, maxY);

    chart->update();
}
