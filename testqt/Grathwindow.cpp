#include "Grathwindow.h"

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
    }