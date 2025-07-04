/*#include "MainWindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
*/
#include "MainWindow.h"
#include <QApplication>
#include <QTimer>
#include <QtMath>
#include <QString>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();

    Grathwindow* acc = window.getAccPlot();
    Grathwindow* alt = window.getAltPlot();
    Textwindow* log1 = window.getLog1();
    Textwindow* log2 = window.getLog2();
    Textwindow* data1 = window.getData1();
    Textwindow* data2 = window.getData2();

    // Set colors for text (optional)
    log1->setTextColor(Qt::darkGreen);
    log2->setTextColor(Qt::darkBlue);
    data1->setTextColor(Qt::black);
    data2->setTextColor(Qt::black);

    // Set logging mode
    log1->setLogMode(true);
    log2->setLogMode(true);

    // Add curves
    int curveAcc = acc->addCurve("Acceleration");
    int curveAlt = alt->addCurve("Altitude");

    int counter = 0;

    QTimer *timer = new QTimer();
    QObject::connect(timer, &QTimer::timeout, [=, &counter]() mutable {
        qreal x = counter;
        qreal accValue = qSin(x * 0.1) * 10 + 10;   // Fake acceleration data
        qreal altValue = x * 0.8 + qSin(x * 0.05) * 5;  // Fake altitude

        // Plot on graphs
        acc->addDataPoint(curveAcc, x, accValue);
        alt->addDataPoint(curveAlt, x, altValue);

        // Log values
        log1->displayMessage(QString("t=%1 | ACC=%.2f").arg(x).arg(accValue));
        log2->displayMessage(QString("t=%1 | ALT=%.2f").arg(x).arg(altValue));

        // Simulate sensor values
        qreal temp = 20.0 + qSin(x * 0.02) * 5;
        qreal longi = 2.33 + qCos(x * 0.01);
        qreal lati = 48.85 + qSin(x * 0.01);
        qreal volt = 3.7 + qSin(x * 0.05) * 0.2;

        QString sensorText = QString("Temp: %1°C\nLong: %2\nLat: %3\nVolt: %4 V")
                                .arg(temp, 0, 'f', 1)
                                .arg(longi, 0, 'f', 4)
                                .arg(lati, 0, 'f', 4)
                                .arg(volt, 0, 'f', 2);

        data1->displayMessage(sensorText);
        data2->displayMessage(sensorText);

        counter++;
    });

    timer->start(500);

    return app.exec();
}
