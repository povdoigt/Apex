/*#include "MainWindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
*/
#include "MainWindow.h"
#include "TelemParser.h"
#include <QApplication>
#include <QTimer>
#include <QString>
#include <fstream>

#define SERIAL_PORT "/dev/ttyACM0"
#define CSV_FILE "output.csv"

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

    log1->setTextColor(Qt::darkGreen);
    log2->setTextColor(Qt::darkBlue);
    data1->setTextColor(Qt::black);
    data2->setTextColor(Qt::black);

    log1->setLogMode(true);
    log2->setLogMode(true);

    int curveAcc = acc->addCurve("Acceleration");
    int curveAlt = alt->addCurve("Altitude");

    int counter = 0;

    std::ifstream* serialInput = new std::ifstream();
    TelemParser* parser = new TelemParser(CSV_FILE);
    bool* serialConnected = new bool(false);

    QTimer* retryTimer = new QTimer();
    QObject::connect(retryTimer, &QTimer::timeout, [=]() {
        if (*serialConnected) return;

        serialInput->open(SERIAL_PORT, std::ios::binary);
        if (serialInput->is_open()) {
            *serialConnected = true;
            log1->displayMessage("Serial port connected.");
            log2->displayMessage("Serial port connected.");
        } else {
            log1->displayMessage("Failed to open serial port. Retrying...");
            log2->displayMessage("Failed to open serial port. Retrying...");
        }
    });
    retryTimer->start(3000);

    QTimer *timer = new QTimer();
    QObject::connect(timer, &QTimer::timeout, [=, &counter]() mutable {
        if (!*serialConnected) {
            // If not connected, do nothing (or show message if you want)
            return;
        }

        TELEM_FORMAT_FT_APEX_DATA_MDR message;
        if (parser->readFromStream(*serialInput, message)) {
            parser->writeToCSV(message);

            qreal x = counter;
            acc->addDataPoint(curveAcc, x, message.acc_z);
            alt->addDataPoint(curveAlt, x, message.alt_gps);

            log1->displayMessage(QString("t=%1 | ACC=%.2f").arg(x).arg(message.acc_z));
            log2->displayMessage(QString("t=%1 | ALT=%.2f").arg(x).arg(message.alt_gps));

            QString sensorText = QString("Temp: -- °C\nLong: %1\nLat: %2\nVolt: %3 V")
                                    .arg(message.long_gps, 0, 'f', 4)
                                    .arg(message.lat_gps, 0, 'f', 4)
                                    .arg(message.volt, 0, 'f', 2);

            data1->displayMessage(sensorText);
            data2->displayMessage(sensorText);

            counter++;
        } else if (serialInput->eof()) {
            log1->displayMessage("⚠ Serial disconnected.");
            log2->displayMessage("⚠ Serial disconnected.");
            serialInput->close();
            *serialConnected = false;
        }
    });
    timer->start(500);

    return app.exec();
}
