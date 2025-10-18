/*#include "MainWindow.h"
#include "Textwindow.h"
#include "Grathwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    // Central widget and main layout
    QWidget *centralWidget = new QWidget(this);
    QGridLayout *mainLayout = new QGridLayout(centralWidget);

    // Add the new sections to the layout
    Grathwindow *sub1 = new Grathwindow(this);
    Textwindow *sub2 = new Textwindow(this);
    Textwindow *sub3 = new Textwindow(this);
    Textwindow *sub4 = new Textwindow(this);

    // Change text 
    sub2->setLogMode(true);  // enable log mode
    sub2->displayMessage("Nouvelle donnée ajoutée...");

    sub3->setLogMode(false); // enable single message mode
    sub3->displayMessage("État actuel: OK");

    sub4->displayMessage("Ici je sait plus");

    // Adding sections to the main layout
    mainLayout->addWidget(sub1, 0, 0, 1, 1);
    mainLayout->addWidget(sub2, 0, 2, 1, 1);
    mainLayout->addWidget(sub3, 2, 0, 1, 1);
    mainLayout->addWidget(sub4, 2, 2, 1, 1);

    // Set row and column stretch to control resizing behavior
    mainLayout->setRowStretch(0, 2); // Top row
    mainLayout->setColumnStretch(0, 2); // Left column

    setCentralWidget(centralWidget);
    setWindowTitle("Data Display Window");
    resize(600, 400);
    }*/

#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    QWidget *centralWidget = new QWidget(this);
    QGridLayout *layout = new QGridLayout(centralWidget);

    // Create widgets
    accPlot = new Grathwindow(this);  // ACC
    altPlot = new Grathwindow(this);  // ALT

    log1 = new Textwindow(this);      // LOG1
    log2 = new Textwindow(this);      // LOG2
    data1 = new Textwindow(this);     // TEMPS/LONG/LAT/VOLT 1
    data2 = new Textwindow(this);     // TEMPS/LONG/LAT/VOLT 2

    resetButton = new QPushButton("RST", this);
    resetButton->setStyleSheet("font-weight: bold; color: red;");

    // Set log mode or static text
    log1->setLogMode(true);
    log2->setLogMode(true);
    data1->setLogMode(false);
    data2->setLogMode(false);

    data1->displayMessage("Temp: --\nLong: --\nLat: --\nVolt: --");
    data2->displayMessage("Temp: --\nLong: --\nLat: --\nVolt: --");

    // Set fixed height for small display boxes
    data1->setMaximumHeight(80);
    data2->setMaximumHeight(80);

    // Data windows side by side
    QHBoxLayout *dataLayout = new QHBoxLayout();
    dataLayout->addWidget(data1);
    dataLayout->addWidget(data2);

    // Data group layout (data windows + reset button below)
    QVBoxLayout *dataGroupLayout = new QVBoxLayout();
    dataGroupLayout->addLayout(dataLayout);
    dataGroupLayout->addWidget(resetButton);

    QWidget *dataGroupWidget = new QWidget(this);
    dataGroupWidget->setLayout(dataGroupLayout);

    // Logs side by side
    QHBoxLayout *logLayout = new QHBoxLayout();
    logLayout->addWidget(log1);
    logLayout->addWidget(log2);

    QWidget *logWidget = new QWidget(this);
    logWidget->setLayout(logLayout);

    // Right column vertical layout: data group above logs
    QVBoxLayout *rightColumnLayout = new QVBoxLayout();
    rightColumnLayout->addWidget(dataGroupWidget);
    rightColumnLayout->addWidget(logWidget);

    QWidget *rightColumnWidget = new QWidget(this);
    rightColumnWidget->setLayout(rightColumnLayout);

    // Add widgets to the main grid layout
    layout->addWidget(accPlot,    0, 0, 1, 2);  // ACC plot top-left
    layout->addWidget(altPlot,    1, 0, 1, 2);  // ALT plot bottom-left
    layout->addWidget(rightColumnWidget, 0, 2, 2, 2); // Right column, spanning 2 rows and 2 columns

    // Stretch settings
    layout->setColumnStretch(0, 3); // For plots
    layout->setColumnStretch(1, 3);
    layout->setColumnStretch(2, 1); // For data and logs container
    layout->setColumnStretch(3, 1);

    layout->setRowStretch(0, 1);
    layout->setRowStretch(1, 1);

    setCentralWidget(centralWidget);
    setWindowTitle("Flight Data Display");
    resize(1200, 600);

    // Connect reset button
    connect(resetButton, &QPushButton::clicked, [this]() {
        accPlot->resetGraph();
        altPlot->resetGraph();
    });
}

// Accessors
Grathwindow* MainWindow::getAccPlot() const { return accPlot; }
Grathwindow* MainWindow::getAltPlot() const { return altPlot; }
Textwindow* MainWindow::getLog1() const { return log1; }
Textwindow* MainWindow::getLog2() const { return log2; }
Textwindow* MainWindow::getData1() const { return data1; }
Textwindow* MainWindow::getData2() const { return data2; }
