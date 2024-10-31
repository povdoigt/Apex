#include "MainWindow.h"
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
    sub2->displayMessage("Ici donnée fusée");
    sub3->displayMessage("Ici update donnée");
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
    }
