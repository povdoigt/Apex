/*#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QTextEdit>
#include <QListWidget>
#include <QPushButton>
#include <iostream>*/
/*#include <QApplication>
#include <QMainWindow>
#include <QGridLayout>

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow(){};
};*/
#pragma once

#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "Grathwindow.h"
#include "Textwindow.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() {}

    Grathwindow* getAccPlot() const;
    Grathwindow* getAltPlot() const;
    Textwindow* getLog1() const;
    Textwindow* getLog2() const;
    Textwindow* getData1() const;
    Textwindow* getData2() const;

private:
    Grathwindow *accPlot;
    Grathwindow *altPlot;
    Textwindow *log1;
    Textwindow *log2;
    Textwindow *data1;
    Textwindow *data2;
    QPushButton *resetButton;
};
