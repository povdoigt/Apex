/*#pragma once
#ifndef MESSAGESSECTION_H
#define MESSAGESSECTION_H
#include <QWidget>

#include"Subwindow.h"
#include <QTextEdit>

class Textwindow : public QWidget {
    Q_OBJECT
    QTextEdit *messagesTextEdit;
public:
    Textwindow(QWidget *parent);
    void displayMessage(const QString &message);
};

#endif // MESSAGESSECTION_H */

#pragma once
#ifndef MESSAGESSECTION_H
#define MESSAGESSECTION_H

#include <QWidget>
#include <QTextEdit>
#include <QColor>

class Textwindow : public QWidget {
    Q_OBJECT

public:
    explicit Textwindow(QWidget *parent = nullptr);
    void displayMessage(const QString &message);
    void displayMessage(int value);
    void displayMessage(double value);
    void setLogMode(bool enabled);
    void setTextColor(const QColor &color);

    // New method to set font size dynamically
    void setFontSize(int pointSize);

private:
    QTextEdit *messagesTextEdit;
    bool logMode = false;
    QColor messageColor = Qt::black;
};

#endif // MESSAGESSECTION_H
