#pragma once
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

#endif // MESSAGESSECTION_H