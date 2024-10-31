#include "Textwindow.h"
#include <QVBoxLayout>


Textwindow::Textwindow(QWidget *parent) : QWidget(parent)  {

    messagesTextEdit = new QTextEdit(this);
    messagesTextEdit->setReadOnly(true);
    messagesTextEdit->setText("Update messages will appear here...");

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(messagesTextEdit);
    setLayout(layout);
}

void Textwindow::displayMessage(const QString &message) {
    messagesTextEdit->setText(message);
}
