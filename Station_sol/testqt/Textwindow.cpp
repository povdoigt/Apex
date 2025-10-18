/*#include "Textwindow.h"
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
*/

#include "Textwindow.h"
#include <QVBoxLayout>

Textwindow::Textwindow(QWidget *parent) : QWidget(parent) {
    messagesTextEdit = new QTextEdit(this);
    messagesTextEdit->setReadOnly(true);
    messagesTextEdit->setText("Update messages will appear here...");

    // Set a bigger font size here
    QFont font = messagesTextEdit->font();
    font.setPointSize(14);  // Adjust this value as needed
    messagesTextEdit->setFont(font);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(messagesTextEdit);
    setLayout(layout);
}

void Textwindow::setLogMode(bool enabled) {
    logMode = enabled;
}

void Textwindow::setTextColor(const QColor &color) {
    messageColor = color;
}

void Textwindow::displayMessage(const QString &message) {
    QString htmlMessage = QString("<span style=\"color:%1;\">%2</span>")
                          .arg(messageColor.name(), message);

    if (logMode) {
        messagesTextEdit->append(htmlMessage);
    } else {
        messagesTextEdit->setHtml(htmlMessage);
    }
}

void Textwindow::displayMessage(int value) {
    displayMessage(QString::number(value));
}

void Textwindow::displayMessage(double value) {
    displayMessage(QString::number(value));
}

// Optional: allow changing font size dynamically
void Textwindow::setFontSize(int pointSize) {
    QFont font = messagesTextEdit->font();
    font.setPointSize(pointSize);
    messagesTextEdit->setFont(font);
}
