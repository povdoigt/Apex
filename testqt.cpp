#include <QApplication>
#include <QPushButton>
#include <QMessageBox>

int main(int argc, char *argv[]) {
    // Create a Qt application
    QApplication app(argc, argv);

    // Create a button
    QPushButton button("Click Me");
    button.resize(200, 100); // Set button size
    button.setWindowTitle("Simple Qt App"); // Set window title

    // Connect button click to a lambda function to show a message box
    QObject::connect(&button, &QPushButton::clicked, [&]() {
        QMessageBox::information(nullptr, "Hello", "Button clicked!");
    });

    // Show the button
    button.show();

    // Start the application event loop
    return app.exec();
}