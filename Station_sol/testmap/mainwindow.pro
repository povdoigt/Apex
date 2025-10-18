# Set the project name
TEMPLATE = app
TARGET = project
CONFIG += c++11

# Include Qt modules
QT += core gui webenginewidgets

# Source and header files
SOURCES += main.cpp \
           mainwindow.cpp

HEADERS += mainwindow.h

# Resource files (if needed)
RESOURCES += 

# UI files (if using Qt Designer)
FORMS += mainwindow.ui

# Compiler settings
CONFIG += warn_on release

# Output directory
DESTDIR = bin
MOC_DIR = build/moc
OBJECTS_DIR = build/obj

# Default installation directory
INSTALLS += target
target.path = /usr/local/bin

# Add WebEngineWidgets support
greaterThan(QT_MAJOR_VERSION, 5): QT += webenginewidgets

# Set build flags
QMAKE_CXXFLAGS += -std=c++11
