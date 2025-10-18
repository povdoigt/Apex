QT += widgets charts
CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS += -Wall -Wextra
SOURCES += main.cpp MainWindow.cpp Subwindow.cpp Textwindow.cpp Grathwindow.cpp TelemParser.cpp
HEADERS += MainWindow.h Subwindow.h Textwindow.h Grathwindow.h TelemParser.h
MOC_DIR = . # To specify where moc files should be placed
