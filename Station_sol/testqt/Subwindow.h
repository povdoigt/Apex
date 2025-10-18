#pragma once
#include <QWidget>

class Subwindow :public QWidget {
    Q_OBJECT
    int row_pos;
    int col_pos;

public:
    Subwindow(int row_pos, int col_pos,QWidget *parent);
};
