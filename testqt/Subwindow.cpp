#include "Subwindow.h"

Subwindow::Subwindow(int row_pos, int col_pos,QWidget *parent) : QWidget(parent){
    this->row_pos=row_pos;
    this->col_pos=col_pos;
}

