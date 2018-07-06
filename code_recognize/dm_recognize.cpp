#include "dm_recognize.h"
#include "ui_dm_recognize.h"


DM_recognize::DM_recognize(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::DM_recognize)
{
    ui->setupUi(this);
}

DM_recognize::~DM_recognize()
{
    delete ui;
}
