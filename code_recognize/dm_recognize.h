#ifndef DM_RECOGNIZE_H
#define DM_RECOGNIZE_H

#include <QMainWindow>

namespace Ui {
class DM_recognize;
}

class DM_recognize : public QMainWindow
{
    Q_OBJECT

public:
    explicit DM_recognize(QWidget *parent = 0);
    ~DM_recognize();

private:
    Ui::DM_recognize *ui;
};

#endif // DM_RECOGNIZE_H
