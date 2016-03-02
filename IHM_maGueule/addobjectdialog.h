#ifndef ADDOBJECTDIALOG_H
#define ADDOBJECTDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QCheckBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QPushButton>
#include <QGroupBox>
#include <QButtonGroup>

enum { face = 0, chessboard = 1, brain = 2, axes = 3 };

class AddObjectDialog : public QDialog
{
    Q_OBJECT
public:
    explicit AddObjectDialog(QWidget *parent = 0);

    QString getObjectName() { return _objectName->text(); }
    QString getObjectPath() { return _objectPath->text(); }


signals:
    void setTemplate(int);

public slots:

private slots:
    void updatePath();

private:
    QLineEdit *_objectName;
    QLineEdit *_objectPath;

    int _templateIndex;
};

#endif // ADDOBJECTDIALOG_H
