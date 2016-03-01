#ifndef MATRIXDIALOG_H
#define MATRIXDIALOG_H

#include <QDialog>
#include <QRadioButton>
#include <QPushButton>
#include <QGroupBox>
#include <QButtonGroup>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include <iostream>

enum { defaultMatrix = 0, existingMatrix = 1, calibrateMatrix = 2 };

class MatrixDialog : public QDialog
{
    Q_OBJECT
public:
    explicit MatrixDialog(QWidget *parent = 0);

    int getChoice() { return _index;}

signals:

public slots:

private slots:
    void updateIndex(int index) { _index = index; }

private:
    QRadioButton* _defaultMatrix;
    QRadioButton* _existingMatrix;
    QRadioButton* _calibrateMatrix;

    QGroupBox* _matrixGroup;
    QButtonGroup* _buttonGroup;

    QLabel* _title;

    QPushButton* _ok;
    QPushButton* _cancel;

    int _index;
};

#endif // MATRIXDIALOG_H
