#ifndef MATRIXDIALOG_H
#define MATRIXDIALOG_H

#include <QDialog>
#include <QRadioButton>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

enum { defaultMatrix = 0, existingMatrix = 1, calibrateMatrix = 2 };

class MatrixDialog : public QDialog
{
    Q_OBJECT
public:
    explicit MatrixDialog(QWidget *parent = 0);

signals:

public slots:

private:
    QRadioButton* _defaultMatrix;
    QRadioButton* _existingMatrix;
    QRadioButton* _calibrateMatrix;

    QGroupBox* _matrixGroup;

    QLabel* _title;

    QPushButton* _ok;
    QPushButton* _cancel;
};

#endif // MATRIXDIALOG_H
