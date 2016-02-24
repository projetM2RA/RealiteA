#ifndef OPTIONSDIALOG_H
#define OPTIONSDIALOG_H

#include <QDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QCheckBox>

class OptionsDialog : public QDialog
{
    Q_OBJECT
public:
    explicit OptionsDialog(int nbrCols, int nbrRows, double chessSize, QWidget *parent = 0);

    int getNbrCols() { return _nbrCols->value(); }
    int getNbrRows() { return _nbrRows->value(); }
    double getChessSize() { return _chessSize->value(); }

signals:

public slots:

private:
    QSpinBox* _nbrCols;
    QSpinBox* _nbrRows;
    QDoubleSpinBox* _chessSize;

    QCheckBox* _chehra;

    QLabel* _nbrRowsLabel;
    QLabel* _nbrColsLabel;
    QLabel* _chessSizeLabel;

    QPushButton* _ok;
    QPushButton* _cancel;

};

#endif // OPTIONSDIALOG_H
