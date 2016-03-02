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
#include <QFile>

#include <iostream>

class OptionsDialog : public QDialog
{
    Q_OBJECT
public:
    explicit OptionsDialog(int nbrCols, int nbrRows, double chessSize, int markerSize, bool chehra, QWidget *parent);

    int getNbrCols() { return _nbrCols->value(); }
    int getNbrRows() { return _nbrRows->value(); }
    int getMarkerSize() { return _markerSize->value(); }
    double getChessSize() { return _chessSize->value(); }
    bool launchChehra() { return _chehra->isChecked(); }

private:
    QCheckBox* _chehra;

    QSpinBox* _nbrCols;
    QSpinBox* _nbrRows;
    QDoubleSpinBox* _chessSize;
    QSpinBox* _markerSize;

    QLabel* _nbrRowsLabel;
    QLabel* _nbrColsLabel;
    QLabel* _chessSizeLabel;
    QLabel* _markerSizeLabel;

    QPushButton* _ok;
    QPushButton* _cancel;

};

#endif // OPTIONSDIALOG_H
