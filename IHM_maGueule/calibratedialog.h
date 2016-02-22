#ifndef CALIBRATEDIALOG_H
#define CALIBRATEDIALOG_H

#undef min

#include <QDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QLabel>
#include <iostream>
#include <QFile>
#include <QMessageBox>
#include <QProgressDialog>

#include "WebcamGraphicsView.h"
#include "webcamgraphicsscene.h"

class CalibrateDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CalibrateDialog(cv::Mat* frame, QWidget *parent = 0);

    int getNbrCols() { return _nbrCols->value(); }
    int getNbrRows() { return _nbrRows->value(); }
    int getChessSize() { return _chessSize->value(); }
    int getImageIndex() { return _imageIndex; }

signals:

public slots:
    void updateWebcam() { _webcamFluxScene->update(); }

private slots:
    void saveSnapShot();
    void delSnapShot();
    void endSnapShot();

private:
    QSpinBox *_nbrCols;
    QSpinBox *_nbrRows;
    QDoubleSpinBox *_chessSize;
    QLabel* _infoMessage;
    QLabel* _imageIndexLabel;
    QPushButton* _save;
    QPushButton* _sup;
    QPushButton* _finish;
    QProgressDialog* _calibration;
    int _imageIndex;
    cv::Mat* _savedFrame;
    WebcamGraphicsScene* _webcamFluxScene;
    WebcamGraphicsScene* _imageSavedScene;

    bool detectChess(cv::Mat chessFrame, cv::Mat* chessSaved);
};

#endif // CALIBRATEDIALOG_H
