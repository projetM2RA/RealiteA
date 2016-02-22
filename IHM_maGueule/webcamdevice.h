#ifndef WEBCAMDEVICE_H
#define WEBCAMDEVICE_H

#include <QThread>
#include <QFileDialog>
#include <QMessageBox>

#include <opencv2/opencv.hpp>
#include <Chehra.h>

#include "calibratedialog.h"
#include "chesscaracteristicsdialog.h"

#define NBR_DETECT              4
#define NBRSAVEDIMAGES          5
#define NBRFACEPOINTSDETECTED   6

#define NBRIMAGESCALIB	20
#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6
#define SAVEDPATH "../rsc/mires/mire"

enum detectMode{noDetection = 0, faceDetection = 1, chessDetection = 2, qrDetection = 3};


class WebcamDevice : public QThread
{
    Q_OBJECT
public:
    explicit WebcamDevice(QObject *parent = 0);
    ~WebcamDevice();

    cv::Mat* getWebcam() { return _frame; }

    void stop() { _isRunning = false; }
    void play() { _isRunning = true; }

    void initMatrix();
    void initModels();

    static int webcamCount();

signals:
    void updateWebcam();
    void updateScene(cv::Mat, cv::Mat);

public slots:
  void switchMode(int mode);
  void pause() { _pause = !_pause; }
  void switchInput(int input);
  void setOptions();

protected:
    void run();

private:
    // member

    void calibrateCam(FileStorage *fs);
    bool detecterVisage(std::vector<cv::Point2f> *pointsVisage);
    bool detectChess(std::vector<cv::Point2f> *chessPoints);
    void faceRT();
    void chessRT();
    void qrRT();


    // attributes

    bool _isRunning;
    bool _pause;
    bool _chessCaracs;
    bool _reset;
    bool _chessDetected;
    detectMode _detect;
    cv::VideoCapture _vcap;
    cv::Mat _rotVecs;
    cv::Mat _tvecs;
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    cv::Mat* _frame;
    cv::Mat _nextFrame;

    Chehra* _chehra;
    std::vector<cv::Point3f> _pointsVisage3D;
    std::vector<cv::Point3f> _pointsChess3D;
    std::vector<std::vector<cv::Point2f>> _images;
    std::vector<std::vector<cv::Point2f>> _chessCornersInit;

    ChessCaracteristicsDialog* _chessDialog;

    int _nbrColChess;
    int _nbrRowChess;
    double _chessSize;

    int _nbrLoopSinceLastDetection;
    double _focalePlane;
    //double _corrector;
};

#endif // WEBCAMDEVICE_H
