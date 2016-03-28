#ifndef WEBCAMDEVICE_H
#define WEBCAMDEVICE_H

#include <QThread>
#include <QFileDialog>
#include <QMessageBox>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <Chehra.h>

#include "CalibrateDialog.h"
#include "OptionsDialog.h"
#include "MatrixDialog.h"

#define NBR_DETECT              4
#define NBRSAVEDIMAGES          5
#define NBRFACEPOINTSDETECTED   13

enum detectMode{noDetection = 0, faceDetection = 1, chessDetection = 2, qrDetection = 3};
enum {cancel = 0, defaultCase = 1, calibrationCase = 2};

class WebcamDevice : public QThread
{
    Q_OBJECT
public:
    explicit WebcamDevice(QObject *parent = 0);
    ~WebcamDevice();

    cv::Mat* getWebcam() { return _frame; }
    detectMode getMode() { return _detect; }

    void stop() { _isRunning = false; }
    void launch() { _isRunning = true; }

    int initMatrix();
    bool initModels();

    static int webcamCount();

signals:
    void updateWebcam();
    void updateScene(cv::Mat, cv::Mat);
    void updateDetect(bool);
    void backToBeginSig();
    void freezeButtons();
    void playVideo();
    void playCam();

public slots:
  void switchMode(int mode);
  void pause() { _pause = !_pause; }
  void switchInput(int input, bool rewind);
  bool setOptions();

  void backward() { if(_actualFps > 4) _actualFps -= 4; }
  void forward() { if(_actualFps < 60) _actualFps += 4; }
  void play() { _pause = !_pause; _actualFps = _initFps; }
  void backToBeginSlot() { switchInput(-1, true); }

protected:
    void run();

private:
    // member

    int calibrateCam(FileStorage *fs);
    bool detecterVisage(std::vector<cv::Point2f> *pointsVisage);
    bool detectChess();
    bool detectMarker();
    void trackingChess();
    void trackingMarker();
    void dbCorrelation();
    void faceRT();
    void chessRT();
    void markerRT();
    float cv_distance(cv::Point2f P, cv::Point2f Q) { return sqrt(pow(abs(P.x - Q.x),2) + pow(abs(P.y - Q.y),2)); }


    // attributes

    bool _isRunning;
    bool _pause;
    bool _chessCaracs;
    bool _reset;
    bool _chessDetected;
    bool _markerDetected;
    bool _vid;
    bool _inputIsSwitched;
    bool _launchChehra;
    detectMode _detect;
    cv::VideoCapture _vcap;
    cv::Mat _rotVecs;
    cv::Mat _tvecs;
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    cv::Mat* _frame;
    cv::Mat _bufferFrame;
    cv::Mat _testFrame;
    cv::Mat _nextFrame;
    cv::Mat _frameCropped;
    QString _path;

    Chehra* _chehra;
    std::vector<cv::Point3f> _pointsVisage3D;
    std::vector<cv::Point3f> _pointsChess3D;
    std::vector<cv::Point3f> _pointsMarker3D;
    std::vector<std::vector<cv::Point2f>> _images;
    std::vector<std::vector<cv::Point2f>> _chessCornersInit;
    std::vector<std::vector<cv::Point2f>> _markerCornersInit;
    std::vector<cv::Mat> _markersModels;

    OptionsDialog* _optionsDialog;

    int _nbrColChess;
    int _nbrRowChess;
    int _markerSize;
    int _resetLabel;
    double _chessSize;

    int _nbrLoopSinceLastDetection;
    int _actualFps;
    int _initFps;
    int _runCount;
    double _focalePlane;
    //double _corrector;
};

#endif // WEBCAMDEVICE_H
