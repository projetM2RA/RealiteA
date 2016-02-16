#ifndef WEBCAMDEVICE_H
#define WEBCAMDEVICE_H

#include <QThread>
#include <QFileDialog>
#include <QMessageBox>

#include <opencv2/opencv.hpp>
#include <Chehra.h>

#define NBR_DETECTION_MODES 4
#define NBRSAVEDIMAGES      5
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

    cv::Mat* getWebcam() { return m_frame; }

    void stop() { m_isRunning = false; }
    void play() { m_isRunning = true; }

signals:
    void updateWebcam();
    void updateScene(cv::Mat, cv::Mat);

public slots:
  void switchMode(int mode) { m_detect = (detectMode)mode; }

protected:
    void run();

private:
    // member

    void calibrateCam(FileStorage *fs);
    void calibrate();
    bool detecterVisage(std::vector<cv::Point2f> *pointsVisage);
    void faceRT();
    void chessRT();
    void qrRT();


    // attributes

    bool m_isRunning;
    detectMode m_detect;
    cv::VideoCapture m_vcap;
    cv::Mat m_rotVecs;
    cv::Mat m_tvecs;
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;
    cv::Mat* m_frame;

    Chehra* m_chehra;
    std::vector<cv::Point3f> m_pointsVisage3D;
    std::vector<std::vector<cv::Point2f>> m_images;


    int m_nbrColChess;
    int m_nbrRowChess;
    float m_chessSize;

    int m_nbrLoopSinceLastDetection;
    double m_focalePlane;
    //double m_corrector;
};

#endif // WEBCAMDEVICE_H
