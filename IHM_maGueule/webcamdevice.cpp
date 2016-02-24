#include "WebcamDevice.h"

// public

WebcamDevice::WebcamDevice(QObject *parent) :
    QThread(parent)
{
    _vcap = cv::VideoCapture(0);
    if(!_vcap.isOpened())
    {
        QMessageBox::StandardButton button = QMessageBox::critical(0, tr("Error"),
                             tr("Unable to find a webcam.\nDo you wish to load a video ?"),
                              QMessageBox::Yes | QMessageBox::No);
    }
    //vcap.set(CV_CAP_PROP_FPS, 30);
    _frame = new cv::Mat(cv::Mat::zeros(_vcap.get(CV_CAP_PROP_FRAME_HEIGHT), _vcap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3));

    do
    {
        _vcap >> *_frame;
    }while(_frame->empty()); // on s'assure que la camera est lancé (#lenteurDuPCDeNico)
    //////////////////////////////////////////////////

    _isRunning = true;
    _pause = false;
    _chessCaracs = false;
    _chessDetected = false;
    _detect = noDetection;

    _nbrColChess = 9;
    _nbrRowChess = 6;
    _chessSize = 26.0;
    //_corrector = (_focalePlane / 2) / (_cameraMatrix.at<double>(1, 2) - _vcap.get(CV_CAP_PROP_FRAME_HEIGHT) / 2);

    this->start();
}

WebcamDevice::~WebcamDevice()
{
    delete _frame;
}

void WebcamDevice::initMatrix()
{
    //////////////////////////////////////////////////
    ////////// initialisation OpenCV /////////////////
    //////////////////////////////////////////////////
    cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

    if(!fs.isOpened())
        calibrateCam(&fs);

    fs["cameraMatrix"] >> _cameraMatrix;
    fs["distCoeffs"] >> _distCoeffs;

    _focalePlane = (_cameraMatrix.at<double>(0, 0) + _cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carrés, fx = fy -> np
    //mais est généralement différent de fy donc on prend (pour l'instant) par défaut la valeur médiane

    fs.release();

    this->setOptions();
}



void WebcamDevice::initModels()
{
    //////////////////////////////////////////////////
    ////////// initialisation variables visage ///////
    //////////////////////////////////////////////////

    _chessCornersInit = std::vector<std::vector<cv::Point2f>>(2);

    std::cout << "initialisation de Chehra..." << std::endl;
    _chehra = new Chehra;
    std::cout << "done" << std::endl;

    // Repere visage
    _pointsVisage3D.push_back(cv::Point3f(-11, -34, 0)); // exterieur narine gauche sur l'image
    _pointsVisage3D.push_back(cv::Point3f(11, -34, 0)); // exterieur narine droite sur l'image
    _pointsVisage3D.push_back(cv::Point3f(0, -26, -14)); // bout du nez
    _pointsVisage3D.push_back(cv::Point3f(-34, -7, 24)); // exterieur oeil gauche sur l'image
    _pointsVisage3D.push_back(cv::Point3f(0, 0, 0)); // haut du nez, centre des yeux
    _pointsVisage3D.push_back(cv::Point3f(34, -7, 24)); // exterieur oeil droit sur l'image

    // Repere chess
    for(int x = -_nbrColChess / 2; x < _nbrColChess / 2 + _nbrColChess % 2; x++)
        for(int y = -_nbrRowChess / 2; y < _nbrRowChess / 2 + _nbrRowChess % 2; y++)
            _pointsChess3D.push_back(cv::Point3f(x * _chessSize, y * _chessSize, 0.0f));
}


//static
int WebcamDevice::webcamCount()
{
    int count = 0;
    for(int i = 0; i < 10; ++i)
    {
        cv::VideoCapture test(i);
        if(test.isOpened())
            ++count;
        test.release();
    }
    return count;
}



// public slot
void WebcamDevice::switchMode(int mode)
{
    _detect = (detectMode)mode;

}

void WebcamDevice::switchInput(int input)
{
    if(input == -1)
    {
        QString videoPath = QFileDialog::getOpenFileName(0, "Open video", "../rsc/video/", "video file (*.avi)");
        if(videoPath == "")
            return;

        else
        {
            _pause = false;
            //_vcap.release();
            _frame->release();
            //delete _frame; // je sais pas si c'est correct...

            std::cout << "test" << std::endl;

            _vcap = cv::VideoCapture(videoPath.toStdString());
            if(!_vcap.isOpened())
            {
                std::cout << "error    h :  " << _vcap.get(CV_CAP_PROP_FRAME_HEIGHT) << "    w : " << _vcap.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
            }

            std::cout << "test" << std::endl;
            _frame = new cv::Mat(cv::Mat::zeros(_vcap.get(CV_CAP_PROP_FRAME_HEIGHT), _vcap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3));

            std::cout << "test" << std::endl;

            do
            {
                _vcap >> *_frame;
            }while(_frame->empty()); // on s'assure que la camera est lancé (#lenteurDuPCDeNico)

            std::cout << "test" << std::endl;
        }
    }
    else
    {
        _pause = false;
        _vcap.release();
        _frame->release();
        delete _frame; // je sais pas si c'est correct...

        _vcap.open(input);
        _frame = new cv::Mat(cv::Mat::zeros(_vcap.get(CV_CAP_PROP_FRAME_HEIGHT), _vcap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3));

        do
        {
            _vcap >> *_frame;
        }while(_frame->empty()); // on s'assure que la camera est lancé (#lenteurDuPCDeNico)
        _pause = true;
    }
}

void WebcamDevice::setOptions()
{
    _optionsDialog = new OptionsDialog(_nbrColChess, _nbrRowChess, _chessSize, (QWidget*)this->parent());
    if(_optionsDialog->exec() == QDialog::Accepted)
    {
        _nbrColChess = _optionsDialog->getNbrCols();
        _nbrRowChess = _optionsDialog->getNbrRows();
        _chessSize = _optionsDialog->getChessSize();
        _chessDetected = false;
    }
    else
        return;
}


// protected

void WebcamDevice::run()
{
    while(_isRunning)
    {
        if(!_pause)
        {
            switch(_detect)
            {
            case noDetection:
                _vcap >> *_frame;

                break;
            case faceDetection:
                _vcap >> *_frame;

                faceRT();

                break;
            case chessDetection:
                _vcap >> *_frame;

                chessRT();

                break;
            case qrDetection:
                _vcap >> *_frame;

                markerRT();

                break;
            default:
                _vcap >> *_frame;

                break;
            }
            emit updateWebcam();
            msleep(33); // 33 fps
        }
    }
}




// private
void WebcamDevice::calibrateCam(cv::FileStorage *fs)
{
    bool stop = false;
    do
    {
        QMessageBox::StandardButton button = QMessageBox::critical(0, tr("Error"),
                             tr("Unable to find calibration matrix.\nHave you already calibrated your camera?"),
                              QMessageBox::Yes | QMessageBox::No);

        switch(button)
        {
            case QMessageBox::No:
            {
                CalibrateDialog calibrateDialog(_frame);
                connect(this, SIGNAL(updateWebcam()), &calibrateDialog, SLOT(updateWebcam()));
                calibrateDialog.exec();
                stop = true;
                break;
            }
            case QMessageBox::Yes:
            {
                cv::Mat test1, test2;
                QString fsPath = QFileDialog::getOpenFileName(0, "Select camera intrinsic matrix", "../rsc/", "FileStorage (*.yml)");
                if(fsPath != "")
                    fs->open(fsPath.toStdString(), cv::FileStorage::READ);
                (*fs)["cameraMatrix"] >> test1;
                (*fs)["distCoeffs"] >> test2;
                if(test1.empty() || test2.empty())
                {
                    QMessageBox::warning(0, tr("incorrect matrix"), tr("the given matrix is incorrect"));
                    fs->release();
                }
                else
                {
                    stop = true;

                    cv::FileStorage fs2("../rsc/intrinsicMatrix.yml", cv::FileStorage::WRITE);
                    fs2 << "cameraMatrix" << test1 << "distCoeffs" << test2;
                    fs2.release();
                }
                break;
            }
            default:
                break;
        }
    }while(!stop);
    fs->open("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);
}

bool WebcamDevice::detecterVisage(std::vector<cv::Point2f> *pointsVisage)
{
    bool visageFound = false;
    cv::Mat imNB;
    cv::Mat points;

    cv::cvtColor(*_frame, imNB, CV_BGR2GRAY);

    visageFound = (*_chehra).track(imNB);

    if(visageFound)
    {
        points = (*_chehra).getTrackedPoints();
        if (points.rows == 98){
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(14, 0), points.at<float>(14 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(18, 0), points.at<float>(18 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(13, 0), points.at<float>(13 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(19, 0), points.at<float>(19 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(10, 0), points.at<float>(10 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(28, 0), points.at<float>(28 + 49, 0)));
        }
        else
            return false;
    }

    return visageFound;
}

bool WebcamDevice::detectChess(std::vector<cv::Point2f> *chessPoints)
{
    bool chessFound = false;
    cv::Mat imGray;

    cv::cvtColor(*_frame, imGray, CV_BGR2GRAY);

    chessFound = cv::findChessboardCorners(imGray, cv::Size(_nbrRowChess, _nbrColChess), *chessPoints, cv::CALIB_CB_FAST_CHECK);

    if(chessFound)
        cv::swap(_nextFrame, imGray);

    return chessFound;
}

void WebcamDevice::trackingChess(cv::Mat *rotVecs)
{
    cv::Mat rvecs;
    cv::Mat imCalib;

    cv::swap(imCalib, _nextFrame);
    _chessCornersInit[0] = _chessCornersInit[1];
    _chessCornersInit[1].clear();

    cv::cornerSubPix(imCalib, _chessCornersInit[0], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    cv::cvtColor(*_frame, _nextFrame, CV_BGR2GRAY);

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(imCalib, _nextFrame, _chessCornersInit[0], _chessCornersInit[1], status, err, cv::Size(31, 31), 3, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1), 0, 0.0001);

    cv::solvePnP(_pointsChess3D, _chessCornersInit[0], _cameraMatrix, _distCoeffs, rvecs, _tvecs);

    _rotVecs = cv::Mat(3, 3, CV_64F);
    cv::Rodrigues(rvecs, (*rotVecs));
}

void WebcamDevice::trackingMarker(cv::Mat *rotVecs)
{
    cv::Mat rvecs;
    cv::Mat imCalib;

    cv::swap(imCalib, _nextFrame);
    _chessCornersInit[0] = _chessCornersInit[1];
    _chessCornersInit[1].clear();

    cv::cornerSubPix(imCalib, _chessCornersInit[0], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    cv::cvtColor(*_frame, _nextFrame, CV_BGR2GRAY);

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(imCalib, _nextFrame, _chessCornersInit[0], _chessCornersInit[1], status, err, cv::Size(31, 31), 3, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1), 0, 0.0001);

    cv::solvePnP(_pointsChess3D, _chessCornersInit[0], _cameraMatrix, _distCoeffs, rvecs, _tvecs);

    _rotVecs = cv::Mat(3, 3, CV_64F);
    cv::Rodrigues(rvecs, (*rotVecs));
}



void WebcamDevice::faceRT()
{
    cv::Mat rvecs;
    std::vector<cv::Point2f> pointsVisage2D;
    std::vector<cv::Point2f> moyPointsVisage2D;
    bool faceDetected = detecterVisage(&pointsVisage2D);

    if(faceDetected)
    {
        _nbrLoopSinceLastDetection = 0;
        _images.push_back(pointsVisage2D);
    }
    else
        _nbrLoopSinceLastDetection++;

    if((_images.size() > NBRSAVEDIMAGES || _nbrLoopSinceLastDetection > NBRSAVEDIMAGES) && !_images.empty())
        _images.erase(_images.begin());

    if(!_images.empty())
    {
        for(int i = 0; i < NBRFACEPOINTSDETECTED; i++)
        {
            cv::Point2f coordonee(0.0f, 0.0f);
            for(int j = 0; j < _images.size(); j++)
            {
                coordonee.x += _images[j][i].x;
                coordonee.y += _images[j][i].y;
            }
            coordonee.x /= _images.size();
            coordonee.y /= _images.size();

            moyPointsVisage2D.push_back(coordonee);
        }

        cv::solvePnP(_pointsVisage3D, moyPointsVisage2D, _cameraMatrix, _distCoeffs, rvecs, _tvecs);

        _rotVecs = cv::Mat(3, 3, CV_64F);
        cv::Rodrigues(rvecs, _rotVecs);

        emit updateScene(_rotVecs, _tvecs);
    }
}

void WebcamDevice::chessRT()
{
    std::vector<cv::Point2f> imagePoints;
    std::vector<double> errors;
    double meanErrors;

    if(_chessDetected)
    {
        this->trackingChess(&_rotVecs);

        cv::projectPoints(_pointsChess3D, _rotVecs, _tvecs, _cameraMatrix, _distCoeffs, imagePoints);

        // Draw chess points
        for(int m = 0; m < _chessCornersInit[0].size(); m++)
            cv::circle(*_frame, cv::Point(_chessCornersInit[0][m].x, _chessCornersInit[0][m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

        emit updateScene(_rotVecs, _tvecs);

        double mean = 0;
        for(int j = 0; j < _nbrColChess * _nbrRowChess; j++)
        {
            double d = sqrt(pow(_chessCornersInit[0][j].y - imagePoints[j].y, 2) + pow(_chessCornersInit[0][j].x - imagePoints[j].x, 2));
            errors.push_back(d);
            mean += d;
        }

        meanErrors = mean / (_nbrColChess * _nbrRowChess);

        if(meanErrors > 2)
            _reset = true;
    }

    if (!_chessDetected || _reset == true)
    {
        imagePoints.clear();
        _chessCornersInit[0].clear();
        _chessCornersInit[1].clear();
        meanErrors = 0;
        errors.clear();
        _nextFrame.release();
        _reset = false;
        _chessDetected = false;

        _chessDetected = detectChess(&_chessCornersInit[1]);
    }
}

void WebcamDevice::markerRT()
{

}
