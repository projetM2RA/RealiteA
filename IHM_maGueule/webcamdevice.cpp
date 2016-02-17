#include "webcamdevice.h"

// public

WebcamDevice::WebcamDevice(QObject *parent) :
    QThread(parent)
{
    _vcap = cv::VideoCapture(0);
    /*
    if(!vcap.isOpened())
    {
        std::cout << "FAIL!" << std::endl;
        return;
    }
    */
    //vcap.set(CV_CAP_PROP_FPS, 30);
    _frame = new cv::Mat(cv::Mat::zeros(_vcap.get(CV_CAP_PROP_FRAME_HEIGHT), _vcap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3));

    do
    {
        _vcap >> *_frame;
    }while(_frame->empty()); // on s'assure que la camera est lancé (#lenteurDuPCDeNico)

    //////////////////////////////////////////////////

    _isRunning = true;
    _detect = noDetection;
    //_corrector = (_focalePlane / 2) / (_cameraMatrix.at<double>(1, 2) - _vcap.get(CV_CAP_PROP_FRAME_HEIGHT) / 2);

    this->start();
}

WebcamDevice::~WebcamDevice()
{
    delete _frame;
}

void WebcamDevice::initModels()
{
    //////////////////////////////////////////////////
    ////////// initialisation variables visage ///////
    //////////////////////////////////////////////////

    std::cout << "initialisation de Chehra..." << std::endl;
    _chehra = new Chehra;
    std::cout << "done" << std::endl;

    _pointsVisage3D.push_back(cv::Point3f(-110, 0, -336)); // exterieur narine gauche sur l'image
    _pointsVisage3D.push_back(cv::Point3f(110, 0, -336)); // exterieur narine droite sur l'image
    _pointsVisage3D.push_back(cv::Point3f(0, -142, -258)); // bout du nez
    _pointsVisage3D.push_back(cv::Point3f(-338, 243, -70)); // exterieur oeil gauche sur l'image
    _pointsVisage3D.push_back(cv::Point3f(0, 0, 0)); // haut du nez, centre des yeux
    _pointsVisage3D.push_back(cv::Point3f(338, 243, -70)); // exterieur oeil droit sur l'image

    //////////////////////////////////////////////////
    ////////// initialisation OpenCV /////////////////
    //////////////////////////////////////////////////
    cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

    while(!fs.isOpened())
        calibrateCam(&fs);

    fs["cameraMatrix"] >> _cameraMatrix;
    fs["distCoeffs"] >> _distCoeffs;

    _focalePlane = (_cameraMatrix.at<double>(0, 0) + _cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carrés, fx = fy -> np
    //mais est généralement différent de fy donc on prend (pour l'instant) par défaut la valeur médiane

    fs.release();
}






// protected

void WebcamDevice::run()
{
    while(_isRunning)
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

            qrRT();

            break;
        default:
            _vcap >> *_frame;

            break;
        }
        emit updateWebcam();
        msleep(33); // 33 fps
    }
}




// private
void WebcamDevice::calibrateCam(cv::FileStorage *fs)
{
    bool stop = false;
    do
    {
        QMessageBox::StandardButton button = QMessageBox::critical(0, tr("Erreur"),
                             tr("Impossible de trouver la matrice de calibration.\nDisposez-vous deja d'une matrice camera ?"),
                              QMessageBox::Yes | QMessageBox::No);

        switch(button)
        {
            case QMessageBox::No:
                CalibrateDialog calibrateDialog(_frame);
                calibrateDialog.exec();
                break;
            case QMessageBox::Yes:
            {
                cv::Mat test1, test2;
                QString fsPath = QFileDialog::getOpenFileName(0, "Ouvrir la matrice camera", "../rsc/", "FileStorage (*.yml)");
                if(fsPath != "")
                    fs->open(fsPath.toStdString(), cv::FileStorage::READ);
                (*fs)["cameraMatrix"] >> test1;
                (*fs)["distCoeffs"] >> test2;
                if(test1.empty() || test2.empty())
                    QMessageBox::warning(0, tr("matrice incorrecte"), tr("La matrice indiquee n'est pas correcte"));
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
                emit shutdownSignal();
                break;
        }
    }while(!stop);
}

void WebcamDevice::calibrate()
{
    cv::Mat imCalib[NBRIMAGESCALIB];
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F), distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    cameraMatrix.at<double>(0,0) = 1.0;

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<std::vector<cv::Point2f>> chessCornersInit;
    std::vector<std::vector<cv::Point3f>> chessCorners3D;

    std::cout << "Debut du calibrage..." << std::endl;

    for(int i = 0; i < NBRIMAGESCALIB; i++)
    {
        std::vector<cv::Point2f> initCorners(ROWCHESSBOARD * COLCHESSBOARD, cv::Point2f(0, 0));
        chessCornersInit.push_back(initCorners);
    }

    for(int i = 0; i < NBRIMAGESCALIB; i++)
    {
        std::ostringstream oss;
        oss << SAVEDPATH << i + 1 << ".png";
        imCalib[i] = cv::imread(oss.str());
        cv::cvtColor(imCalib[i], imCalib[i], CV_BGR2GRAY);

        bool patternfound = cv::findChessboardCorners(imCalib[i], cv::Size(ROWCHESSBOARD, COLCHESSBOARD), chessCornersInit[i]);

        if(patternfound)
            cv::cornerSubPix(imCalib[i], chessCornersInit[i], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        cv::drawChessboardCorners(imCalib[i], cv::Size(ROWCHESSBOARD, COLCHESSBOARD), cv::Mat(chessCornersInit[i]), patternfound);

        //cv::imshow("image", imCalib[i]);

        //cv::waitKey(0);
    }

    for(int i = 0; i < NBRIMAGESCALIB; i++)
    {
        std::vector<cv::Point3f> initCorners3D;
        for(int j = 0; j < COLCHESSBOARD; j++)
        {
            for(int k = 0; k < ROWCHESSBOARD; k++)
            {
                cv::Point3f corner(j * 26.0f, k * 26.0f, 0.0f);
                initCorners3D.push_back(corner);
            }
        }
        chessCorners3D.push_back(initCorners3D);
    }

    cv::calibrateCamera(chessCorners3D, chessCornersInit, cv::Size(imCalib[0].size()), cameraMatrix, distCoeffs, rvecs, tvecs);

    std::string filename = "../rsc/intrinsicMatrix.yml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;

    std::cout << "Sauvegarde de la matrice intrinseque : " << filename << std::endl;

    fs.release();
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
        if (points.rows == 98)
            for(int i = 0; i < 49; i++)
            {
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

}

void WebcamDevice::qrRT()
{

}
