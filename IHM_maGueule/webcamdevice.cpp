#include "WebcamDevice.h"

// public

WebcamDevice::WebcamDevice(QObject *parent) :
    QThread(parent)
{
    _vcap = cv::VideoCapture(0);
    _frame = new cv::Mat(cv::Mat::zeros(640,480, CV_8UC3));
    _rotVecs = cv::Mat::zeros(3, 3, CV_64F);
    _initFps = 25;
    _actualFps = 25;

    _runCount = 0;
    _resetLabel = 0;

    _isRunning = true;
    _pause = false;
    _chessCaracs = false;
    _chessDetected = false;
    _markerDetected = false;
    _vid = false;
    _inputIsSwitched = false;
    _launchChehra = true;
    _detect = noDetection;

    _nbrColChess = 9;
    _nbrRowChess = 6;
    _chessSize = 26.0;
    _markerSize = 50;

    if(!_vcap.isOpened())
    {
        QMessageBox::StandardButton button = QMessageBox::critical(0, tr("Error"),
                                                                   tr("Unable to find a webcam.\nDo you wish to load a video ?"),
                                                                   QMessageBox::Yes | QMessageBox::No);

        if(button == QMessageBox::Yes)
            this->switchInput(-1, false);
        else
            return;
    }
    //vcap.set(CV_CAP_PROP_FPS, 30);
    else
    {
        do
        {
            _vcap >> *_frame;
        }while(_frame->empty()); // on s'assure que la camera est lanc� (#lenteurDuPCDeNico)
    }
    //////////////////////////////////////////////////

    //_corrector = (_focalePlane / 2) / (_cameraMatrix.at<double>(1, 2) - _vcap.get(CV_CAP_PROP_FRAME_HEIGHT) / 2);

    this->start();
}

WebcamDevice::~WebcamDevice()
{
    delete _frame;
}

/*
 * membre qui va initialiser les matrices intrinseques
 * selon le choix de l'utilisateur (par defaut ou
 * en lisant un fichier yml)
 */
int WebcamDevice::initMatrix()
{
    //////////////////////////////////////////////////
    ////////// initialisation OpenCV /////////////////
    //////////////////////////////////////////////////
    cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

    if(!fs.isOpened())
    {
        switch(calibrateCam(&fs))
        {
        case cancel:
        {
            return cancel;
            break;
        }
        case defaultCase:
        {
            _cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);
            _distCoeffs = cv::Mat::zeros(1, 5, CV_32F);

            _cameraMatrix.at<double>(0,0) = 680.0;
            _cameraMatrix.at<double>(0,1) = 0;
            _cameraMatrix.at<double>(0,2) = 320.0;
            _cameraMatrix.at<double>(1,0) = 0;
            _cameraMatrix.at<double>(1,1) = 680.0;
            _cameraMatrix.at<double>(1,1) = 240.0;
            _cameraMatrix.at<double>(2,0) = 0;
            _cameraMatrix.at<double>(2,1) = 0;
            _cameraMatrix.at<double>(2,2) = 1;

            _distCoeffs.at<double>(0,0) = 0.0;
            _distCoeffs.at<double>(0,1) = 0.0;
            _distCoeffs.at<double>(0,2) = 0.0;
            _distCoeffs.at<double>(0,3) = 0.0;
            _distCoeffs.at<double>(0,4) = 0.0;

            _focalePlane = (_cameraMatrix.at<double>(0, 0) + _cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carr�s, fx = fy -> np
            //mais est g�n�ralement diff�rent de fy donc on prend (pour l'instant) par d�faut la valeur m�diane
            fs.release();

            if(this->setOptions())
                return defaultCase;
            else
                return cancel;

            break;
        }
        case calibrationCase:
        {
            fs["cameraMatrix"] >> _cameraMatrix;
            fs["distCoeffs"] >> _distCoeffs;

            _focalePlane = (_cameraMatrix.at<double>(0, 0) + _cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carr�s, fx = fy -> np
            //mais est g�n�ralement diff�rent de fy donc on prend (pour l'instant) par d�faut la valeur m�diane

            fs.release();

            if(this->setOptions())
                return calibrationCase;
            else
                return cancel;

            break;
        }
        }
    }
    else
    {
        fs["cameraMatrix"] >> _cameraMatrix;
        fs["distCoeffs"] >> _distCoeffs;

        _focalePlane = (_cameraMatrix.at<double>(0, 0) + _cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carr�s, fx = fy -> np
        //mais est g�n�ralement diff�rent de fy donc on prend (pour l'instant) par d�faut la valeur m�diane

        fs.release();

        if(this->setOptions())
            return calibrationCase;
        else
            return cancel;
    }
}

/*
 * membre qui va initialiser les vector de points 3D
 * qui repr�sentent les modeles des objets r�els
 * detect�s
 * (c'est ici aussi qu'on instancie chehra)
 */
bool WebcamDevice::initModels()
{
    //////////////////////////////////////////////////
    ////////// initialisation variables visage ///////
    //////////////////////////////////////////////////

    _launchChehra = true;

    _chessCornersInit = std::vector<std::vector<cv::Point2f>>(2);
    _markerCornersInit = std::vector<std::vector<cv::Point2f>>(2);

    // Lecture des marqueurs
    for (int i = 0; i < 3; i++)
    {
        std::ostringstream oss;
        oss << ":/markers/mark" << i+1;
        QImage marker(QString::fromStdString(oss.str()));
        //        marker.convertToFormat(QImage::Format_RGB16);
        cv::Mat imMQR(marker.height(), marker.width(), CV_8UC4);
        imMQR.data = marker.bits();
        //        cv::Mat imMQR = cv::imread(oss.str());
        cv::cvtColor(imMQR, imMQR, CV_RGBA2GRAY);
        _markersModels.push_back(imMQR);
    }

    if(_optionsDialog->launchChehra())
    {
        _chehra = new Chehra;
        _launchChehra = true;
    }
    else
    {
        _chehra = 0;
        _launchChehra = false;
    }

    // Repere visage
    _pointsVisage3D.push_back(cv::Point3f(-53, -11, -38));    // exterieur oeil gauche sur l'image
    _pointsVisage3D.push_back(cv::Point3f(-17, -11, -38));    // interieur oeil gauche sur l'image
    _pointsVisage3D.push_back(cv::Point3f(0, 0, 0));          // haut du nez, centre des yeux
    _pointsVisage3D.push_back(cv::Point3f(17, -11, -38));     // interieur oeil droit sur l'image
    _pointsVisage3D.push_back(cv::Point3f(53, -11, -38));     // exterieur oeil droit sur l'image
    _pointsVisage3D.push_back(cv::Point3f(0, -40, 7));        // milieu haut du nez
    _pointsVisage3D.push_back(cv::Point3f(0, -40, 14));       // milieu bas du nez
    _pointsVisage3D.push_back(cv::Point3f(0, -40, 22));       // bout du nez
    _pointsVisage3D.push_back(cv::Point3f(-17, -52, 0));      // exterieur narine gauche sur l'image
    _pointsVisage3D.push_back(cv::Point3f(-8, -56, 0));       // milieu narine gauche sur l'image
    _pointsVisage3D.push_back(cv::Point3f(0, -60, 0));        // milieu narines sur l'image
    _pointsVisage3D.push_back(cv::Point3f(8, -56, 0));        // milieu narine droite sur l'image
    _pointsVisage3D.push_back(cv::Point3f(17, -52, 0));       // exterieur narine droite sur l'image

    // Repere chess
    for(int x = -_nbrColChess / 2; x < _nbrColChess / 2 + _nbrColChess % 2; x++)
        for(int y = -_nbrRowChess / 2; y < _nbrRowChess / 2 + _nbrRowChess % 2; y++)
            _pointsChess3D.push_back(cv::Point3f(x * _chessSize, y * _chessSize, 0.0f));

    // Repere marker
    _pointsMarker3D.push_back(cv::Point3f(-_markerSize, 0, 0));              // A
    _pointsMarker3D.push_back(cv::Point3f(_markerSize, 0, 0));               // B
    _pointsMarker3D.push_back(cv::Point3f(_markerSize, _markerSize, 0));      // C
    _pointsMarker3D.push_back(cv::Point3f(-_markerSize/2, _markerSize/2, 0)); // D
    _pointsMarker3D.push_back(cv::Point3f(0, 0, 0));                        // E
    _pointsMarker3D.push_back(cv::Point3f(_markerSize/2, _markerSize/2, 0));  // F

    return _launchChehra;
}


//static
/*
 * permet d'obtenir le nombre de webcam branch�es
 * ne fonctionne que si elles ne sont pas utilis�es
 */
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

/*
 * on change ici l'entree video.
 * soit on charge une video, soit on ouvre une nouvelle webcam
 *
 * /!\ A noter que toutes les entr�es video seront uutilis�es
 * avec la meme matrice intrinseque. il faudrait donc � l'avenir
 * pr�voir une m�thode qui permettra d'associer une matrice �
 * une entr�e video.
 */
void WebcamDevice::switchInput(int input, bool rewind)
{
    _inputIsSwitched = true;

    if(!rewind)
    {
        emit freezeButtons();

        if(input == -1)
        {
            QString videoPath = QFileDialog::getOpenFileName((QWidget*)this->parent(), "Open video", "../rsc/video/", "video file (*.avi *.mp4 *.mkv)");
            if(videoPath == "")
                return;

            else
            {
                _pause = true;

                _path = videoPath;
                _vcap.open(videoPath.toStdString());
                _initFps = _vcap.get(CV_CAP_PROP_FPS);
                _actualFps = _vcap.get(CV_CAP_PROP_FPS);
                if(!_vcap.isOpened())
                {
                    std::cout << "error" << std::endl;
                }
                do
                {
                    _vcap >> _bufferFrame;
                    cv::resize(_bufferFrame, (*_frame), cv::Size(640,480), 0, 0, CV_INTER_AREA);
                    emit updateWebcam();
                    emit playVideo();
                    //std::cout << _bufferFrame.size() << std::endl;
                }while(_bufferFrame.empty()); // on s'assure que la camera est lanc� (#lenteurDuPCDeNico)
            }
        }
        else
        {
            _pause = true;

            _vcap.open(input);
            _initFps = 25;
            _actualFps = 25;
            if(!_vcap.isOpened())
            {
                std::cout << "error" << std::endl;
            }
            do
            {
                _vcap >> _bufferFrame;
                cv::resize(_bufferFrame, (*_frame), cv::Size(640,480), 0, 0, CV_INTER_AREA);
                emit updateWebcam();
                emit playCam();
                //std::cout << _bufferFrame.size() << std::endl;
            }while(_bufferFrame.empty()); // on s'assure que la camera est lanc� (#lenteurDuPCDeNico)
            _pause = false;
        }
    }
    else
    {
        _pause = true;

        _vcap.open(_path.toStdString());
        _initFps = _vcap.get(CV_CAP_PROP_FPS);
        _actualFps = _vcap.get(CV_CAP_PROP_FPS);
        if(!_vcap.isOpened())
        {
            std::cout << "error" << std::endl;
        }
        do
        {
            _vcap >> _bufferFrame;
            cv::resize(_bufferFrame, (*_frame), cv::Size(640,480), 0, 0, CV_INTER_AREA);
            emit updateWebcam();
            emit playVideo();
            //std::cout << _bufferFrame.size() << std::endl;
        }while(_bufferFrame.empty()); // on s'assure que la camera est lanc� (#lenteurDuPCDeNico)

    }

    _inputIsSwitched = false;
}

/*
 * permet de gerer les options de detection
 */
bool WebcamDevice::setOptions()
{
    _optionsDialog = new OptionsDialog(_nbrColChess, _nbrRowChess, _chessSize, _markerSize, _launchChehra, (QWidget*)this->parent());
    if(_optionsDialog->exec() == QDialog::Accepted)
    {
        _nbrColChess = _optionsDialog->getNbrCols();
        _nbrRowChess = _optionsDialog->getNbrRows();
        _chessSize = _optionsDialog->getChessSize();
        _markerSize = _optionsDialog->getMarkerSize();
        _chessDetected = false;
        _markerDetected = false;
        return true;
    }
    else
        return false;
}


// protected
/*
 * run du thread
 * va recup�rer l'entr�e video puis effectuer le calcul de pose
 * selon le mode de detection choisi par l'utilisateur
 */
void WebcamDevice::run()
{
    while(_isRunning)
    {
        if(!_pause)
        {
            _vid = _vcap.read(_bufferFrame);

            if(_vid)
            {
                cv::resize(_bufferFrame, (*_frame), cv::Size(640,480), 0, 0, CV_INTER_AREA);
                cv::resize(_bufferFrame, _testFrame, cv::Size(640,480), 0, 0, CV_INTER_AREA);
            }
            else
            {
                _vcap.release();
                do {
                    _testFrame.copyTo(*_frame);
                    emit backToBeginSig();
                }while(!_inputIsSwitched);
            }

            switch(_detect)
            {
            case noDetection:
                break;
            case faceDetection:
                faceRT();
                break;
            case chessDetection:
                chessRT();
                break;
            case qrDetection:
                markerRT();
                break;
            default:
                break;
            }

            emit updateWebcam();
            msleep(1000 / _actualFps);
        }
    }
}



// private
/*
 * membre appel� lorsque le soft ne trouve pas de matrice intrinseque en ../rsc
 * il va proposer � l'utilisateur d'en cr�er une, d'en utiliser une par defaut,
 * ou d'indiquer le chemin vers un fichier correct
 */
int WebcamDevice::calibrateCam(cv::FileStorage *fs)
{
    bool stop = false;
    int ret = cancel;

    do
    {
        MatrixDialog* matrixDialog = new MatrixDialog();

        if(matrixDialog->exec() == QDialog::Accepted)
        {
            switch(matrixDialog->getChoice())
            {
            case defaultMatrix:
            {
                ret = defaultCase;
                stop = true;
                break;
            }
            case existingMatrix:
            {
                cv::Mat test1, test2;
                QString fsPath = QFileDialog::getOpenFileName((QWidget*)this->parent(), "Select camera intrinsic matrix", "../rsc/", "FileStorage (*.yml)");
                if(fsPath != "")
                    fs->open(fsPath.toStdString(), cv::FileStorage::READ);
                (*fs)["cameraMatrix"] >> test1;
                (*fs)["distCoeffs"] >> test2;
                if(test1.empty() || test2.empty())
                {
                    QMessageBox::warning((QWidget*)this->parent(), tr("Incorrect matrix"), tr("The given matrix is incorrect.\nPlease select another one."));
                    fs->release();
                }
                else
                {
                    stop = true;
                    ret = calibrationCase;

                    cv::FileStorage fs2("../rsc/intrinsicMatrix.yml", cv::FileStorage::WRITE);
                    fs2 << "cameraMatrix" << test1 << "distCoeffs" << test2;
                    fs2.release();
                    fs->open("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);
                }
                break;
            }
            case calibrateMatrix:
            {
                CalibrateDialog* calibrateDialog = new CalibrateDialog(_frame);
                connect(this, SIGNAL(updateWebcam()), calibrateDialog, SLOT(updateWebcam()));

                if(calibrateDialog->exec() == QDialog::Accepted)
                {
                    fs->open("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);
                    stop = true;
                    ret = calibrationCase;
                    break;
                }
                else
                    break;
            }
            default:
                break;
            }
        }
        else
            return ret;
    }while(!stop);

    return ret;
}

/*
 * les membres suivant servent � detecter le repere choisi par l'utilisateur
 */
bool WebcamDevice::detecterVisage(std::vector<cv::Point2f> *pointsVisage)
{
    bool visageFound = false;
    cv::Mat imNB;
    cv::Mat points;

    cv::cvtColor(*_frame, imNB, CV_BGR2GRAY);

    visageFound = _chehra->track(imNB);

    if(visageFound)
    {
        points = _chehra->getTrackedPoints();
        if (points.rows == 98){
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(19, 0), points.at<float>(19 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(22, 0), points.at<float>(22 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(10, 0), points.at<float>(10 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(25, 0), points.at<float>(25 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(28, 0), points.at<float>(28 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(11, 0), points.at<float>(11 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(12, 0), points.at<float>(12 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(13, 0), points.at<float>(13 + 49, 0)));/*
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(14, 0), points.at<float>(14 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(15, 0), points.at<float>(15 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(16, 0), points.at<float>(16 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(17, 0), points.at<float>(17 + 49, 0)));
            (*pointsVisage).push_back(cv::Point2f(points.at<float>(18, 0), points.at<float>(18 + 49, 0)));*/
        }
        else
            return false;
    }

    return visageFound;
}

bool WebcamDevice::detectChess()
{
    bool chessFound = false;
    cv::Mat imGray;

    cv::cvtColor(*_frame, imGray, CV_BGR2GRAY);

    chessFound = cv::findChessboardCorners(imGray, cv::Size(_nbrRowChess, _nbrColChess), _chessCornersInit[1], cv::CALIB_CB_FAST_CHECK);

    if(chessFound)
        cv::swap(_nextFrame, imGray);

    return chessFound;
}

/*
 * membre qui va detecter le marqueur et le cropper afin de permettre par
 * la suite un appareillement.
 * peut etre am�lior� en effectuant une homographie afin de remettre l'image
 * au centre dans le bon sens et permettre une meilleur indentification
 */
bool WebcamDevice::detectMarker()
{
    cv::Mat imCalibGray;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    int A = 0, B = 0, C = 0;
    int mark;
    bool patternFound = false;

    cv::Mat edges((*_frame).size(),CV_MAKETYPE((*_frame).depth(), 1));
    cv::cvtColor((*_frame), imCalibGray, CV_BGR2GRAY);
    Canny(imCalibGray, edges, 100 , 200, 3);

    cv::findContours(edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    mark = 0;

    std::vector<cv::Moments> mu(contours.size());
    std::vector<cv::Point2f> mc(contours.size());

    for( int i = 0; i < contours.size(); i++ )
    {
        mu[i] = moments( contours[i], false );
        mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }

    for( int i = 0; i < contours.size(); i++ )
    {
        int k = i;
        int c = 0;

        while(hierarchy[k][2] != -1)
        {
            k = hierarchy[k][2] ;
            ++c;
        }
        if(hierarchy[k][2] != -1)
            ++c;

        if (c >= 5)
        {
            if (mark == 0)
                A = i;
            else if (mark == 1)
                B = i;      // i.e., A is already found, assign current contour to B
            else if(mark == 2)
                C = i;      // i.e., A and B are already found, assign current contour to C
            ++mark;
        }
    }

    if (A !=0 && B !=0 && C!=0)
    {
        _markerCornersInit[1].push_back(mc[A]);
        //cv::circle((*_frame), cv::Point((*pointQR)[0].x, (*pointQR)[0].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
        _markerCornersInit[1].push_back(mc[B]);
        //cv::circle((*_frame), cv::Point((*pointQR)[1].x, (*pointQR)[1].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
        _markerCornersInit[1].push_back(mc[C]);
        //cv::circle((*_frame), cv::Point((*pointQR)[2].x, (*pointQR)[2].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

        //        std::vector<cv::Point2f> eqLine;
        //        for(int i = 0; i < 3; ++i)
        //        {
        //            cv::Point2f eq()
        //        }


        //* partie Nico qui "fonctionne"
        float minDist;
        float dist2;
        cv::Point2f minPoint;

        minDist = sqrt(pow(_markerCornersInit[1][0].x, 2) + pow(_markerCornersInit[1][0].y, 2));
        minPoint = _markerCornersInit[1][0];

        for (int i = 1; i < 3; i++)
        {
            dist2 = sqrt(pow(_markerCornersInit[1][i].x, 2) + pow(_markerCornersInit[1][i].y, 2));

            if (dist2 <= minDist)
            {
                minDist = dist2;
                minPoint = _markerCornersInit[1][i];
            }
        }

        float distCrop;
        std::vector<float> tabDistCrop;

        for (int i = 0; i < 3; i++)
        {
            distCrop = sqrt(pow((_markerCornersInit[1][i].x - minPoint.x) ,2) + pow((_markerCornersInit[1][i].y - minPoint.y),2));
            if (distCrop != 0)
                tabDistCrop.push_back(distCrop);
        }

        _frame->copyTo(_frameCropped);
        cv::Rect ROI(minPoint.x, minPoint.y, tabDistCrop[1], tabDistCrop[0]);

        if(ROI.x >= 0 && ROI.y >= 0 && (ROI.width + ROI.x) < _frameCropped.cols && (ROI.height + ROI.y) < _frameCropped.rows)
        {
            cv::Mat croppedRef(_frameCropped, ROI);
            cv::cvtColor(croppedRef, _frameCropped, CV_BGR2GRAY);

            cv::Point2f D(0.0f,0.0f);
            cv::Point2f E(0.0f,0.0f);
            cv::Point2f F(0.0f,0.0f);

            D.x = (mc[A].x + mc[B].x)/2;
            E.x = (mc[B].x + mc[C].x)/2;
            F.x = (mc[C].x + mc[A].x)/2;

            D.y = (mc[A].y + mc[B].y)/2;
            E.y = (mc[B].y + mc[C].y)/2;
            F.y = (mc[C].y + mc[A].y)/2;

            _markerCornersInit[1].push_back(D);
            //cv::circle((*_frame), cv::Point((*pointQR)[3].x, (*pointQR)[3].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
            _markerCornersInit[1].push_back(E);
            //cv::circle((*_frame), cv::Point((*pointQR)[4].x, (*pointQR)[4].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
            _markerCornersInit[1].push_back(F);
            //cv::circle((*_frame), cv::Point((*pointQR)[5].x, (*pointQR)[5].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

            patternFound = true;
        }
        //*/
    }

    if(patternFound)
        cv::swap(_nextFrame, imCalibGray);

    return patternFound;
}

/*
 * tracking � l'aide de flot optique
 */
void WebcamDevice::trackingChess()
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

    cv::Rodrigues(rvecs, _rotVecs);
}

void WebcamDevice::trackingMarker()
{
    cv::Mat rvecs;
    cv::Mat imCalib;

    cv::swap(imCalib, _nextFrame);
    _markerCornersInit[0] = _markerCornersInit[1];
    _markerCornersInit[1].clear();

    cv::cornerSubPix(imCalib, _markerCornersInit[0], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    cv::cvtColor(*_frame, _nextFrame, CV_BGR2GRAY);

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(imCalib, _nextFrame, _markerCornersInit[0], _markerCornersInit[1], status, err, cv::Size(31, 31), 3, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1), 0, 0.0001);

    cv::solvePnP(_pointsMarker3D, _markerCornersInit[0], _cameraMatrix, _distCoeffs, rvecs, _tvecs);

    cv::Rodrigues(rvecs, _rotVecs);
}

/*
 * membre qui fait ici l'appareillement SIFT entre l'image du marqueur detect� et
 * les images pr�sentes dans la base de donn�es.
 * peut etre am�lior� en utilisant une m�thode d'identification moins "lourde"
 * (fft par exemple).
 */
void WebcamDevice::dbCorrelation()
{
    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints1, keypoints2, keypoints3;
    detector.detect(_frameCropped, keypoints1);
    detector.detect(_markersModels[0], keypoints2);
    detector.detect(_markersModels[1], keypoints3);

    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create("SIFT");
    cv::Mat descriptors1, descriptors2, descriptors3;
    descriptor->compute(_frameCropped, keypoints1, descriptors1);
    descriptor->compute(_markersModels[0], keypoints2, descriptors2);
    descriptor->compute(_markersModels[1], keypoints3, descriptors3);

    if(!descriptors1.empty())
    {
        cv::FlannBasedMatcher matcher;
        std::vector< cv::DMatch > matches;
        std::vector< cv::DMatch > good_matches1;
        std::vector< cv::DMatch > good_matches2;
        double max_dist = 0; double min_dist = 100;

        matcher.match(descriptors2, descriptors1, matches);

        for(int i = 0; i < descriptors2.rows; i++)
        {
            double dist = matches[i].distance;
            if(dist < min_dist) min_dist = dist;
            if(dist > max_dist) max_dist = dist;

            if(matches[i].distance <= 2 * min_dist)
                good_matches1.push_back(matches[i]);
        }

        /*for( int i = 0; i < descriptors2.rows; i++ )
            if( matches[i].distance <= 2*min_dist )
                good_matches1.push_back( matches[i]);*/
        matcher.match( descriptors3, descriptors1, matches );

        for( int i = 0; i < descriptors3.rows; i++ )
        {
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;

            if( matches[i].distance <= 2*min_dist )
                good_matches2.push_back( matches[i]);
        }

        /* for( int i = 0; i < descriptors3.rows; i++ )
            if( matches[i].distance <= 2*min_dist )
                good_matches2.push_back( matches[i]);*/

        if(good_matches1.size() > good_matches2.size())
            std::cout << "cerveau" << std::endl;
        else
            std::cout << "os" << std::endl;
    }
}

/*
 * les membre suivant vont effectuer le calcul de pose avec solvePNP
 * ils enverront ensuite le r�ultat au travers d'un signal reli� �
 * updateSceneRT du mainWindow
 */
void WebcamDevice::faceRT()
{
    cv::Mat rvecs;
    std::vector<cv::Point2f> pointsVisage2D;
    std::vector<cv::Point2f> moyPointsVisage2D;

    bool faceDetected = detecterVisage(&pointsVisage2D);

    if(faceDetected)
    {
        _nbrLoopSinceLastDetection = 0;
        _resetLabel = 0;
        _images.push_back(pointsVisage2D);
    }
    else
    {
        _resetLabel++;
        _nbrLoopSinceLastDetection++;
    }

    if((_images.size() > NBRSAVEDIMAGES || _nbrLoopSinceLastDetection > NBRSAVEDIMAGES) && !_images.empty())
        _images.erase(_images.begin());

    if(!_images.empty() && faceDetected)
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

        cv::Rodrigues(rvecs, _rotVecs);

        emit updateScene(_rotVecs, _tvecs);
        emit updateDetect(true);
    }

    if(_resetLabel > 5)
        emit updateDetect(false);
}

void WebcamDevice::chessRT()
{
    std::vector<cv::Point2f> imagePoints;
    std::vector<double> errors;
    double meanErrors;

    if(_chessDetected)
    {
        this->trackingChess();

        cv::projectPoints(_pointsChess3D, _rotVecs, _tvecs, _cameraMatrix, _distCoeffs, imagePoints);

        // Draw chess points
        //        for(int m = 0; m < _chessCornersInit[0].size(); m++)
        //            cv::circle(*_frame, cv::Point(_chessCornersInit[0][m].x, _chessCornersInit[0][m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

        emit updateScene(_rotVecs, _tvecs);
        emit updateDetect(true);

        double mean = 0;
        for(int j = 0; j < _nbrColChess * _nbrRowChess; j++)
        {
            double d = sqrt(pow(_chessCornersInit[0][j].y - imagePoints[j].y, 2) + pow(_chessCornersInit[0][j].x - imagePoints[j].x, 2));
            errors.push_back(d);
            mean += d;
        }

        meanErrors = mean / (_nbrColChess * _nbrRowChess);

        if(meanErrors > 2)
        {
            emit updateDetect(false);
            _reset = true;
        }
    }

    if (!_chessDetected || _reset == true)
    {
        _chessCornersInit[0].clear();
        _chessCornersInit[1].clear();
        _nextFrame.release();
        _reset = false;
        _chessDetected = false;

        _chessDetected = detectChess();
    }
}

void WebcamDevice::markerRT()
{
    std::vector<cv::Point2f> imagePoints;
    std::vector<double> errors;
    double meanErrors;


    if(_markerDetected)
    {
        this->trackingMarker();

        cv::projectPoints(_pointsMarker3D, _rotVecs, _tvecs, _cameraMatrix, _distCoeffs, imagePoints);

        // Draw chess points
        for(int m = 0; m < _markerCornersInit[0].size(); m++)
            cv::circle(*_frame, cv::Point(_markerCornersInit[0][m].x, _markerCornersInit[0][m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

        //        if(_runCount == _actualFps)
        //        {
        //            this->dbCorrelation();
        //            _runCount = 0;
        //        }

        emit updateScene(_rotVecs, _tvecs);
        emit updateDetect(true);

        double mean = 0;
        for(int j = 0; j < _markerCornersInit[0].size(); j++)
        {
            double d = sqrt(pow(_markerCornersInit[0][j].y - imagePoints[j].y, 2) + pow(_markerCornersInit[0][j].x - imagePoints[j].x, 2));
            errors.push_back(d);
            mean += d;
        }

        meanErrors = mean / (_markerCornersInit[0].size());

        if(meanErrors > 10)
        {
            //            emit updateDetect(false);
            _reset = true;
        }

        ++_runCount;
    }

    if(!_markerDetected) // TODO a retirer apres demo
        emit updateDetect(false);

    if (!_markerDetected || _reset == true)
    {
        _markerCornersInit[0].clear();
        _markerCornersInit[1].clear();
        _nextFrame.release();
        _frameCropped.release();
        _reset = false;
        _markerDetected = false;

        _markerDetected = detectMarker();
    }
}
