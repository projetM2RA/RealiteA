#include "webcamdevice.h"

// public

WebcamDevice::WebcamDevice(QObject *parent) :
    QThread(parent)
{
    //////////////////////////////////////////////////
    ////////// initialisation variables visage ///////
    //////////////////////////////////////////////////

    std::cout << "initialisation de Chehra..." << std::endl;
    m_chehra = new Chehra;
    std::cout << "done" << std::endl;

    m_pointsVisage3D.push_back(cv::Point3f(-110, 0, -336)); // exterieur narine gauche sur l'image
    m_pointsVisage3D.push_back(cv::Point3f(110, 0, -336)); // exterieur narine droite sur l'image
    m_pointsVisage3D.push_back(cv::Point3f(0, -142, -258)); // bout du nez
    m_pointsVisage3D.push_back(cv::Point3f(-338, 243, -70)); // exterieur oeil gauche sur l'image
    m_pointsVisage3D.push_back(cv::Point3f(0, 0, 0)); // haut du nez, centre des yeux
    m_pointsVisage3D.push_back(cv::Point3f(338, 243, -70)); // exterieur oeil droit sur l'image

    //////////////////////////////////////////////////
    ////////// initialisation OpenCV /////////////////
    //////////////////////////////////////////////////
    cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

    while(!fs.isOpened())
        calibrateCam(&fs);

    fs["cameraMatrix"] >> m_cameraMatrix;
    fs["distCoeffs"] >> m_distCoeffs;

    m_focalePlane = (m_cameraMatrix.at<double>(0, 0) + m_cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carrés, fx = fy -> np
    //mais est généralement différent de fy donc on prend (pour l'instant) par défaut la valeur médiane

    fs.release();

    m_vcap = cv::VideoCapture(0);
    /*
    if(!vcap.isOpened())
    {
        std::cout << "FAIL!" << std::endl;
        return;
    }
    */
    //vcap.set(CV_CAP_PROP_FPS, 30);
    m_frame = new cv::Mat(cv::Mat::zeros(m_vcap.get(CV_CAP_PROP_FRAME_HEIGHT), m_vcap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3));

    do
    {
        m_vcap >> *m_frame;
    }while(m_frame->empty());

    //////////////////////////////////////////////////

    m_isRunning = true;
    m_detect = noDetection;
    //m_corrector = (m_focalePlane / 2) / (m_cameraMatrix.at<double>(1, 2) - m_vcap.get(CV_CAP_PROP_FRAME_HEIGHT) / 2);

    this->start();
}

WebcamDevice::~WebcamDevice()
{
    delete m_frame;
}






// protected

void WebcamDevice::run()
{
    while(m_isRunning)
    {
        switch(m_detect)
        {
        case noDetection:
            m_vcap >> *m_frame;

            break;
        case faceDetection:
            m_vcap >> *m_frame;

            faceRT();

            break;
        case chessDetection:
            m_vcap >> *m_frame;

            chessRT();

            break;
        case qrDetection:
            m_vcap >> *m_frame;

            qrRT();

            break;
        default:
            m_vcap >> *m_frame;

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
        QMessageBox::StandardButton button = QMessageBox::critical(0, tr("matrice camera manquante"),
                             tr("impossible de trouver la matrice camera.\nDisposez-vous deja d'une matrice de calibration ?."),
                              QMessageBox::Yes | QMessageBox::No);

        switch(button)
        {
        case QMessageBox::No:

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
            stop = true;
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

    cv::cvtColor(*m_frame, imNB, CV_BGR2GRAY);

    visageFound = (*m_chehra).track(imNB);

    if(visageFound)
    {
        points = (*m_chehra).getTrackedPoints();
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
        m_nbrLoopSinceLastDetection = 0;
        m_images.push_back(pointsVisage2D);
    }
    else
        m_nbrLoopSinceLastDetection++;

    if((m_images.size() > NBRSAVEDIMAGES || m_nbrLoopSinceLastDetection > NBRSAVEDIMAGES) && !m_images.empty())
        m_images.erase(m_images.begin());

    if(!m_images.empty())
    {
        for(int i = 0; i < NBRFACEPOINTSDETECTED; i++)
        {
            cv::Point2f coordonee(0.0f, 0.0f);
            for(int j = 0; j < m_images.size(); j++)
            {
                coordonee.x += m_images[j][i].x;
                coordonee.y += m_images[j][i].y;
            }
            coordonee.x /= m_images.size();
            coordonee.y /= m_images.size();

            moyPointsVisage2D.push_back(coordonee);
        }

        cv::solvePnP(m_pointsVisage3D, moyPointsVisage2D, m_cameraMatrix, m_distCoeffs, rvecs, m_tvecs);

        m_rotVecs = cv::Mat(3, 3, CV_64F);
        cv::Rodrigues(rvecs, m_rotVecs);

        emit updateScene(m_rotVecs, m_tvecs);
    }
}

void WebcamDevice::chessRT()
{

}

void WebcamDevice::qrRT()
{

}
