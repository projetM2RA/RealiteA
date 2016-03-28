#include "CalibrateDialog.h"

CalibrateDialog::CalibrateDialog(cv::Mat* frame, QWidget *parent) :
    QDialog(parent)
{
    this->setWindowTitle("Camera calibration");
    this->setWindowIcon(QIcon(":/icons/icon"));

    _rows = frame->rows;
    _cols = frame->cols;
    _type = frame->type();

    _imageIndex = 1;
    _savedFrame = new cv::Mat(cv::Mat::zeros(_rows, _cols, _type));

    _save = new QPushButton(tr("Save screenshot"));
    _sup = new QPushButton(tr("Delete last screenshot"));
    _finish = new QPushButton(tr("End Calibration"));

    _finish->setEnabled(false);
    _sup->setEnabled(false);

    QLabel* nbrRowsLabel = new QLabel(tr("Number of rows : "));
    QLabel* nbrColsLabel = new QLabel(tr("Number of columns : "));
    QLabel* chessSizeLabel = new QLabel(tr("Size of one square side : "));

    _imageIndexLabel = new QLabel("Image : " + QString::number(_imageIndex-1) + " / 15");
    _infoMessage = new QLabel();
    _infoMessage->setObjectName("infoLabel"); // CSS

    _webcamFluxScene = new WebcamGraphicsScene(frame);
    _imageSavedScene = new WebcamGraphicsScene(_savedFrame);

    WebcamGraphicsView* webcamFlux = new WebcamGraphicsView();
    webcamFlux->setScene(_webcamFluxScene);

    WebcamGraphicsView* imageSaved = new WebcamGraphicsView();
    imageSaved->setScene(_imageSavedScene);

    _nbrRows = new QSpinBox;
    _nbrRows->setValue(6);
    _nbrCols = new QSpinBox;
    _nbrCols->setValue(9);
    _chessSize = new QDoubleSpinBox;
    _chessSize->setValue(26);
    _chessSize->setSuffix(" mm");

    QHBoxLayout* optionsLayout = new QHBoxLayout;
    optionsLayout->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    optionsLayout->addWidget(nbrRowsLabel);
    optionsLayout->addWidget(_nbrRows);
    optionsLayout->addWidget(nbrColsLabel);
    optionsLayout->addWidget(_nbrCols);
    optionsLayout->addWidget(chessSizeLabel);
    optionsLayout->addWidget(_chessSize);
    optionsLayout->addWidget(_infoMessage, 1, Qt::AlignRight);
    optionsLayout->addWidget(_imageIndexLabel, 1, Qt::AlignRight);

    QHBoxLayout* imageLayout = new QHBoxLayout;
    imageLayout->addWidget(webcamFlux);
    imageLayout->addWidget(imageSaved);

    QHBoxLayout* buttonsLayout = new QHBoxLayout;
    buttonsLayout->setAlignment(Qt::AlignBottom);
    buttonsLayout->addWidget(_save);
    buttonsLayout->addWidget(_sup);
    buttonsLayout->addWidget(_finish);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(optionsLayout);
    mainLayout->addLayout(imageLayout);
    mainLayout->addLayout(buttonsLayout);

    this->setLayout(mainLayout);
    this->setWindowFlags(this->windowFlags() & (~Qt::WindowContextHelpButtonHint));
    this->setWindowFlags(this->windowFlags() & (Qt::WindowMaximizeButtonHint));

    connect(_save, SIGNAL(clicked()), this, SLOT(saveSnapShot()));
    connect(_sup, SIGNAL(clicked()), this, SLOT(delSnapShot()));
    connect(_finish, SIGNAL(clicked()), this, SLOT(endSnapShot()));

    for(int i = 1; i <= 30; ++i)
        if(QFile::exists(QString("../rsc/mires/mire") + QString::number(i) + QString(".png")))
            QFile::remove(QString("../rsc/mires/mire") + QString::number(i) + QString(".png"));
}

bool CalibrateDialog::detectChess(cv::Mat chessFrame, cv::Mat* chessSaved)
{
    cv::Mat imgGray;
    std::vector<cv::Point2f> chessCorners;
    bool chessFound = false;
    chessCorners.clear();
    (*chessSaved) = chessFrame.clone();

    cv::cvtColor(chessFrame, imgGray, CV_BGR2GRAY);
    chessFound = cv::findChessboardCorners(imgGray, cv::Size(_nbrRows->value(), _nbrCols->value()), chessCorners, CV_CALIB_CB_FAST_CHECK);

    if(chessFound)
    {
        for(int m = 0; m < chessCorners.size(); m++)
            cv::circle((*chessSaved), cv::Point(chessCorners[m].x, chessCorners[m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
    }

    return chessFound;
}

void CalibrateDialog::saveSnapShot()
{
    if(!QDir("../rsc/mires/").exists())
        QDir().mkdir("../rsc/mires/");
    std::ostringstream oss;
    oss << "../rsc/mires/mire" << _imageIndex << ".png";

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    bool chessFound = detectChess(_webcamFluxScene->getFrame(), _savedFrame);

    if(chessFound == false)
        _infoMessage->setText("No chessboard detection!");
    else
    {
        if(_imageIndex <= 30)
        {
            cv::imwrite(oss.str(), _webcamFluxScene->getFrame(), compression_params);
            _infoMessage->setText(QString("Image ") + QString::number(_imageIndex) + QString(" saved."));
            _imageSavedScene->update();
        }

        _sup->setEnabled(true);

        if(_imageIndex == 1)
            _imageIndexLabel->setText("Image : " + QString::number(_imageIndex) + " / 15");
        else if(_imageIndex > 1 && _imageIndex <=15)
        {
            _imageIndexLabel->setText("Images : " + QString::number(_imageIndex) + " / 15");
            if(_imageIndex == 15)
                _finish->setEnabled(true);
        }
        else if(_imageIndex > 15 && _imageIndex <= 30)
        {
            if(_imageIndex == 30)
                _save->setEnabled(false);
            _imageIndexLabel->setText("Images : " + QString::number(_imageIndex) + " / " + QString::number(_imageIndex));
        }
        else
        {
            _save->setEnabled(false);
            _infoMessage->setText("Too many screenshots.");
        }

        ++_imageIndex;
        chessFound = false;
    }
}

void CalibrateDialog::delSnapShot()
{
    --_imageIndex;
    std::ostringstream oss;
    oss << "../rsc/mires/mire" << _imageIndex << ".png";

    if(_imageIndex <= 30)
    {
        QFile::remove(QString::fromStdString(oss.str()));
        _infoMessage->setText(QString("Image ") + QString::number(_imageIndex) + QString(" deleted."));
        std::ostringstream oss;
        oss << "../rsc/mires/mire" << _imageIndex-1 << ".png";

        if(_imageIndex == 1)
            (*_savedFrame) = cv::Scalar(0, 0, 0);
        else
            (*_savedFrame) = cv::imread(oss.str());
        _imageSavedScene->update();
    }

    if(_imageIndex == 1)
    {
        _imageIndexLabel->setText("Image : " + QString::number(_imageIndex - 1) + " / 15");
        _sup->setEnabled(false);        
    }
    else if(_imageIndex > 1 && _imageIndex <= 15)
    {
        _imageIndexLabel->setText("Images : " + QString::number(_imageIndex - 1) + " / 15");
        if(_imageIndex == 15)
            _finish->setEnabled(true);
    }
    else if(_imageIndex > 15 && _imageIndex <= 30)
    {
        if(_imageIndex == 30)
            _save->setEnabled(false);
        _imageIndexLabel->setText("Images : " + QString::number(_imageIndex - 1) + " / " + QString::number(_imageIndex));
    }
    else
    {
        _save->setEnabled(false);
        _infoMessage->setText("Too many screenshots.");
    }
}

void CalibrateDialog::endSnapShot()
{
    _calibration = new QProgressDialog("Calibration...", "Abort Calibration", 0, 100, this);
    _calibration->setWindowModality(Qt::WindowModal);
    _calibration->setCancelButton(0);
    _calibration->setMinimumDuration(0);

    std::vector<cv::Mat> imCalib;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    cameraMatrix.at<double>(0, 0) = 1.0;

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<std::vector<cv::Point2f>> chessCornersInit;
    std::vector<std::vector<cv::Point3f>> chessCorners3D;

    rvecs.clear();
    tvecs.clear();
    chessCorners3D.clear();
    chessCornersInit.clear();

    for(int i = 1; i < _imageIndex; i++)
    {
        std::vector<cv::Point2f> initCorners(_nbrRows->value() * _nbrCols->value(), cv::Point2f(0, 0));
        chessCornersInit.push_back(initCorners);
    }

    _calibration->setValue(33);

    for(int i = 0; i < _imageIndex - 1; i++)
    {
        std::ostringstream oss;
        oss << "../rsc/mires/mire" << i + 1 << ".png";
        imCalib.push_back(cv::imread(oss.str()));

        cv::cvtColor(imCalib[i], imCalib[i], CV_BGR2GRAY);

        bool patternfound = cv::findChessboardCorners(imCalib[i], cv::Size(_nbrRows->value(), _nbrCols->value()), chessCornersInit[i]);

        if(patternfound)
            cv::cornerSubPix(imCalib[i], chessCornersInit[i], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }

    _calibration->setValue(66);

    for(int i = 1; i < _imageIndex; i++)
    {
        std::vector<cv::Point3f> initCorners3D;

        for(int j = 0; j < _nbrCols->value(); j++)
        {
            for(int k = 0; k < _nbrRows->value(); k++)
            {
                 _calibration->setValue(i);
                cv::Point3f corner(j * _chessSize->value(), k * _chessSize->value(), 0.0f);
                initCorners3D.push_back(corner);
            }
        }
        chessCorners3D.push_back(initCorners3D);
    }

    _calibration->setValue(90);

    cv::calibrateCamera(chessCorners3D, chessCornersInit, cv::Size(imCalib[0].size()), cameraMatrix, distCoeffs, rvecs, tvecs);

    _calibration->setValue(99);

    std::string filename = "../rsc/intrinsicMatrix.yml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);    

    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;

    _calibration->setValue(100);

    QMessageBox::information(this, "Calibration", "Calibration done with success.\nYour intrinsinc matrix as been successfuly created.");
    this->accept();

    fs.release();

}
