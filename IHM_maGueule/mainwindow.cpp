#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    this->setWindowTitle(tr("Augmented Reality Of Neuroskeleton"));
    this->setWindowIcon(QIcon(":/icons/icon"));
    _objectID = 0;
    _nbrCam = WebcamDevice::webcamCount(); // a mettre au debut sinon conflit avec les webcams lancees
    _webcamDevice = new WebcamDevice(this);
    _fullScreen = false;

    setFirstWindow();


    this->resize(QDesktopWidget().availableGeometry(this).size() * 0.65);
}

MainWindow::~MainWindow()
{
    for(int i = 0; i < NBR_DETECT; i++)
    {
        delete _detectActions[i];
    }
    delete _detectActions;

    delete _addObjectAction;
    delete _fullScreenAction;

    delete _mainView;

    delete _objectChoiceComboBox;
    delete _deleteObjectButton;
    delete _isPrintedBox;

    for(int i = 0; i < NBR_CHARACTERISTICS; i++)
    {
        delete _objectCharacteristicsSpinSliders[i];
    }

    delete _objectCharacteristicsSpinSliders;

    delete _sideView;

    //////////////////////////////////////////////////

    _webcamDevice->stop();
    delete _webcamDevice;
}



// private slots
void MainWindow::start()
{
    QSplashScreen splash(QPixmap(":/icons/splash"));

    _webcamDevice->initMatrix();

    splash.show();

    this->initObjectsList();

    this->setMainWindow();
    this->setCursor(QCursor(Qt::WaitCursor));
    _webcamDevice->initModels();
    this->setCursor(QCursor(Qt::ArrowCursor));

    this->createFullScreenWidget();

    this->connectAll();
    this->setShortcuts();

    splash.close();
}

void MainWindow::calibrateCamera()
{
    CalibrateDialog calibrateDialog(_webcamDevice->getWebcam());
    connect(_webcamDevice, SIGNAL(updateWebcam()), &calibrateDialog, SLOT(updateWebcam()));
    calibrateDialog.exec();
}

void MainWindow::addObject()
{
    AddObjectDialog objectDialog;

    if(objectDialog.exec() == QDialog::Rejected)
        return;

    QString objectPath = objectDialog.getObjectPath();
    QString objectName = objectDialog.getObjectName();

    if(objectPath == "" || objectName == "")
    {
        QMessageBox::warning(this, tr("Error loading object"), tr("The object's path and/or name have not been specified."));
        return;
    }

    _objectsList.push_back(new Our3DObject(objectPath));
    _objectsList[0]->addChild(_objectsList[_objectsList.size() - 1]);
    _objectsList2.push_back(new Our3DObject(objectPath));
    _objectsList2[0]->addChild(_objectsList2[_objectsList2.size() - 1]);

    int nbrObjects = _objectChoiceComboBox->itemText(0).mid(14).toInt();
    _objectChoiceComboBox->setItemText(0, "All objects : " + QString::number(nbrObjects + 1));
    _objectChoiceComboBox->addItem(objectName);
}

void MainWindow::updateObjectCharacteristics(int objectID)
{
    if(objectID == _objectID)
        return;
    // _objectList[0] <=> globalMat
    // _objectList[1] <=> hud

    Our3DObject* object = _objectsList[_objectID];
    Our3DObject* object2 = _objectsList2[_objectID];

    if(_objectID != 1)
    {
        disconnect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
        disconnect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
        disconnect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
        disconnect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
        disconnect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
        disconnect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
        disconnect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

        //////////////////////////////////////////////////

        disconnect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object2, SLOT(setSizeX(int)));
        disconnect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object2, SLOT(setSizeY(int)));
        disconnect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object2, SLOT(setSizeZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object2, SLOT(setRotX(int)));
        disconnect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object2, SLOT(setRotY(int)));
        disconnect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object2, SLOT(setRotZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object2, SLOT(setTransX(int)));
        disconnect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object2, SLOT(setTransY(int)));
        disconnect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object2, SLOT(setTransZ(int)));
    }
    else
    {
        _objectCharacteristicsSpinSliders[sizeX]->setEnabled(true);
        _objectCharacteristicsSpinSliders[sizeY]->setEnabled(true);
        _objectCharacteristicsSpinSliders[sizeZ]->setEnabled(true);

        _objectCharacteristicsSpinSliders[rotX]->setEnabled(true);
        _objectCharacteristicsSpinSliders[rotY]->setEnabled(true);
        _objectCharacteristicsSpinSliders[rotZ]->setEnabled(true);

        _objectCharacteristicsSpinSliders[transX]->setEnabled(true);
        _objectCharacteristicsSpinSliders[transY]->setEnabled(true);
        _objectCharacteristicsSpinSliders[transZ]->setEnabled(true);

        _isPrintedBox->setEnabled(true);
    }

    disconnect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object2, SLOT(setAlpha(int)));



    _objectID = objectID;

    object = _objectsList[_objectID];
    object2 = _objectsList2[_objectID];


    _isPrintedBox->setChecked(object->getNodeMask());
    _isPrintedBox2->setChecked(object2->getNodeMask());

    if(_objectID == 1)
    {
        _objectCharacteristicsSpinSliders[sizeX]->setValue(0);
        _objectCharacteristicsSpinSliders[sizeY]->setValue(0);
        _objectCharacteristicsSpinSliders[sizeZ]->setValue(0);

        _objectCharacteristicsSpinSliders[rotX]->setValue(0);
        _objectCharacteristicsSpinSliders[rotY]->setValue(0);
        _objectCharacteristicsSpinSliders[rotZ]->setValue(0);

        _objectCharacteristicsSpinSliders[transX]->setValue(0);
        _objectCharacteristicsSpinSliders[transY]->setValue(0);
        _objectCharacteristicsSpinSliders[transZ]->setValue(0);

        _objectCharacteristicsSpinSliders[sizeX]->setEnabled(false);
        _objectCharacteristicsSpinSliders[sizeY]->setEnabled(false);
        _objectCharacteristicsSpinSliders[sizeZ]->setEnabled(false);

        _objectCharacteristicsSpinSliders[rotX]->setEnabled(false);
        _objectCharacteristicsSpinSliders[rotY]->setEnabled(false);
        _objectCharacteristicsSpinSliders[rotZ]->setEnabled(false);

        _objectCharacteristicsSpinSliders[transX]->setEnabled(false);
        _objectCharacteristicsSpinSliders[transY]->setEnabled(false);
        _objectCharacteristicsSpinSliders[transZ]->setEnabled(false);

        _isPrintedBox->setEnabled(false);

        _objectCharacteristicsSpinSliders[alpha]->setValue(object2->getAlpha());
        connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object2, SLOT(setAlpha(int)));
    }
    else
    {
        _objectCharacteristicsSpinSliders[sizeX]->setValue(object->getSizeX());
        _objectCharacteristicsSpinSliders[sizeY]->setValue(object->getSizeY());
        _objectCharacteristicsSpinSliders[sizeZ]->setValue(object->getSizeZ());

        _objectCharacteristicsSpinSliders[rotX]->setValue(object->getRotX());
        _objectCharacteristicsSpinSliders[rotY]->setValue(object->getRotY());
        _objectCharacteristicsSpinSliders[rotZ]->setValue(object->getRotZ());

        _objectCharacteristicsSpinSliders[transX]->setValue(object->getTransX());
        _objectCharacteristicsSpinSliders[transY]->setValue(object->getTransY());
        _objectCharacteristicsSpinSliders[transZ]->setValue(object->getTransZ());

        connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
        connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
        connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

        connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
        connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
        connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

        connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
        connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
        connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

        _objectCharacteristicsSpinSliders[alpha]->setValue(object->getAlpha());
        connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

        //////////////////////////////////////////////////

        connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object2, SLOT(setSizeX(int)));
        connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object2, SLOT(setSizeY(int)));
        connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object2, SLOT(setSizeZ(int)));

        connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object2, SLOT(setRotX(int)));
        connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object2, SLOT(setRotY(int)));
        connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object2, SLOT(setRotZ(int)));

        connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object2, SLOT(setTransX(int)));
        connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object2, SLOT(setTransY(int)));
        connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object2, SLOT(setTransZ(int)));

        connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object2, SLOT(setAlpha(int)));
    }
}

void MainWindow::updateDetectMode()
{
    for(int i = 0; i < NBR_DETECT; i++)
    {
        if(_detectActions[i]->isChecked())
        {
            _webcamDevice->switchMode(i);
            return;
        }
    }
}

void MainWindow::switchInput()
{
    for(int i = 0; i < _nbrCam; i++)
    {
        if(_webcamActions[i]->isChecked())
        {
            _webcamDevice->switchInput(i);
            _pause->setEnabled(false);
            _play->setEnabled(false);
            _fast->setEnabled(false);
            _slow->setEnabled(false);
            return;
        }
    }
    _webcamDevice->switchInput(-1);
    _play->setEnabled(true);
}

void MainWindow::displayObjectInScene(bool display)
{
    if(_objectID == 0)
        displayObjects(!display);
    else
    {
        if(display)
            _objectsList[_objectID]->setNodeMask(display);
        else
            _objectsList[_objectID]->setNodeMask(display);
    }
}

void MainWindow::displayObjectInSideView(bool display)
{
    if(_objectID == 0)
        displayObjectsInSideView(!display);
    else
    {
        if(display)
            _objectsList2[_objectID]->setNodeMask(display);
        else
            _objectsList2[_objectID]->setNodeMask(display);
    }
}

void MainWindow::displayFullScreen()
{
    if(!_fullScreen)
        _fullScreenWidget->showFullScreen();
    else
    {
        _fullScreenWidget->showNormal();
        _fullScreenWidget->move(-5000, -5000); //(necessaire pour cacher correctement le full screen #truanderie)
        _fullScreenWidget->hide();
    }

    _fullScreenView->update();

    _fullScreen = !_fullScreen;
}

void MainWindow::updateSceneRT(cv::Mat rotVec, cv::Mat tvecs)
{
    double t3 = tvecs.at<double>(2, 0);
    double t2 = tvecs.at<double>(1, 0) + t3 / _corrector; // #truanderie
    double t1 = tvecs.at<double>(0, 0);

    double r11 = rotVec.at<double>(0, 0);
    double r12 = rotVec.at<double>(0, 1);
    double r13 = rotVec.at<double>(0, 2);
    double r21 = rotVec.at<double>(1, 0);
    double r22 = rotVec.at<double>(1, 1);
    double r23 = rotVec.at<double>(1, 2);
    double r31 = rotVec.at<double>(2, 0);
    double r32 = rotVec.at<double>(2, 1);
    double r33 = rotVec.at<double>(2, 2);


    osg::Matrixd matrixR; // rotation (transposee de rotVec)
    matrixR.set(
                r11,	r21,	r31,	0,
                r12,	r22,	r32,	0,
                r13,	r23,	r33,	0,
                0,		0,		0,		1);

    double rotX = atan2(r32, r33);
    double rotY = atan2(r21, r11);
    double rotZ = atan2(-r31, sqrt((r32 * r32) + (r33 * r33)));

    std::cout << "rot x : " << rotX << std::endl;
    std::cout << "rot y : " << rotY << std::endl;
    std::cout << "rot z : " << rotZ << std::endl;
    std::cout << std::endl << std::endl;
    std::cout << "t x : " << t1 << std::endl;
    std::cout << "t y : " << t2 << std::endl;
    std::cout << "t z : " << t3 << std::endl;
    std::cout << std::endl << std::endl;

    osg::Matrixd matrixR2; // rotation corrigee
    matrixR2.makeRotate(rotX, osg::Vec3d(1.0, 0.0, 0.0), rotZ, osg::Vec3d(0.0, 1.0, 0.0), rotY, osg::Vec3d(0.0, 0.0, 1.0));

    //    std::cout << "rotations : " << std::endl
    //              << "x : " << rotX << std::endl
    //              << "y : " << rotY << std::endl
    //              << "z : " << rotZ << std::endl << std::endl;
    //    std::cout << "translations : " << std::endl
    //              << "x : " << t1 << std::endl
    //              << "y : " << t2 << std::endl
    //              << "z : " << t3 << std::endl << std::endl << std::endl;


    osg::Matrixd matrixT; // translation
    matrixT.makeTranslate(t1, t2, t3);

    osg::Matrixd matrix90; // rotation de repere entre opencv et osg
    matrix90.makeRotate(osg::Quat(osg::DegreesToRadians(-90.0f), osg::Vec3d(1.0, 0.0, 0.0)));

    _mainMat->setMatrix(matrixR * matrixT * matrix90);
    _mainMat2->setMatrix(matrixR * matrixT * matrix90);
}



// private
void MainWindow::setFirstWindow()
{
    //////////////////////////////////////////////////
    ////////// menu bar //////////////////////////////
    //////////////////////////////////////////////////

    QMenuBar* menuBar = new QMenuBar(this);
    QMenu* menuFile = menuBar->addMenu(tr("&File"));
    QMenu* menuDetect = menuBar->addMenu(tr("&Detection"));
    QMenu* menuCam = menuBar->addMenu(tr("Video &Input"));
    QMenu* menuHelp = menuBar->addMenu(tr("&?"));



    _startAction = menuFile->addAction(tr("&Start"));
    _addObjectAction = menuFile->addAction(tr("&Add object to the scene"));
    _addObjectAction->setEnabled(false);
    _calibrateAction = menuFile->addAction(tr("&Calibrate camera"));
    _optionsAction = menuFile->addAction(tr("&Options"));
    _fullScreenAction = menuFile->addAction(tr("F&ull Screen"));
    _fullScreenAction->setEnabled(false);



    _detectGroup = new QActionGroup(menuDetect);
    _detectActions = new QAction*[NBR_DETECT];
    for(int i = 0; i < NBR_DETECT; i++)
    {
        _detectActions[i] = new QAction(_detectGroup);
        _detectActions[i]->setCheckable(true);
        _detectActions[i]->setEnabled(false);
        _detectGroup->addAction(_detectActions[i]);
        menuDetect->addAction(_detectActions[i]);
    }

    _detectActions[noDetect]->setText(tr("&No detection"));
    _detectActions[noDetect]->setChecked(true);
    _detectActions[chehra]->setText(tr("F&ace detection"));
    _detectActions[chess]->setText(tr("&Board detection"));
    _detectActions[QR]->setText(tr("&Marker detection"));



    _webcamGroup = new QActionGroup(menuCam);
    if(_nbrCam > 0)
    {
        _webcamActions = new QAction*[_nbrCam];
        for(int i = 0; i < _nbrCam; ++i)
        {
            _webcamActions[i] = menuCam->addAction(QString("Webcam ") + QString::number(i + 1));
            _webcamGroup->addAction(_webcamActions[i]);
            _webcamActions[i]->setCheckable(true);
            _webcamActions[i]->setEnabled(false);
        }
        _webcamActions[0]->setChecked(true);
    }

    menuCam->addSeparator();
    _videoAction = menuCam->addAction(tr("&Open video file"));
    _webcamGroup->addAction(_videoAction);
    _videoAction->setCheckable(true);
    _videoAction->setEnabled(false);



    connect(_startAction, SIGNAL(triggered()), this, SLOT(start()));
    connect(_calibrateAction, SIGNAL(triggered()), this, SLOT(calibrateCamera()));
    connect(_optionsAction, SIGNAL(triggered()), _webcamDevice, SLOT(setOptions()));

    this->setMenuBar(menuBar);
}

void MainWindow::setMainWindow()
{
    //////////////////////////////////////////////////
    ////////// enable actions ////////////////////////
    //////////////////////////////////////////////////
    _startAction->setEnabled(false);
    _addObjectAction->setEnabled(true);
    _fullScreenAction->setEnabled(true);

        for(int i = 0; i < NBR_DETECT; i++)
            _detectActions[i]->setEnabled(true);
    for(int i = 0; i < _nbrCam; i++)
        _webcamActions[i]->setEnabled(true);
    _videoAction->setEnabled(true);    // ca bug ; dunno why

    //////////////////////////////////////////////////
    //////////////////////////////////////////////////
    //////////////////////////////////////////////////

    QWidget* mainWidget = new QWidget();
    QHBoxLayout* mainLayout = new QHBoxLayout();

    QVBoxLayout* objectLayout = new QVBoxLayout();

    //////////////////////////////////////////////////
    ////////// webcam layout /////////////////////////
    //////////////////////////////////////////////////

    _mainView = new OSGWidget(_webcamDevice->getWebcam(), _mainMat, _objectsList[1], this);
    mainLayout->addWidget(_mainView);

    //////////////////////////////////////////////////
    ////////// object layout /////////////////////////
    //////////////////////////////////////////////////


    _objectChoiceComboBox = new QComboBox();
    _objectChoiceComboBox->addItem("All objects : 0");
    _objectChoiceComboBox->addItem("video");
    objectLayout->addWidget(_objectChoiceComboBox);

    QHBoxLayout* objectDispalyOptionsLayout = new QHBoxLayout();
    _isPrintedBox = new QCheckBox(tr("Display object in main view"));
    _isPrintedBox->setChecked(false);
    objectDispalyOptionsLayout->addWidget(_isPrintedBox);
    _isPrintedBox2 = new QCheckBox(tr("Display object in side view"));
    _isPrintedBox2->setChecked(true);
    objectDispalyOptionsLayout->addWidget(_isPrintedBox2);
    _deleteObjectButton = new QPushButton(tr("Delete object"));
    _deleteObjectButton->setEnabled(false);
    objectDispalyOptionsLayout->addWidget(_deleteObjectButton);

    objectLayout->addLayout(objectDispalyOptionsLayout);

    _objectCharacteristicsSpinSliders = new QSlider*[NBR_CHARACTERISTICS];
    for(int i = 0; i < NBR_CHARACTERISTICS; i++)
    {
        _objectCharacteristicsSpinSliders[i] = new QSlider();
        _objectCharacteristicsSpinSliders[i]->setOrientation(Qt::Horizontal);
    }

    QFormLayout *resizeLayout = new QFormLayout;
    QGroupBox *resizeGroup = new QGroupBox(tr(" Resize object "));

    _objectCharacteristicsSpinSliders[sizeX]->setRange(-100, 100);
    _objectCharacteristicsSpinSliders[sizeX]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[sizeX]->setValue(10.0);
    resizeLayout->addRow("x : ", _objectCharacteristicsSpinSliders[sizeX]);
    _objectCharacteristicsSpinSliders[sizeY]->setRange(-100, 100);
    _objectCharacteristicsSpinSliders[sizeY]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[sizeY]->setValue(10.0);
    resizeLayout->addRow("y : ", _objectCharacteristicsSpinSliders[sizeY]);
    _objectCharacteristicsSpinSliders[sizeZ]->setRange(-100, 100);
    _objectCharacteristicsSpinSliders[sizeZ]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[sizeZ]->setValue(10.0);
    resizeLayout->addRow("z : ", _objectCharacteristicsSpinSliders[sizeZ]);

    resizeGroup->setLayout(resizeLayout);
    objectLayout->addWidget(resizeGroup);

    QFormLayout *rotateLayout = new QFormLayout;
    QGroupBox *rotateGroup = new QGroupBox(tr(" Rotate object "));

    _objectCharacteristicsSpinSliders[rotX]->setRange(-180, 180);
    _objectCharacteristicsSpinSliders[rotX]->setSingleStep(1);
    rotateLayout->addRow("x : ", _objectCharacteristicsSpinSliders[rotX]);
    _objectCharacteristicsSpinSliders[rotY]->setRange(-180, 180);
    _objectCharacteristicsSpinSliders[rotY]->setSingleStep(1);
    rotateLayout->addRow("y : ", _objectCharacteristicsSpinSliders[rotY]);
    _objectCharacteristicsSpinSliders[rotZ]->setRange(-180, 180);
    _objectCharacteristicsSpinSliders[rotZ]->setSingleStep(1);
    rotateLayout->addRow("z : ", _objectCharacteristicsSpinSliders[rotZ]);

    rotateGroup->setLayout(rotateLayout);
    objectLayout->addWidget(rotateGroup);

    QFormLayout *translateLayout = new QFormLayout;
    QGroupBox *translateGroup = new QGroupBox(tr(" Translate object "));

    _objectCharacteristicsSpinSliders[transX]->setRange(-500, 500);
    _objectCharacteristicsSpinSliders[transX]->setSingleStep(1);
    translateLayout->addRow("x : ", _objectCharacteristicsSpinSliders[transX]);
    _objectCharacteristicsSpinSliders[transY]->setRange(-500, 500);
    _objectCharacteristicsSpinSliders[transY]->setSingleStep(1);
    translateLayout->addRow("y : ", _objectCharacteristicsSpinSliders[transY]);
    _objectCharacteristicsSpinSliders[transZ]->setRange(-500, 500);
    _objectCharacteristicsSpinSliders[transZ]->setSingleStep(1);
    translateLayout->addRow("z : ", _objectCharacteristicsSpinSliders[transZ]);

    translateGroup->setLayout(translateLayout);
    objectLayout->addWidget(translateGroup);

    QFormLayout *alphaLayout = new QFormLayout;
    QGroupBox *alphaGroup = new QGroupBox(tr(" Transparency "));

    _objectCharacteristicsSpinSliders[alpha]->setRange(0, 100);
    _objectCharacteristicsSpinSliders[alpha]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[alpha]->setValue(100);
    alphaLayout->addRow("α : ", _objectCharacteristicsSpinSliders[alpha]);

    alphaGroup->setLayout(alphaLayout);
    objectLayout->addWidget(alphaGroup);

    QHBoxLayout *videoLayout = new QHBoxLayout;
    QGroupBox *videoGroup = new QGroupBox();
    _play = new QPushButton(QIcon(":/icons/play"), "");
    _pause = new QPushButton(QIcon(":/icons/pause"), "");
    _fast = new QPushButton(QIcon(":/icons/forw"), "");
    _slow = new QPushButton(QIcon(":/icons/back"), "");

    videoLayout->addWidget(_slow);
    videoLayout->addWidget(_play);
    videoLayout->addWidget(_pause);
    videoLayout->addWidget(_fast);
    _pause->setEnabled(false);
    _play->setEnabled(false);
    _fast->setEnabled(false);
    _slow->setEnabled(false);
    videoGroup->setLayout(videoLayout);
    objectLayout->addWidget(videoGroup);

    _sideView = new SideViewOsgWidet(_webcamDevice->getWebcam(), _mainMat2, _objectsList2[1], this);
    objectLayout->addWidget(_sideView);

    objectLayout->setStretchFactor(_sideView, 2);
    mainLayout->addLayout(objectLayout);

    //////////////////////////////////////////////////

    mainLayout->setStretch(0, 2);
    mainLayout->setStretch(1, 1);
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);
}

void MainWindow::connectAll()
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(_webcamDevice, SIGNAL(updateWebcam()), this, SLOT(updateCam()));

    connect(_detectGroup, SIGNAL(triggered(QAction*)), this, SLOT(updateDetectMode()));
    connect(_webcamGroup, SIGNAL(triggered(QAction*)), this, SLOT(switchInput()));

    connect(_addObjectAction, SIGNAL(triggered()), this, SLOT(addObject()));
    connect(_fullScreenAction, SIGNAL(triggered()), this, SLOT(displayFullScreen()));
    connect(_mainView, SIGNAL(changeFullScreenState()), this, SLOT(displayFullScreen()));
    connect(_fullScreenView, SIGNAL(changeFullScreenState()), this, SLOT(displayFullScreen()));

    connect(_detectActions[noDetect], SIGNAL(toggled(bool)), this, SLOT(displayObjects(bool)));
    connect(_isPrintedBox, SIGNAL(clicked(bool)), this, SLOT(displayObjectInScene(bool)));
    connect(_isPrintedBox2, SIGNAL(clicked(bool)), this, SLOT(displayObjectInSideView(bool))); // #truanderie

    connect(_objectChoiceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObjectCharacteristics(int)));

    connect(_webcamDevice, SIGNAL(updateScene(cv::Mat, cv::Mat)), this, SLOT(updateSceneRT(cv::Mat, cv::Mat)));


    Our3DObject* object = _objectsList[0];

    connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
    connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
    connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

    connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
    connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
    connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

    connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
    connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
    connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

    connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

    object = _objectsList2[0];

    connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
    connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
    connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

    connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
    connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
    connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

    connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
    connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
    connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

    connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

    connect(_slow, SIGNAL(clicked()), _webcamDevice, SLOT(backward()));
    connect(_fast, SIGNAL(clicked()), _webcamDevice, SLOT(forward()));
    connect(_pause, SIGNAL(clicked()), _webcamDevice, SLOT(pause()));
    connect(_play, SIGNAL(clicked()), this, SLOT(play()));
}

void MainWindow::setShortcuts()
{
    _fullScreenShortcut = new QShortcut(Qt::CTRL + Qt::Key_F, this);
    connect(_fullScreenShortcut, SIGNAL(activated()), this, SLOT(displayFullScreen()));
    _fullScreenShortcut2 = new QShortcut(Qt::CTRL + Qt::Key_F, _fullScreenView); // necessaire pour utiliser le shortcut en plein ecran
    connect(_fullScreenShortcut2, SIGNAL(activated()), this, SLOT(displayFullScreen())); // #truanderie
    _leaveFullScreen = new QShortcut(Qt::Key_Escape, _fullScreenView);
    connect(_leaveFullScreen, SIGNAL(activated()), this, SLOT(displayFullScreen()));

    _pauseShortcut = new QShortcut(Qt::Key_Space, this);
    connect(_pauseShortcut, SIGNAL(activated()), _webcamDevice, SLOT(pause()));
    _pauseShortcut2 = new QShortcut(Qt::Key_Space, _fullScreenView);
    connect(_pauseShortcut2, SIGNAL(activated()), _webcamDevice, SLOT(pause()));
}

void MainWindow::createFullScreenWidget() // #truanderie
{
    QRect rec = QApplication::desktop()->screenGeometry();
    int screenHeight = rec.height();
    int screenWidth = rec.width();

    cv::Mat* webcam = _webcamDevice->getWebcam();
    int camHeight = webcam->rows;
    int camWidth = webcam->cols;

    _fullScreenView = new OSGWidget(webcam, _mainMat, _objectsList[1], 0);
    _fullScreenWidget = new QWidget;
    _fullScreenWidget->setContentsMargins(0, 0, 0, 0);

    int resizeH = screenHeight / camHeight;
    int resizeW = screenWidth / camWidth;

    if(resizeW > resizeH)
    {
        QHBoxLayout *layout = new QHBoxLayout;
        QGraphicsView *sideLayer = new QGraphicsView;
        QGraphicsView *sideLayer2 = new QGraphicsView;

        sideLayer->setBackgroundBrush(QBrush(Qt::black));
        sideLayer->setFixedWidth((screenWidth - (camWidth * resizeH)) / 2);
        sideLayer->setScene(new QGraphicsScene());
        sideLayer2->setBackgroundBrush(QBrush(Qt::black));
        sideLayer2->setFixedWidth((screenWidth - (camWidth * resizeH)) / 2);
        sideLayer2->setScene(new QGraphicsScene());

        layout->addWidget(sideLayer);
        layout->addWidget(_fullScreenView);
        layout->addWidget(sideLayer2);

        layout->setSpacing(0);
        layout->setContentsMargins(1, 0, 1, 0);

        _fullScreenWidget->setLayout(layout);
    }
    else if(resizeW < resizeH)
    {
        QVBoxLayout *layout = new QVBoxLayout;
        QGraphicsView *sideLayer = new QGraphicsView;
        QGraphicsView *sideLayer2 = new QGraphicsView;

        sideLayer->setBackgroundBrush(QBrush(Qt::black));
        sideLayer->setFixedHeight((screenHeight - (camHeight * resizeW)) / 2);
        sideLayer->setScene(new QGraphicsScene());
        sideLayer2->setBackgroundBrush(QBrush(Qt::black));
        sideLayer2->setFixedHeight((screenHeight - (camHeight * resizeW)) / 2);
        sideLayer2->setScene(new QGraphicsScene());

        layout->addWidget(sideLayer);
        layout->addWidget(_fullScreenView);
        layout->addWidget(sideLayer2);

        layout->setSpacing(0);
        layout->setContentsMargins(1, 0, 1, 0);

        _fullScreenWidget->setLayout(layout);
    }
    else
        _fullScreenWidget = _fullScreenView;
}

void MainWindow::initObjectsList()
{
    _objectsList.push_back(new Our3DObject()); // globalMat en _objectsList[0]
    _mainMat = new osg::MatrixTransform();
    _mainMat->addChild(_objectsList[0]);
    _objectsList[0]->setNodeMask(0);

    _objectsList2.push_back(new Our3DObject()); // globalMat2 en _objectsList2[0]
    _mainMat2 = new osg::MatrixTransform();
    _mainMat2->addChild(_objectsList2[0]);
    _objectsList2[0]->setNodeMask(1);

    _backgroundImage = new osg::Image;

    cv::Mat *webcamMat = _webcamDevice->getWebcam();

    cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

    cv::Mat cameraMatrix;

    fs["cameraMatrix"] >> cameraMatrix;

    int cx = cameraMatrix.at<double>(0, 2);
    int cy = cameraMatrix.at<double>(1, 2);
    double n = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2;
    // NEAR (n) = distance focale ; si pixels carres, fx = fy -> np
    //mais est generalement different de fy donc on prend (pour l'instant) par defaut la valeur mediane
    _corrector = (n / 2) / (cameraMatrix.at<double>(1, 2) - webcamMat->rows / 2);

    fs.release();

    _backgroundImage->setImage(webcamMat->cols, webcamMat->rows, 3,
                               GL_RGB, GL_BGR, GL_UNSIGNED_BYTE,
                               (uchar*)(webcamMat->data),
                               osg::Image::AllocationMode::NO_DELETE, 1);
    {
        osg::Geometry* geoQuad = new osg::Geometry;

        osg::Vec3Array* tabSommet = new osg::Vec3Array;
        tabSommet->push_back(osg::Vec3(-cx, n, -cy));
        tabSommet->push_back(osg::Vec3(webcamMat->cols - cx, n, -cy));
        tabSommet->push_back(osg::Vec3(webcamMat->cols - cx, n, webcamMat->rows - cy));
        tabSommet->push_back(osg::Vec3(-cx, n, webcamMat->rows - cy));
        geoQuad->setVertexArray(tabSommet);

        osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
        primitive->push_back(0);
        primitive->push_back(1);
        primitive->push_back(2);
        primitive->push_back(3);
        geoQuad->addPrimitiveSet(primitive);

        // Nous creons ensuite une tableau qui contiendra nos coordonnees de texture.
        osg::Vec2Array* coordonneeTexture = new osg::Vec2Array(4);
        (*coordonneeTexture)[0].set(0.0f, 1.0f);
        (*coordonneeTexture)[1].set(1.0f, 1.0f);
        (*coordonneeTexture)[2].set(1.0f, 0.0f);
        (*coordonneeTexture)[3].set(0.0f, 0.0f);
        geoQuad->setTexCoordArray(0, coordonneeTexture);

        osg::Geode* noeudGeo = new osg::Geode;
        osg::StateSet* statuts = noeudGeo->getOrCreateStateSet();
        statuts->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        noeudGeo->addDrawable(geoQuad);

        // Nous creons une Texture2D.
        osg::Texture2D* texture = new osg::Texture2D;

        // Nous associons notre image a notre objet Texture2D.
        texture->setImage(_backgroundImage);
        texture->setResizeNonPowerOfTwoHint(false);

        // Enfin nous activons les texture de notre objet Geometry a travers l'objet statuts.
        statuts->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
        statuts->setMode(GL_BLEND,osg::StateAttribute::ON);
        statuts->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

        _objectsList.push_back(new Our3DObject(noeudGeo)); // mat en _objectsList[1] qui contient le hud
    }

    {
        osg::Geometry* geoQuad = new osg::Geometry;

        osg::Vec3Array* tabSommet = new osg::Vec3Array;
        tabSommet->push_back(osg::Vec3(-cx, n, -cy));
        tabSommet->push_back(osg::Vec3(webcamMat->cols - cx, n, -cy));
        tabSommet->push_back(osg::Vec3(webcamMat->cols - cx, n, webcamMat->rows - cy));
        tabSommet->push_back(osg::Vec3(-cx, n, webcamMat->rows - cy));
        geoQuad->setVertexArray(tabSommet);

        osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
        primitive->push_back(0);
        primitive->push_back(1);
        primitive->push_back(2);
        primitive->push_back(3);
        geoQuad->addPrimitiveSet(primitive);

        // Nous creons ensuite une tableau qui contiendra nos coordonnees de texture.
        osg::Vec2Array* coordonneeTexture = new osg::Vec2Array(4);
        (*coordonneeTexture)[0].set(0.0f, 1.0f);
        (*coordonneeTexture)[1].set(1.0f, 1.0f);
        (*coordonneeTexture)[2].set(1.0f, 0.0f);
        (*coordonneeTexture)[3].set(0.0f, 0.0f);
        geoQuad->setTexCoordArray(0, coordonneeTexture);

        osg::Geode* noeudGeo = new osg::Geode;
        osg::StateSet* statuts = noeudGeo->getOrCreateStateSet();
        statuts->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        noeudGeo->addDrawable(geoQuad);

        // Nous creons une Texture2D.
        osg::Texture2D* texture = new osg::Texture2D;

        // Nous associons notre image a notre objet Texture2D.
        texture->setImage(_backgroundImage);
        texture->setResizeNonPowerOfTwoHint(false);

        // Enfin nous activons les texture de notre objet Geometry a travers l'objet statuts.
        statuts->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
        statuts->setMode(GL_BLEND,osg::StateAttribute::ON);
        statuts->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

        _objectsList2.push_back(new Our3DObject(noeudGeo));
    }
}
