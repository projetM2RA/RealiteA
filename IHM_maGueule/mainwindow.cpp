#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    this->setWindowTitle(tr("Augmented Reality Of Neuroskeleton"));
    this->setWindowIcon(QIcon("../rsc/icons/icon.png"));
    _webcamDevice = new WebcamDevice(this);
    connect(_webcamDevice, SIGNAL(shutdownSignal()), this, SLOT(close()));

    setWindow();

    _webcamDevice->initModels();

    _objectID = 0;

    connectAll();

    this->resize(1000, 700);
}

MainWindow::~MainWindow()
{
    for(int i = 0; i < NBR_DETECT; i++)
    {
        delete _detectBoxes[i];
    }
    delete _detectBoxes;

    delete _addObjectButton;
    delete _fullScreenButton;

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

    _mainView->addObjectToScene(objectPath);
    _sideView->addObjectToScene(objectPath);

    int nbrData = _objectChoiceComboBox->itemText(0).mid(18).toInt();
    _objectChoiceComboBox->setItemText(0, "All objects : " + QString::number(nbrData + 1));
    _objectChoiceComboBox->addItem(objectName);
}

void MainWindow::updateObjectCharacteristics(int objectID)
{
    if(objectID == _objectID)
        return;

    Our3DObject* object = NULL;

    if(_objectID != 0)
        object = _mainView->getObject(_objectID - 1);
    else
        object = _mainView->getGlobalMat();

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



    _objectID = objectID;

    if(_objectID > 0)
        object = _mainView->getObject(objectID - 1);
    else
        object = _mainView->getGlobalMat();


    _isPrintedBox->setChecked(object->isObjectPrinted());

    _objectCharacteristicsSpinSliders[sizeX]->setValue(object->getSizeX());
    _objectCharacteristicsSpinSliders[sizeY]->setValue(object->getSizeY());
    _objectCharacteristicsSpinSliders[sizeZ]->setValue(object->getSizeZ());

    _objectCharacteristicsSpinSliders[rotX]->setValue(object->getRotX());
    _objectCharacteristicsSpinSliders[rotY]->setValue(object->getRotY());
    _objectCharacteristicsSpinSliders[rotZ]->setValue(object->getRotZ());

    _objectCharacteristicsSpinSliders[transX]->setValue(object->getTransX());
    _objectCharacteristicsSpinSliders[transY]->setValue(object->getTransY());
    _objectCharacteristicsSpinSliders[transZ]->setValue(object->getTransZ());

    _objectCharacteristicsSpinSliders[alpha]->setValue(object->getAlpha());

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
}



// private
void MainWindow::setWindow()
{
    QWidget* mainWidget = new QWidget();
    QHBoxLayout* mainLayout = new QHBoxLayout();

    QVBoxLayout* optionsLayout = new QVBoxLayout();
    QVBoxLayout* objectLayout = new QVBoxLayout();

    //////////////////////////////////////////////////
    ////////// options layout ////////////////////////
    //////////////////////////////////////////////////

    _detectGroup = new QButtonGroup();
    _detectBoxes = new QCheckBox*[NBR_DETECT];
    for(int i = 0; i < NBR_DETECT; i++)
    {
        _detectBoxes[i] = new QCheckBox();
        _detectGroup->addButton(_detectBoxes[i]);
        _detectGroup->setId(_detectBoxes[i], i);
        optionsLayout->addWidget(_detectBoxes[i]);
    }

    _detectBoxes[noDetect]->setText(tr("No detection"));
    _detectBoxes[noDetect]->setChecked(true);
    _detectBoxes[chehra]->setText(tr("Face detection"));
    _detectBoxes[chess]->setText(tr("Board detection"));
    _detectBoxes[chess]->setEnabled(false);
    _detectBoxes[QR]->setText(tr("Marker detection"));
    _detectBoxes[QR]->setEnabled(false);

    _addObjectButton = new QPushButton(tr("Add object to the scene"));
    optionsLayout->addWidget(_addObjectButton);
    _calibrateButton = new QPushButton(tr("Calibrate camera"));
    _calibrateButton->setEnabled(false);
    optionsLayout->addWidget(_calibrateButton);
    _fullScreenButton = new QPushButton(tr("Full Screen"));
    _fullScreenButton->setEnabled(false);
    optionsLayout->addWidget(_fullScreenButton);

    mainLayout->addLayout(optionsLayout);

    //////////////////////////////////////////////////
    ////////// webcam layout /////////////////////////
    //////////////////////////////////////////////////

    _mainView = new OSGWidget(_webcamDevice->getWebcam());
    mainLayout->addWidget(_mainView);

    //////////////////////////////////////////////////
    ////////// object layout /////////////////////////
    //////////////////////////////////////////////////


    _objectChoiceComboBox = new QComboBox();
    _objectChoiceComboBox->addItem("All objects : 0");
    objectLayout->addWidget(_objectChoiceComboBox);

    QHBoxLayout* objectDispalyOptionsLayout = new QHBoxLayout();
    _isPrintedBox = new QCheckBox(tr("Display object"));
    _isPrintedBox->setChecked(true);
    objectDispalyOptionsLayout->addWidget(_isPrintedBox);
    _deleteObjectButton = new QPushButton(tr("Delete object"));
    _deleteObjectButton->setEnabled(false);
    objectDispalyOptionsLayout->addWidget(_deleteObjectButton);

    objectLayout->addLayout(objectDispalyOptionsLayout);

    _objectCharacteristicsSpinSliders = new QSlider*[NBR_CHARACTERISTICS];
    QLabel** objectCharacteristicsLabels = new QLabel*[NBR_CHARACTERISTICS];
    for(int i = 0; i < NBR_CHARACTERISTICS; i++)
    {
        _objectCharacteristicsSpinSliders[i] = new QSlider();
        _objectCharacteristicsSpinSliders[i]->setOrientation(Qt::Horizontal);
        objectCharacteristicsLabels[i] = new QLabel();
        objectLayout->addWidget(objectCharacteristicsLabels[i]);
        objectLayout->addWidget(_objectCharacteristicsSpinSliders[i]);
    }

    objectCharacteristicsLabels[sizeX]->setText(tr("Resize by x"));
    _objectCharacteristicsSpinSliders[sizeX]->setRange(-10000, 10000);
    _objectCharacteristicsSpinSliders[sizeX]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[sizeX]->setValue(100.0);
    objectCharacteristicsLabels[sizeY]->setText(tr("Resize by y"));
    _objectCharacteristicsSpinSliders[sizeY]->setRange(-10000, 10000);
    _objectCharacteristicsSpinSliders[sizeY]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[sizeY]->setValue(100.0);
    objectCharacteristicsLabels[sizeZ]->setText(tr("Resize by z"));
    _objectCharacteristicsSpinSliders[sizeZ]->setRange(-10000, 10000);
    _objectCharacteristicsSpinSliders[sizeZ]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[sizeZ]->setValue(100.0);

    objectCharacteristicsLabels[rotX]->setText(tr("Rotate around x"));
    _objectCharacteristicsSpinSliders[rotX]->setRange(-180, 180);
    _objectCharacteristicsSpinSliders[rotX]->setSingleStep(1);
    objectCharacteristicsLabels[rotY]->setText(tr("Rotate around y"));
    _objectCharacteristicsSpinSliders[rotY]->setRange(-180, 180);
    _objectCharacteristicsSpinSliders[rotY]->setSingleStep(1);
    objectCharacteristicsLabels[rotZ]->setText(tr("Rotate around z"));
    _objectCharacteristicsSpinSliders[rotZ]->setRange(-180, 180);
    _objectCharacteristicsSpinSliders[rotZ]->setSingleStep(1);

    objectCharacteristicsLabels[transX]->setText(tr("Translate across x"));
    _objectCharacteristicsSpinSliders[transX]->setRange(-5000, 5000);
    _objectCharacteristicsSpinSliders[transX]->setSingleStep(1);
    objectCharacteristicsLabels[transY]->setText(tr("Translate across y"));
    _objectCharacteristicsSpinSliders[transY]->setRange(-5000, 5000);
    _objectCharacteristicsSpinSliders[transY]->setSingleStep(1);
    objectCharacteristicsLabels[transZ]->setText(tr("Translate across z"));
    _objectCharacteristicsSpinSliders[transZ]->setRange(-5000, 5000);
    _objectCharacteristicsSpinSliders[transZ]->setSingleStep(1);

    objectCharacteristicsLabels[alpha]->setText(tr("Transparency"));
    _objectCharacteristicsSpinSliders[alpha]->setRange(0, 100);
    _objectCharacteristicsSpinSliders[alpha]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[alpha]->setValue(100);

    _sideView = new SideViewOsgWidet(_webcamDevice->getWebcam());
    objectLayout->addWidget(_sideView);

    objectLayout->setStretchFactor(_sideView, 2);
    mainLayout->addLayout(objectLayout);

    //////////////////////////////////////////////////

    mainLayout->setStretch(1, 6);
    mainLayout->setStretch(2, 2);
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);
}

void MainWindow::connectAll()
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(_webcamDevice, SIGNAL(updateWebcam()), _mainView, SLOT(repaint()));
    connect(_webcamDevice, SIGNAL(updateWebcam()), _sideView, SLOT(repaint()));

    connect(_detectGroup, SIGNAL(buttonClicked(int)), _webcamDevice, SLOT(switchMode(int)));

    connect(_addObjectButton, SIGNAL(clicked()), this, SLOT(addObject()));

    connect(_detectBoxes[noDetect], SIGNAL(toggled(bool)), _mainView, SLOT(displayObjects(bool)));
    connect(_isPrintedBox, SIGNAL(clicked(bool)), this, SLOT(displayObjectInScene(bool)));

    connect(_objectChoiceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObjectCharacteristics(int)));

    connect(_webcamDevice, SIGNAL(updateScene(cv::Mat,cv::Mat)), _mainView, SLOT(updateSceneRT(cv::Mat,cv::Mat)));
    connect(_webcamDevice, SIGNAL(updateScene(cv::Mat,cv::Mat)), _sideView, SLOT(updateSceneRT(cv::Mat,cv::Mat)));


    Our3DObject* object = _mainView->getGlobalMat();

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
}
