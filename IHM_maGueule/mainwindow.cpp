#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("Projet M2 ISEN : application de realite augmentee"));

    m_webcamDevice = new WebcamDevice();

    setWindow();
    m_objectID = 0;

    connectAll();

    this->resize(1000, 700);
}

MainWindow::~MainWindow()
{
    for(int i = 0; i < NBR_DETECT; i++)
    {
        delete m_detectBoxes[i];
    }
    delete m_detectBoxes;

    delete m_addObjectButton;
    delete m_fullScreenButton;

    delete m_mainView;

    delete m_objectChoiceComboBox;
    delete m_deleteObjectButton;
    delete m_isPrintedBox;

    for(int i = 0; i < NBR_CHARACTERISTICS; i++)
    {
        delete m_objectCharacteristicsSpinSliders[i];
    }

    delete m_objectCharacteristicsSpinSliders;

    delete m_sideView;

    //////////////////////////////////////////////////

    m_webcamDevice->stop();
    delete m_webcamDevice;
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
        QMessageBox::warning(this, tr("erreur de chargement de l'objet"), tr("le chemin d'acces ou le nom n'ont pas ete renseigne"));
        return;
    }

    m_mainView->addObjectToScene(objectPath);
    m_sideView->addObjectToScene(objectPath);

    int nbrData = m_objectChoiceComboBox->itemText(0).mid(18).toInt();
    m_objectChoiceComboBox->setItemText(0, "tous les objets : " + QString::number(nbrData + 1));
    m_objectChoiceComboBox->addItem(objectName);
}

void MainWindow::updateObjectCharacteristics(int objectID)
{
    if(objectID == m_objectID)
        return;

    Our3DObject* object = NULL;

    if(m_objectID != 0)
        object = m_mainView->getObject(m_objectID - 1);
    else
        object = m_mainView->getGlobalMat();

    disconnect(m_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
    disconnect(m_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
    disconnect(m_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

    disconnect(m_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
    disconnect(m_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
    disconnect(m_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

    disconnect(m_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
    disconnect(m_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
    disconnect(m_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

    disconnect(m_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));



    m_objectID = objectID;

    if(m_objectID > 0)
        object = m_mainView->getObject(objectID - 1);
    else
        object = m_mainView->getGlobalMat();


    m_isPrintedBox->setChecked(object->isObjectPrinted());

    m_objectCharacteristicsSpinSliders[sizeX]->setValue(object->getSizeX());
    m_objectCharacteristicsSpinSliders[sizeY]->setValue(object->getSizeY());
    m_objectCharacteristicsSpinSliders[sizeZ]->setValue(object->getSizeZ());

    m_objectCharacteristicsSpinSliders[rotX]->setValue(object->getRotX());
    m_objectCharacteristicsSpinSliders[rotY]->setValue(object->getRotY());
    m_objectCharacteristicsSpinSliders[rotZ]->setValue(object->getRotZ());

    m_objectCharacteristicsSpinSliders[transX]->setValue(object->getTransX());
    m_objectCharacteristicsSpinSliders[transY]->setValue(object->getTransY());
    m_objectCharacteristicsSpinSliders[transZ]->setValue(object->getTransZ());

    m_objectCharacteristicsSpinSliders[alpha]->setValue(object->getAlpha());

    connect(m_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
    connect(m_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
    connect(m_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

    connect(m_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
    connect(m_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
    connect(m_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

    connect(m_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
    connect(m_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
    connect(m_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

    connect(m_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));
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

    m_detectGroup = new QButtonGroup();
    m_detectBoxes = new QCheckBox*[NBR_DETECT];
    for(int i = 0; i < NBR_DETECT; i++)
    {
        m_detectBoxes[i] = new QCheckBox();
        m_detectGroup->addButton(m_detectBoxes[i]);
        m_detectGroup->setId(m_detectBoxes[i], i);
        optionsLayout->addWidget(m_detectBoxes[i]);
    }

    m_detectBoxes[noDetect]->setText(tr("pas de detection"));
    m_detectBoxes[noDetect]->setChecked(true);
    m_detectBoxes[chehra]->setText(tr("detection visage"));
    m_detectBoxes[chess]->setText(tr("detection echiquier"));
    m_detectBoxes[chess]->setEnabled(false);
    m_detectBoxes[QR]->setText(tr("detection QR code"));
    m_detectBoxes[QR]->setEnabled(false);

    m_addObjectButton = new QPushButton(tr("ajouter objet a la scene"));
    optionsLayout->addWidget(m_addObjectButton);
    m_calibrateButton = new QPushButton(tr("calibrer camera"));
    m_calibrateButton->setEnabled(false);
    optionsLayout->addWidget(m_calibrateButton);
    m_fullScreenButton = new QPushButton(tr("afficher en plein ecran"));
    m_fullScreenButton->setEnabled(false);
    optionsLayout->addWidget(m_fullScreenButton);

    mainLayout->addLayout(optionsLayout);

    //////////////////////////////////////////////////
    ////////// webcam layout /////////////////////////
    //////////////////////////////////////////////////

    m_mainView = new OSGWidget(m_webcamDevice->getWebcam());
    mainLayout->addWidget(m_mainView);

    //////////////////////////////////////////////////
    ////////// object layout /////////////////////////
    //////////////////////////////////////////////////


    m_objectChoiceComboBox = new QComboBox();
    m_objectChoiceComboBox->addItem("tous les objets : 0");
    objectLayout->addWidget(m_objectChoiceComboBox);

    QHBoxLayout* objectDispalyOptionsLayout = new QHBoxLayout();
    m_isPrintedBox = new QCheckBox(tr("afficher l'objet"));
    m_isPrintedBox->setChecked(true);
    objectDispalyOptionsLayout->addWidget(m_isPrintedBox);
    m_deleteObjectButton = new QPushButton(tr("supprimer l'objet"));
    m_deleteObjectButton->setEnabled(false);
    objectDispalyOptionsLayout->addWidget(m_deleteObjectButton);

    objectLayout->addLayout(objectDispalyOptionsLayout);

    m_objectCharacteristicsSpinSliders = new QSlider*[NBR_CHARACTERISTICS];
    QLabel** objectCharacteristicsLabels = new QLabel*[NBR_CHARACTERISTICS];
    for(int i = 0; i < NBR_CHARACTERISTICS; i++)
    {
        m_objectCharacteristicsSpinSliders[i] = new QSlider();
        m_objectCharacteristicsSpinSliders[i]->setOrientation(Qt::Horizontal);
        objectCharacteristicsLabels[i] = new QLabel();
        objectLayout->addWidget(objectCharacteristicsLabels[i]);
        objectLayout->addWidget(m_objectCharacteristicsSpinSliders[i]);
    }

    objectCharacteristicsLabels[sizeX]->setText(tr("taille selon x"));
    m_objectCharacteristicsSpinSliders[sizeX]->setRange(-10000, 10000);
    m_objectCharacteristicsSpinSliders[sizeX]->setSingleStep(1);
    m_objectCharacteristicsSpinSliders[sizeX]->setValue(100.0);
    objectCharacteristicsLabels[sizeY]->setText(tr("taille selon y"));
    m_objectCharacteristicsSpinSliders[sizeY]->setRange(-10000, 10000);
    m_objectCharacteristicsSpinSliders[sizeY]->setSingleStep(1);
    m_objectCharacteristicsSpinSliders[sizeY]->setValue(100.0);
    objectCharacteristicsLabels[sizeZ]->setText(tr("taille selon z"));
    m_objectCharacteristicsSpinSliders[sizeZ]->setRange(-10000, 10000);
    m_objectCharacteristicsSpinSliders[sizeZ]->setSingleStep(1);
    m_objectCharacteristicsSpinSliders[sizeZ]->setValue(100.0);

    objectCharacteristicsLabels[rotX]->setText(tr("rotation selon x"));
    m_objectCharacteristicsSpinSliders[rotX]->setRange(-180, 180);
    m_objectCharacteristicsSpinSliders[rotX]->setSingleStep(1);
    objectCharacteristicsLabels[rotY]->setText(tr("rotation selon y"));
    m_objectCharacteristicsSpinSliders[rotY]->setRange(-180, 180);
    m_objectCharacteristicsSpinSliders[rotY]->setSingleStep(1);
    objectCharacteristicsLabels[rotZ]->setText(tr("rotation selon z"));
    m_objectCharacteristicsSpinSliders[rotZ]->setRange(-180, 180);
    m_objectCharacteristicsSpinSliders[rotZ]->setSingleStep(1);

    objectCharacteristicsLabels[transX]->setText(tr("translation selon x"));
    m_objectCharacteristicsSpinSliders[transX]->setRange(-5000, 5000);
    m_objectCharacteristicsSpinSliders[transX]->setSingleStep(1);
    objectCharacteristicsLabels[transY]->setText(tr("translation selon y"));
    m_objectCharacteristicsSpinSliders[transY]->setRange(-5000, 5000);
    m_objectCharacteristicsSpinSliders[transY]->setSingleStep(1);
    objectCharacteristicsLabels[transZ]->setText(tr("translation selon z"));
    m_objectCharacteristicsSpinSliders[transZ]->setRange(-5000, 5000);
    m_objectCharacteristicsSpinSliders[transZ]->setSingleStep(1);

    objectCharacteristicsLabels[alpha]->setText(tr("transparence"));
    m_objectCharacteristicsSpinSliders[alpha]->setRange(0, 100);
    m_objectCharacteristicsSpinSliders[alpha]->setSingleStep(1);
    m_objectCharacteristicsSpinSliders[alpha]->setValue(100);

    m_sideView = new SideViewOsgWidet(m_webcamDevice->getWebcam());
    objectLayout->addWidget(m_sideView);

    objectLayout->setStretchFactor(m_sideView, 2);
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
    connect(m_webcamDevice, SIGNAL(updateWebcam()), m_mainView, SLOT(repaint()));
    connect(m_webcamDevice, SIGNAL(updateWebcam()), m_sideView, SLOT(repaint()));

    connect(m_detectGroup, SIGNAL(buttonClicked(int)), m_webcamDevice, SLOT(switchMode(int)));

    connect(m_addObjectButton, SIGNAL(clicked()), this, SLOT(addObject()));

    connect(m_detectBoxes[noDetect], SIGNAL(toggled(bool)), m_mainView, SLOT(displayObjects(bool)));
    connect(m_isPrintedBox, SIGNAL(toggled(bool)), this, SLOT(displayObjectInScene(bool)));

    connect(m_objectChoiceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObjectCharacteristics(int)));

    connect(m_webcamDevice, SIGNAL(updateScene(cv::Mat,cv::Mat)), m_mainView, SLOT(updateSceneRT(cv::Mat,cv::Mat)));
    connect(m_webcamDevice, SIGNAL(updateScene(cv::Mat,cv::Mat)), m_sideView, SLOT(updateSceneRT(cv::Mat,cv::Mat)));


    Our3DObject* object = m_mainView->getGlobalMat();

    connect(m_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
    connect(m_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
    connect(m_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

    connect(m_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
    connect(m_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
    connect(m_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

    connect(m_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
    connect(m_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
    connect(m_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

    connect(m_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));
}
