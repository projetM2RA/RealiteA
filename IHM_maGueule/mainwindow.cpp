#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("Projet M2 ISEN : application de realite augmentee"));

    setWindow();
}

MainWindow::~MainWindow()
{

}

void MainWindow::setWindow()
{
    QWidget* mainWidget = new QWidget();
    QHBoxLayout* mainLayout = new QHBoxLayout();

    QVBoxLayout* optionsLayout = new QVBoxLayout();
    QVBoxLayout* objectLayout = new QVBoxLayout();

    //////////////////////////////////////////////////
    ////////// options layout ////////////////////////
    //////////////////////////////////////////////////

    QButtonGroup* detectGroup = new QButtonGroup();
    m_detectBoxes = new QCheckBox*[NBR_DETECT];
    for(int i = 0; i < NBR_DETECT; i++)
    {
        m_detectBoxes[i] = new QCheckBox();
        detectGroup->addButton(m_detectBoxes[i]);
        optionsLayout->addWidget(m_detectBoxes[i]);
    }

    m_detectBoxes[chehra]->setText(tr("detection visage"));
    m_detectBoxes[chess]->setText(tr("detection echiquier"));
    m_detectBoxes[QR]->setText(tr("detection QR code"));

    m_occlusionBox = new QCheckBox(tr("occlusion"));
    optionsLayout->addWidget(m_occlusionBox);
    m_addObjectButton = new QPushButton(tr("ajouter objet a la scene"));
    optionsLayout->addWidget(m_addObjectButton);
    m_3DSlicerButton = new QPushButton(tr("lancer 3DSlicer"));
    optionsLayout->addWidget(m_3DSlicerButton);
    m_fullScreenButton = new QPushButton(tr("afficher en plein ecran"));
    optionsLayout->addWidget(m_fullScreenButton);

    mainLayout->addLayout(optionsLayout);

    //////////////////////////////////////////////////
    ////////// webcam layout /////////////////////////
    //////////////////////////////////////////////////

    OSGWidget *widgetTemporaire1 = new OSGWidget();
    mainLayout->addWidget(widgetTemporaire1);

    //////////////////////////////////////////////////
    ////////// object layout /////////////////////////
    //////////////////////////////////////////////////


    m_objectChoiceComboBox = new QComboBox();
    objectLayout->addWidget(m_objectChoiceComboBox);

    QHBoxLayout* objectDispalyOptionsLayout = new QHBoxLayout();
    m_isPrintedBox = new QCheckBox(tr("afficher l'objet"));
    m_isPrintedBox->setChecked(true);
    objectDispalyOptionsLayout->addWidget(m_isPrintedBox);
    m_isMaskBox = new QCheckBox(tr("masque"));
    objectDispalyOptionsLayout->addWidget(m_isMaskBox);

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
    m_objectCharacteristicsSpinSliders[sizeX]->setRange(-1000, 1000);
    m_objectCharacteristicsSpinSliders[sizeX]->setValue(100.0);
    objectCharacteristicsLabels[sizeY]->setText(tr("taille selon y"));
    m_objectCharacteristicsSpinSliders[sizeY]->setRange(-1000, 1000);
    m_objectCharacteristicsSpinSliders[sizeY]->setValue(100.0);
    objectCharacteristicsLabels[sizeZ]->setText(tr("taille selon z"));
    m_objectCharacteristicsSpinSliders[sizeZ]->setRange(-1000, 1000);
    m_objectCharacteristicsSpinSliders[sizeZ]->setValue(100.0);

    objectCharacteristicsLabels[rotX]->setText(tr("rotation selon x"));
    m_objectCharacteristicsSpinSliders[rotX]->setRange(-180, 180);
    objectCharacteristicsLabels[rotY]->setText(tr("rotation selon y"));
    m_objectCharacteristicsSpinSliders[rotY]->setRange(-180, 180);
    objectCharacteristicsLabels[rotZ]->setText(tr("rotation selon z"));
    m_objectCharacteristicsSpinSliders[rotZ]->setRange(-180, 180);

    objectCharacteristicsLabels[transX]->setText(tr("translation selon x"));
    m_objectCharacteristicsSpinSliders[transX]->setRange(-5000, 5000);
    objectCharacteristicsLabels[transY]->setText(tr("translation selon y"));
    m_objectCharacteristicsSpinSliders[transY]->setRange(-5000, 5000);
    objectCharacteristicsLabels[transZ]->setText(tr("translation selon z"));
    m_objectCharacteristicsSpinSliders[transZ]->setRange(-5000, 5000);

    OSGWidget *widgetTemporaire2 = new OSGWidget();
    objectLayout->addWidget(widgetTemporaire2);

    objectLayout->setStretchFactor(widgetTemporaire2, 2);
    mainLayout->addLayout(objectLayout);

    //////////////////////////////////////////////////

    mainLayout->setStretch(1, 6);
    mainLayout->setStretch(2, 2);
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);
}
