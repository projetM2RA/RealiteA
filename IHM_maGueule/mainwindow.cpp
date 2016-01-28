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

    QPushButton *widgetTemporaire1 = new QPushButton();
    mainLayout->addWidget(widgetTemporaire1);

    //////////////////////////////////////////////////
    ////////// object layout /////////////////////////
    //////////////////////////////////////////////////


    m_objectChoiceComboBox = new QComboBox();
    objectLayout->addWidget(m_objectChoiceComboBox);

    m_objectCharacteristicsSpinBoxes = new QDoubleSpinBox*[NBR_CHARACTERISTICS];
    QLabel** objectCharacteristicsLabels = new QLabel*[NBR_CHARACTERISTICS];
    for(int i = 0; i < NBR_CHARACTERISTICS; i++)
    {
        m_objectCharacteristicsSpinBoxes[i] = new QDoubleSpinBox();
        objectCharacteristicsLabels[i] = new QLabel();
        objectLayout->addWidget(objectCharacteristicsLabels[i]);
        objectLayout->addWidget(m_objectCharacteristicsSpinBoxes[i]);
    }

    objectCharacteristicsLabels[sizeX]->setText(tr("taille selon x"));
    objectCharacteristicsLabels[sizeY]->setText(tr("taille selon y"));
    objectCharacteristicsLabels[sizeZ]->setText(tr("taille selon z"));

    objectCharacteristicsLabels[rotX]->setText(tr("rotation selon x"));
    objectCharacteristicsLabels[rotY]->setText(tr("rotation selon y"));
    objectCharacteristicsLabels[rotZ]->setText(tr("rotation selon z"));

    objectCharacteristicsLabels[transX]->setText(tr("translation selon x"));
    objectCharacteristicsLabels[transY]->setText(tr("translation selon y"));
    objectCharacteristicsLabels[transZ]->setText(tr("translation selon z"));

    QPushButton *widgetTemporaire2 = new QPushButton("VISU 3D");
    objectLayout->addWidget(widgetTemporaire2);

    mainLayout->addLayout(objectLayout);

    //////////////////////////////////////////////////

   mainWidget->setLayout(mainLayout);
   this->setCentralWidget(mainWidget);
}
