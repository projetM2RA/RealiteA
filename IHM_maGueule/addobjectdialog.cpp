#include "AddObjectDialog.h"

AddObjectDialog::AddObjectDialog(QWidget *parent) :
    QDialog(parent)
{
    _objectName = new QLineEdit;
    _objectName->setText(tr("Name"));

    _objectPath = new QLineEdit;
    _objectPath->setText(tr(""));

    _texturePath = new QLineEdit;
    _texturePath->setText(tr(""));

    QLabel* templateLabel = new QLabel("Templates");
    QLabel* customLabel = new QLabel("Custom object");

    QPushButton* ok = new QPushButton(tr("Ok"));
    QPushButton* cancel = new QPushButton(tr("Cancel"));
    QPushButton* path = new QPushButton(QIcon(":/icons/open"), "");
    QPushButton* texturePath = new QPushButton(QIcon(":/icons/open"), "");
    QPushButton* faceTemplate = new QPushButton("");
    QPushButton* chessTemplate = new QPushButton("");
    QPushButton* brainTemplate = new QPushButton("");
    QPushButton* axesTemplate = new QPushButton("");

    QFrame* separator = new QFrame();
    separator->setFrameShape(QFrame::HLine);

    //CSS
    faceTemplate->setObjectName("faceTemplate");
    chessTemplate->setObjectName("chessTemplate");
    brainTemplate->setObjectName("brainTemplate");
    axesTemplate->setObjectName("axesTemplate");
    templateLabel->setObjectName("infoLabel");
    customLabel->setObjectName("infoLabel");

    QButtonGroup* templateGroup = new QButtonGroup();
    templateGroup->addButton(faceTemplate, face);
    templateGroup->addButton(chessTemplate, chessboard);
    templateGroup->addButton(brainTemplate, brain);
    templateGroup->addButton(axesTemplate, axes);

    QHBoxLayout* templateLayout = new QHBoxLayout;
    templateLayout->addWidget(faceTemplate);
    templateLayout->addWidget(chessTemplate);
    templateLayout->addWidget(brainTemplate);
    templateLayout->addWidget(axesTemplate);

    QHBoxLayout* pathLayout = new QHBoxLayout;
    pathLayout->addWidget(_objectPath);
    pathLayout->addWidget(path);

    QHBoxLayout* textureLayout = new QHBoxLayout;
    textureLayout->addWidget(_texturePath);
    textureLayout->addWidget(texturePath);

    QFormLayout* optionsLayout = new QFormLayout;
    optionsLayout->addRow(tr("Object name"), _objectName);
    optionsLayout->addRow(tr("Path of the object : "), pathLayout);
    optionsLayout->addRow(tr("Texture path : "), textureLayout);

    QHBoxLayout* buttonsLayout = new QHBoxLayout;
    buttonsLayout->setAlignment(Qt::AlignRight);
    buttonsLayout->addWidget(ok);
    buttonsLayout->addWidget(cancel);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(templateLabel);
    mainLayout->addLayout(templateLayout);
    mainLayout->addWidget(separator);
    mainLayout->addWidget(customLabel);
    mainLayout->addLayout(optionsLayout);
    mainLayout->addLayout(buttonsLayout);

    setLayout(mainLayout);

    mainLayout->setSizeConstraint(QLayout::SetFixedSize);

    this->setWindowTitle("Add 3D object");
    this->setWindowIcon(QIcon(":/icons/icon"));
    this->setWindowFlags(this->windowFlags() & (~Qt::WindowContextHelpButtonHint));

    connect(templateGroup, SIGNAL(buttonClicked(int)), this, SLOT(emitIndex(int)));
    connect(templateGroup, SIGNAL(buttonClicked(int)), this, SLOT(close()));
    connect(path, SIGNAL(clicked()), this, SLOT(updatePath()));
    connect(texturePath, SIGNAL(clicked()), this, SLOT(updateTexture()));
    connect(cancel, SIGNAL(clicked()), this, SLOT(reject()));
    connect(ok, SIGNAL(clicked()), this, SLOT(accept()));
}

void AddObjectDialog::updatePath()
{
    QString objectPath = QFileDialog::getOpenFileName(this, "Open 3D Object", "../rsc/objets3D/", "3D object (*.obj *.3ds *.stl *.osgt *.osg)");
    if(objectPath != "")
        _objectPath->setText(objectPath);
}

void AddObjectDialog::updateTexture()
{
    QString texturePath = QFileDialog::getOpenFileName(this, "Open 3D Texture", "../rsc/objets3D/Textures/", "texture image (*.bmp)");
    if(texturePath != "")
        _texturePath->setText(texturePath);
}
