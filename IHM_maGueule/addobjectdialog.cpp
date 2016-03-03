#include "AddObjectDialog.h"

AddObjectDialog::AddObjectDialog(QWidget *parent) :
    QDialog(parent)
{
    _objectName = new QLineEdit;
    _objectName->setText(tr("Name"));

    _objectPath = new QLineEdit;
    _objectPath->setText(tr(""));

    QLabel* templateLabel = new QLabel("Templates");

    QPushButton* ok = new QPushButton(tr("Ok"));
    QPushButton* cancel = new QPushButton(tr("Cancel"));
    QPushButton* path = new QPushButton(QIcon(":/icons/open"), "");
    QPushButton* faceTemplate = new QPushButton("");
    QPushButton* chessTemplate = new QPushButton("");
    QPushButton* brainTemplate = new QPushButton("");
    QPushButton* axesTemplate = new QPushButton("");

    //CSS
    faceTemplate->setObjectName("faceTemplate");
    chessTemplate->setObjectName("chessTemplate");
    brainTemplate->setObjectName("brainTemplate");
    axesTemplate->setObjectName("axesTemplate");
    templateLabel->setObjectName("infoLabel");

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

    QFormLayout* optionsLayout = new QFormLayout;
    optionsLayout->addRow(tr("Object name"), _objectName);
    optionsLayout->addRow(tr("Path of the object : "), pathLayout);

    QHBoxLayout* buttonsLayout = new QHBoxLayout;
    buttonsLayout->setAlignment(Qt::AlignRight);
    buttonsLayout->addWidget(ok);
    buttonsLayout->addWidget(cancel);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(templateLabel);
    mainLayout->addLayout(templateLayout);
    mainLayout->addSpacing(20);
    mainLayout->addLayout(optionsLayout);
    mainLayout->addLayout(buttonsLayout);

    setLayout(mainLayout);

    mainLayout->setSizeConstraint( QLayout::SetFixedSize );

    this->setWindowTitle("Add 3D object");
    this->setWindowIcon(QIcon(":/icons/icon"));
    this->setWindowFlags(this->windowFlags() & (~Qt::WindowContextHelpButtonHint));

    //connect(templateGroup, SIGNAL(buttonClicked(int)), this, SLOT(emitIndex(int)));
    //connect(templateGroup, SIGNAL(buttonClicked(int)), this, SLOT(close()));
    connect(path, SIGNAL(clicked()), this, SLOT(updatePath()));
    connect(cancel, SIGNAL(clicked()), this, SLOT(reject()));
    connect(ok, SIGNAL(clicked()), this, SLOT(accept()));
}

void AddObjectDialog::updatePath()
{
    QString objectPath = QFileDialog::getOpenFileName(this, "Open 3D Object", "../rsc/objets3D/", "3D object (*.obj *.3ds *.stl *.osgt *.osg)");
    if(objectPath != "")
        _objectPath->setText(objectPath);
}
