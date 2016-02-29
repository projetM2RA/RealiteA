#include "MatrixDialog.h"

MatrixDialog::MatrixDialog(QWidget *parent) :
    QDialog(parent)
{
    this->setWindowTitle("Matrix choice");
    this->setWindowIcon(QIcon(":/icons/icon"));

    _defaultMatrix = new QRadioButton("Use default calibration matrix.");
    _existingMatrix = new QRadioButton("Import existing calibration matrix.");
    _calibrateMatrix = new QRadioButton("Create a new calibration matrix.");

    _ok = new QPushButton(tr("Ok"));
    _cancel = new QPushButton(tr("Cancel"));

    QGroupBox* matrixGroup = new QGroupBox();

    QVBoxLayout* choiceLayout = new QVBoxLayout;
    choiceLayout->addWidget(_defaultMatrix);
    choiceLayout->addWidget(_existingMatrix);
    choiceLayout->addWidget(_calibrateMatrix);
    matrixGroup->setLayout(choiceLayout);

    QVBoxLayout* choiceLayoutGlobal = new QVBoxLayout;
    choiceLayoutGlobal->addWidget(matrixGroup);

    QHBoxLayout* buttonsLayout = new QHBoxLayout;
    buttonsLayout->setAlignment(Qt::AlignRight);
    buttonsLayout->addWidget(_ok);
    buttonsLayout->addWidget(_cancel);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(choiceLayoutGlobal);
    mainLayout->addLayout(buttonsLayout);
    mainLayout->setSizeConstraint( QLayout::SetFixedSize );

    this->setLayout(mainLayout);
    this->setWindowFlags(this->windowFlags() & (~Qt::WindowContextHelpButtonHint));

    connect(_cancel, SIGNAL(clicked()), this, SLOT(reject()));
    connect(_ok, SIGNAL(clicked()), this, SLOT(accept()));
}
