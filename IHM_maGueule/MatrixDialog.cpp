#include "MatrixDialog.h"

MatrixDialog::MatrixDialog(QWidget *parent) :
    QDialog(parent)
{
    this->setWindowTitle("Calibration matrix");
    this->setWindowIcon(QIcon(":/icons/icon"));

    _defaultMatrix = new QRadioButton("Use default calibration matrix (might be less accurate).");
    _existingMatrix = new QRadioButton("Import existing calibration matrix.");
    _calibrateMatrix = new QRadioButton("Create a new calibration matrix.");

    _title = new QLabel("Unable to find calibration matrix.\nPlease select your choice of calibration.");

    _ok = new QPushButton(tr("Ok"));
    _cancel = new QPushButton(tr("Cancel"));

    _matrixGroup = new QGroupBox();

    QVBoxLayout* choiceLayout = new QVBoxLayout;
    choiceLayout->addWidget(_defaultMatrix);
    choiceLayout->addWidget(_existingMatrix);
    choiceLayout->addWidget(_calibrateMatrix);
    _matrixGroup->setLayout(choiceLayout);

    QVBoxLayout* choiceLayoutGlobal = new QVBoxLayout;
    choiceLayoutGlobal->addWidget(_title);
    choiceLayoutGlobal->addWidget(_matrixGroup);

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
