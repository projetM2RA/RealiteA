#include "OptionsDialog.h"

OptionsDialog::OptionsDialog(int nbrCols, int nbrRows, double chessSize, int markerSize, QWidget *parent) :
    QDialog(parent)
{
    this->setWindowTitle("Options");
    this->setWindowIcon(QIcon(":/icons/icon"));

    _chehra = new QCheckBox("Activate face detection (might take up to 1min to load)");
    _chehra->setChecked(true);

    _nbrRowsLabel = new QLabel(tr("Number of rows : "));
    _nbrColsLabel = new QLabel(tr("Number of columns : "));
    _chessSizeLabel = new QLabel(tr("Size of one square side : "));

    _markerSizeLabel = new QLabel(tr("Size of the marker (side) : "));

    _nbrRows = new QSpinBox;
    _nbrRows->setValue(nbrRows);
	_nbrRows->setRange(5,10);
    _nbrCols = new QSpinBox;
    _nbrCols->setValue(nbrCols);
	_nbrCols->setRange(5,10);
    _chessSize = new QDoubleSpinBox;
    _chessSize->setValue(chessSize);
    _chessSize->setSuffix(" mm");

    _markerSize = new QSpinBox;
    _markerSize->setValue(markerSize);
    _markerSize->setMinimum(40);

    _ok = new QPushButton(tr("Ok"));
    _cancel = new QPushButton(tr("Cancel"));

    QGroupBox *faceGroup = new QGroupBox(tr("Face detection"));
    QGroupBox *chessGroup = new QGroupBox(tr("Chess detection"));
    QGroupBox *markerGroup = new QGroupBox(tr("Marker detection"));

    QHBoxLayout* faceLayout = new QHBoxLayout;
    faceLayout->addWidget(_chehra);
    faceGroup->setLayout(faceLayout);

    QHBoxLayout* chessLayout = new QHBoxLayout;
    chessLayout->addWidget(_nbrRowsLabel);
    chessLayout->addWidget(_nbrRows);
    chessLayout->addWidget(_nbrColsLabel);
    chessLayout->addWidget(_nbrCols);
    chessLayout->addWidget(_chessSizeLabel);
    chessLayout->addWidget(_chessSize);
    chessGroup->setLayout(chessLayout);

    QHBoxLayout* markerLayout = new QHBoxLayout;
    markerLayout->setAlignment(Qt::AlignLeft);
    markerLayout->addWidget(_markerSizeLabel);
    markerLayout->addWidget(_markerSize);
    markerGroup->setLayout(markerLayout);

    QVBoxLayout* caracsLayout = new QVBoxLayout;
    caracsLayout->addWidget(faceGroup);
    caracsLayout->addWidget(chessGroup);
    caracsLayout->addWidget(markerGroup);

    QHBoxLayout* buttonsLayout = new QHBoxLayout;
    buttonsLayout->setAlignment(Qt::AlignRight);
    buttonsLayout->addWidget(_ok);
    buttonsLayout->addWidget(_cancel);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(caracsLayout);
    mainLayout->addLayout(buttonsLayout);
    mainLayout->setSizeConstraint( QLayout::SetFixedSize );

    this->setLayout(mainLayout);
    this->setWindowFlags(this->windowFlags() & (~Qt::WindowContextHelpButtonHint));

    connect(_cancel, SIGNAL(clicked()), this, SLOT(reject()));
    connect(_ok, SIGNAL(clicked()), this, SLOT(accept()));
}
