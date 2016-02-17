#include "addobjectdialog.h"

AddObjectDialog::AddObjectDialog(QWidget *parent) :
    QDialog(parent)
{
    _objectName = new QLineEdit;
    _objectName->setText(tr("Name"));

    _objectPath = new QLineEdit;
    _objectPath->setText(tr(""));

    QPushButton* ok = new QPushButton(tr("Ok"));
    QPushButton* cancel = new QPushButton(tr("Cancel"));
    QPushButton* path = new QPushButton(QIcon("../rsc/icons/open-file.png"), "");

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
    mainLayout->addLayout(optionsLayout);
    mainLayout->addLayout(buttonsLayout);

    setLayout(mainLayout);

    setFixedSize(350, 100);
    setWindowTitle("Add object");

    connect(path, SIGNAL(clicked()), this, SLOT(updatePath()));

    connect(cancel, SIGNAL(clicked()), this, SLOT(reject()));
    connect(ok, SIGNAL(clicked()), this, SLOT(accept()));
}

void AddObjectDialog::updatePath()
{
    QString objectPath = QFileDialog::getOpenFileName(this, "Open 3D Object", "../rsc/objets3D/", "3D object (*.obj *.3ds *.stl)");
    if(objectPath != "")
        _objectPath->setText(objectPath);

}
