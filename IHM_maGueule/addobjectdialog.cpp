#include "addobjectdialog.h"

AddObjectDialog::AddObjectDialog(QWidget *parent) :
    QDialog(parent)
{
    m_objectName = new QLineEdit;
    m_objectName->setText(tr("name"));

    m_objectPath = new QLineEdit;
    m_objectPath->setText(tr(""));
    //m_objectPath->setEnabled(false);

    m_maskBox = new QCheckBox();

    QPushButton* ok = new QPushButton(tr("ok"));
    QPushButton* cancel = new QPushButton(tr("annuler"));
    QPushButton* path = new QPushButton("");

    QHBoxLayout* pathLayout = new QHBoxLayout;
    pathLayout->addWidget(m_objectPath);
    pathLayout->addWidget(path);

    QFormLayout* optionsLayout = new QFormLayout;
    optionsLayout->addRow(tr("nom de l'objet"), m_objectName);
    optionsLayout->addRow(tr("chemin d'acces de l'objet"), pathLayout);
    optionsLayout->addRow(tr("objet masque"), m_maskBox);

    QHBoxLayout* buttonsLayout = new QHBoxLayout;
    buttonsLayout->setAlignment(Qt::AlignRight);
    buttonsLayout->addWidget(ok);
    buttonsLayout->addWidget(cancel);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(optionsLayout);
    mainLayout->addLayout(buttonsLayout);

    setLayout(mainLayout);

    setFixedSize(300, 200);

    connect(path, SIGNAL(clicked()), this, SLOT(updatePath()));

    connect(cancel, SIGNAL(clicked()), this, SLOT(reject()));
    connect(ok, SIGNAL(clicked()), this, SLOT(accept()));
}

void AddObjectDialog::updatePath()
{
    QString objectPath = QFileDialog::getOpenFileName(this, "Ouvrir un objet 3D", "../rsc/objets3D/", "objets 3D (*.obj *.3ds *.stl)");
    if(objectPath != "")
        m_objectPath->setText(objectPath);

}
