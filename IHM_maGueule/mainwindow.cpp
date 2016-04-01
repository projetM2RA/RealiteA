#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    this->setWindowTitle(tr("Augmented Reality Of Neuroskeleton"));
    this->setWindowIcon(QIcon(":/icons/icon"));
    _objectID = 0;
    _nbrCam = WebcamDevice::webcamCount(); // a mettre au debut sinon conflit avec les webcams lancees
    _webcamDevice = new WebcamDevice(this);
    _fullScreen = false;
    _delete = false;
    _playVideo = false;
    _playCam = false;

    //    _file.open ("../rsc/angles.txt"); // utilisé à des fins de débug
    //    _file << "   rotX   " << "   rotY   " << "   rotZ   " << "   tX   " << "   tY   " << "   tZ   " << std::endl;

    setFirstWindow();

    this->resize(QDesktopWidget().availableGeometry(this).size() * 0.65);
}

MainWindow::~MainWindow()
{
    //_file.close();
    if(_delete)
    {
        for(int i = 0; i < NBR_DETECT; i++)
            delete _detectActions[i];
        delete _detectActions;

        delete _addObjectAction;
        delete _fullScreenAction;

        delete _mainView;

        delete _objectChoiceComboBox;
        delete _deleteObjectButton;
        delete _isPrintedBox;

        for(int i = 0; i < NBR_CHARACTERISTICS; i++)
            delete _objectCharacteristicsSpinSliders[i];

        delete _objectCharacteristicsSpinSliders;

        delete _sideView;
    }

    _webcamDevice->stop();
    _sleep(100); // on s'assure que le thread s'est bien arreté
    delete _webcamDevice;
}



// private slots

/*
 * ce membre sera appelé lorsque l'utilisateur va cliquer sur "start".
 * il va réllement lancer le programme en appelant notamment les membres suivant :
 * initOjectsList()
 * setMainWindow()
 * connectAll()
 */
bool MainWindow::start()
{
    int initCalib = 0;
    /*
     * meme si un QSplashScreen semble plus indiqué, ça ne fonctionne pas ici
     * (le chargement de chehra prend trop de ressources ?)
     * on utilise donc un QWidget sur lequel on applique un mask
     * à noter qu'y appliquer une "simple" image ne fonctionne pas non plus
     */
    //QSplashScreen splash(QPixmap(":/icons/splash"), Qt::WindowStaysOnTopHint);
    QWidget splash;
    QBitmap bit(":/icons/splash");
    splash.setMask(bit);
    splash.setFixedSize(bit.size());

    initCalib = _webcamDevice->initMatrix();

    if(initCalib == cancel)
        return _delete;

    splash.show();
    this->setCursor(QCursor(Qt::WaitCursor));

    if(initCalib == defaultCase)
    {
        this->initObjectsList(defaultCase);
        this->setMainWindow(defaultCase);
        this->createFullScreenWidget(defaultCase);
    }
    else if(initCalib == calibrationCase)
    {
        this->initObjectsList(calibrationCase);
        this->setMainWindow(calibrationCase);
        this->createFullScreenWidget(calibrationCase);
    }

    if(_webcamDevice->initModels())
        _detectActions[chehra]->setEnabled(true);
    else
        _detectActions[chehra]->setEnabled(false);

    this->connectAll();
    this->setShortcuts();

    this->setCursor(QCursor(Qt::ArrowCursor));
    this->show();
    splash.close();
    _delete = true;
}

void MainWindow::calibrateCamera()
{
    CalibrateDialog calibrateDialog(_webcamDevice->getWebcam());
    connect(_webcamDevice, SIGNAL(updateWebcam()), &calibrateDialog, SLOT(updateWebcam()));
    calibrateDialog.exec();
}

void MainWindow::addObject()
{
    _objectDialog = new AddObjectDialog(this);

    connect(_objectDialog, SIGNAL(setTemplate(int)), this, SLOT(addTemplate(int)));

    if(_objectDialog->exec() == QDialog::Rejected)
        return;

    QString objectPath = _objectDialog->getObjectPath();
    QString objectName = _objectDialog->getObjectName();

    this->addObject(objectName, objectPath);
    QString texPath = _objectDialog->getTexturePath();

    if(QFile::exists(texPath))
    {
        _objectsList[_objectsList.size() - 1]->applyTexture(texPath);
        _objectsList2[_objectsList2.size() - 1]->applyTexture(texPath);
    }
}

bool MainWindow::addObject(QString name, QString path)
{
    if(path == "" || name == "")
    {
        QMessageBox::warning(this, tr("Error loading object"), tr("The object's path and/or name have not been specified."));
        return false;
    }

    if(!QFile::exists(path))
    {
        QMessageBox::warning(this, tr("Error loading object"), "The object's path doesn't exist.\n(" + path + ")");
        return false;
    }

    _objectsList.push_back(new Our3DObject(path));
    _objectsList[0]->addChild(_objectsList[_objectsList.size() - 1]);
    _objectsList2.push_back(new Our3DObject(path));
    _objectsList2[0]->addChild(_objectsList2[_objectsList2.size() - 1]);

    int nbrObjects = _objectChoiceComboBox->itemText(0).mid(14).toInt();
    _objectChoiceComboBox->setItemText(0, "All objects : " + QString::number(nbrObjects + 1));
    _objectChoiceComboBox->addItem(name);

    return true;
}

void MainWindow::setTexture()
{
    if(_objectID <= 1)
        return;
    QString texPath = QFileDialog::getOpenFileName(this, "Open 3D Texture", "../rsc/objets3D/Textures/", "texture image (*.bmp)");
    if(!QFile::exists(texPath))
        return;
    _objectsList[_objectID]->applyTexture(texPath);
    _objectsList2[_objectID]->applyTexture(texPath);
}

/*
 * membre qui va parser le fichier objectTemplate.xml
 * (inclut dans les resources .qrc)
 * et va ajouter les différents modeles 3D avec leurs
 * caractéristiques propre à la scene
 */
void MainWindow::addTemplate(int templateID)
{
    QXmlStreamReader reader;
    QFile file(":/templates/objectsTemplate.xml");
    file.open(QFile::ReadOnly | QFile::Text);
    reader.setDevice(&file);
    QString name, path;

    while(!reader.atEnd() && !reader.hasError())
    {
        /* Read next element.*/
        reader.readNext();
        if(reader.tokenType() == QXmlStreamReader::StartDocument)
            continue;
        if(reader.tokenType() == QXmlStreamReader::StartElement)
        {
            if(reader.name() == "objectsTemplate")
                continue;
            if(reader.name() == "template")
            {
                QXmlStreamAttributes attributes = reader.attributes();
                if(attributes.value("id").toInt() == templateID)
                {
                    while(!(reader.tokenType() == QXmlStreamReader::EndElement && reader.name() == "template"))
                    {
                        if(reader.name() == "object" && reader.tokenType() == QXmlStreamReader::StartElement)
                        {
                            while(!(reader.tokenType() == QXmlStreamReader::EndElement && reader.name() == "object"))
                            {
                                if(reader.name() == "name" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    name = reader.text().toString();
                                }
                                if(reader.name() == "path" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    path = reader.text().toString();
                                }
                                /*
                                 * les templates on été créés avant qu'on mette en place la gestion des textures
                                 * il faudrait donc les rajouter dans le fichier .xml
                                 */

                                /*
                                if(reader.name() == "texture" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->applyTexture(reader.text().toString());
                                    _objectsList2[_objectsList2.size() - 1]->applyTexture(reader.text().toString());
                                }
                                */

                                if(reader.name() == "sx" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setSizeX(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setSizeX(reader.text().toInt());
                                }
                                if(reader.name() == "sy" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setSizeY(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setSizeY(reader.text().toInt());
                                }
                                if(reader.name() == "sz" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setSizeZ(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setSizeZ(reader.text().toInt());
                                }

                                if(reader.name() == "rx" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setRotX(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setRotX(reader.text().toInt());
                                }
                                if(reader.name() == "ry" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setRotY(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setRotY(reader.text().toInt());
                                }
                                if(reader.name() == "rz" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setRotZ(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setRotZ(reader.text().toInt());
                                }

                                if(reader.name() == "tx" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setTransX(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setTransX(reader.text().toInt());
                                }
                                if(reader.name() == "ty" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setTransY(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setTransY(reader.text().toInt());
                                }
                                if(reader.name() == "tz" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setTransZ(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setTransZ(reader.text().toInt());
                                }

                                if(reader.name() == "alpha" && reader.tokenType() == QXmlStreamReader::StartElement)
                                {
                                    while(reader.tokenType() != QXmlStreamReader::Characters)
                                        reader.readNext();
                                    _objectsList[_objectsList.size() - 1]->setAlpha(reader.text().toInt());
                                    _objectsList2[_objectsList2.size() - 1]->setAlpha(reader.text().toInt());
                                }

                                reader.readNext();
                            }

                            if(name != "" && path != "") // je sais pas trop où le mettre ce if
                            {
                                if(!this->addObject(name, path))
                                    return;
                                name = "";
                                path = "";
                            }
                        }
                        reader.readNext();
                    }
                    reader.clear();
                    return;
                }
            }
        }
    }
    // Error handling.
    if(reader.hasError())
    {
        QMessageBox::critical(this,
                              "QXSRExample::parseXML",
                              reader.errorString(),
                              QMessageBox::Ok);
    }
    /* Removes any device() or data from the reader
        * and resets its internal state to the initial state. */
    reader.clear();
}

/*
 * membre qui va mettre à jour les valeurs des sliders
 * lorsque l'utilisateur change l'objet selectionné dans
 * la spinbox
 * (il va aussi relier correctement les signaux des sliders
 * aux slots de l'objet correspondant)
 */
void MainWindow::updateObjectCharacteristics(int objectID)
{
    if(objectID == _objectID)
        return;
    // _objectList[0] <=> globalMat
    // _objectList[1] <=> hud

    Our3DObject* object = _objectsList[_objectID];
    Our3DObject* object2 = _objectsList2[_objectID];

    if(_objectID != 1)
    {
        disconnect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
        disconnect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
        disconnect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
        disconnect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
        disconnect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
        disconnect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
        disconnect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

        //////////////////////////////////////////////////

        disconnect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object2, SLOT(setSizeX(int)));
        disconnect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object2, SLOT(setSizeY(int)));
        disconnect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object2, SLOT(setSizeZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object2, SLOT(setRotX(int)));
        disconnect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object2, SLOT(setRotY(int)));
        disconnect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object2, SLOT(setRotZ(int)));

        disconnect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object2, SLOT(setTransX(int)));
        disconnect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object2, SLOT(setTransY(int)));
        disconnect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object2, SLOT(setTransZ(int)));
    }
    else
    {
        _objectCharacteristicsSpinSliders[sizeX]->setEnabled(true);
        _objectCharacteristicsSpinSliders[sizeY]->setEnabled(true);
        _objectCharacteristicsSpinSliders[sizeZ]->setEnabled(true);

        _objectCharacteristicsSpinSliders[rotX]->setEnabled(true);
        _objectCharacteristicsSpinSliders[rotY]->setEnabled(true);
        _objectCharacteristicsSpinSliders[rotZ]->setEnabled(true);

        _objectCharacteristicsSpinSliders[transX]->setEnabled(true);
        _objectCharacteristicsSpinSliders[transY]->setEnabled(true);
        _objectCharacteristicsSpinSliders[transZ]->setEnabled(true);

        _isPrintedBox->setEnabled(true);
    }

    disconnect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object2, SLOT(setAlpha(int)));



    _objectID = objectID;

    object = _objectsList[_objectID];
    object2 = _objectsList2[_objectID];


    _isPrintedBox->setChecked(object->getNodeMask());
    _isPrintedBox2->setChecked(object2->getNodeMask());

    if(_objectID == 1)
    {
        _objectCharacteristicsSpinSliders[sizeX]->setValue(0);
        _objectCharacteristicsSpinSliders[sizeY]->setValue(0);
        _objectCharacteristicsSpinSliders[sizeZ]->setValue(0);

        _objectCharacteristicsSpinSliders[rotX]->setValue(0);
        _objectCharacteristicsSpinSliders[rotY]->setValue(0);
        _objectCharacteristicsSpinSliders[rotZ]->setValue(0);

        _objectCharacteristicsSpinSliders[transX]->setValue(0);
        _objectCharacteristicsSpinSliders[transY]->setValue(0);
        _objectCharacteristicsSpinSliders[transZ]->setValue(0);

        _objectCharacteristicsSpinSliders[sizeX]->setEnabled(false);
        _objectCharacteristicsSpinSliders[sizeY]->setEnabled(false);
        _objectCharacteristicsSpinSliders[sizeZ]->setEnabled(false);

        _objectCharacteristicsSpinSliders[rotX]->setEnabled(false);
        _objectCharacteristicsSpinSliders[rotY]->setEnabled(false);
        _objectCharacteristicsSpinSliders[rotZ]->setEnabled(false);

        _objectCharacteristicsSpinSliders[transX]->setEnabled(false);
        _objectCharacteristicsSpinSliders[transY]->setEnabled(false);
        _objectCharacteristicsSpinSliders[transZ]->setEnabled(false);

        _isPrintedBox->setEnabled(false);

        _objectCharacteristicsSpinSliders[alpha]->setValue(object2->getAlpha());
        connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object2, SLOT(setAlpha(int)));
    }
    else
    {
        _objectCharacteristicsSpinSliders[sizeX]->setValue(object->getSizeX());
        _objectCharacteristicsSpinSliders[sizeY]->setValue(object->getSizeY());
        _objectCharacteristicsSpinSliders[sizeZ]->setValue(object->getSizeZ());

        _objectCharacteristicsSpinSliders[rotX]->setValue(object->getRotX());
        _objectCharacteristicsSpinSliders[rotY]->setValue(object->getRotY());
        _objectCharacteristicsSpinSliders[rotZ]->setValue(object->getRotZ());

        _objectCharacteristicsSpinSliders[transX]->setValue(object->getTransX());
        _objectCharacteristicsSpinSliders[transY]->setValue(object->getTransY());
        _objectCharacteristicsSpinSliders[transZ]->setValue(object->getTransZ());

        _objectCharacteristicsSpinSliders[alpha]->setValue(object->getAlpha());

        connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
        connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
        connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

        connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
        connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
        connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

        connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
        connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
        connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));
        connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

        //////////////////////////////////////////////////

        connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object2, SLOT(setSizeX(int)));
        connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object2, SLOT(setSizeY(int)));
        connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object2, SLOT(setSizeZ(int)));

        connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object2, SLOT(setRotX(int)));
        connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object2, SLOT(setRotY(int)));
        connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object2, SLOT(setRotZ(int)));

        connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object2, SLOT(setTransX(int)));
        connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object2, SLOT(setTransY(int)));
        connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object2, SLOT(setTransZ(int)));

        connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object2, SLOT(setAlpha(int)));
    }
}

/*
 * membre qui va supprimer l'objet selectionné dans
 * la spinbox de la scene 3D
 * A noter qu'il ne fera rien sur la video
 * et qu'il supprimera tous les objets si la spinbox
 * est sur le "all objects"
 */
void MainWindow::removeObject()
{
    if(_objectID == 1)
        return;
    else if(_objectID == 0)
    {
        for(int i = _objectsList.size() - 1; i > 1; --i)
        {
            _objectID = i;
            this->removeObject();
        }
        _objectID = 1;
        this->updateObjectCharacteristics(0);
    }
    else
    {
        disconnect(_objectChoiceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObjectCharacteristics(int)));
        {
            Our3DObject* object = _objectsList[_objectID];
            Our3DObject* object2 = _objectsList2[_objectID];

            disconnect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
            disconnect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
            disconnect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

            disconnect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
            disconnect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
            disconnect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

            disconnect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
            disconnect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
            disconnect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

            disconnect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

            //////////////////////////////////////////////////

            disconnect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object2, SLOT(setSizeX(int)));
            disconnect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object2, SLOT(setSizeY(int)));
            disconnect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object2, SLOT(setSizeZ(int)));

            disconnect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object2, SLOT(setRotX(int)));
            disconnect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object2, SLOT(setRotY(int)));
            disconnect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object2, SLOT(setRotZ(int)));

            disconnect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object2, SLOT(setTransX(int)));
            disconnect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object2, SLOT(setTransY(int)));
            disconnect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object2, SLOT(setTransZ(int)));

            disconnect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object2, SLOT(setAlpha(int)));
        }

        _objectsList[0]->removeChild(_objectsList[_objectID]);
        _objectsList.erase(_objectsList.begin() += _objectID);
        _objectsList2[0]->removeChild(_objectsList2[_objectID]);
        _objectsList2.erase(_objectsList2.begin() += _objectID);
        int nbrObjects = _objectChoiceComboBox->itemText(0).mid(14).toInt();
        _objectChoiceComboBox->setItemText(0, "All objects : " + QString::number(nbrObjects - 1));
        _objectChoiceComboBox->removeItem(_objectID);

        if(_objectID >= _objectsList.size())
            _objectID--;
        {
            Our3DObject* object = _objectsList[_objectID];
            Our3DObject* object2 = _objectsList2[_objectID];

            _isPrintedBox->setChecked(object->getNodeMask());
            _isPrintedBox2->setChecked(object2->getNodeMask());

            if(_objectID == 1)
            {
                _objectCharacteristicsSpinSliders[sizeX]->setValue(0);
                _objectCharacteristicsSpinSliders[sizeY]->setValue(0);
                _objectCharacteristicsSpinSliders[sizeZ]->setValue(0);

                _objectCharacteristicsSpinSliders[rotX]->setValue(0);
                _objectCharacteristicsSpinSliders[rotY]->setValue(0);
                _objectCharacteristicsSpinSliders[rotZ]->setValue(0);

                _objectCharacteristicsSpinSliders[transX]->setValue(0);
                _objectCharacteristicsSpinSliders[transY]->setValue(0);
                _objectCharacteristicsSpinSliders[transZ]->setValue(0);

                _objectCharacteristicsSpinSliders[sizeX]->setEnabled(false);
                _objectCharacteristicsSpinSliders[sizeY]->setEnabled(false);
                _objectCharacteristicsSpinSliders[sizeZ]->setEnabled(false);

                _objectCharacteristicsSpinSliders[rotX]->setEnabled(false);
                _objectCharacteristicsSpinSliders[rotY]->setEnabled(false);
                _objectCharacteristicsSpinSliders[rotZ]->setEnabled(false);

                _objectCharacteristicsSpinSliders[transX]->setEnabled(false);
                _objectCharacteristicsSpinSliders[transY]->setEnabled(false);
                _objectCharacteristicsSpinSliders[transZ]->setEnabled(false);

                _isPrintedBox->setEnabled(false);

                _objectCharacteristicsSpinSliders[alpha]->setValue(object2->getAlpha());
                connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object2, SLOT(setAlpha(int)));
            }
            else
            {
                _objectCharacteristicsSpinSliders[sizeX]->setValue(object->getSizeX());
                _objectCharacteristicsSpinSliders[sizeY]->setValue(object->getSizeY());
                _objectCharacteristicsSpinSliders[sizeZ]->setValue(object->getSizeZ());

                _objectCharacteristicsSpinSliders[rotX]->setValue(object->getRotX());
                _objectCharacteristicsSpinSliders[rotY]->setValue(object->getRotY());
                _objectCharacteristicsSpinSliders[rotZ]->setValue(object->getRotZ());

                _objectCharacteristicsSpinSliders[transX]->setValue(object->getTransX());
                _objectCharacteristicsSpinSliders[transY]->setValue(object->getTransY());
                _objectCharacteristicsSpinSliders[transZ]->setValue(object->getTransZ());

                _objectCharacteristicsSpinSliders[alpha]->setValue(object->getAlpha());

                connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
                connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
                connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

                connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
                connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
                connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

                connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
                connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
                connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

                connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

                //////////////////////////////////////////////////

                connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object2, SLOT(setSizeX(int)));
                connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object2, SLOT(setSizeY(int)));
                connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object2, SLOT(setSizeZ(int)));

                connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object2, SLOT(setRotX(int)));
                connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object2, SLOT(setRotY(int)));
                connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object2, SLOT(setRotZ(int)));

                connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object2, SLOT(setTransX(int)));
                connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object2, SLOT(setTransY(int)));
                connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object2, SLOT(setTransZ(int)));

                connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object2, SLOT(setAlpha(int)));
            }
        }
        connect(_objectChoiceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObjectCharacteristics(int)));
    }
}

/*
 * membre appelé lorsque l'utilisateur change
 * le mode de detection
 */
void MainWindow::updateDetectMode()
{
    _detectionLabel2->setVisible(false);

    for(int i = 0; i < NBR_DETECT; i++)
    {
        if(_detectActions[i]->isChecked())
        {
            _webcamDevice->switchMode(i);

            switch(i)
            {
            case 0:
                _detectionLabel->setText("No active detection.");
                break;
            case 1:
                _detectionLabel->setText("Face detection");
                break;
            case 2:
                _detectionLabel->setText("Board detection");
                break;
            case 3:
                _detectionLabel->setText("Marker detection");
                break;
            default:
                _detectionLabel->setText("No active detection.");
                break;
            }
            return;
        }
    }
}

/*
 * membre qui va afficher s'il y a detection ou non
 */
void MainWindow::updateDetectLabel(bool detect)
{
    _detectionLabel2->setVisible(true);

    for(int i = 0; i < NBR_DETECT; i++)
    {
        if(_detectActions[i]->isChecked())
        {
            switch(i)
            {
            case 0:
                _detectionLabel2->setText("");
                break;
            case 1:
                if(detect)
                    _detectionLabel2->setText("-> face detected");
                else
                    _detectionLabel2->setText("-> no detection");
                break;
            case 2:
                if(detect)
                    _detectionLabel2->setText("-> board detected");
                else
                    _detectionLabel2->setText("-> no detection");
                break;
            case 3:
                if(detect)
                    _detectionLabel2->setText("-> marker detected");
                else
                    _detectionLabel2->setText("-> no detection");
                break;
            default:
                _detectionLabel2->setText("");
                break;
            }
            return;
        }
    }
}

/*
 * membre appelé lorsque l'utilisateur change
 * d'entrée video (autre camera ou fichier video)
 */
void MainWindow::switchInput()
{
    for(int i = 0; i < _nbrCam; i++)
    {
        if(_webcamActions[i]->isChecked())
        {
            _webcamDevice->switchInput(i, false);
            _pause->setEnabled(false);
            _play->setEnabled(false);
            _fast->setEnabled(false);
            _slow->setEnabled(false);
            return;
        }
    }
    _webcamDevice->switchInput(-1, false);
    _play->setEnabled(true);
}

/*
 * membre appelé lorsque l'utilisateur choisi
 * d'afficher ou non un objet 3D dans la scene
 */
void MainWindow::displayObjectInScene(bool display)
{
    if(_objectID == 0)
        displayObjects(!display);
    else
    {
        if(display)
            _objectsList[_objectID]->setNodeMask(display);
        else
            _objectsList[_objectID]->setNodeMask(display);
    }
}

/*
 * membre appelé lorsque l'utilisateur choisi
 * d'afficher ou non un objet 3D dans la sideView
 */
void MainWindow::displayObjectInSideView(bool display)
{
    if(_objectID == 0)
        displayObjectsInSideView(!display);
    else
    {
        if(display)
            _objectsList2[_objectID]->setNodeMask(display);
        else
            _objectsList2[_objectID]->setNodeMask(display);
    }
}

/*
 * membre appelé lorsque l'utilisateur choisi d'afficher
 * la vue principale en full screen.
 * A noter que comme on utilise un autre widget,
 * s'il fait Alt+f4 en full screen, le widget disparaitra
 * et il ne pourra plus passer en full screen
 * (#truanderie)
 */
void MainWindow::displayFullScreen()
{
    if(!_fullScreen)
        _fullScreenWidget->showFullScreen();
    else
    {
        _fullScreenWidget->showNormal();
        _fullScreenWidget->move(-5000, -5000); //(necessaire pour cacher correctement le widget de full screen)
        _fullScreenWidget->hide();
    }

    _fullScreenView->update();

    _fullScreen = !_fullScreen;
}

/* /!\ membre le plus important d'un point de vue technique /!\
 *
 * ce membre est connecté au signal emis à chaque frame par le webcamDevice
 * qui transmets les matrice rotation et translation (RT) du repere 3D réel
 * par rapport à la caméra.
 *
 * il va donc appliquer ces matrices R et T à la mainMat
 * (voir graphe de scene, annexe 2 du rapport)
 *
 * normalement, le "simple" code commenté ci-dessous fonctionne
 * (ça a été testé et il fonctionne parfaitement avec la mire)
 * mais pour une raison encore inconnue, il ne fonctionne pas
 * à partir d'une certaine rotation du visage selon l'axe y.
 * Plusieurs tests ont été effectués et au final les valeurs
 * de rotations et translation ont été modifiées de maniere
 * arbitraire lors de la detection du visage afin d'obtenir
 * un résultat qui semble cohérent. (#truanderie)
 *
 * A noter qu'on n'utilise pas exactement le morceau de code
 * commenté ci-dessous pour la detection de la mire car on a
 * appliqué un effet miroir à notre texture video
 * (cf this->initObjectList)
 *
 */

/*
    double t3 = tvecs.at<double>(2, 0);
    double t2 = tvecs.at<double>(1, 0) + t3 / _corrector;
    double t1 = tvecs.at<double>(0, 0);

    double r11 = rotVec.at<double>(0, 0);
    double r12 = rotVec.at<double>(0, 1);
    double r13 = rotVec.at<double>(0, 2);
    double r21 = rotVec.at<double>(1, 0);
    double r22 = rotVec.at<double>(1, 1);
    double r23 = rotVec.at<double>(1, 2);
    double r31 = rotVec.at<double>(2, 0);
    double r32 = rotVec.at<double>(2, 1);
    double r33 = rotVec.at<double>(2, 2);

    osg::Matrixd matrixR; // rotation (transposee de rotVec)
    matrixR.set(
                r11,	r21,    r31,    0,
                r12,	r22,    r32,    0,
                r13,	r23,    r33,    0,
                0,		0,		0,		1);


    osg::Matrixd matrixT; // translation
    matrixT.makeTranslate(t1, t2, t3);

    osg::Matrixd matrix90; // rotation de repere entre opencv et osg
    matrix90.makeRotate(osg::Quat(osg::DegreesToRadians(90.0f), osg::Vec3d(1.0, 0.0, 0.0)));

    _mainMat->setMatrix(matrixR * matrixT * matrix90);
 */
void MainWindow::updateSceneRT(cv::Mat rotVec, cv::Mat tvecs)
{
    bool rpy = false; // utilise-ton roll/pitch/yaw de chehra (cf WebcamDevice.cpp)

    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    double rx, ry, rz;
    double t1, t2, t3;

    cv::Mat r;

    if(!rpy)
    {
        r11 = rotVec.at<double>(0, 0);
        r12 = rotVec.at<double>(0, 1);
        r13 = rotVec.at<double>(0, 2);
        r21 = rotVec.at<double>(1, 0);
        r22 = rotVec.at<double>(1, 1);
        r23 = rotVec.at<double>(1, 2);
        r31 = rotVec.at<double>(2, 0);
        r32 = rotVec.at<double>(2, 1);
        r33 = rotVec.at<double>(2, 2);

        cv::Rodrigues(rotVec, r);
    }

    t3 = tvecs.at<double>(2, 0);
    t2 = tvecs.at<double>(1, 0) + t3 / _corrector; // #truanderie
    t1 = tvecs.at<double>(0, 0);

    if(_webcamDevice->getMode() == chehra)
    {
        if(rpy)
        {
            rx = rotVec.at<double>(0, 0) * PI / 180.0;
            ry = rotVec.at<double>(1, 0) * PI / 180.0;
            rz = rotVec.at<double>(2, 0) * PI / 180.0;

            ry += atan(t1 / t3);
        }
        else
        {
            rx = r.at<double>(2, 0);
            ry = r.at<double>(1, 0) + PI;
            rz = r.at<double>(0, 0);

            if(ry > PI)
            {
                rx *= -1;
                rz *= -1;
                ry -= 2 * PI;
            }

            rx -= 2 * atan(t2 / t3);
            ry -= 2 * atan(t1 / t3);
            ry *= -1;
            rz *= 0.75; // la rotation autour de z semble excessive ; no idea why

            if(ry > 0 && rz > 0)
                ry -= rz / 4;
            else if(ry < 0 && rz > 0)
                ry += rz / 4;
            else if(ry < 0 && rz < 0)
                ry -= rz / 4;
            else if(ry > 0 && rz < 0)
                ry += rz / 4;
            if(rz > 0)
                rx += rz / 4;
            else if(rz < 0)
                rx -= rz / 4;
            if(ry < 0)
            {
                ry += rx / 4;
                //rx -= rz / 2;
            }
        }

        /* utilisé à des fins de debug
        _file << (rx*180)/PI << " ; " << (ry*180)/PI << " ; "  << (rz*180)/PI << " ; "  << std::endl;
        std::cout << "r x : " << rx * 180 / PI << std::endl;
        std::cout << "r y : " << ry * 180 / PI << std::endl;
        std::cout << "r z : " << rz * 180 / PI << std::endl;
        std::cout << std::endl << std::endl;
        std::cout << "t x : " << t1 << std::endl;
        std::cout << "t y : " << t2 << std::endl;
        std::cout << "t z : " << t3 << std::endl;
        std::cout << std::endl << std::endl;
        */

        osg::Matrixd matrixR; // rotation corrigee
        matrixR.makeRotate(rx, osg::Vec3d(1.0, 0.0, 0.0), ry, osg::Vec3d(0.0, 1.0, 0.0), rz, osg::Vec3d(0.0, 0.0, 1.0));

        osg::Matrixd matrixT; // translation
        matrixT.makeTranslate(t1, t2, t3);

        osg::Matrixd matrix90; // rotation de repere entre opencv et osg
        matrix90.makeRotate(osg::Quat(osg::DegreesToRadians(+90.0f), osg::Vec3d(1.0, 0.0, 0.0)));

        _mainMat->setMatrix(matrixR * matrixT * matrix90);
        _mainMat2->setMatrix(matrixR * matrixT * matrix90);
    }
    else
    {
        if(rpy) // en théorie non necessaire, utilisé à des fins de test en utilisant les membres de chehra sur le visage
        {
            r11 = rotVec.at<double>(0, 0);
            r12 = rotVec.at<double>(0, 1);
            r13 = rotVec.at<double>(0, 2);
            r21 = rotVec.at<double>(1, 0);
            r22 = rotVec.at<double>(1, 1);
            r23 = rotVec.at<double>(1, 2);
            r31 = rotVec.at<double>(2, 0);
            r32 = rotVec.at<double>(2, 1);
            r33 = rotVec.at<double>(2, 2);

            cv::Rodrigues(rotVec, r); // si roll/pitch/yaw avec chehra
        }

        rx = atan2(r32, r33);
        ry = atan2(r21, r11);
        rz = -atan2(-r31, sqrt((r32 * r32) + (r33 * r33)));

        osg::Matrixd matrixR; // rotation corrigee
        matrixR.makeRotate(rx, osg::Vec3d(1.0, 0.0, 0.0), rz, osg::Vec3d(0.0, 1.0, 0.0), -ry, osg::Vec3d(0.0, 0.0, 1.0));

        osg::Matrixd matrixT; // translation
        matrixT.makeTranslate(-t1, t2, t3);

        osg::Matrixd matrix90; // rotation de repere entre opencv et osg
        matrix90.makeRotate(osg::Quat(osg::DegreesToRadians(-90.0f), osg::Vec3d(1.0, 0.0, 0.0)));

        _mainMat->setMatrix(matrixR * matrixT * matrix90);
        _mainMat2->setMatrix(matrixR * matrixT * matrix90);
    }
}

/*
 * les membres suivant servent à la lecture des videos
 * (play/pause fonctionne aussi pour les flux webcam)
 */
void MainWindow::playNpause()
{
    if(_playCam)
    {
        _backBegin->setVisible(false);
        _play->setVisible(false);
        _pause->setVisible(false);
        _fast->setVisible(false);
        _slow->setVisible(false);
        _videoGroup->setVisible(false);
        _camGroup->setVisible(true);

        _playCam = false;
    }
    else
    {
        if(_playVideo)
        {
            _pause->setVisible(true);
            _pause->setEnabled(true);
            _play->setVisible(false);
            _play->setEnabled(false);
            _fast->setEnabled(true);
            _slow->setEnabled(true);
            _webcamDevice->play();

            _playVideo = false;
        }
        else
        {
            _pause->setVisible(false);
            _pause->setEnabled(false);
            _play->setVisible(true);
            _play->setEnabled(true);
            _fast->setEnabled(false);
            _slow->setEnabled(false);
            _webcamDevice->pause();

            _playVideo = true;
        }
    }

}

void MainWindow::back2Begin()
{
    _backBegin->setVisible(true);
    _backBegin->setEnabled(true);
    _play->setVisible(false);
    _pause->setVisible(false);
    _fast->setVisible(false);
    _slow->setVisible(false);
}

void MainWindow::freezeButtons()
{
    _backBegin->setVisible(false);
    _pause->setEnabled(false);
    _play->setEnabled(false);
    _fast->setEnabled(false);
    _slow->setEnabled(false);
}

void MainWindow::playVideo()
{
    _camGroup->setVisible(false);
    _playVideo = true;
    _playCam = false;
    _play->setVisible(true);
    _play->setEnabled(true);
    _backBegin->setVisible(false);
    _pause->setVisible(false);
    _fast->setVisible(true);
    _fast->setEnabled(false);
    _slow->setVisible(true);
    _slow->setEnabled(false);
    _videoGroup->setVisible(true);
}

void MainWindow::playCam()
{
    _videoGroup->setVisible(false);
    _playVideo = false;
    _playCam = true;
    _backBegin->setVisible(false);
    _play->setVisible(false);
    _pause->setVisible(false);
    _fast->setVisible(false);
    _slow->setVisible(false);
    _camGroup->setVisible(true);
}

/*
 * les membre suivant servent à l'affichage
 * des menus "about" des bibliotheques utilisées
 * et du projet.
 */
void MainWindow::displayAboutCV()
{
    QMessageBox aboutCVMessageBox(QMessageBox::NoIcon, "About OpenCV",
                                  "",
                                  QMessageBox::Ok, this);
    QSpacerItem* horizontalSpacer = new QSpacerItem(5000, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

    aboutCVMessageBox.setText("<b><big>About OpenCV</big></b><br><br>"
                              "This program uses OpenCV version 2.4.11.<br><br>"
                              "<a href=http://opencv.org/about.html>OpenCV</a> (Open Source Computer Vision Library) is an open source computer vision and machine learning software library."
                              "OpenCV was built to provide a common infrastructure for computer vision applications and to accelerate the use of machine perception in the commercial products."
                              "Being a BSD-licensed product, OpenCV makes it easy for businesses to utilize and modify the code.<br><br>"
                              "The library has more than 2500 optimized algorithms, which includes a comprehensive set of both classic and state-of-the-art computer vision and machine learning algorithms."
                              "These algorithms can be used to detect and recognize faces, identify objects, classify human actions in videos, track camera movements, track moving objects, extract 3D models of objects, produce 3D point clouds from stereo cameras, stitch images together to produce a high resolution image of an entire scene, find similar images from an image database, remove red eyes from images taken using flash, follow eye movements, recognize scenery and establish markers to overlay it with augmented reality, etc."
                              "OpenCV has more than 47 thousand people of user community and estimated number of downloads exceeding <a href=https://sourceforge.net/projects/opencvlibrary/files/stats/timeline?dates=2001-09-20+to+2013-09-26>7 million</a>."
                              "The library is used extensively in companies, research groups and by governmental bodies.<br><br>"
                              "Along with well-established companies like Google, Yahoo, Microsoft, Intel, IBM, Sony, Honda, Toyota that employ the library, there are many startups such as Applied Minds, VideoSurf, and Zeitera, that make extensive use of OpenCV."
                              "OpenCV’s deployed uses span the range from stitching streetview images together, detecting intrusions in surveillance video in Israel, monitoring mine equipment in China, helping robots navigate and pick up objects at Willow Garage, detection of swimming pool drowning accidents in Europe, running interactive art in Spain and New York, checking runways for debris in Turkey, inspecting labels on products in factories around the world on to rapid face detection in Japan.<br><br>"
                              "It has C++, C, Python, Java and MATLAB interfaces and supports Windows, Linux, <a href=http://opencv.org/platforms/android.html>Android</a> and Mac OS."
                              "OpenCV leans mostly towards real-time vision applications and takes advantage of MMX and SSE instructions when available."
                              "A full-featured <a href=http://opencv.org/platforms/cuda.html>CUDA</a> and OpenCL interfaces are being actively developed right now."
                              "There are over 500 algorithms and about 10 times as many functions that compose or support those algorithms."
                              "OpenCV is written natively in C++ and has a templated interface that works seamlessly with STL containers.<br><br>"
                              "<a href=http://itseez.com>OpenCV Developers Team</a> :<br><br>");

    aboutCVMessageBox.setIconPixmap(QPixmap(":/icons/opencv"));
    QGridLayout* layout = (QGridLayout*)aboutCVMessageBox.layout();
    QLabel* logo = new QLabel("");
    logo->setPixmap(QPixmap(":/icons/itseez"));
    layout->addWidget(logo, 1, 2, 1, 1);

    layout->addItem(horizontalSpacer, layout->rowCount(), 0, 1, layout->columnCount());
    //    aboutCVMessageBox.setFixedSize(800, 600);
    aboutCVMessageBox.exec();
}

void MainWindow::displayAboutOsg()
{
    QMessageBox aboutOSGMessageBox(QMessageBox::NoIcon, "About OpenSceneGraph",
                                   "",
                                   QMessageBox::Ok, this);
    QSpacerItem* horizontalSpacer = new QSpacerItem(5000, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

    aboutOSGMessageBox.setText("<b><big>About OpenSceneGraph</big></b><br><br>"
                               "This program uses OpenSceneGraph version 3.4.0.<br><br>"
                               "<b>Usage and Markets</b><br>"
                               "The OpenSceneGraph is open source, real-time graphics middle-ware used by application developers in fields that range from visual simulation (flight, marine, vehicle, space simulator) to virtual and augmented reality, to medical and scientific visualisation, to education and games.<br><br>"
                               "<b>Cross Platform</b><br>"
                               "The OpenSceneGraph is cross platform running on small devices such as embedded graphics platforms to phones, tablets that use OpenGL ES,  to laptops and desktops using OpenGL all the way up to dedicated image generator clusters used in full scale simulators and immersive 3D displays.<br><br>"
                               "<b>Licensing</b><br>"
                               "The OpenSceneGraph is published under the OpenSceneGraph Public License, which is a relaxation of the Less GNU Public License (LGPL) that permits usage in commercial application that require static linking or embed systems.<br><br>"
                               "<b>Technology</b><br>"
                               "The OpenSceneGraph is written in Standard C++, taking advantage of the standard template library (STL) for containers.  The software uses the scene graph approach to representing 3D worlds as a graph of node that logical and spatially group subgraphs for behaviour and high performance.<br>"
                               "OpenGL 1.0 through to OpenGL 4.2, and OpenGL ES 1.1 and 2.0 are supported making it possible to support both old hardware and operating systems through to the latest mobile devices and all the features of cutting edge desktop graphics systems thanks to the software run time extension checking.<br>"
                               "Design Patterns are used throughout the software making it easier to maintain and understand how our software works as well as providing a good example of usage. The software is kept modular and extensible enabling end users to only utilize the components they need and to allow customisation when required.<br>"
                               "The key strengths of OpenSceneGraph are :<br>"
                               "<b><ul><li>Performance</li><li>Productivity</li><li>Database loaders</li><li>Node Kits</li><li>Portability</li><li>Scalability</li><li>Multi-language support</li></ul></b><br>"
                               "see more <a href=http://www.openscenegraph.org/index.php/about/features>here</a>.<br><br>"
                               "<a href=http://www.openscenegraph.org/index.php/about/licensing>Licensing</a> <a href=http://www.openscenegraph.org/index.php/about/history>History</a> <a href=http://www.openscenegraph.org/index.php/about/contributors>Contributors</a>");
    aboutOSGMessageBox.setIconPixmap(QPixmap(":/icons/osg"));

    QGridLayout* layout = (QGridLayout*)aboutOSGMessageBox.layout();
    layout->addItem(horizontalSpacer, layout->rowCount(), 0, 1, layout->columnCount());
    //    aboutCVMessageBox.setFixedSize(800, 600);
    aboutOSGMessageBox.exec();
}

void MainWindow::displayAboutChehra()
{
    QMessageBox aboutOSGMessageBox(QMessageBox::NoIcon, "About Chehra",
                                   "",
                                   QMessageBox::Ok, this);
    QSpacerItem* horizontalSpacer = new QSpacerItem(5000, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

    aboutOSGMessageBox.setText("<b><big>About Chehra</big></b><br><br>"
                               "This program uses Chehra version 0.1.<br><br>"
                               "Chehra (meaning \"face\" in Hindi) is a fully-automatic real-time face and eyes landmark detection and tracking software capable of handling faces under uncontrolled natural setting.<br>"
                               "It has been developed by <a href=https://wp.doc.ic.ac.uk/szafeiri>Akshay Asthana</a> and <a href=https://sites.google.com/site/akshayasthana>Stefanos Zafeiriou</a> at Imperial College London.<br>"
                               "For any further questions, please email me at <a href=mailto:chehra.team@gmail.com>chehra.team@gmail.com</a>.<br><br>"
                               "This software has been made available for non-commercial and academic purposes only.<br><br><br>"
                               "see also : <a href=http://ibug.doc.ic.ac.uk/media/uploads/aasthanacvpr2014.pdf>Incremental Face Alignment in the Wild</a>.");
    aboutOSGMessageBox.setIconPixmap(QPixmap(":/icons/chehra").scaledToWidth(128));

    QGridLayout* layout = (QGridLayout*)aboutOSGMessageBox.layout();
    layout->addItem(horizontalSpacer, layout->rowCount(), 0, 1, layout->columnCount());
    //    aboutCVMessageBox.setFixedSize(800, 600);
    aboutOSGMessageBox.exec();
}

void MainWindow::displayAbout()
{
    QMessageBox aboutOSGMessageBox(QMessageBox::NoIcon, "About",
                                   "",
                                   QMessageBox::Ok, this);
    QSpacerItem* horizontalSpacer = new QSpacerItem(5000, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

    aboutOSGMessageBox.setText("<b><big>About</big></b><br><br>"
                               "This is a software realised by Nicolas Charlet, Vivien Fragni and Leo Ricouard as a school project supervised by <a href=http://michael.aron.free.fr>Michael Aron</a> at the ISEN Brest. "
                               "It was meant to be a first approach to computer vision through augmented reality with medical applications.<br><br>"
                               "The point of this project was to segment IRM pictures of a brain in order to reconstitute it in a 3D model with 3DSlicer. "
                               "This model is then displayed on the patient's head using augmented reality.<br>"
                               "In the end, many features linked to this subject were added to the software. "
                               "Here's the exhaustive list of this software's features : <br>"
                               "<ul>"
                               "<li><b>Camera calibration</b> : In order to have the best projection, it is necessary to have the intrinsics parameter of the camera. "
                               "This is why this software allows the user to calibrate himself his camera (provided he has a chessboard). "
                               "It is also possible to use a default matrix.</li>"
                               "<li><b>Chessboard detection</b> : It is possible to choose to project 3D models on a chessboard ; chessboard detection being more accurate than face detection.</li>"
                               "<li><b>Face detection</b> : The main use of this software. The 3D models are projected on the user's face.</li>"
                               "<li><b>Custom marker detection</b> : The user can apply the projection on custom markers displaying differents models. (not completely functional for now).</li>"
                               "<li><b>Video and webcam input</b> : The user can either choose to read a video from a file or a webcam (up to 10 differents webcam). "
                               "Note than in the current state of things, the intrinsic matrix defined at the launching will be use in all cases.</li>"
                               "<li><b>Possibility to use any 3D model</b> : The software can import .obj, .3ds, .stl, .osg and .osgt models. "
                               "Note than 4 templates have been created, importing frequently used models.</li>"
                               "<li><b>Possibility to resize, rotate and translate the 3D models</b> : The user can adapt the model's characteristics to the scene</li>"
                               "<li><b>Possibility to set transparency and textures to the 3D models</b> : The user can customize the model with textures or can use it as a mask by setting transparency to 0.</li>"
                               "</ul><br>"
                               "The source code is available and free to use.");
    aboutOSGMessageBox.setIconPixmap(QPixmap(":/icons/aronBlack").scaledToWidth(128));

    QGridLayout* layout = (QGridLayout*)aboutOSGMessageBox.layout();
    layout->addItem(horizontalSpacer, layout->rowCount(), 0, 1, layout->columnCount());
    //    aboutCVMessageBox.setFixedSize(800, 600);
    aboutOSGMessageBox.exec();
}

void MainWindow::displayHelp()
{
    HelpWindow hw;
    hw.exec();
}

// private

/*
 * ce membre va initialiser une premiere fenetre
 */
void MainWindow::setFirstWindow()
{
    //////////////////////////////////////////////////
    ////////// menu bar //////////////////////////////
    //////////////////////////////////////////////////

    QMenuBar* menuBar = new QMenuBar(this);
    QMenu* menuFile = menuBar->addMenu(tr("&File"));
    QMenu* menuDetect = menuBar->addMenu(tr("&Detection"));
    QMenu* menuCam = menuBar->addMenu(tr("Video &Input"));
    QMenu* menuHelp = menuBar->addMenu(tr("&?"));



    _startAction = menuFile->addAction(tr("&Start"));
    _addObjectAction = menuFile->addAction(tr("&Add object to the scene"));
    _addObjectAction->setEnabled(false);
    _calibrateAction = menuFile->addAction(tr("&Calibrate camera"));
    _optionsAction = menuFile->addAction(tr("&Options"));
    _fullScreenAction = menuFile->addAction(tr("F&ull Screen"));
    _fullScreenAction->setEnabled(false);



    _detectGroup = new QActionGroup(menuDetect);
    _detectActions = new QAction*[NBR_DETECT];
    for(int i = 0; i < NBR_DETECT; i++)
    {
        _detectActions[i] = new QAction(_detectGroup);
        _detectActions[i]->setCheckable(true);
        _detectActions[i]->setEnabled(false);
        _detectGroup->addAction(_detectActions[i]);
        menuDetect->addAction(_detectActions[i]);
    }

    _detectActions[noDetect]->setText(tr("&No detection"));
    _detectActions[noDetect]->setChecked(true);
    _detectActions[chehra]->setText(tr("F&ace detection"));
    _detectActions[chess]->setText(tr("&Board detection"));
    _detectActions[QR]->setText(tr("&Marker detection"));



    _webcamGroup = new QActionGroup(menuCam);
    if(_nbrCam > 0)
    {
        _webcamActions = new QAction*[_nbrCam];
        for(int i = 0; i < _nbrCam; ++i)
        {
            _webcamActions[i] = menuCam->addAction(QString("Webcam ") + QString::number(i + 1));
            _webcamGroup->addAction(_webcamActions[i]);
            _webcamActions[i]->setCheckable(true);
            _webcamActions[i]->setEnabled(false);
        }
        _webcamActions[0]->setChecked(true);
    }

    menuCam->addSeparator();
    _videoAction = menuCam->addAction(tr("&Open video file"));
    _webcamGroup->addAction(_videoAction);
    _videoAction->setCheckable(true);
    _videoAction->setEnabled(false);



    QAction* appAction = menuHelp->addAction(tr("&About"));
    connect(appAction, SIGNAL(triggered()), this, SLOT(displayAbout()));
    QAction* howToAction = menuHelp->addAction(tr("&How to use"));
    connect(howToAction, SIGNAL(triggered()), this, SLOT(displayHelp()));
    menuHelp->addSeparator();
    QAction* qtAction = menuHelp->addAction(tr("About &Qt"));
    connect(qtAction, SIGNAL(triggered()), this, SLOT(displayAboutQt()));
    QAction* ocvAction = menuHelp->addAction(tr("About OpenC&V"));
    connect(ocvAction, SIGNAL(triggered()), this, SLOT(displayAboutCV()));
    QAction* osgAction = menuHelp->addAction(tr("About Open&SceneGraph"));
    connect(osgAction, SIGNAL(triggered()), this, SLOT(displayAboutOsg()));
    QAction* chehraAction = menuHelp->addAction(tr("About &Chehra"));
    connect(chehraAction, SIGNAL(triggered()), this, SLOT(displayAboutChehra()));



    connect(_startAction, SIGNAL(triggered()), this, SLOT(start()));
    connect(_calibrateAction, SIGNAL(triggered()), this, SLOT(calibrateCamera()));
    connect(_optionsAction, SIGNAL(triggered()), _webcamDevice, SLOT(setOptions()));

    this->setMenuBar(menuBar);
}

/*
 * membre qui va créer la fenetre principale
 * en instanciant et positionnant chaque
 * widget et layout
 */
void MainWindow::setMainWindow(int mode)
{
    //////////////////////////////////////////////////
    ////////// enable actions ////////////////////////
    //////////////////////////////////////////////////
    _startAction->setEnabled(false);
    _addObjectAction->setEnabled(true);
    _fullScreenAction->setEnabled(true);

    for(int i = 0; i < NBR_DETECT; i++)
        _detectActions[i]->setEnabled(true);

    _detectActions[noDetect]->setEnabled(true);

    for(int i = 0; i < _nbrCam; i++)
        _webcamActions[i]->setEnabled(true);
    _videoAction->setEnabled(true);

    //////////////////////////////////////////////////
    //////////////////////////////////////////////////
    //////////////////////////////////////////////////

    QWidget* mainWidget = new QWidget();
    QSplitter* mainSplitter = new QSplitter(Qt::Horizontal);

    QVBoxLayout* objectLayout = new QVBoxLayout();

    //////////////////////////////////////////////////
    ////////// webcam layout /////////////////////////
    //////////////////////////////////////////////////

    _mainView = new OSGWidget(_webcamDevice->getWebcam(), _mainMat, _objectsList[1], mode, this);
    //mainLayout->addWidget(_mainView);

    //////////////////////////////////////////////////
    ////////// object layout /////////////////////////
    //////////////////////////////////////////////////

    QGridLayout* gridLayout = new QGridLayout();

    _objectChoiceComboBox = new QComboBox();
    _objectChoiceComboBox->addItem("All objects : 0");
    _objectChoiceComboBox->addItem("Video");
    gridLayout->addWidget(_objectChoiceComboBox, 0, 0, 1, 4);

    QHBoxLayout* objectDispalyOptionsLayout = new QHBoxLayout();
    _isPrintedBox = new QCheckBox(tr("Display object in main view"));
    _isPrintedBox->setChecked(false);
    objectDispalyOptionsLayout->addWidget(_isPrintedBox);
    _isPrintedBox2 = new QCheckBox(tr("Display object in side view"));
    _isPrintedBox2->setChecked(true);
    objectDispalyOptionsLayout->addWidget(_isPrintedBox2);
    _deleteObjectButton = new QPushButton(tr("Delete object"));
    objectDispalyOptionsLayout->addWidget(_deleteObjectButton);
    _setTextureButton = new QPushButton(tr("Apply texture"));
    objectDispalyOptionsLayout->addWidget(_setTextureButton);

    gridLayout->addWidget(_isPrintedBox, 1, 0, 1, 1);
    gridLayout->addWidget(_isPrintedBox2, 1, 1, 1, 1);
    gridLayout->addWidget(_deleteObjectButton, 1, 2, 1, 1);
    gridLayout->addWidget(_setTextureButton, 1, 3, 1, 1);

    //objectLayout->addLayout(objectDispalyOptionsLayout);

    _objectCharacteristicsSpinSliders = new QSlider*[NBR_CHARACTERISTICS];
    for(int i = 0; i < NBR_CHARACTERISTICS; i++)
    {
        _objectCharacteristicsSpinSliders[i] = new QSlider();
        _objectCharacteristicsSpinSliders[i]->setOrientation(Qt::Horizontal);
    }

    QFormLayout *resizeLayout = new QFormLayout;
    QGroupBox *resizeGroup = new QGroupBox(tr(" Resize object "));

    _objectCharacteristicsSpinSliders[sizeX]->setRange(-1000, 1000);
    _objectCharacteristicsSpinSliders[sizeX]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[sizeX]->setValue(0);
    resizeLayout->addRow("x : ", _objectCharacteristicsSpinSliders[sizeX]);
    _objectCharacteristicsSpinSliders[sizeY]->setRange(-1000, 1000);
    _objectCharacteristicsSpinSliders[sizeY]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[sizeY]->setValue(0);
    resizeLayout->addRow("y : ", _objectCharacteristicsSpinSliders[sizeY]);
    _objectCharacteristicsSpinSliders[sizeZ]->setRange(-1000, 1000);
    _objectCharacteristicsSpinSliders[sizeZ]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[sizeZ]->setValue(0);
    resizeLayout->addRow("z : ", _objectCharacteristicsSpinSliders[sizeZ]);

    resizeGroup->setLayout(resizeLayout);
    //objectLayout->addWidget(resizeGroup);

    QFormLayout *rotateLayout = new QFormLayout;
    QGroupBox *rotateGroup = new QGroupBox(tr(" Rotate object "));

    _objectCharacteristicsSpinSliders[rotX]->setRange(-180, 180);
    _objectCharacteristicsSpinSliders[rotX]->setSingleStep(1);
    rotateLayout->addRow("x : ", _objectCharacteristicsSpinSliders[rotX]);
    _objectCharacteristicsSpinSliders[rotY]->setRange(-180, 180);
    _objectCharacteristicsSpinSliders[rotY]->setSingleStep(1);
    rotateLayout->addRow("y : ", _objectCharacteristicsSpinSliders[rotY]);
    _objectCharacteristicsSpinSliders[rotZ]->setRange(-180, 180);
    _objectCharacteristicsSpinSliders[rotZ]->setSingleStep(1);
    rotateLayout->addRow("z : ", _objectCharacteristicsSpinSliders[rotZ]);

    rotateGroup->setLayout(rotateLayout);
    //objectLayout->addWidget(rotateGroup);

    QFormLayout *translateLayout = new QFormLayout;
    QGroupBox *translateGroup = new QGroupBox(tr(" Translate object "));

    _objectCharacteristicsSpinSliders[transX]->setRange(-500, 500);
    _objectCharacteristicsSpinSliders[transX]->setSingleStep(1);
    translateLayout->addRow("x : ", _objectCharacteristicsSpinSliders[transX]);
    _objectCharacteristicsSpinSliders[transY]->setRange(-500, 500);
    _objectCharacteristicsSpinSliders[transY]->setSingleStep(1);
    translateLayout->addRow("y : ", _objectCharacteristicsSpinSliders[transY]);
    _objectCharacteristicsSpinSliders[transZ]->setRange(-500, 500);
    _objectCharacteristicsSpinSliders[transZ]->setSingleStep(1);
    translateLayout->addRow("z : ", _objectCharacteristicsSpinSliders[transZ]);

    translateGroup->setLayout(translateLayout);
    //objectLayout->addWidget(translateGroup);

    // CSS
    translateGroup->setObjectName("sliderGroup");
    rotateGroup->setObjectName("sliderGroup");
    resizeGroup->setObjectName("sliderGroup");

    QFormLayout *alphaLayout = new QFormLayout;
    QGroupBox *alphaGroup = new QGroupBox(tr(" Transparency "));

    _objectCharacteristicsSpinSliders[alpha]->setRange(0, 100);
    _objectCharacteristicsSpinSliders[alpha]->setSingleStep(1);
    _objectCharacteristicsSpinSliders[alpha]->setValue(100);
    alphaLayout->addRow("α : ", _objectCharacteristicsSpinSliders[alpha]);

    alphaGroup->setLayout(alphaLayout);
    gridLayout->addWidget(alphaGroup, 2, 0, 1, 4);
    //objectLayout->addWidget(alphaGroup);

    QHBoxLayout *videoLayout = new QHBoxLayout;
    _videoGroup = new QGroupBox();
    _play = new QPushButton(QIcon(":/icons/play"), "");
    _pause = new QPushButton(QIcon(":/icons/pause"), "");
    _fast = new QPushButton(QIcon(":/icons/forw"), "");
    _slow = new QPushButton(QIcon(":/icons/back"), "");
    _backBegin = new QPushButton(QIcon(":/icons/backBegin"), "");

    videoLayout->addWidget(_backBegin);
    videoLayout->addWidget(_slow);
    videoLayout->addWidget(_play);
    videoLayout->addWidget(_pause);
    videoLayout->addWidget(_fast);
    _videoGroup->setLayout(videoLayout);
    _videoGroup->setVisible(false);
    objectLayout->addWidget(_videoGroup);

    QHBoxLayout *camLayout = new QHBoxLayout;
    _camGroup = new QGroupBox();
    _detectionLabel = new QLabel("No active detection.");
    _detectionLabel->setObjectName("infoLabel"); // CSS
    _detectionLabel2 = new QLabel("");
    _detectionLabel2->setObjectName("infoLabel"); // CSS

    camLayout->addWidget(_detectionLabel);
    camLayout->addWidget(_detectionLabel2);
    camLayout->setAlignment(Qt::AlignCenter);
    _camGroup->setLayout(camLayout);
    objectLayout->addWidget(_camGroup);

    _sideView = new SideViewOsgWidet(_webcamDevice->getWebcam(), _mainMat2, _objectsList2[1], mode, this);
    objectLayout->addWidget(_sideView);

    objectLayout->setStretchFactor(_sideView, 2);
    mainWidget->setLayout(objectLayout);
    //mainLayout->addLayout(objectLayout);

    //////////////////////////////////////////////////
    //              TOOLBAR - TODO                  //
    //////////////////////////////////////////////////

    QWidget* widgetToolbar = new QWidget();
    widgetToolbar->setLayout(gridLayout);

    QToolBar* toolbar = new QToolBar();
    toolbar->addWidget(widgetToolbar);
    toolbar->addWidget(resizeGroup);
    toolbar->addWidget(rotateGroup);
    toolbar->addWidget(translateGroup);
    toolbar->setMovable(true);
    //    this->addToolBar(Qt::BottomToolBarArea, toolbar);
    this->addToolBar(Qt::TopToolBarArea, toolbar);

    ///////////////////////////////////////////////////

    //mainLayout->setStretch(0, 2);
    //mainLayout->setStretch(1, 1);
    //mainWidget->setLayout(mainLayout);
    mainSplitter->addWidget(_mainView);
    mainSplitter->addWidget(mainWidget);
    mainSplitter->setCollapsible(0, false);
    mainSplitter->setCollapsible(1, false);
    mainSplitter->setStretchFactor(0, 2);
    mainSplitter->setStretchFactor(1, 1);
    this->setCentralWidget(mainSplitter);


}

/*
 * membre qui va connecter tous les signaux à leurs slots associés
 */
void MainWindow::connectAll()
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(_webcamDevice, SIGNAL(updateWebcam()), this, SLOT(updateCam()));
    connect(_webcamDevice, SIGNAL(backToBeginSig()), this, SLOT(back2Begin()));
    connect(_webcamDevice, SIGNAL(freezeButtons()), this, SLOT(freezeButtons()));
    connect(_webcamDevice, SIGNAL(playVideo()), this, SLOT(playVideo()));
    connect(_webcamDevice, SIGNAL(playCam()), this, SLOT(playCam()));

    connect(_detectGroup, SIGNAL(triggered(QAction*)), this, SLOT(updateDetectMode()));
    connect(_webcamDevice, SIGNAL(updateDetect(bool)), this, SLOT(updateDetectLabel(bool)));
    connect(_webcamGroup, SIGNAL(triggered(QAction*)), this, SLOT(switchInput()));

    connect(_addObjectAction, SIGNAL(triggered()), this, SLOT(addObject()));
    connect(_fullScreenAction, SIGNAL(triggered()), this, SLOT(displayFullScreen()));
    connect(_mainView, SIGNAL(changeFullScreenState()), this, SLOT(displayFullScreen()));
    connect(_fullScreenView, SIGNAL(changeFullScreenState()), this, SLOT(displayFullScreen()));

    connect(_detectActions[noDetect], SIGNAL(toggled(bool)), this, SLOT(displayObjects(bool)));
    connect(_webcamDevice, SIGNAL(updateDetect(bool)), this, SLOT(displaySceneAuto(bool)));
    connect(_webcamDevice, SIGNAL(updateDetect(bool)), this, SLOT(displayInSideViewAuto(bool)));
    connect(_isPrintedBox, SIGNAL(clicked(bool)), this, SLOT(displayObjectInScene(bool)));
    connect(_isPrintedBox2, SIGNAL(clicked(bool)), this, SLOT(displayObjectInSideView(bool))); // #truanderie

    connect(_objectChoiceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObjectCharacteristics(int)));
    connect(_deleteObjectButton, SIGNAL(clicked()), this, SLOT(removeObject()));
    connect(_setTextureButton, SIGNAL(clicked()), this, SLOT(setTexture()));

    connect(_webcamDevice, SIGNAL(updateScene(cv::Mat, cv::Mat)), this, SLOT(updateSceneRT(cv::Mat, cv::Mat)));


    Our3DObject* object = _objectsList[0];

    connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
    connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
    connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

    connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
    connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
    connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

    connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
    connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
    connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

    connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

    object = _objectsList2[0];

    connect(_objectCharacteristicsSpinSliders[sizeX], SIGNAL(valueChanged(int)), object, SLOT(setSizeX(int)));
    connect(_objectCharacteristicsSpinSliders[sizeY], SIGNAL(valueChanged(int)), object, SLOT(setSizeY(int)));
    connect(_objectCharacteristicsSpinSliders[sizeZ], SIGNAL(valueChanged(int)), object, SLOT(setSizeZ(int)));

    connect(_objectCharacteristicsSpinSliders[rotX], SIGNAL(valueChanged(int)), object, SLOT(setRotX(int)));
    connect(_objectCharacteristicsSpinSliders[rotY], SIGNAL(valueChanged(int)), object, SLOT(setRotY(int)));
    connect(_objectCharacteristicsSpinSliders[rotZ], SIGNAL(valueChanged(int)), object, SLOT(setRotZ(int)));

    connect(_objectCharacteristicsSpinSliders[transX], SIGNAL(valueChanged(int)), object, SLOT(setTransX(int)));
    connect(_objectCharacteristicsSpinSliders[transY], SIGNAL(valueChanged(int)), object, SLOT(setTransY(int)));
    connect(_objectCharacteristicsSpinSliders[transZ], SIGNAL(valueChanged(int)), object, SLOT(setTransZ(int)));

    connect(_objectCharacteristicsSpinSliders[alpha], SIGNAL(valueChanged(int)), object, SLOT(setAlpha(int)));

    connect(_slow, SIGNAL(clicked()), _webcamDevice, SLOT(backward()));
    connect(_fast, SIGNAL(clicked()), _webcamDevice, SLOT(forward()));
    connect(_pause, SIGNAL(clicked()), this, SLOT(playNpause()));
    connect(_play, SIGNAL(clicked()), this, SLOT(playNpause()));
    connect(_backBegin, SIGNAL(clicked()), _webcamDevice, SLOT(backToBeginSlot()));
}

/*
 * membre qui va créer quelques raccourcis clavier et les connecter
 * à leurs slots associés
 */
void MainWindow::setShortcuts()
{
    _fullScreenShortcut = new QShortcut(Qt::CTRL + Qt::Key_F, this);
    connect(_fullScreenShortcut, SIGNAL(activated()), this, SLOT(displayFullScreen()));
    _fullScreenShortcut2 = new QShortcut(Qt::CTRL + Qt::Key_F, _fullScreenView); // necessaire pour utiliser le shortcut en plein ecran
    connect(_fullScreenShortcut2, SIGNAL(activated()), this, SLOT(displayFullScreen())); // #truanderie
    _leaveFullScreen = new QShortcut(Qt::Key_Escape, _fullScreenView);
    connect(_leaveFullScreen, SIGNAL(activated()), this, SLOT(displayFullScreen()));

    _pauseShortcut = new QShortcut(Qt::Key_Space, this);
    connect(_pauseShortcut, SIGNAL(activated()), this, SLOT(playNpause()));
    _pauseShortcut2 = new QShortcut(Qt::Key_Space, _fullScreenView);
    connect(_pauseShortcut2, SIGNAL(activated()), this, SLOT(playNpause()));
}

/*
 * membre qui va créer un widget permettant l'affichage en plein ecran
 * il se compose d'un second OSGWidget (représentant la vue principale)
 * et de deux bandes noires afin d'afficher l'image avec le bon ratio
 */
void MainWindow::createFullScreenWidget(int mode)
{
    QRect rec = QApplication::desktop()->screenGeometry();
    int screenHeight = rec.height();
    int screenWidth = rec.width();

    cv::Mat* webcam = _webcamDevice->getWebcam();
    int camHeight = webcam->rows;
    int camWidth = webcam->cols;

    _fullScreenView = new OSGWidget(webcam, _mainMat, _objectsList[1], mode);
    _fullScreenWidget = new QWidget;
    _fullScreenWidget->setContentsMargins(0, 0, 0, 0);

    int resizeH = screenHeight / camHeight;
    int resizeW = screenWidth / camWidth;

    if(resizeW > resizeH)
    {
        QHBoxLayout *layout = new QHBoxLayout;
        QGraphicsView *sideLayer = new QGraphicsView;
        QGraphicsView *sideLayer2 = new QGraphicsView;

        sideLayer->setBackgroundBrush(QBrush(Qt::black));
        sideLayer->setFixedWidth((screenWidth - (camWidth * resizeH)) / 2);
        sideLayer->setScene(new QGraphicsScene());
        sideLayer2->setBackgroundBrush(QBrush(Qt::black));
        sideLayer2->setFixedWidth((screenWidth - (camWidth * resizeH)) / 2);
        sideLayer2->setScene(new QGraphicsScene());

        layout->addWidget(sideLayer);
        layout->addWidget(_fullScreenView);
        layout->addWidget(sideLayer2);

        layout->setSpacing(0);
        layout->setContentsMargins(1, 0, 1, 0);

        _fullScreenWidget->setLayout(layout);
    }
    else if(resizeW < resizeH)
    {
        QVBoxLayout *layout = new QVBoxLayout;
        QGraphicsView *sideLayer = new QGraphicsView;
        QGraphicsView *sideLayer2 = new QGraphicsView;

        sideLayer->setBackgroundBrush(QBrush(Qt::black));
        sideLayer->setFixedHeight((screenHeight - (camHeight * resizeW)) / 2);
        sideLayer->setScene(new QGraphicsScene());
        sideLayer2->setBackgroundBrush(QBrush(Qt::black));
        sideLayer2->setFixedHeight((screenHeight - (camHeight * resizeW)) / 2);
        sideLayer2->setScene(new QGraphicsScene());

        layout->addWidget(sideLayer);
        layout->addWidget(_fullScreenView);
        layout->addWidget(sideLayer2);

        layout->setSpacing(0);
        layout->setContentsMargins(1, 0, 1, 0);

        _fullScreenWidget->setLayout(layout);
    }
    else
        _fullScreenWidget = _fullScreenView;
}

/*
 * membre qui va instancier le vector de Our3DObject
 * a noter que le _objectList[0] contiendra la matrixTransform principale
 * et que le _objectList[1] contiendra la texture video.
 * Ainsi, _objectList[0] aura comme "child" dans le graphe de scene tous
 * les autres objets contenus dans le vector à partir de _objectList[2]
 * Il y a aussi un vector d'Our3DObject par vue (mainView et sideView)
 */
void MainWindow::initObjectsList(int mode)
{
    _objectsList.push_back(new Our3DObject()); // globalMat en _objectsList[0]
    _mainMat = new osg::MatrixTransform();
    _mainMat->addChild(_objectsList[0]);
    _objectsList[0]->setNodeMask(0);

    _objectsList2.push_back(new Our3DObject()); // globalMat2 en _objectsList2[0]
    _mainMat2 = new osg::MatrixTransform();
    _mainMat2->addChild(_objectsList2[0]);
    _objectsList2[0]->setNodeMask(1);

    _backgroundImage = new osg::Image;

    cv::Mat *webcamMat = _webcamDevice->getWebcam();
    int cx;
    int cy;
    double n;

    if(mode == 1)
    {
        cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);

        cameraMatrix.at<double>(0,0) = 680.0;
        cameraMatrix.at<double>(0,1) = 0;
        cameraMatrix.at<double>(0,2) = 320.0;
        cameraMatrix.at<double>(1,0) = 0;
        cameraMatrix.at<double>(1,1) = 680;
        cameraMatrix.at<double>(1,1) = 240.0;
        cameraMatrix.at<double>(2,0) = 0;
        cameraMatrix.at<double>(2,1) = 0;
        cameraMatrix.at<double>(2,2) = 1;

        cx = cameraMatrix.at<double>(0, 2);
        cy = cameraMatrix.at<double>(1, 2);
        n = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2;
        // NEAR (n) = distance focale ; si pixels carres, fx = fy -> np
        //mais est generalement different de fy donc on prend (pour l'instant) par defaut la valeur mediane
        _corrector = (n / 2) / (cy - webcamMat->rows / 2);
    }

    else if(mode == 2)
    {
        cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

        cv::Mat cameraMatrix;

        fs["cameraMatrix"] >> cameraMatrix;

        cx = cameraMatrix.at<double>(0, 2);
        cy = cameraMatrix.at<double>(1, 2);
        n = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2;
        // NEAR (n) = distance focale ; si pixels carres, fx = fy -> np
        //mais est generalement different de fy donc on prend (pour l'instant) par defaut la valeur mediane
        _corrector = (n / 2) / (cy - webcamMat->rows / 2);

        fs.release();
    }

    _backgroundImage->setImage(webcamMat->cols, webcamMat->rows, 3,
                               GL_RGB, GL_BGR, GL_UNSIGNED_BYTE,
                               (uchar*)(webcamMat->data),
                               osg::Image::AllocationMode::NO_DELETE, 1);
    {
        osg::Geometry* geoQuad = new osg::Geometry;

        osg::Vec3Array* tabSommet = new osg::Vec3Array;
        tabSommet->push_back(osg::Vec3(-cx, n, -cy));
        tabSommet->push_back(osg::Vec3(webcamMat->cols - cx, n, -cy));
        tabSommet->push_back(osg::Vec3(webcamMat->cols - cx, n, webcamMat->rows - cy));
        tabSommet->push_back(osg::Vec3(-cx, n, webcamMat->rows - cy));
        geoQuad->setVertexArray(tabSommet);

        osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
        primitive->push_back(0);
        primitive->push_back(1);
        primitive->push_back(2);
        primitive->push_back(3);
        geoQuad->addPrimitiveSet(primitive);

        // Nous creons ensuite une tableau qui contiendra nos coordonnees de texture.
        osg::Vec2Array* coordonneeTexture = new osg::Vec2Array(4);
        (*coordonneeTexture)[0].set(1.0f, 1.0f);
        (*coordonneeTexture)[1].set(0.0f, 1.0f);
        (*coordonneeTexture)[2].set(0.0f, 0.0f);
        (*coordonneeTexture)[3].set(1.0f, 0.0f);
        geoQuad->setTexCoordArray(0, coordonneeTexture);

        osg::Geode* noeudGeo = new osg::Geode;
        osg::StateSet* statuts = noeudGeo->getOrCreateStateSet();
        statuts->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        noeudGeo->addDrawable(geoQuad);

        // Nous creons une Texture2D.
        osg::Texture2D* texture = new osg::Texture2D;

        // Nous associons notre image a notre objet Texture2D.
        texture->setImage(_backgroundImage);
        texture->setResizeNonPowerOfTwoHint(false);

        // Enfin nous activons les texture de notre objet Geometry a travers l'objet statuts.
        statuts->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
        statuts->setMode(GL_BLEND,osg::StateAttribute::ON);
        statuts->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

        _objectsList.push_back(new Our3DObject(noeudGeo)); // mat en _objectsList[1] qui contient le hud
    }

    {
        osg::Geometry* geoQuad = new osg::Geometry;

        osg::Vec3Array* tabSommet = new osg::Vec3Array;
        tabSommet->push_back(osg::Vec3(-cx, n, -cy));
        tabSommet->push_back(osg::Vec3(webcamMat->cols - cx, n, -cy));
        tabSommet->push_back(osg::Vec3(webcamMat->cols - cx, n, webcamMat->rows - cy));
        tabSommet->push_back(osg::Vec3(-cx, n, webcamMat->rows - cy));
        geoQuad->setVertexArray(tabSommet);

        osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
        primitive->push_back(0);
        primitive->push_back(1);
        primitive->push_back(2);
        primitive->push_back(3);
        geoQuad->addPrimitiveSet(primitive);

        // Nous creons ensuite une tableau qui contiendra nos coordonnees de texture.
        osg::Vec2Array* coordonneeTexture = new osg::Vec2Array(4);
        (*coordonneeTexture)[0].set(1.0f, 1.0f);
        (*coordonneeTexture)[1].set(0.0f, 1.0f);
        (*coordonneeTexture)[2].set(0.0f, 0.0f);
        (*coordonneeTexture)[3].set(1.0f, 0.0f);
        geoQuad->setTexCoordArray(0, coordonneeTexture);

        osg::Geode* noeudGeo = new osg::Geode;
        osg::StateSet* statuts = noeudGeo->getOrCreateStateSet();
        statuts->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        noeudGeo->addDrawable(geoQuad);

        // Nous creons une Texture2D.
        osg::Texture2D* texture = new osg::Texture2D;

        // Nous associons notre image a notre objet Texture2D.
        texture->setImage(_backgroundImage);
        texture->setResizeNonPowerOfTwoHint(false);

        // Enfin nous activons les texture de notre objet Geometry a travers l'objet statuts.
        statuts->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
        statuts->setMode(GL_BLEND,osg::StateAttribute::ON);
        statuts->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

        _objectsList2.push_back(new Our3DObject(noeudGeo));
    }
}
