#include "osgwidget.h"

OSGWidget::OSGWidget(cv::Mat* webcamMat, QWidget* parent, const QGLWidget* shareWidget)
    : QGLWidget( parent, shareWidget)
    , _graphicsWindow( new osgViewer::GraphicsWindowEmbedded( this->x(),
                                                               this->y(),
                                                               this->width(),
                                                               this->height() ) )
    , _viewer( new osgViewer::Viewer )
{
    //////////////////////////////////////////////////
    ////////// initialisation OpenCV /////////////////
    //////////////////////////////////////////////////

    cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

    cv::Mat cameraMatrix;

    fs["cameraMatrix"] >> cameraMatrix;

    double NEAR_VALUE = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carrés, fx = fy -> np
    //mais est généralement différent de fy donc on prend (pour l'instant) par défaut la valeur médiane
    double FAR_VALUE = 2000 * NEAR_VALUE;

    fs.release();

    //////////////////////////////////////////////////
    ////////// initialisation OpenSceneGraph /////////
    //////////////////////////////////////////////////

    _backgroundImage = new osg::Image;
    _backgroundImage->setImage(webcamMat->cols, webcamMat->rows, 3,
                                GL_RGB, GL_BGR, GL_UNSIGNED_BYTE,
                                (uchar*)(webcamMat->data),
                                osg::Image::AllocationMode::NO_DELETE, 1);

    _hud = createHUD(_backgroundImage, webcamMat->cols, webcamMat->rows, cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2), NEAR_VALUE);

    _group = new osg::Group();
    _mainMat = new osg::MatrixTransform();
    _globalMat = new Our3DObject();
    _mainCam = new osg::Camera();
    _hudCam = new osg::Camera();

    // Projection

    osg::Matrixd projectionMatrix;

    projectionMatrix.makeFrustum(
                -cameraMatrix.at<double>(0, 2),		webcamMat->cols - cameraMatrix.at<double>(0, 2),
                -cameraMatrix.at<double>(1, 2),		webcamMat->rows - cameraMatrix.at<double>(1, 2),
                NEAR_VALUE,							FAR_VALUE);

    _corrector = (NEAR_VALUE / 2) / (cameraMatrix.at<double>(1, 2) - webcamMat->rows / 2);

    osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, FAR_VALUE, 0.0f), normal(0.0f, 0.0f, 1.0f);

    _hudCam->setProjectionMatrix(projectionMatrix);
    _hudCam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _hudCam->setViewMatrixAsLookAt(eye, target, normal);
    _hudCam->setClearMask(GL_DEPTH_BUFFER_BIT);
    _hudCam->setRenderOrder(osg::Camera::PRE_RENDER);

    _mainCam->setProjectionMatrix(projectionMatrix);
    _mainCam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _mainCam->setViewMatrixAsLookAt(eye, target, normal);
    _mainCam->setClearMask(GL_DEPTH_BUFFER_BIT);
    _mainCam->setRenderOrder(osg::Camera::POST_RENDER);

    //_mainCam->addChild(_mainMat);
    _mainMat->addChild(_globalMat);
    _hudCam->addChild(_hud);

    _group->addChild(_mainCam);
    _group->addChild(_hudCam);

    //float aspectRatio = static_cast<float>(this->width()) / static_cast<float>( this->height() );
    _viewer->setSceneData(_group.get());
    _viewer->getCamera()->setProjectionMatrix(projectionMatrix);
    _viewer->getCamera()->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _viewer->getCamera()->setViewMatrixAsLookAt(eye, target, normal);
    _viewer->getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT);
    _viewer->getCamera()->setGraphicsContext(_graphicsWindow);

    _viewer->setThreadingModel( osgViewer::Viewer::SingleThreaded );
    _viewer->realize();

    this->setAutoBufferSwap(false);
}

OSGWidget::~OSGWidget()
{
}


Our3DObject* OSGWidget::getObject(int objectID)
{
    if(objectID < 0 || objectID >= _objectsList.size())
        return NULL;
    else
        return _objectsList[objectID];
}



// public slots

void OSGWidget::updateSceneRT(cv::Mat rotVec, cv::Mat tvecs)
{
    double t3 = tvecs.at<double>(1, 0);
    double t1 = -tvecs.at<double>(0, 0);
    double t2 = -tvecs.at<double>(2, 0);// - t3 / _corrector; // and now, magic !

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
                r11,	r21,	r31,	0,
                r12,	r22,	r32,	0,
                r13,	r23,	r33,	0,
                0,		0,		0,		1);

    double rotX = atan2(r32, r33);
    double rotY =  atan2(r21, r11);
    double rotZ = -atan2(-r31, sqrt((r32 * r32) + (r33 * r33)));

    std::cout << "rotations : " << std::endl
            << "x : " << rotX << std::endl
            << "y : " << rotY << std::endl
            << "z : " << rotZ << std::endl << std::endl;
    std::cout << "translations : " << std::endl
              << "x : " << t1 << std::endl
              << "y : " << t2 << std::endl
              << "z : " << t3 << std::endl << std::endl << std::endl;


    osg::Matrixd matrixT; // translation
    matrixT.makeTranslate(t1, t2, t3);

    osg::Matrixd matrix90; // rotation de repere entre opencv et osg
    matrix90.makeRotate(osg::Quat(osg::DegreesToRadians(90.0f), osg::Vec3d(1.0, 0.0, 0.0)));

    osg::Matrixd matrix180; // rotation de repere entre opencv et osg
    matrix180.makeRotate(osg::Quat(osg::DegreesToRadians(180.0f), osg::Vec3d(0.0, 0.0, 1.0)));

    _mainMat->setMatrix(matrixR * matrix90 * matrix180 * matrixT);
}

void OSGWidget::addObjectToScene(QString objectPath)
{
    _objectsList.push_back(new Our3DObject(objectPath));
    _globalMat->addChild(_objectsList[_objectsList.size() - 1]);
}

void OSGWidget::displayObjectInScene(int objectID, bool display)
{
    std::cout << "ID : " << objectID << "    display : " << display << "    nbrObj : " << _objectsList.size() << std::endl;
    if(objectID > 0 && objectID < _objectsList.size())
    {
        if(display)
            _globalMat->addChild(_objectsList[objectID - 1]);
        else
            _globalMat->removeChild(_objectsList[objectID - 1]);
        _objectsList[objectID - 1]->printObject(display);
    }
    else if(objectID == 0)
    {
        if(display)
            _mainMat->addChild(_globalMat);
        else
            _mainMat->removeChild(_globalMat);
        _globalMat->printObject(display);
    }
    else
        return;
}




// protected

void OSGWidget::paintEvent( QPaintEvent* /* paintEvent */ )
{
    this->makeCurrent();

    QPainter painter( this );
    painter.setRenderHint( QPainter::Antialiasing );

    this->paintGL();

    painter.end();

    this->swapBuffers();
    this->doneCurrent();
}

void OSGWidget::paintGL()
{
    _backgroundImage->dirty();
    _viewer->frame();
}

void OSGWidget::resizeGL( int width, int height )
{
    _graphicsWindow->resized( this->x(), this->y(), width, height );

    _viewer->getCamera()->setViewport( 0, 0, this->width(), this->height() );
}


// private

void OSGWidget::onHome()
{
    _viewer->home();
}

osg::Geode* OSGWidget::createHUD(osg::Image* bgImage, int camWidth, int camHeight, double cx, double cy, double n)
{
    osg::Geometry* geoQuad = new osg::Geometry;

    osg::Vec3Array* tabSommet = new osg::Vec3Array;
    tabSommet->push_back(osg::Vec3(-cx, n, -cy));
    tabSommet->push_back(osg::Vec3(camWidth - cx, n, -cy));
    tabSommet->push_back(osg::Vec3(camWidth - cx, n, camHeight - cy));
    tabSommet->push_back(osg::Vec3(-cx, n, camHeight - cy));
    geoQuad->setVertexArray(tabSommet);

    osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    primitive->push_back(0);
    primitive->push_back(1);
    primitive->push_back(2);
    primitive->push_back(3);
    geoQuad->addPrimitiveSet(primitive);

    // Nous créons ensuite une tableau qui contiendra nos coordonnées de texture.
    osg::Vec2Array* coordonneeTexture = new osg::Vec2Array(4);
    (*coordonneeTexture)[0].set(0.0f, 1.0f);
    (*coordonneeTexture)[1].set(1.0f, 1.0f);
    (*coordonneeTexture)[2].set(1.0f, 0.0f);
    (*coordonneeTexture)[3].set(0.0f, 0.0f);
    geoQuad->setTexCoordArray(0, coordonneeTexture);

    osg::Geode* noeudGeo = new osg::Geode;
    osg::StateSet* statuts = noeudGeo->getOrCreateStateSet();
    statuts->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    noeudGeo->addDrawable(geoQuad);

    // Nous créons une Texture2D.
    osg::Texture2D* texture = new osg::Texture2D;

    // Nous associons notre image à notre objet Texture2D.
    texture->setImage(bgImage);
    texture->setResizeNonPowerOfTwoHint(false);

    // Enfin nous activons les texture de notre objet Geometry à travers l'objet statuts.
    statuts->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
    statuts->setMode(GL_BLEND,osg::StateAttribute::ON);
    statuts->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    return noeudGeo;
}
