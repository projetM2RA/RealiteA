#include "osgwidget.h"

OSGWidget::OSGWidget(cv::Mat* webcamMat, QWidget* parent, const QGLWidget* shareWidget)
    : QGLWidget( parent, shareWidget)
    , m_graphicsWindow( new osgViewer::GraphicsWindowEmbedded( this->x(),
                                                               this->y(),
                                                               this->width(),
                                                               this->height() ) )
    , m_viewer( new osgViewer::Viewer )
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

    m_backgroundImage = new osg::Image;
    m_backgroundImage->setImage(webcamMat->cols, webcamMat->rows, 3,
                                GL_RGB, GL_BGR, GL_UNSIGNED_BYTE,
                                (uchar*)(webcamMat->data),
                                osg::Image::AllocationMode::NO_DELETE, 1);

    m_hud = createHUD(m_backgroundImage, webcamMat->cols, webcamMat->rows, cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2), NEAR_VALUE);

    m_group = new osg::Group();
    m_mainMat = new osg::MatrixTransform();
    m_globalMat = new Our3DObject();
    m_mainCam = new osg::Camera();
    m_hudCam = new osg::Camera();

    // Projection

    osg::Matrixd projectionMatrix;

    projectionMatrix.makeFrustum(
                -cameraMatrix.at<double>(0, 2),		webcamMat->cols - cameraMatrix.at<double>(0, 2),
                -cameraMatrix.at<double>(1, 2),		webcamMat->rows - cameraMatrix.at<double>(1, 2),
                NEAR_VALUE,							FAR_VALUE);

    m_corrector = (NEAR_VALUE / 2) / (cameraMatrix.at<double>(1, 2) - webcamMat->rows / 2);

    osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, FAR_VALUE, 0.0f), normal(0.0f, 0.0f, 1.0f);

    m_hudCam->setProjectionMatrix(projectionMatrix);
    m_hudCam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    m_hudCam->setViewMatrixAsLookAt(eye, target, normal);
    m_hudCam->setClearMask(GL_DEPTH_BUFFER_BIT);
    m_hudCam->setRenderOrder(osg::Camera::PRE_RENDER);

    m_mainCam->setProjectionMatrix(projectionMatrix);
    m_mainCam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    m_mainCam->setViewMatrixAsLookAt(eye, target, normal);
    m_mainCam->setClearMask(GL_DEPTH_BUFFER_BIT);
    m_mainCam->setRenderOrder(osg::Camera::POST_RENDER);

    //m_mainCam->addChild(m_mainMat);
    m_mainMat->addChild(m_globalMat);
    m_hudCam->addChild(m_hud);

    m_group->addChild(m_mainCam);
    m_group->addChild(m_hudCam);

    //float aspectRatio = static_cast<float>(this->width()) / static_cast<float>( this->height() );
    m_viewer->setSceneData(m_group.get());
    m_viewer->getCamera()->setProjectionMatrix(projectionMatrix);
    m_viewer->getCamera()->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    m_viewer->getCamera()->setViewMatrixAsLookAt(eye, target, normal);
    m_viewer->getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT);
    m_viewer->getCamera()->setGraphicsContext(m_graphicsWindow);

    m_viewer->setThreadingModel( osgViewer::Viewer::SingleThreaded );
    m_viewer->realize();

    this->setAutoBufferSwap(false);
}

OSGWidget::~OSGWidget()
{
}


Our3DObject* OSGWidget::getObject(int objectID)
{
    if(objectID < 0 || objectID >= m_objectsList.size())
        return NULL;
    else
        return m_objectsList[objectID];
}



// public slots

void OSGWidget::updateSceneRT(cv::Mat rotVec, cv::Mat tvecs)
{
    double t3 = tvecs.at<double>(1, 0);
    double t1 = -tvecs.at<double>(0, 0);
    double t2 = -tvecs.at<double>(2, 0);// - t3 / m_corrector; // and now, magic !

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

    m_mainMat->setMatrix(matrixR * matrix90 * matrix180 * matrixT);
}

void OSGWidget::addObjectToScene(QString objectPath)
{
    m_objectsList.push_back(new Our3DObject(objectPath));
    m_globalMat->addChild(m_objectsList[m_objectsList.size() - 1]);
}

void OSGWidget::displayObjectInScene(int objectID, bool display)
{
    std::cout << "ID : " << objectID << "    display : " << display << "    nbrObj : " << m_objectsList.size() << std::endl;
    if(objectID > 0 && objectID < m_objectsList.size())
    {
        if(display)
            m_globalMat->addChild(m_objectsList[objectID - 1]);
        else
            m_globalMat->removeChild(m_objectsList[objectID - 1]);
        m_objectsList[objectID - 1]->printObject(display);
    }
    else if(objectID == 0)
    {
        if(display)
            m_mainMat->addChild(m_globalMat);
        else
            m_mainMat->removeChild(m_globalMat);
        m_globalMat->printObject(display);
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
    m_backgroundImage->dirty();
    m_viewer->frame();
}

void OSGWidget::resizeGL( int width, int height )
{
    m_graphicsWindow->resized( this->x(), this->y(), width, height );

    m_viewer->getCamera()->setViewport( 0, 0, this->width(), this->height() );
}


// private

void OSGWidget::onHome()
{
    m_viewer->home();
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
