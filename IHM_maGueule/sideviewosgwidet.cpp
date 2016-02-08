#include "sideviewosgwidet.h"

SideViewOsgWidet::SideViewOsgWidet(cv::Mat* webcamMat, QWidget* parent, const QGLWidget* shareWidget)
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

    double NEAR_VALUE = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2;

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

    m_group = new osg::Group;
    m_mat = new osg::MatrixTransform();

    // Projection

    osg::Matrixd projectionMatrix;

    projectionMatrix.makeFrustum(
                -cameraMatrix.at<double>(0, 2),		webcamMat->cols - cameraMatrix.at<double>(0, 2),
                -cameraMatrix.at<double>(1, 2),		webcamMat->rows - cameraMatrix.at<double>(1, 2),
                NEAR_VALUE / 2,							20000*200);

    m_corrector = (NEAR_VALUE / 2) / (cameraMatrix.at<double>(1, 2) - webcamMat->rows / 2);

    osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, 1000.0, 0.0f), normal(0.0f, 0.0f, 1.0f);

    m_group->addChild(m_mat);
    m_group->addChild(m_hud);

    //float aspectRatio = static_cast<float>(this->width()) / static_cast<float>( this->height() );
    m_viewer->setSceneData(m_group.get());

    osgGA::TrackballManipulator* manipulator = new osgGA::TrackballManipulator;
    manipulator->setAllowThrow( false );

    m_viewer->setCameraManipulator( manipulator );
    m_viewer->getCamera()->setProjectionMatrix(projectionMatrix);
    m_viewer->getCamera()->setViewMatrixAsLookAt(eye, target, normal);
    m_viewer->getCamera()->setGraphicsContext(m_graphicsWindow);

    m_viewer->setThreadingModel( osgViewer::Viewer::SingleThreaded );
    m_viewer->realize();

    this->setAutoBufferSwap(false);
}

SideViewOsgWidet::~SideViewOsgWidet()
{
}



// public slots
void SideViewOsgWidet::updateSceneRT(cv::Mat rotVec, cv::Mat tvecs)
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

    m_mat->setMatrix(matrixR * matrix90 * matrix180 * matrixT);
}

void SideViewOsgWidet::addObjectToScene(QString objectPath)
{
    m_objectsList.push_back(new Our3DObject(objectPath));
    m_mat->addChild(m_objectsList[0]);
}




// protected

void SideViewOsgWidet::paintEvent( QPaintEvent* /* paintEvent */ )
{
    this->makeCurrent();

    QPainter painter( this );
    painter.setRenderHint( QPainter::Antialiasing );

    this->paintGL();

    painter.end();

    this->swapBuffers();
    this->doneCurrent();
}

void SideViewOsgWidet::paintGL()
{
    m_backgroundImage->dirty();
    m_viewer->frame();
}

void SideViewOsgWidet::resizeGL( int width, int height )
{
    m_graphicsWindow->resized( this->x(), this->y(), width, height );

    m_viewer->getCamera()->setViewport( 0, 0, this->width(), this->height() );
}


// private

void SideViewOsgWidet::onHome()
{
    m_viewer->home();
}

osg::Geode* SideViewOsgWidet::createHUD(osg::Image* bgImage, int camWidth, int camHeight, double cx, double cy, double n)
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
