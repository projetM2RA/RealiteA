#include "sideviewosgwidet.h"

SideViewOsgWidet::SideViewOsgWidet(cv::Mat* webcamMat, osg::MatrixTransform *mainMat, Our3DObject* hud, QWidget* parent, const QGLWidget* shareWidget)
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

    double NEAR_VALUE = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2;

    fs.release();

    //////////////////////////////////////////////////
    ////////// initialisation OpenSceneGraph /////////
    //////////////////////////////////////////////////

    _group = new osg::Group;
    _mat = new osg::MatrixTransform();

    // Projection

    osg::Matrixd projectionMatrix;

    projectionMatrix.makeFrustum(
                -cameraMatrix.at<double>(0, 2),		webcamMat->cols - cameraMatrix.at<double>(0, 2),
                -cameraMatrix.at<double>(1, 2),		webcamMat->rows - cameraMatrix.at<double>(1, 2),
                NEAR_VALUE / 2,							20000*200);

    _corrector = (NEAR_VALUE / 2) / (cameraMatrix.at<double>(1, 2) - webcamMat->rows / 2);

    osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, 1000.0, 0.0f), normal(0.0f, 0.0f, 1.0f);

    _group->addChild(mainMat);
    _group->addChild(hud);

    //float aspectRatio = static_cast<float>(this->width()) / static_cast<float>( this->height() );
    _viewer->setSceneData(_group.get());

    osgGA::TrackballManipulator* manipulator = new osgGA::TrackballManipulator;
    manipulator->setAllowThrow( false );

    _viewer->setCameraManipulator( manipulator );
    _viewer->getCamera()->setProjectionMatrix(projectionMatrix);
    _viewer->getCamera()->setViewMatrixAsLookAt(eye, target, normal);
    _viewer->getCamera()->setGraphicsContext(_graphicsWindow);

    _viewer->setThreadingModel( osgViewer::Viewer::SingleThreaded );
    _viewer->realize();

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

    _mat->setMatrix(matrixR * matrix90 * matrix180 * matrixT);
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
    _viewer->frame();
}

void SideViewOsgWidet::resizeGL( int width, int height )
{
    _graphicsWindow->resized( this->x(), this->y(), width, height );

    _viewer->getCamera()->setViewport( 0, 0, this->width(), this->height() );
}



// private

void SideViewOsgWidet::onHome()
{
    _viewer->home();
}
