#include "osgwidget.h"

OSGWidget::OSGWidget(cv::Mat* webcamMat, osg::MatrixTransform *mainMat, Our3DObject* hud, QWidget* parent, const QGLWidget* shareWidget)
    : QGLWidget( parent, shareWidget)
    , _graphicsWindow( new osgViewer::GraphicsWindowEmbedded( this->x(),
                                                               this->y(),
                                                               this->width(),
                                                               this->height()) )
    , _viewer( new osgViewer::Viewer )
{

    cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

    cv::Mat cameraMatrix;

    fs["cameraMatrix"] >> cameraMatrix;

    double NEAR_VALUE = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2;
    double FAR_VALUE = 2000 * NEAR_VALUE;

    fs.release();

    //////////////////////////////////////////////////
    ////////// initialisation OpenSceneGraph /////////
    //////////////////////////////////////////////////

    _group = new osg::Group();
    _mainCam = new osg::Camera();
    _hudCam = new osg::Camera();

    // Projection

    osg::Matrixd projectionMatrix;

    projectionMatrix.makeFrustum(
                -cameraMatrix.at<double>(0, 2),		webcamMat->cols - cameraMatrix.at<double>(0, 2),
                -cameraMatrix.at<double>(1, 2),		webcamMat->rows - cameraMatrix.at<double>(1, 2),
                NEAR_VALUE,							FAR_VALUE);

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

    _mainCam->addChild(mainMat);
    _hudCam->addChild(hud);

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
