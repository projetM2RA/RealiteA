#include "sideviewosgwidet.h"

SideViewOsgWidet::SideViewOsgWidet(cv::Mat* webcamMat, osg::MatrixTransform *mainMat, QWidget* parent, const QGLWidget* shareWidget)
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
