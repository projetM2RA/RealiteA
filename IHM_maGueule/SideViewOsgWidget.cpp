#include "SideViewOsgWidget.h"

SideViewOsgWidet::SideViewOsgWidet(cv::Mat* webcamMat, osg::MatrixTransform *mainMat, Our3DObject *hud, QWidget* parent, const QGLWidget* shareWidget)
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

    // Projection

    osg::Matrixd projectionMatrix;

    projectionMatrix.makeFrustum(
                -cameraMatrix.at<double>(0, 2),		webcamMat->cols - cameraMatrix.at<double>(0, 2),
                -cameraMatrix.at<double>(1, 2),		webcamMat->rows - cameraMatrix.at<double>(1, 2),
                NEAR_VALUE / 2,							20000*200);

    _corrector = (NEAR_VALUE / 2) / (cameraMatrix.at<double>(1, 2) - webcamMat->rows / 2);

    osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, 1000.0, 0.0f), normal(0.0f, 0.0f, 1.0f);

    _group->addChild(hud);
    _group->addChild(mainMat);

    //float aspectRatio = static_cast<float>(this->width()) / static_cast<float>( this->height() );
    _viewer->setSceneData(_group.get());

    osgGA::TrackballManipulator* manipulator = new osgGA::TrackballManipulator;
    manipulator->setAllowThrow(false);

    _viewer->setCameraManipulator( manipulator );
    _viewer->getCamera()->setProjectionMatrix(projectionMatrix);
    _viewer->getCamera()->setViewMatrixAsLookAt(eye, target, normal);
    _viewer->getCamera()->setGraphicsContext(_graphicsWindow);

    _viewer->setThreadingModel( osgViewer::Viewer::SingleThreaded );
    _viewer->realize();

    this->setAutoBufferSwap(false);
    this->setMouseTracking(true);
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



void SideViewOsgWidet::mouseMoveEvent( QMouseEvent* event )
{
    this->getEventQueue()->mouseMotion( static_cast<float>( event->x() ),
                                        static_cast<float>( event->y() ) );
}

void SideViewOsgWidet::mousePressEvent( QMouseEvent* event )
{
    // 1 = left mouse button
    // 2 = middle mouse button
    // 3 = right mouse button

    unsigned int button = 0;

    switch( event->button() )
    {
    case Qt::LeftButton:
        button = 1;
        break;

    case Qt::MiddleButton:
        button = 2;
        break;

    case Qt::RightButton:
        button = 3;
        break;

    default:
        break;
    }

    this->getEventQueue()->mouseButtonPress( static_cast<float>( event->x() ),
                                             static_cast<float>( event->y() ),
                                             button );
}

void SideViewOsgWidet::mouseReleaseEvent(QMouseEvent* event)
{
    // 1 = left mouse button
    // 2 = middle mouse button
    // 3 = right mouse button

    unsigned int button = 0;

    switch( event->button() )
    {
    case Qt::LeftButton:
        button = 1;
        break;

    case Qt::MiddleButton:
        button = 2;
        break;

    case Qt::RightButton:
        button = 3;
        break;

    default:
        break;
    }

    this->getEventQueue()->mouseButtonRelease( static_cast<float>( event->x() ),
                                               static_cast<float>( event->y() ),
                                               button );
}

void SideViewOsgWidet::wheelEvent( QWheelEvent* event )
{
    event->accept();
    int delta = event->delta();

    osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?   osgGA::GUIEventAdapter::SCROLL_UP
                                                                 : osgGA::GUIEventAdapter::SCROLL_DOWN;

    this->getEventQueue()->mouseScroll( motion );
}

bool SideViewOsgWidet::event( QEvent* event )
{
    bool handled = QGLWidget::event( event );

    // This ensures that the OSG widget is always going to be repainted after the
    // user performed some interaction. Doing this in the event handler ensures
    // that we don't forget about some event and prevents duplicate code.
    switch( event->type() )
    {
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
    case QEvent::Wheel:
        this->update();
        break;

    default:
        break;
    }

    return handled;
}
// private

void SideViewOsgWidet::onHome()
{
    _viewer->home();
}

osgGA::EventQueue* SideViewOsgWidet::getEventQueue() const
{
    osgGA::EventQueue* eventQueue = _graphicsWindow->getEventQueue();

    if( eventQueue )
        return eventQueue;
    else
        throw std::runtime_error( "Unable to obtain valid event queue");
}
