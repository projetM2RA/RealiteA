#ifndef OSGWidget_h__
#define OSGWidget_h__

#include <QGLWidget>

#include <osg/ref_ptr>

#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>

#include <osg/Camera>

#include <osg/DisplaySettings>
#include <osg/Geode>
#include <osg/Material>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>

#include <osgDB/ReadFile>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osg/TextureRectangle>
#include <osg/Array>

#include <opencv2/opencv.hpp>

#include <vector>

#include "our3dobject.h"

class OSGWidget : public QGLWidget
{
    Q_OBJECT

public:
    OSGWidget( cv::Mat *webcamMat, QWidget* parent = 0,
               const QGLWidget* shareWidget = 0);

    virtual ~OSGWidget();

    Our3DObject* getObject(int objectID);
    Our3DObject* getGlobalMat() { return _globalMat; }

public slots:
    void displayObjects(bool removeObjects) { if(!removeObjects) _mainCam->addChild(_mainMat); else _mainCam->removeChild(_mainMat); }
    void updateSceneRT(cv::Mat rotVec, cv::Mat tvecs);

    void addObjectToScene(QString objectPath);
    void displayObjectInScene(int objectID, bool display);

protected:
    virtual void paintEvent( QPaintEvent* paintEvent );
    virtual void paintGL();
    virtual void resizeGL( int width, int height );

private:
    // members

    virtual void onHome();
    osg::Geode* createHUD(osg::Image* bgImage, int camWidth, int camHeight, double cx, double cy, double n);

    //attributes

    double _corrector;

    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _graphicsWindow;
    osg::ref_ptr<osgViewer::Viewer> _viewer;
    osg::ref_ptr<osg::Image> _backgroundImage;

    osg::ref_ptr<osg::Geode> _hud;
    osg::ref_ptr<osg::Group> _group;
    osg::ref_ptr<osg::MatrixTransform> _mainMat;
    Our3DObject* _globalMat;
    osg::ref_ptr<osg::Camera> _mainCam;
    osg::ref_ptr<osg::Camera> _hudCam;

    std::vector<Our3DObject*> _objectsList;
};

#endif
