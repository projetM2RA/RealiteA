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
    Our3DObject* getGlobalMat() { return m_globalMat; }

public slots:
    void displayObjects(bool removeObjects) { if(!removeObjects) m_mainCam->addChild(m_mainMat); else m_mainCam->removeChild(m_mainMat); }
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

    double m_corrector;

    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> m_graphicsWindow;
    osg::ref_ptr<osgViewer::Viewer> m_viewer;
    osg::ref_ptr<osg::Image> m_backgroundImage;

    osg::ref_ptr<osg::Geode> m_hud;
    osg::ref_ptr<osg::Group> m_group;
    osg::ref_ptr<osg::MatrixTransform> m_mainMat;
    Our3DObject* m_globalMat;
    osg::ref_ptr<osg::Camera> m_mainCam;
    osg::ref_ptr<osg::Camera> m_hudCam;

    std::vector<Our3DObject*> m_objectsList;
};

#endif
