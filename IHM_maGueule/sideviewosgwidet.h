#ifndef SIDEVIEWOSGWIDET_H
#define SIDEVIEWOSGWIDET_H

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
#include <osgGA/TrackballManipulator>

#include <opencv2/opencv.hpp>

#include <vector>

#include "our3dobject.h"



class SideViewOsgWidet : public QGLWidget
{
    Q_OBJECT

public:
    SideViewOsgWidet(cv::Mat *webcamMat, osg::MatrixTransform *mainMat, Our3DObject *hud, QWidget* parent = 0,
               const QGLWidget* shareWidget = 0);

    virtual ~SideViewOsgWidet();

public slots:
    void updateSceneRT(cv::Mat rotVec, cv::Mat tvecs);

protected:
    virtual void paintEvent( QPaintEvent* paintEvent );
    virtual void paintGL();
    virtual void resizeGL( int width, int height );

private:
    // members

    virtual void onHome();

    //attributes

    double _corrector;

    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _graphicsWindow;
    osg::ref_ptr<osgViewer::Viewer> _viewer;

    osg::ref_ptr<osg::Group> _group;
    osg::ref_ptr<osg::MatrixTransform> _mat;

    std::vector<Our3DObject*> _objectsList;

};

#endif // SIDEVIEWOSGWIDET_H
