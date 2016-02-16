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
    SideViewOsgWidet( cv::Mat *webcamMat, QWidget* parent = 0,
               const QGLWidget* shareWidget = 0);

    virtual ~SideViewOsgWidet();

public slots:
    void updateSceneRT(cv::Mat rotVec, cv::Mat tvecs);

    void addObjectToScene(QString objectPath);

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
    osg::ref_ptr<osg::MatrixTransform> m_mat;

    std::vector<Our3DObject*> m_objectsList;

};

#endif // SIDEVIEWOSGWIDET_H
