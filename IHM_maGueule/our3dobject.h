#ifndef OUR3DOBJECT_H
#define OUR3DOBJECT_H

#include <QWidget>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osg/Texture2D>

class Our3DObject : public osg::MatrixTransform
{
public:
    Our3DObject(QString objectPath, QString name, bool isMask);

    bool applyTexture(QString texturePath);

    // getters
    QString getName() { return m_name; }
    bool isMask() { return m_isMask; }

    double getSizeX() { return m_sizeX; }
    double getSizeY() { return m_sizeY; }
    double getSizeZ() { return m_sizeZ; }

    double getRotX() { return m_rotX; }
    double getRotY() { return m_rotY; }
    double getRotZ() { return m_rotZ; }

    double getTransX() { return m_transX; }
    double getTransY() { return m_transY; }
    double getTransZ() { return m_transZ; }

    // setters
    void setSizeX(double s) { m_sizeX = s; update(); }
    void setSizeY(double s) { m_sizeY = s; update(); }
    void setSizeZ(double s) { m_sizeZ = s; update(); }

    void setRotX(double r) { m_rotX = r; update(); }
    void setRotY(double r) { m_rotY = r; update(); }
    void setRotZ(double r) { m_rotZ = r; update(); }

    void setTransX(double t) { m_transX = t; update(); }
    void setTransY(double t) { m_transY = t; update(); }
    void setTransZ(double t) { m_transZ = t; update(); }

    void update();

private:
    QString m_name;
    bool m_isMask;
    osg::Node* object;

    double m_sizeX;
    double m_sizeY;
    double m_sizeZ;

    double m_rotX;
    double m_rotY;
    double m_rotZ;

    double m_transX;
    double m_transY;
    double m_transZ;
};

#endif // OUR3DOBJECT_H
