#ifndef OUR3DOBJECT_H
#define OUR3DOBJECT_H

#include <QWidget>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osg/Texture2D>

class Our3DObject : public QObject, public osg::MatrixTransform
{
    Q_OBJECT

public:
    Our3DObject();
    Our3DObject(QString objectPath);
    //Our3DObject(const Our3DObject &obj);
    ~Our3DObject();

    bool applyTexture(QString texturePath);

    void update();




public slots:

    // getters

    int getSizeX() { if(m_sizeX >= 1) return 100 * (m_sizeX - 1); else return -((100 / m_sizeX) - 100); }
    int getSizeY() { if(m_sizeY >= 1) return 100 * (m_sizeY - 1); else return -((100 / m_sizeY) - 100); }
    int getSizeZ() { if(m_sizeZ >= 1) return 100 * (m_sizeZ - 1); else return -((100 / m_sizeZ) - 100); }

    int getRotX() { return m_rotX; }
    int getRotY() { return m_rotY; }
    int getRotZ() { return m_rotZ; }

    int getTransX() { return m_transX; }
    int getTransY() { return m_transY; }
    int getTransZ() { return m_transZ; }

    int getAlpha() { return m_alpha * 100; }

    bool isObjectPrinted() { return m_isPrinted; }

    // setters

    void setSizeX(int s) { if(s >= 0) m_sizeX = 1 + s / 100; else m_sizeX = 100 / (100 - s); update(); }
    void setSizeY(int s) { if(s >= 0) m_sizeY = 1 + s / 100; else m_sizeY = 100 / (100 - s); update(); }
    void setSizeZ(int s) { if(s >= 0) m_sizeZ = 1 + s / 100; else m_sizeZ = 100 / (100 - s); update(); }

    void setRotX(int r) { m_rotX = r; update(); }
    void setRotY(int r) { m_rotY = r; update(); }
    void setRotZ(int r) { m_rotZ = r; update(); }

    void setTransX(int t) { m_transX = t; update(); }
    void setTransY(int t) { m_transY = t; update(); }
    void setTransZ(int t) { m_transZ = t; update(); }

    void setAlpha(int alpha);

    void printObject(bool print) { m_isPrinted = print; }

private:

    // members

    // attributes

    osg::Node* m_object;
    osg::Material* m_material;

    float m_sizeX;
    float m_sizeY;
    float m_sizeZ;

    double m_rotX;
    double m_rotY;
    double m_rotZ;

    int m_transX;
    int m_transY;
    int m_transZ;

    float m_alpha;

    bool m_isPrinted;
};

#endif // OUR3DOBJECT_H
