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

    int getSizeX() { if(_sizeX >= 1) return 100 * (_sizeX - 1); else return -((100 / _sizeX) - 100); }
    int getSizeY() { if(_sizeY >= 1) return 100 * (_sizeY - 1); else return -((100 / _sizeY) - 100); }
    int getSizeZ() { if(_sizeZ >= 1) return 100 * (_sizeZ - 1); else return -((100 / _sizeZ) - 100); }

    int getRotX() { return _rotX; }
    int getRotY() { return _rotY; }
    int getRotZ() { return _rotZ; }

    int getTransX() { return _transX; }
    int getTransY() { return _transY; }
    int getTransZ() { return _transZ; }

    int getAlpha() { return _alpha * 100; }

    bool isObjectPrinted() { return _isPrinted; }

    // setters

    void setSizeX(int s) { if(s >= 0) _sizeX = 1 + s / 100; else _sizeX = 100 / (100 - s); update(); }
    void setSizeY(int s) { if(s >= 0) _sizeY = 1 + s / 100; else _sizeY = 100 / (100 - s); update(); }
    void setSizeZ(int s) { if(s >= 0) _sizeZ = 1 + s / 100; else _sizeZ = 100 / (100 - s); update(); }

    void setRotX(int r) { _rotX = r; update(); }
    void setRotY(int r) { _rotY = r; update(); }
    void setRotZ(int r) { _rotZ = r; update(); }

    void setTransX(int t) { _transX = t; update(); }
    void setTransY(int t) { _transY = t; update(); }
    void setTransZ(int t) { _transZ = t; update(); }

    void setAlpha(int alpha);

    void printObject(bool print) { _isPrinted = print; }

private:

    // members

    // attributes

    osg::Node* _object;
    osg::Material* _material;

    float _sizeX;
    float _sizeY;
    float _sizeZ;

    double _rotX;
    double _rotY;
    double _rotZ;

    int _transX;
    int _transY;
    int _transZ;

    float _alpha;

    bool _isPrinted;
};

#endif // OUR3DOBJECT_H
