#include "Our3DObject.h"

Our3DObject::Our3DObject() :
    QObject(0), _printed(true),
    _sizeX(10.0), _sizeY(10.0), _sizeZ(10.0), _alpha(10.0),
    _rotX(0.0), _rotY(0.0), _rotZ(0.0), _transX(0.0), _transY(0.0), _transZ(0.0)
{
    _object = NULL;

    update();
}

Our3DObject::Our3DObject(osg::Node *object) :
    QObject(0), _printed(true),
    _sizeX(1.0), _sizeY(1.0), _sizeZ(1.0), _alpha(1.0),
    _rotX(0.0), _rotY(0.0), _rotZ(0.0), _transX(0.0), _transY(0.0), _transZ(0.0),
    _object(object)
{
    _material = new osg::Material;
    this->addChild(_object);
    update();
}

Our3DObject::Our3DObject(QString objectPath) :
    QObject(0), _printed(true),
    _sizeX(1.0), _sizeY(1.0), _sizeZ(1.0), _alpha(1.0),
    _rotX(0.0), _rotY(0.0), _rotZ(0.0), _transX(0.0), _transY(0.0), _transZ(0.0)
{
    _material = new osg::Material;
    _object = osgDB::readNodeFile(objectPath.toStdString());
    this->addChild(_object);

    update();
}

Our3DObject::~Our3DObject()
{
    this->removeChild(_object);
}

bool Our3DObject::applyTexture(QString texturePath)
{
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setDataVariance(osg::Object::DYNAMIC);
    osg::Image* textureImage = osgDB::readImageFile(texturePath.toStdString());
    if (!textureImage)
        return false;
    else
        texture->setImage(textureImage);

    _object->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
    return true;
}

void Our3DObject::update()
{
    osg::Matrixd matrixS; // scale
    matrixS.set(
                _sizeX,0,      0,      0,
                0,      _sizeY,0,      0,
                0,      0,      _sizeZ,0,
                0,      0,      0,      1);

    osg::Matrixd matrixRot;
    matrixRot.makeRotate(osg::DegreesToRadians(_rotX), osg::Vec3d(1.0, 0.0, 0.0),
                         osg::DegreesToRadians(_rotY), osg::Vec3d(0.0, 1.0, 0.0),
                         osg::DegreesToRadians(_rotZ), osg::Vec3d(0.0, 0.0, 1.0));

    osg::Matrixd matrixTrans;
    matrixTrans.makeTranslate(_transX, _transY, _transZ);

    this->setMatrix(matrixS * matrixRot * matrixTrans);
}

void Our3DObject::setAlpha(int alpha)
{
    _alpha = alpha / 100.0f;
    if(_object)
    {
        _material->setAlpha(osg::Material::FRONT_AND_BACK, _alpha); //Making alpha channel
        _object->getOrCreateStateSet()->setAttributeAndModes(_material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        _object->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    }
}
