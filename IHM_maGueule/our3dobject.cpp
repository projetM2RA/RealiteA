#include "our3dobject.h"

Our3DObject::Our3DObject() :
    QObject(0), m_isPrinted(true),
    m_sizeX(100.0), m_sizeY(100.0), m_sizeZ(100.0), m_alpha(1.0),
    m_rotX(0.0), m_rotY(0.0), m_rotZ(0.0), m_transX(0.0), m_transY(0.0), m_transZ(0.0)
{
    m_object = NULL;
    if(0)
    {
        m_material = new osg::Material;

        m_material->setAlpha(osg::Material::FRONT_AND_BACK, 1.0f); //Making alpha channel
        m_object->getOrCreateStateSet()->setAttributeAndModes(m_material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        m_object->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    }

    update();
}

Our3DObject::Our3DObject(QString objectPath) :
    QObject(0), m_isPrinted(true),
    m_sizeX(1.0), m_sizeY(1.0), m_sizeZ(1.0), m_alpha(1.0),
    m_rotX(0.0), m_rotY(0.0), m_rotZ(0.0), m_transX(0.0), m_transY(0.0), m_transZ(0.0)
{
    m_material = new osg::Material;
    m_object = osgDB::readNodeFile(objectPath.toStdString());
    if(0)
    {
        m_material = new osg::Material;

        m_material->setAlpha(osg::Material::FRONT_AND_BACK, 1.0f); //Making alpha channel
        m_object->getOrCreateStateSet()->setAttributeAndModes(m_material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        m_object->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    }
    this->addChild(m_object);

    update();
}

Our3DObject::~Our3DObject()
{
    if(m_isPrinted)
        this->removeChild(m_object);
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

    m_object->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
    return true;
}

void Our3DObject::update()
{
    osg::Matrixd matrixS; // scale
    matrixS.set(
        m_sizeX,0,      0,      0,
        0,      m_sizeY,0,      0,
        0,      0,      m_sizeZ,0,
        0,      0,      0,      1);

    osg::Matrixd matrixRot;
    matrixRot.makeRotate(osg::DegreesToRadians(m_rotX), osg::Vec3d(1.0, 0.0, 0.0),
                         osg::DegreesToRadians(m_rotY), osg::Vec3d(0.0, 1.0, 0.0),
                         osg::DegreesToRadians(m_rotZ), osg::Vec3d(0.0, 0.0, 1.0));

    osg::Matrixd matrixTrans;
    matrixTrans.makeTranslate(m_transX, m_transY, m_transZ);

    this->setMatrix(matrixS * matrixRot * matrixTrans);
}

void Our3DObject::setAlpha(int alpha)
{
    m_alpha = alpha / 100.0f;
    if(m_object)
    {
        m_material->setAlpha(osg::Material::FRONT_AND_BACK, m_alpha); //Making alpha channel
        m_object->getOrCreateStateSet()->setAttributeAndModes(m_material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        m_object->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    }
}


//void Our3DObject::printObject(bool print)
//{
//    if(m_isPrinted == print)
//        return;

//    m_isPrinted = print;
//    if(m_isPrinted)
//        this->addChild(m_object);
//    else
//        this->removeChild(m_object);
//}
