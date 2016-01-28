#include "our3dobject.h"

Our3DObject::Our3DObject(QString objectPath, QString name, bool isMask) :
    m_isMask(isMask), m_name(name), m_sizeX(100.0), m_sizeY(100.0), m_sizeZ(100.0),
    m_rotX(0.0), m_rotY(0.0), m_rotZ(0.0), m_transX(0.0), m_transY(0.0), m_transZ(0.0)
{
    object = osgDB::readNodeFile(objectPath.toStdString());
    if(m_isMask)
    {
        osg::ref_ptr<osg::Material> material = new osg::Material;

        material->setAlpha(osg::Material::FRONT_AND_BACK, 0.0f); //Making alpha channel
        object->getOrCreateStateSet()->setAttributeAndModes(material.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        object->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    }

    update();
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

    object->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
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
