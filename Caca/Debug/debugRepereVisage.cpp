#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Camera>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/TextureRectangle>
#include <osg/Array>
#include <osgViewer/CompositeViewer>
#include <osg/ShapeDrawable>

#include <opencv2/opencv.hpp>
#include <Chehra.h>

#include <stdio.h>
#include <time.h>

#define COLCHESSBOARD			9
#define ROWCHESSBOARD			6
#define SIZEMIRE				26

#define NBRSAVEDIMAGES			5
#define NBRFACEPOINTSDETECTED	6
#define PI						3.14159265359

bool detecterVisage(cv::Mat* imCalibColor, Chehra *chehra, std::vector<cv::Point2f> *pointsVisage)
{
    bool visageFound = false;
    cv::Mat imNB;
    cv::Mat points;

    cv::cvtColor(*imCalibColor, imNB, CV_BGR2GRAY);

    visageFound = (*chehra).track(imNB);

    if(visageFound)
    {
        points = (*chehra).getTrackedPoints();
        if (points.rows == 98)
            for(int i = 0; i < 49; i++)
            {
                (*pointsVisage).push_back(cv::Point2f(points.at<float>(14, 0), points.at<float>(14 + 49, 0)));
                (*pointsVisage).push_back(cv::Point2f(points.at<float>(18, 0), points.at<float>(18 + 49, 0)));
                (*pointsVisage).push_back(cv::Point2f(points.at<float>(13, 0), points.at<float>(13 + 49, 0)));
                (*pointsVisage).push_back(cv::Point2f(points.at<float>(19, 0), points.at<float>(19 + 49, 0)));
                (*pointsVisage).push_back(cv::Point2f(points.at<float>(10, 0), points.at<float>(10 + 49, 0)));
                (*pointsVisage).push_back(cv::Point2f(points.at<float>(28, 0), points.at<float>(28 + 49, 0)));
            }
    }

    return visageFound; // si echap est presse, patternFound = false
}

osg::Geode* createHUD(osg::Image* bgImage, int camWidth, int camHeight, double cx, double cy, double n)
{
    osg::Geometry* geoQuad = new osg::Geometry;

    osg::Vec3Array* tabSommet = new osg::Vec3Array;
    tabSommet->push_back(osg::Vec3(-cx, n, -cy));
    tabSommet->push_back(osg::Vec3(camWidth - cx, n, -cy));
    tabSommet->push_back(osg::Vec3(camWidth - cx, n, camHeight - cy));
    tabSommet->push_back(osg::Vec3(-cx, n, camHeight - cy));
    geoQuad->setVertexArray(tabSommet);

    osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    primitive->push_back(0);
    primitive->push_back(1);
    primitive->push_back(2);
    primitive->push_back(3);
    geoQuad->addPrimitiveSet(primitive);

    // Nous créons ensuite une tableau qui contiendra nos coordonnées de texture.
    osg::Vec2Array* coordonneeTexture = new osg::Vec2Array(4);
    (*coordonneeTexture)[0].set(0.0f, 1.0f);
    (*coordonneeTexture)[1].set(1.0f, 1.0f);
    (*coordonneeTexture)[2].set(1.0f, 0.0f);
    (*coordonneeTexture)[3].set(0.0f, 0.0f);
    geoQuad->setTexCoordArray(0, coordonneeTexture);

    osg::Geode* noeudGeo = new osg::Geode;
    osg::StateSet* statuts = noeudGeo->getOrCreateStateSet();
    statuts->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    noeudGeo->addDrawable(geoQuad);

    // Nous créons une Texture2D.
    osg::Texture2D* texture = new osg::Texture2D;

    // Nous associons notre image à notre objet Texture2D.
    texture->setImage(bgImage);
    texture->setResizeNonPowerOfTwoHint(false);

    // Enfin nous activons les texture de notre objet Geometry à travers l'objet statuts.
    statuts->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
    statuts->setMode(GL_BLEND,osg::StateAttribute::ON);
    statuts->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    return noeudGeo;
}

osg::MatrixTransform* chargerCerveau2()
{

    osg::ref_ptr<osg::Node> objetCerveau = osgDB::readNodeFile("../rsc/objets3D/grayMatter.stl");

    if(objetCerveau == NULL)
        std::cout << "PAS DE CERVEAU !" << std::endl;

    osg::StateSet* cerveauStateset = objetCerveau->getOrCreateStateSet();
    cerveauStateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    //cerveauStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
    osg::ref_ptr<osg::Material> material = new osg::Material;

    material->setAlpha(osg::Material::FRONT_AND_BACK, 0.4f); // Alpha channel
    cerveauStateset->setAttributeAndModes( material.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    cerveauStateset->setMode( GL_BLEND, osg::StateAttribute::ON );
    //masqueStateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    objetCerveau->setStateSet(cerveauStateset);

    osg::MatrixTransform* mat = new osg::MatrixTransform();
    mat->addChild(objetCerveau);

    double sx = 9;
    double sy = 9;
    double sz = 9;

    osg::Matrixd matrixS; // scale
    matrixS.set(
        sx,	0,	0,	0,
        0,	sy,	0,	0,
        0,	0,	sz,	0,
        0,	0,	0,	1);

    osg::Matrixd matrixRot;
    matrixRot.makeRotate(osg::Quat(osg::DegreesToRadians(180.0f), osg::Vec3d(0.0, 0.0, 1.0)));

    osg::Matrixd matrixRotBis;
    matrixRotBis.makeRotate(osg::Quat(osg::DegreesToRadians(-10.0f), osg::Vec3d(0.0, 1.0, 0.0)));

    osg::Matrixd matrixTrans;
    matrixTrans.makeTranslate(0,1000,0);

    mat->setMatrix(matrixS * matrixRot * matrixRotBis * matrixTrans);

    return mat;
}

osg::MatrixTransform* chargerCerveau()
{

    osg::ref_ptr<osg::Node> objetCerveau = osgDB::readNodeFile("../rsc/objets3D/whiteMatter.stl");

    if(objetCerveau == NULL)
        std::cout << "PAS DE CERVEAU !" << std::endl;

//    osg::StateSet* cerveauStateset = objetCerveau->getOrCreateStateSet();
//    cerveauStateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
//    //cerveauStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
//    osg::ref_ptr<osg::Material> material = new osg::Material;

//    material->setAlpha(osg::Material::FRONT_AND_BACK, 0.5f); // Alpha channel
//    cerveauStateset->setAttributeAndModes( material.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

//    cerveauStateset->setMode( GL_BLEND, osg::StateAttribute::ON );
//    //masqueStateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

//    objetCerveau->setStateSet(cerveauStateset);

    osg::MatrixTransform* mat = new osg::MatrixTransform();
    mat->addChild(objetCerveau);

    double sx = 9;
    double sy = 9;
    double sz = 9;

    osg::Matrixd matrixS; // scale
    matrixS.set(
        sx,	0,	0,	0,
        0,	sy,	0,	0,
        0,	0,	sz,	0,
        0,	0,	0,	1);

    osg::Matrixd matrixRot;
    matrixRot.makeRotate(osg::Quat(osg::DegreesToRadians(180.0f), osg::Vec3d(0.0, 0.0, 1.0)));

    osg::Matrixd matrixRotBis;
    matrixRotBis.makeRotate(osg::Quat(osg::DegreesToRadians(-10.0f), osg::Vec3d(0.0, 1.0, 0.0)));

    osg::Matrixd matrixTrans;
    matrixTrans.makeTranslate(0,1000,0);

    mat->setMatrix(matrixS * matrixRot * matrixRotBis * matrixTrans);

    return mat;
}

osg::MatrixTransform* chargerMasque()
{
    osg::ref_ptr<osg::Node> masque = osgDB::readNodeFile("../rsc/objets3D/head.obj");

    if(masque == NULL)
        std::cout << "PAS DE TETE !" << std::endl;

    osg::StateSet* masqueStateset = masque->getOrCreateStateSet();
    masqueStateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    //masqueStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
    osg::ref_ptr<osg::Material> material = new osg::Material;

    material->setAlpha(osg::Material::FRONT_AND_BACK, 0.0f); // Alpha channel
    masqueStateset->setAttributeAndModes( material.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    masqueStateset->setMode( GL_BLEND, osg::StateAttribute::ON );
    //masqueStateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    masque->setStateSet(masqueStateset);

    osg::MatrixTransform* mat = new osg::MatrixTransform();
    mat->addChild(masque);

    double sx = 120;
    double sy = 80;
    double sz = 130;

    osg::Matrixd matrixS; // scale
    matrixS.set(
        sx,	0,	0,	0,
        0,	sy,	0,	0,
        0,	0,	sz,	0,
        0,	0,	0,	1);

    osg::Matrixd matrixRot;
    matrixRot.makeRotate(osg::Quat(osg::DegreesToRadians(0.0f), osg::Vec3d(0.0, 0.0, 1.0)));

    osg::Matrixd matrixRotBis;
    matrixRotBis.makeRotate(osg::Quat(osg::DegreesToRadians(-30.0f), osg::Vec3d(1.0, 0.0, 0.0)));

    osg::Matrixd matrixTrans;
    matrixTrans.makeTranslate(0,0,30);

    mat->setMatrix(matrixS);

    return mat;
}

osg::MatrixTransform* chargerLunettes()
{
    osg::ref_ptr<osg::Node> objetGlasses = osgDB::readNodeFile("../rsc/objets3D/GlassesSwag.3DS");

    //obectStateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    //obectStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

    osg::MatrixTransform* mat = new osg::MatrixTransform();
    mat->addChild(objetGlasses);

    double sx = 45;
    double sy = 42;
    double sz = 42;

    osg::Matrixd matrixS; // scale
    matrixS.set(
        sx,	0,	0,	0,
        0,	sy,	0,	0,
        0,	0,	sz,	0,
        0,	0,	0,	1);

    osg::Matrixd matrixRot;
    matrixRot.makeRotate(osg::Quat(osg::DegreesToRadians(-90.0f), osg::Vec3d(0.0, 0.0, 1.0)));

    osg::Matrixd matrixRotBis;
    matrixRotBis.makeRotate(osg::Quat(osg::DegreesToRadians(-35.0f), osg::Vec3d(1.0, 0.0, 0.0)));

    osg::Matrixd matrixTrans;
    matrixTrans.makeTranslate(0,665,-153);

    mat->setMatrix(matrixS * matrixTrans);

    return mat;
}

osg::MatrixTransform* chargerBunny()
{
    osg::ref_ptr<osg::Node> objetBunny = osgDB::readNodeFile("../rsc/objets3D/bunny/ItmUsagiHat.obj");

    osg::StateSet* obectStateset3 = objetBunny->getOrCreateStateSet();
    obectStateset3->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    //obectStateset3->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

    osg::Texture2D* textureLapin = new osg::Texture2D;
    textureLapin->setDataVariance(osg::Object::DYNAMIC);
    osg::Image* texture3ON = osgDB::readImageFile("../rsc/objets3D/bunny/earfix.bmp");
    if (!texture3ON)
    {
        std::cout << " couldn't find texture, quiting." << std::endl;
        // SORTIE
    }
    else
        textureLapin->setImage(texture3ON);

    obectStateset3->setTextureAttributeAndModes(0,textureLapin,osg::StateAttribute::ON);
    objetBunny->setStateSet(obectStateset3);

    osg::MatrixTransform* mat = new osg::MatrixTransform();
    mat->addChild(objetBunny);

    double s = 300;

    osg::Matrixd matrixS; // scale
    matrixS.set(
        s,	0,	0,	0,
        0,	s,	0,	0,
        0,	0,	s,	0,
        0,	0,	0,	1);

    osg::Matrixd matrixRot;
    matrixRot.makeRotate(osg::Quat(osg::DegreesToRadians(-55.0f), osg::Vec3d(1.0, 0.0, 0.0)));

    osg::Matrixd matrixRotBis;
    matrixRotBis.makeRotate(osg::Quat(osg::DegreesToRadians(180.0f), osg::Vec3d(0.0, 0.0, 1.0)));

    osg::Matrixd matrixTrans;
    matrixTrans.makeTranslate(0,800,150);

    mat->setMatrix(matrixS * matrixTrans);

    return mat;
}

osg::MatrixTransform* chargerMoustache()
{
    osg::ref_ptr<osg::Node> objetMoustache = osgDB::readNodeFile("../rsc/objets3D/Mustache.obj");

    osg::StateSet* obectStateset2 = objetMoustache->getOrCreateStateSet();
    obectStateset2->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    //obectStateset2->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

    osg::Texture2D* textureMoustache = new osg::Texture2D;
    textureMoustache->setDataVariance(osg::Object::DYNAMIC);
    osg::Image* texture2ON = osgDB::readImageFile("../rsc/objets3D/MustacheUV-textureMap.bmp");
    if (!texture2ON)
    {
        std::cout << " couldn't find texture, quiting." << std::endl;
        // SORTIE
    }
    else
        textureMoustache->setImage(texture2ON);

    obectStateset2->setTextureAttributeAndModes(0,textureMoustache,osg::StateAttribute::ON);
    objetMoustache->setStateSet(obectStateset2);

    osg::MatrixTransform* mat = new osg::MatrixTransform();
    mat->addChild(objetMoustache);

    double sx = 230;
    double sy = 180;
    double sz = 160;

    osg::Matrixd matrixS; // scale
    matrixS.set(
        sx,	0,	0,	0,
        0,	sy,	0,	0,
        0,	0,	sz,	0,
        0,	0,	0,	1);

    osg::Matrixd matrixRot;
    matrixRot.makeRotate(osg::Quat(osg::DegreesToRadians(180.0f), osg::Vec3d(0.0, 0.0, 1.0)));

    osg::Matrixd matrixTrans;
    matrixTrans.makeTranslate(0,-100,-480);

    mat->setMatrix(matrixS * matrixRot * matrixTrans);

    return mat;
}

osg::MatrixTransform* chargerAxes()
{
    osg::ref_ptr<osg::Node> objetGlasses = osgDB::readNodeFile("../rsc/objets3D/osg-data-master/axes.osgt");

    //obectStateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    //obectStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

    osg::MatrixTransform* mat = new osg::MatrixTransform();
    mat->addChild(objetGlasses);

    double sx = 420;
    double sy = 420;
    double sz = 420;

    osg::Matrixd matrixS; // scale
    matrixS.set(
        sx,	0,	0,	0,
        0,	sy,	0,	0,
        0,	0,	sz,	0,
        0,	0,	0,	1);

    osg::Matrixd matrixRot;
    matrixRot.makeRotate(osg::Quat(osg::DegreesToRadians(-90.0f), osg::Vec3d(0.0, 0.0, 1.0)));

    osg::Matrixd matrixRotBis;
    matrixRotBis.makeRotate(osg::Quat(osg::DegreesToRadians(-35.0f), osg::Vec3d(1.0, 0.0, 0.0)));

    osg::Matrixd matrixTrans;
    matrixTrans.makeTranslate(0,665,-153);

    mat->setMatrix(matrixS);

    return mat;
}

std::vector<cv::Point2f> dessinerPoints(cv::Mat* imCalibColor, const std::vector<cv::Point3f> & objectPoints, const cv::Mat & rotVec, const cv::Mat & tvecs, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs, bool dessiner)
{
    std::vector<cv::Point2f> imagePoints, repereMire;
    std::vector<cv::Point3f> repere3D;
    repere3D.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
    repere3D.push_back(cv::Point3f(78.0f, 0.0f, 0.0f));
    repere3D.push_back(cv::Point3f(0.0f, 78.0f, 0.0f));
    repere3D.push_back(cv::Point3f(0.0f, 0.0f, 78.0f));

    // Projection des points
    cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);
    cv::projectPoints(repere3D, rotVec, tvecs, cameraMatrix, distCoeffs, repereMire);

    // Dessin des points projetes
    if(dessiner)
        for(int m = 0; m < objectPoints.size(); m++)
            cv::circle(*imCalibColor, cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

    cv::circle(*imCalibColor, cv::Point(repereMire[0].x, repereMire[0].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
    cv::circle(*imCalibColor, cv::Point(repereMire[1].x, repereMire[1].y), 3, cv::Scalar(0, 255, 0), 1, 8, 0);
    cv::circle(*imCalibColor, cv::Point(repereMire[2].x, repereMire[2].y), 3, cv::Scalar(255, 0, 0), 1, 8, 0);
    cv::circle(*imCalibColor, cv::Point(repereMire[3].x, repereMire[3].y), 3, cv::Scalar(255, 0, 255), 1, 8, 0);

    cv::line(*imCalibColor, repereMire[0], repereMire[1], cv::Scalar(0,255,0), 2, 8);
    cv::line(*imCalibColor, repereMire[0], repereMire[2], cv::Scalar(255,0,0), 2, 8);
    cv::line(*imCalibColor, repereMire[0], repereMire[3], cv::Scalar(255,0,255), 2, 8);


    return imagePoints;
    //return repereMire;
}

void main()
{
    //////////////////////////////////////////////////
    //////////	initialisation des variables /////////
    //////////////////////////////////////////////////

    bool detectionVisage = false;
    bool objectAdded = true;

    int nbrLoopSinceLastDetection = 0;

    std::cout << "initialisation de Chehra..." << std::endl;
    Chehra chehra;
    std::cout << "done" << std::endl;

    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat rvecs, tvecs;

    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point2f> pointsVisage2D;
    std::vector<cv::Point2f> moyPointsVisage2D;
    std::vector<cv::Point3f> pointsVisage3D;

    std::vector<std::vector<cv::Point2f>> images;
    std::vector<cv::Mat> frames;
    /*
    pointsVisage3D.push_back(cv::Point3f(-110, -573, -104)); // exterieur narine gauche
    pointsVisage3D.push_back(cv::Point3f(110, -573, -104)); // exterieur narine droite
    pointsVisage3D.push_back(cv::Point3f(0, -715, 25)); // bout du nez
    pointsVisage3D.push_back(cv::Point3f(-343, -322, 163)); // exterieur oeil gauche
    pointsVisage3D.push_back(cv::Point3f(0, -568, 250)); // haut du nez, centre des yeux
    pointsVisage3D.push_back(cv::Point3f(343, -322, 163)); // exterieur oeil droit
    */
//    pointsVisage3D.push_back(cv::Point3f(-110, 0, -336)); // exterieur narine gauche sur l'image
//    pointsVisage3D.push_back(cv::Point3f(110, 0, -336)); // exterieur narine droite sur l'image
//    pointsVisage3D.push_back(cv::Point3f(0, -142, -258)); // bout du nez
//    pointsVisage3D.push_back(cv::Point3f(-338, 243, -70)); // exterieur oeil gauche sur l'image
//    pointsVisage3D.push_back(cv::Point3f(0, 0, 0)); // haut du nez, centre des yeux
//    pointsVisage3D.push_back(cv::Point3f(338, 243, -70)); // exterieur oeil droit sur l'image

    double R = 676/120;

    pointsVisage3D.push_back(cv::Point3f(-110/R, 0, -336/R));       // exterieur narine gauche sur l'image
    pointsVisage3D.push_back(cv::Point3f(110/R, 0, -336/R));        // exterieur narine droite sur l'image
    pointsVisage3D.push_back(cv::Point3f(0, -142/R, -258/R));       // bout du nez
    pointsVisage3D.push_back(cv::Point3f(-338/R, 243/R, -70/R));    // exterieur oeil gauche sur l'image
    pointsVisage3D.push_back(cv::Point3f(0, 0, 0));                 // haut du nez, centre des yeux
    pointsVisage3D.push_back(cv::Point3f(338/R, 243/R, -70/R));     // exterieur oeil droit sur l'image

    /*
    pointsVisage3D.push_back(cv::Point3f(90,0,-680)); // exterieur narine gauche
    pointsVisage3D.push_back(cv::Point3f(-90,0,-680)); // exterieur narine droite
    pointsVisage3D.push_back(cv::Point3f(0,0,-600)); // bout du nez
    pointsVisage3D.push_back(cv::Point3f(600,0,0)); // exterieur oeil gauche
    pointsVisage3D.push_back(cv::Point3f(0,0,0)); // haut du nez, centre des yeux
    pointsVisage3D.push_back(cv::Point3f(-600,0,0)); // exterieur oeil droit
    */

    //////////////////////////////////////////////////
    //////////	initialisation OpenCV ////////////////
    //////////////////////////////////////////////////

    cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;

    double f = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carrés, fx = fy -> np
    //mais est généralement différent de fy donc on prend (pour l'instant) par défaut la valeur médiane
    double g = 2000 * f;

    fs.release();

    cv::VideoCapture vcap(0);
    if(!vcap.isOpened())
    {
        std::cout << "FAIL!" << std::endl;
        return;
    }
    //vcap.set(CV_CAP_PROP_FPS, 30);
    cv::Mat *frame = new cv::Mat(cv::Mat::zeros(vcap.get(CV_CAP_PROP_FRAME_HEIGHT), vcap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3));

    do
    {
        vcap >> *frame;
    }while(frame->empty());

    //////////////////////////////////////////////////
    ////////// initialisation OpenSceneGraph /////////
    //////////////////////////////////////////////////

    osg::ref_ptr<osg::Image> backgroundImage = new osg::Image;
    backgroundImage->setImage(frame->cols, frame->rows, 3,
        GL_RGB, GL_BGR, GL_UNSIGNED_BYTE,
        (uchar*)(frame->data),
        osg::Image::AllocationMode::NO_DELETE, 1);

    // read the scene from the list of file specified commandline args.
    osg::ref_ptr<osg::Group> group = new osg::Group;
    osg::ref_ptr<osg::Group> group2 = new osg::Group;
    osg::ref_ptr<osg::Geode> hud = createHUD(backgroundImage, vcap.get(CV_CAP_PROP_FRAME_WIDTH), vcap.get(CV_CAP_PROP_FRAME_HEIGHT), cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2), f);
    osg::ref_ptr<osg::Camera> cam = new osg::Camera;

    std::cout << "initialisation des objets 3D..." << std::endl;

    osg::ref_ptr<osg::MatrixTransform> mat = new osg::MatrixTransform();
    osg::ref_ptr<osg::MatrixTransform> matGlasses = chargerLunettes();
    osg::ref_ptr<osg::MatrixTransform> matMoustache = chargerMoustache();
    osg::ref_ptr<osg::MatrixTransform> matBunny = chargerBunny();
    osg::ref_ptr<osg::MatrixTransform> matMasque = chargerMasque();
    osg::ref_ptr<osg::MatrixTransform> matCerveau = chargerCerveau();
    osg::ref_ptr<osg::MatrixTransform> matCerveau2 = chargerCerveau2();
    osg::ref_ptr<osg::MatrixTransform> matAxes = chargerAxes();

    std::cout << "done" << std::endl;

    // construction du viewer
    osgViewer::CompositeViewer compositeViewer;
    osgViewer::View* viewer = new osgViewer::View;
    osgViewer::View* viewer2 = new osgViewer::View;

    // Insertion des objets

    //mat->addChild(matMasque);
    //mat->addChild(matGlasses);
    //mat->addChild(matMoustache);
    //mat->addChild(matBunny);
    //mat->addChild(matCerveau);
    //mat->addChild(matCerveau2);
    mat->addChild(matAxes);

    double sx = 0.19;
    double sy = 0.19;
    double sz = 0.19;

    osg::Matrixd matrixS; // scale
    matrixS.set(
        sx,	0,	0,	0,
        0,	sy,	0,	0,
        0,	0,	sz,	0,
        0,	0,	0,	1);


    // Projection

    osg::Matrixd projectionMatrix;

    projectionMatrix.makeFrustum(
        -cameraMatrix.at<double>(0, 2),		vcap.get(CV_CAP_PROP_FRAME_WIDTH) - cameraMatrix.at<double>(0, 2),
        -cameraMatrix.at<double>(1, 2),		vcap.get(CV_CAP_PROP_FRAME_HEIGHT) - cameraMatrix.at<double>(1, 2),
        f,									g);

    double correcteurY = (f/2)/(cameraMatrix.at<double>(1, 2)-vcap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);
    //double correcteurX = (f/2)/(cameraMatrix.at<double>(2, 2)-vcap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);

    osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, g, 0.0f), normal(0.0f, 0.0f, 1.0f);

    cam->setProjectionMatrix(projectionMatrix);
    cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    cam->setViewMatrixAsLookAt(eye, target, normal);
    cam->setClearMask(GL_DEPTH_BUFFER_BIT);
    cam->setRenderOrder(osg::Camera::PRE_RENDER);

    cam->addChild(hud);
    group->addChild(mat);
    group->addChild(cam);

    // Paramètres de scène
    viewer->setSceneData(group.get());
    viewer->setUpViewOnSingleScreen();
    //viewer->setUpViewInWindow(0, 0, 1920, 1080);
    viewer->getCamera()->setProjectionMatrix(projectionMatrix);
    viewer->getCamera()->setViewMatrixAsLookAt(eye, target, normal);
    viewer->getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT);

    //////////////////////////////////////////////////
    ////////// truc de merde à retirer asap //////////
    //////////////////////////////////////////////////

    group2->addChild(hud);
    group2->addChild(mat);

    viewer2->setSceneData(group2.get());
    viewer2->setUpViewInWindow(1920 / 2, 0, 1920 / 2, 1080 / 2);
    viewer2->getCamera()->setProjectionMatrix(projectionMatrix);
    osg::Vec3d eye2(2 * f, -10 * f, -2 * f), target2(0.0f, f, 0.0f), normal2(0.0f, 0.0f, 1.0f);
    viewer2->getCamera()->setViewMatrixAsLookAt(eye2, target2, normal2);

    compositeViewer.addView(viewer2);
    compositeViewer.addView(viewer);

    compositeViewer.realize();

    /////// DEBUG /////////

    std::ofstream file;
    file.open ("../rsc/angles.txt");
    file << "   rotX   " << "   rotY   " << "   rotZ   " << "   tX   " << "   tY   " << "   tZ   " << std::endl;

    //////////////////////////////////////////////////
    ////////////	boucle principale   //////////////
    //////////////////////////////////////////////////

    do
    {
        detectionVisage = false;

        moyPointsVisage2D.clear();
        pointsVisage2D.clear();

        std::cout << "recherche de pattern" << std::endl
            << "nbr images sauvegardees : " << images.size() << std::endl;

        vcap >> *frame;

        detectionVisage = detecterVisage(frame, &chehra, &pointsVisage2D);

        if(detectionVisage)
        {
            images.push_back(pointsVisage2D);
            nbrLoopSinceLastDetection = 0;
            if(!objectAdded)
            {
                group->addChild(mat);
                objectAdded = true;
            }
        }
        else
            nbrLoopSinceLastDetection++;

        if((images.size() > NBRSAVEDIMAGES || nbrLoopSinceLastDetection > NBRSAVEDIMAGES) && !images.empty())
            images.erase(images.begin());

        if(images.empty() && objectAdded)
        {
            group->removeChild(mat);
            objectAdded = false;
        }

        else if(!images.empty())
        {
            for(int i = 0; i < NBRFACEPOINTSDETECTED; i++)
            {
                cv::Point2f coordonee(0.0f, 0.0f);
                for(int j = 0; j < images.size(); j++)
                {
                    coordonee.x += images[j][i].x;
                    coordonee.y += images[j][i].y;
                }
                coordonee.x /= images.size();
                coordonee.y /= images.size();

                moyPointsVisage2D.push_back(coordonee);
            }

            cv::solvePnP(pointsVisage3D, moyPointsVisage2D, cameraMatrix, distCoeffs, rvecs, tvecs);

            cv::Mat rotVec(3, 3, CV_64F);
            cv::Rodrigues(rvecs, rotVec);

            imagePoints = dessinerPoints(frame, pointsVisage3D, rotVec, tvecs, cameraMatrix, distCoeffs, true);

            // Translations d'origine
//            double t3 = -tvecs.at<double>(2, 0);
//            double t2 = tvecs.at<double>(1, 0) - t3 / correcteurY;
//            double t1 = tvecs.at<double>(0, 0);

            // Translations -> angles rad
            double t1 = tvecs.at<double>(0, 0);
            double t3 = tvecs.at<double>(2, 0);
            double t2 = -tvecs.at<double>(1, 0) - t3 / correcteurY;

            std::cout << "t1 = " << t1 << " ; t2 = " << t2-t3/correcteurY << " ; t3 = " << t3 << std::endl;

            // Rotations
            double r11 = rotVec.at<double>(0, 0);
            double r12 = rotVec.at<double>(0, 1);
            double r13 = rotVec.at<double>(0, 2);
            double r21 = rotVec.at<double>(1, 0);
            double r22 = rotVec.at<double>(1, 1);
            double r23 = rotVec.at<double>(1, 2);
            double r31 = rotVec.at<double>(2, 0);
            double r32 = rotVec.at<double>(2, 1);
            double r33 = rotVec.at<double>(2, 2);

            osg::Matrixd matrixR; // Matrice rotation (transposee de rotVec)
            matrixR.set(
                r11,	r21,	r31,	0,
                r12,	r22,	r32,	0,
                r13,	r23,	r33,	0,
                0,		0,		0,		1);

            // Valeurs d'angles en rad
            double rotX = atan2(r32, r33);
            double rotY = atan2(r21, r11);
            double rotZ = -atan2(-r31, sqrt((r32 * r32) + (r33 * r33)));

            file << (rotX*180)/PI << " ; " << (rotY*180)/PI << " ; "  << (rotZ*180)/PI << " ; "  << t1 << " ; "  << t2-t3/correcteurY << " ; "  << t3 << " ; "  << std::endl;
            std::cout << "rotX = " << (rotX*180)/PI << " ; rotY = " << (rotY*180)/PI << " ; rotZ = " << (rotZ*180)/PI << std::endl;

            osg::Matrixd matrixT; // Matrice translation
            matrixT.makeTranslate(t1, t2, t3);

            osg::Matrixd matrixR2;
            matrixR2.makeRotate(rotY, osg::Vec3d(0.0, 1.0, 0.0), -rotZ, osg::Vec3d(0.0, 0.0, 1.0), -rotX, osg::Vec3d(1.0, 0.0, 0.0));

            osg::Matrixd matrix90; // rotation de repere entre opencv et osg
            matrix90.makeRotate(osg::Quat(osg::DegreesToRadians(-90.0f), osg::Vec3d(1.0, 0.0, 0.0)));

            osg::Matrixd matrix180; // rotation de repere entre opencv et osg
            matrix180.makeRotate(osg::Quat(osg::DegreesToRadians(180.0f), osg::Vec3d(0.0, 0.0, 1.0)));

            mat->setMatrix(matrixS * matrixR2 * matrixT * matrix90 * matrix180);

        }

        backgroundImage->dirty();
        compositeViewer.frame();

    }while(!compositeViewer.done());

    file.close();
}
