/* OpenSceneGraph example, osghud.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/Camera>
#include <osg/RenderInfo>
#include <osg/TextureRectangle>
#include <osgDB/WriteFile>
#include <osgText/Text>
#include <osg/PositionAttitudeTransform>

#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <cmath>

#define COLCHESSBOARD   9
#define ROWCHESSBOARD   6


std::vector<cv::Point2f> dessinerPoints(cv::Mat* imCalibColor, const std::vector<cv::Point3f> & objectPoints, const cv::Mat & rotVec, const cv::Mat & tvecs, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs)
{   
    std::vector<cv::Point2f> imagePoints;
    //Projection
    cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

    // Dessin des points projetes
    for(int m = 0; m < objectPoints.size(); m++)
        cv::circle(*imCalibColor, cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

    return imagePoints;
}

bool detecterMire(cv::Mat* imCalibColor, std::vector<cv::Point2f> *pointsMire, cv::Mat *imCalibNext)
{
    bool patternFound = false;
    cv::Mat imNB;

    cv::cvtColor(*imCalibColor, imNB, CV_BGR2GRAY);
            
    patternFound = cv::findChessboardCorners(imNB, cv::Size(ROWCHESSBOARD, COLCHESSBOARD), *pointsMire, cv::CALIB_CB_FAST_CHECK);
    
    if(patternFound)
        *imCalibNext = imNB;
    
    return patternFound; // normalement, si echap est presse, patternFound = false
}

cv::Mat trackingMire(cv::Mat *imCalibColor, cv::Mat *imCalibNext, std::vector<std::vector<cv::Point2f>> *chessCornersInit, std::vector<cv::Point3f> *chessCorners3D, cv::Mat *cameraMatrix, cv::Mat *distCoeffs, cv::Mat *tvecs)
{
    cv::Mat imCalib, rvecs;
    cv::swap(imCalib, *imCalibNext); // copie de l'ancienne image pour le flot optique
    (*chessCornersInit)[0] = (*chessCornersInit)[1];
    (*chessCornersInit)[1].clear();

    cv::cornerSubPix(imCalib, (*chessCornersInit)[0], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    cv::cvtColor(*imCalibColor, *imCalibNext, CV_BGR2GRAY);

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(imCalib, *imCalibNext, (*chessCornersInit)[0], (*chessCornersInit)[1], status, err, cv::Size(31, 31), 3, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1), 0, 0.0001);

    cv::solvePnP(*chessCorners3D, (*chessCornersInit)[0], *cameraMatrix, *distCoeffs, rvecs, *tvecs);

    cv::Mat rotVec(3, 3, CV_64F);
    cv::Rodrigues(rvecs, rotVec);

    return rotVec;
}



osg::Camera* createHUD(osg::Image* bgImage)
{
    // create a camera to set up the projection and model view matrices, and the subgraph to draw in the HUD
    osg::Camera* camera = new osg::Camera;

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0, 1.0f, 1.0f, 0));
    // set the view matrix
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::NESTED_RENDER);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    camera->setAllowEventFocus(false);



    // add to this camera a subgraph to render
    {

        osg::Geode* geode = new osg::Geode();

        // turn lighting off for the text and disable depth test to ensure it's always ontop.
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

        {
            auto texturedQuad = osg::createTexturedQuadGeometry(
                osg::Vec3(0.f, 0.f, 0.f), 
                osg::Vec3(1.0f, 0.f, 0.f),
                osg::Vec3(0.f, 1.0f, 0.f),
                bgImage->s(), 
                bgImage->t());
            auto textureRect = new osg::TextureRectangle(bgImage);
            textureRect->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
            textureRect->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            textureRect->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            texturedQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, textureRect, osg::StateAttribute::ON);
            texturedQuad->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
            geode->addDrawable(texturedQuad);
        }
        camera->addChild(geode);
    }

    return camera;
}



void main()
{
    bool patternfound = false;
    bool reset = false;
    bool resetAuto = false;
    int nbImages = 0;
    double moyFinale = 0;

    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
    cv::Size winSize(31, 31);
    
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat imCalib;
    cv::Mat imCalibColor;
    cv::Mat imCalibNext;
    cv::Mat rvecs, tvecs;
    
    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point3f> cubeObjectPoints;
    std::vector<std::vector<cv::Point2f>> chessCornersInit(2);
    std::vector<cv::Point3f> chessCorners3D;
    std::vector<double> distances;
    double moyDistances;
    

    // Creation des points a projeter
    for(int x = 0; x < COLCHESSBOARD; x++)
        for(int y = 0; y < ROWCHESSBOARD; y++)
            objectPoints.push_back(cv::Point3f(x * 26.0f, y * 26.0f, 0.0f));

    // Creation des points a projeter
    cubeObjectPoints.push_back(cv::Point3f(52, 26, 0));
    cubeObjectPoints.push_back(cv::Point3f(156, 26, 0));
    cubeObjectPoints.push_back(cv::Point3f(156, 128, 0));
    cubeObjectPoints.push_back(cv::Point3f(52, 128, 0));
    cubeObjectPoints.push_back(cv::Point3f(52, 26, 104));
    cubeObjectPoints.push_back(cv::Point3f(156, 26, 104));
    cubeObjectPoints.push_back(cv::Point3f(156, 128, 104));
    cubeObjectPoints.push_back(cv::Point3f(52, 128, 104));

    // Creation des coins de la mire
    for(int x = 0; x < COLCHESSBOARD; x++)
        for(int y = 0; y < ROWCHESSBOARD; y++)
            chessCorners3D.push_back(cv::Point3f(x * 26.0f, y * 26.0f, 0.0f));  

    cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;

    fs.release();

    cv::VideoCapture vcap(0); 
    if(!vcap.isOpened()){
        std::cout << "FAIL!" << std::endl;
        return;
    }

    cv::Mat *frame = new cv::Mat(cv::Mat::zeros(vcap.get(CV_CAP_PROP_FRAME_HEIGHT), vcap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3));
    
    do
    {
        vcap >> *frame;
    }while(frame->empty());

    osg::ref_ptr<osg::Image> backgroundImage = new osg::Image;
    backgroundImage->setImage(frame->cols, frame->rows, 3,
        GL_RGB, GL_BGR, GL_UNSIGNED_BYTE,
        (uchar*)(frame->data),
        osg::Image::AllocationMode::NO_DELETE, 1);

    // read the scene from the list of file specified commandline args.
    osg::ref_ptr<osg::Group> group = new osg::Group;
    osg::ref_ptr<osg::Node> objet3D = osgDB::readNodeFile("dumptruck.osgt");
    osg::ref_ptr<osg::Camera> cam = createHUD(backgroundImage);
    osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
    osg::ref_ptr<osg::PositionAttitudeTransform> pat2 = new osg::PositionAttitudeTransform;

    // construct the viewer.
    osgViewer::Viewer viewer;

    // add the HUD subgraph.
    group->addChild(cam);
    pat->addChild(pat2);
    pat2->addChild(objet3D);
    group->addChild(pat);

    // set the scene to render
    viewer.setSceneData(group.get());
    viewer.realize();  // set up windows and associated threads.

    char key = 0;
    bool detectionMire = false;

    cv::Mat Rc, C = cv::Mat(3, 1, CV_64F), rotVecInv;
    pat->setScale(osg::Vec3d(0.08, 0.08, 0.08));
    pat->setAttitude(osg::Quat(osg::DegreesToRadians(180.0), osg::Vec3d(1.0, 0.0, 0.0)));
    pat2->setPosition(osg::Vec3d(15.0, 4.0, 5.0));

/*  
    // projection
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(1, 2);
    double cy = cameraMatrix.at<double>(1, 0);
    double W  = (double)frame->cols;
    double H  = (double)frame->rows;
    double near = .1;
    double far = 100.0;

    osg::Matrixd projectionMatrix;
    projectionMatrix.set(
        2 * fx / W, 0, 0, 0, 
        0, 2 * fy / H, 0, 0, 
        2 * (cx / W) - 1, 2 * (cy - H) - 1, (far + near) / (far - near), 1,
        0, 0, 2 * far * near / (near - far), 0);
        
    projectionMatrix.set(
        2 * fx / W, 0, 0, 0, 
        0, 2 * fy / H, 0, 0,
        2 * (cx / W) - 1, 2 * (cy / H) - 1, (far + near) / (far - near), 1,
        0, 0, 2 * far * near / (near - far), 0);

    viewer.getCamera()->setProjectionMatrix(projectionMatrix);*/


    do
    {       
        patternfound = false;
        resetAuto = false;
        detectionMire = false;
            
        imagePoints.clear();
        chessCornersInit[0].clear();
        chessCornersInit[1].clear();
        moyDistances = 0;
        distances.clear();
        imCalibNext.release();
        
        group->removeChild(pat);
        std::cout << "recherche de mire" << std::endl;

        do
        {
            vcap >> *frame;
            backgroundImage->dirty();
            detectionMire = detecterMire(frame, &chessCornersInit[1], &imCalibNext);
            viewer.frame();
        }while(!detectionMire && !viewer.done());

        if(viewer.done())
            break;

        std::cout << "mire detectee" << std::endl << std::endl;
        group->addChild(pat);

        do
        {           
            vcap >> *frame;
            
            cv::Mat rotVec = trackingMire(frame, &imCalibNext, &chessCornersInit, &chessCorners3D, &cameraMatrix, &distCoeffs, &tvecs);

            imagePoints = dessinerPoints(frame, objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs);
            
            cv::transpose(rotVec, Rc);
            cv::invert(rotVec, rotVecInv);

            for(int i = 0; i < 3; i++)
                C.at<double>(i, 0) = -1 * (
                rotVecInv.at<double>(i, 0) * tvecs.at<double>(0, 0) +
                rotVecInv.at<double>(i, 1) * tvecs.at<double>(1, 0) +
                rotVecInv.at<double>(i, 2) * tvecs.at<double>(2, 0));
            
            osg::Matrixd viewMatrixR, viewMatrixT, viewMatrix90;

            viewMatrixT.makeTranslate(
                -C.at<double>(0, 0) / 100,
                C.at<double>(1, 0) / 100,
                C.at<double>(2, 0) / 100);
            
            double r11 = rotVec.at<double>(0, 0);
            double r21 = rotVec.at<double>(1, 0);
            double r31 = rotVec.at<double>(2, 0);
            double r32 = rotVec.at<double>(2, 1);
            double r33 = rotVec.at<double>(2, 2);

            viewMatrixR.makeRotate(
                atan2(r32, r33), osg::Vec3d(1.0, 0.0, 0.0),
                -atan2(-r31, sqrt((r32 * r32) + (r33 * r33))), osg::Vec3d(0.0, 1.0, 0.0),
                -atan2(r21, r11), osg::Vec3d(0.0, 0.0, 1.0));
            

            viewMatrix90.makeRotate(osg::DegreesToRadians(-90.0), osg::Vec3d(1.0, 0.0, 0.0));
            
            viewer.getCamera()->setViewMatrix(viewMatrixT * viewMatrixR);
            
            // Calcul d'erreur de reprojection
            double moy = 0;
            for(int j = 0; j < COLCHESSBOARD * ROWCHESSBOARD; j++)
            {
                double d = sqrt(pow(chessCornersInit[0][j].y - imagePoints[j].y, 2) + pow(chessCornersInit[0][j].x - imagePoints[j].x, 2));
                distances.push_back(d);
                moy += d;
            }

            moyDistances = moy / (COLCHESSBOARD * ROWCHESSBOARD);

            if(moyDistances > 2) // si l'ecart de reproj est trop grand, reset
                resetAuto = true;

            key = cv::waitKey(33);

            backgroundImage->dirty();
            viewer.frame();
        }while(!viewer.done() && !resetAuto && key != 32);

    }while(!viewer.done());
}
