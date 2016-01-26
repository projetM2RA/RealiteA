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

#include <opencv2/opencv.hpp>
#include <C:/Libs/Chehra/Chehra.h>

#include <stdio.h>
#include <time.h>

#define COLCHESSBOARD   9
#define ROWCHESSBOARD   6
#define SIZEMIRE		26


std::vector<cv::Point2f> dessinerPoints(cv::Mat* imCalibColor, const std::vector<cv::Point3f> & objectPoints, const cv::Mat & rotVec, const cv::Mat & tvecs, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs)
{   
	std::vector<cv::Point2f> imagePoints;
	//Projection
	cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

	// Dessin des points projetes
	for(int m = 0; m < objectPoints.size(); m++)
		cv::circle(*imCalibColor, cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(255, 0, 0), 1, 8, 0);

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

bool detecterVisage(cv::Mat* imCalibColor, Chehra *chehra, std::vector<cv::Point2f> *pointsVisage, std::vector<cv::Point3f> *visage, std::vector<cv::Point3f> *pointsVisage3D, cv::Mat *imCalibNext)
{
	bool visageFound = false;
	cv::Mat imNB;
	cv::Mat points;

	cv::cvtColor(*imCalibColor, imNB, CV_BGR2GRAY);

	visageFound = (*chehra).track(imNB);	

	if(visageFound){
		*imCalibNext = imNB;
		points = (*chehra).getTrackedPoints();
		if (points.rows == 98) {
			for(int i = 0; i < 49; i++){
				(*visage).push_back(cv::Point3f(points.at<float>(i,0),points.at<float>(i+49,0),0));	
				//(*pointsVisage3D).push_back(cv::Point3f(points.at<float>(i,0),points.at<float>(i+49,0),0));	
				//(*pointsVisage).push_back(cv::Point2f(points.at<float>(i,0),points.at<float>(i+49,0)));	
			}
		}

		(*pointsVisage).push_back(cv::Point2f((*visage)[18].x,(*visage)[18].y));
		(*pointsVisage).push_back(cv::Point2f((*visage)[13].x,(*visage)[13].y));	
		(*pointsVisage).push_back(cv::Point2f((*visage)[19].x,(*visage)[19].y));
		(*pointsVisage).push_back(cv::Point2f((*visage)[10].x,(*visage)[10].y));
		(*pointsVisage).push_back(cv::Point2f((*visage)[28].x,(*visage)[28].y));
		
		(*pointsVisage3D).push_back(cv::Point3f(0,0,0));
		(*pointsVisage3D).push_back(cv::Point3f(-8,8,0));
		(*pointsVisage3D).push_back(cv::Point3f(-60,60,0));
		(*pointsVisage3D).push_back(cv::Point3f(0,60,0));
		(*pointsVisage3D).push_back(cv::Point3f(60,60,0));	

	}

	return visageFound; // normalement, si echap est presse, patternFound = false
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

	return noeudGeo;
}



void main()
{
    bool patternfound = false;
    bool reset = false;
    bool resetAuto = false;
    int nbImages = 0;
    double moyFinale = 0;
    char key = 0;
    bool detectionMire = false;
	bool detectionVisage = false;
	int cpt = 0, moyCpt = 0, i = 0;

	std::cout << "initialisation de Chehra..." << std::endl;
	Chehra chehra;
	std::cout << "done" << std::endl;

    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
    cv::Size winSize(31, 31);
    
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat imCalib;
    cv::Mat imCalibColor;
    cv::Mat imCalibNext;
    cv::Mat rvecs, tvecs;
    cv::Mat Rc, C = cv::Mat(3, 1, CV_64F), rotVecInv;
    
    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point3f> cubeObjectPoints;
	std::vector<cv::Point3f> dessinPointsVisage;
    std::vector<std::vector<cv::Point2f>> chessCornersInit(2);
	std::vector<std::vector<cv::Point2f>> pointsVisageInit(2);
    std::vector<cv::Point3f> chessCorners3D;
	std::vector<cv::Point3f> pointsVisage3D;
	std::vector<cv::Point3f> visage;
    std::vector<double> distances;
    double moyDistances;

    // Creation des coins de la mire
    for(int x = 0; x < COLCHESSBOARD; x++)
        for(int y = 0; y < ROWCHESSBOARD; y++)
            chessCorners3D.push_back(cv::Point3f(x * SIZEMIRE, y * SIZEMIRE, 0.0f));  

    // Creation des points a projeter
    for(int x = 0; x < COLCHESSBOARD; x++)
        for(int y = 0; y < ROWCHESSBOARD; y++)
            objectPoints.push_back(cv::Point3f(x * SIZEMIRE, y * SIZEMIRE, 0.0f));
	
	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	double f = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carrés, fx = fy -> np 
	//mais est généralement différent de fy donc on prend (pour l'instant) par défaut la valeur médiane
	double g = 2000 * f; // je sais pas pourquoi. au pif.

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
	osg::ref_ptr<osg::Geode> cam = createHUD(backgroundImage, vcap.get(CV_CAP_PROP_FRAME_WIDTH), vcap.get(CV_CAP_PROP_FRAME_HEIGHT), cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2), f);

	std::cout << "initialisation de l'objet 3D..." << std::endl;
	osg::ref_ptr<osg::Node> objet3D = osgDB::readNodeFile("../rsc/objets3D/Creature.obj");
	std::cout << "done" << std::endl;
   
	osg::StateSet* obectStateset = objet3D->getOrCreateStateSet();
       obectStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	osg::ref_ptr<osg::MatrixTransform> mat = new osg::MatrixTransform();
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform();

	// construct the viewer.
	osgViewer::CompositeViewer compositeViewer;
	osgViewer::View* viewer = new osgViewer::View;
	osgViewer::View* viewer2 = new osgViewer::View;

	// add the HUD subgraph.
	group->addChild(cam);

	mat->addChild(objet3D);
	pat->addChild(mat);
	group->addChild(pat);

    pat->setScale(osg::Vec3d(3, 3, 3));

	osg::Matrixd projectionMatrix;

	projectionMatrix.makeFrustum(
		-cameraMatrix.at<double>(0, 2),		vcap.get(CV_CAP_PROP_FRAME_WIDTH) - cameraMatrix.at<double>(0, 2),
		-cameraMatrix.at<double>(1, 2),		vcap.get(CV_CAP_PROP_FRAME_HEIGHT) - cameraMatrix.at<double>(1, 2),
		f,								g);

	osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, g, 0.0f), normal(0.0f, 0.0f, 1.0f);

	// set the scene to render
	viewer->setSceneData(group.get());
	viewer->setUpViewInWindow(0, 0, 1920 / 2, 1080 / 2); 
	viewer->getCamera()->setProjectionMatrix(projectionMatrix);
	viewer->getCamera()->setViewMatrixAsLookAt(eye, target, normal);

	viewer2->setSceneData(group.get());
	viewer2->setUpViewInWindow(1920 / 2, 0, 1920 / 2, 1080 / 2); 
	viewer2->getCamera()->setProjectionMatrix(projectionMatrix);
	osg::Vec3d eye2(4 * f, 3 * f / 2, 0.0f), target2(0.0f, f, 0.0f), normal2(0.0f, 0.0f, 1.0f);
	viewer2->getCamera()->setViewMatrixAsLookAt(eye2, target2, normal2);

	compositeViewer.addView(viewer);
	compositeViewer.addView(viewer2);

	compositeViewer.realize();  // set up windows and associated threads.



    do
    {       
		group->removeChild(pat);
        patternfound = false;
        resetAuto = false;
        detectionMire = false;
		detectionVisage = false;
            
        imagePoints.clear();
        chessCornersInit[0].clear();
        chessCornersInit[1].clear();
		pointsVisageInit[0].clear();
		pointsVisageInit[1].clear();
		pointsVisage3D.clear();
		dessinPointsVisage.clear();
		visage.clear();
        moyDistances = 0;
        distances.clear();
        imCalibNext.release();
        
        std::cout << "recherche de pattern" << std::endl;

		time_t start = clock();
		double timer = 0;
		
        do
        {
			start = clock();

            vcap >> *frame;

			backgroundImage->dirty();
            //detectionMire = detecterMire(frame, &chessCornersInit[1], &imCalibNext);
			detectionVisage = detecterVisage(frame, &chehra, &pointsVisageInit[1], &visage, &pointsVisage3D, &imCalibNext);

			cpt++;
			double duree = (clock() - start)/(double) CLOCKS_PER_SEC;
			timer += duree;

			if(timer >= 1){
				std::cout << cpt << " fps" << std::endl;
				moyCpt += cpt;
				timer = 0;
				duree = 0;
				i++;
				cpt = 0;
				start = clock();
			}

            compositeViewer.frame();
        }while(!detectionMire && !detectionVisage && !compositeViewer.done());

        if(compositeViewer.done())
            break;

        std::cout << "pattern detectee" << std::endl << std::endl;

		group->addChild(pat);
		
        do
        {           
			start = clock();

            vcap >> *frame;
            
			cv::Mat rotVec = trackingMire(frame, &imCalibNext, &pointsVisageInit, &pointsVisage3D, &cameraMatrix, &distCoeffs, &tvecs);
            //cv::Mat rotVec = trackingMire(frame, &imCalibNext, &chessCornersInit, &chessCorners3D, &cameraMatrix, &distCoeffs, &tvecs);

            //imagePoints = dessinerPoints(frame, objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs);
			imagePoints = dessinerPoints(frame, pointsVisage3D, rotVec, tvecs, cameraMatrix, distCoeffs);
            
            double r11 = rotVec.at<double>(0, 0);
            double r21 = rotVec.at<double>(1, 0);
            double r31 = rotVec.at<double>(2, 0);
            double r32 = rotVec.at<double>(2, 1);
            double r33 = rotVec.at<double>(2, 2);

			osg::Matrixd matrixR;
            matrixR.makeRotate(
                atan2(r32, r33), osg::Vec3d(1.0, 0.0, 0.0),
                -atan2(-r31, sqrt((r32 * r32) + (r33 * r33))), osg::Vec3d(0.0, 0.0, 1.0),
                atan2(r21, r11), osg::Vec3d(0.0, 1.0, 0.0));
            
            mat->setMatrix(matrixR);
			pat->setPosition(osg::Vec3d(tvecs.at<double>(0, 0), tvecs.at<double>(2, 0), -tvecs.at<double>(1, 0)));

			//std::cout << "x = " << tvecs.at<double>(0, 0) << " - y = " << tvecs.at<double>(1, 0) << " - z = " << tvecs.at<double>(2, 0) << std::endl;

            // Calcul d'erreur de reprojection
            double moy = 0;
            for(int j = 0; j < pointsVisageInit[1].size() ; j++)
			{
				double d = sqrt(pow(pointsVisageInit[0][j].y - imagePoints[j].y, 2) + pow(pointsVisageInit[0][j].x - imagePoints[j].x, 2));
				distances.push_back(d);
				moy += d;
			}

            moyDistances = moy / pointsVisageInit[1].size();

            if(moyDistances > 1) // si l'ecart de reproj est trop grand, reset
                resetAuto = true;

			double duree = (clock() - start)/(double) CLOCKS_PER_SEC;


				std::cout << (int)(1/duree) << " fps" << std::endl;
				moyCpt += (int)(1/duree);
				duree = 0;
				i++;
			
            backgroundImage->dirty();
            compositeViewer.frame();
        }while(!compositeViewer.done() && !resetAuto);
		
    }while(!compositeViewer.done());

	std::cout << std::endl << "Moyenne des fps : " << moyCpt/i << std::endl;

	std::system("PAUSE");
}
