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

bool detecterVisage(cv::Mat* imCalibColor, Chehra *chehra, std::vector<cv::Point2f> *pointsVisage, std::vector<cv::Point3f> *visage)
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
				(*visage).push_back(cv::Point3f(points.at<float>(i,0),points.at<float>(i+49,0),0));	
		
		(*pointsVisage).push_back(cv::Point2f((*visage)[14].x,(*visage)[14].y));
		(*pointsVisage).push_back(cv::Point2f((*visage)[18].x,(*visage)[18].y));
		(*pointsVisage).push_back(cv::Point2f((*visage)[13].x,(*visage)[13].y));	
		(*pointsVisage).push_back(cv::Point2f((*visage)[19].x,(*visage)[19].y));
		(*pointsVisage).push_back(cv::Point2f((*visage)[10].x,(*visage)[10].y));
		(*pointsVisage).push_back(cv::Point2f((*visage)[28].x,(*visage)[28].y));
	}

	return visageFound; // normalement, si echap est presse, patternFound = false
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
	bool resetAuto = false;
	int nbImages = 0;
	double moyFinale = 0;
	bool detectionVisage = false;

	int nbrLoopSinceLastDetection = 0;
	int criticalValueOfLoopWithoutDetection = 15;

	std::cout << "initialisation de Chehra..." << std::endl;
	Chehra chehra;
	std::cout << "done" << std::endl;

	cv::Mat cameraMatrix, distCoeffs;
	cv::Mat rvecs, tvecs;

	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point2f> pointsVisage2D;
	std::vector<cv::Point2f> moyPointsVisage2D;
	std::vector<cv::Point3f> pointsVisage3D;
	std::vector<cv::Point3f> visage;
	std::vector<double> distances;
	double moyDistances;

	std::vector<std::vector<cv::Point2f>> images;
	std::vector<cv::Mat> frames;

	double s = 10.0f;

	osg::Matrixd matrixS; // scale
	matrixS.set(
		s,	0,	0,	0,
		0,	s,	0,	0,
		0,	0,	s,	0,
		0,	0,	0,	1);
	
	pointsVisage3D.push_back(cv::Point3f(90,0,-80));
	pointsVisage3D.push_back(cv::Point3f(-90,0,-80));
	pointsVisage3D.push_back(cv::Point3f(0,0,0));
	pointsVisage3D.push_back(cv::Point3f(600,0,600));
	pointsVisage3D.push_back(cv::Point3f(0,0,600));
	pointsVisage3D.push_back(cv::Point3f(-600,0,600));
	/*
	pointsVisage3D.push_back(cv::Point3f(13.1, -98.1,108.3)); // exterieur narine gauche
	pointsVisage3D.push_back(cv::Point3f(-13.1, -98.1,108.3)); // exterieur narine droite
	pointsVisage3D.push_back(cv::Point3f(0, -87.2, 124.2)); // bout du nez
	pointsVisage3D.push_back(cv::Point3f(44.4, -57.9, 83.7)); // exterieur oeil gauche
	pointsVisage3D.push_back(cv::Point3f(0, 55.4, 101.4)); // haut du nez, centre des yeux
	pointsVisage3D.push_back(cv::Point3f(-44.4, -57.9, 83.7)); // exterieur oeil droit
	*/
	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	double f = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carrés, fx = fy -> np 
	//mais est généralement différent de fy donc on prend (pour l'instant) par défaut la valeur médiane
	double g = 2000 * f; // je sais pas pourquoi. au pif.

	fs.release();

	cv::VideoCapture vcap(0); 
	if(!vcap.isOpened())
	{
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
	osg::ref_ptr<osg::Node> objet3D = osgDB::readNodeFile("../rsc/objets3D/brain.obj");
	//osg::ref_ptr<osg::Node> objet3D = osgDB::readNodeFile("../rsc/objets3D/dumptruck.osgt");

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
	/*
	osg::Sphere* unitSphere = new osg::Sphere(osg::Vec3(0, -1000, 1000), 100.0);
	osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(unitSphere);

	osg::Geode* objet3D = new osg::Geode();

	objet3D->addDrawable(unitSphereDrawable);
	*/
	//osg::StateSet* sphereStateset = unitSphereDrawable->getOrCreateStateSet();
	//sphereStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	//sphereStateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::cout << "done" << std::endl;

	osg::StateSet* obectStateset = objet3D->getOrCreateStateSet();
	obectStateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	obectStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	osg::ref_ptr<osg::MatrixTransform> mat = new osg::MatrixTransform();

	// construct the viewer.
	osgViewer::CompositeViewer compositeViewer;
	osgViewer::View* viewer = new osgViewer::View;
	osgViewer::View* viewer2 = new osgViewer::View;

	// add the HUD subgraph.
	group->addChild(cam);

	mat->addChild(objet3D);
	group->addChild(mat);

	osg::Matrixd projectionMatrix;

	projectionMatrix.makeFrustum(
		-cameraMatrix.at<double>(0, 2),		vcap.get(CV_CAP_PROP_FRAME_WIDTH) - cameraMatrix.at<double>(0, 2),
		-cameraMatrix.at<double>(1, 2),		vcap.get(CV_CAP_PROP_FRAME_HEIGHT) - cameraMatrix.at<double>(1, 2),
		f,									g);

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

	compositeViewer.addView(viewer2);
	compositeViewer.addView(viewer);

	compositeViewer.realize();  // set up windows and associated threads.

	do
	{       
		patternfound = false;
		resetAuto = false;
		detectionVisage = false;

		moyPointsVisage2D.clear();
		pointsVisage2D.clear();
		visage.clear();
		moyDistances = 0;
		distances.clear();

		std::cout << "recherche de pattern" << std::endl
			<< "nbr images sauvegardees : " << images.size() << std::endl;

		vcap >> *frame;
		frames.push_back(*frame);

		detectionVisage = detecterVisage(frame, &chehra, &pointsVisage2D, &visage);

		if(detectionVisage)
		{
			images.push_back(pointsVisage2D);
			nbrLoopSinceLastDetection = 0;
			group->addChild(mat);
		}
		else
			nbrLoopSinceLastDetection++;

		if((images.size() > NBRSAVEDIMAGES || nbrLoopSinceLastDetection > criticalValueOfLoopWithoutDetection) && !images.empty())
			images.erase(images.begin());

		if(images.empty())
			group->removeChild(mat);

		else
		{
			//cv::cornerSubPix(*frame, pointsVisage2D, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

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
			
			imagePoints = dessinerPoints(frame, pointsVisage3D, rotVec, tvecs, cameraMatrix, distCoeffs);

			double t3 = tvecs.at<double>(2, 0);
			double t1 = tvecs.at<double>(0, 0);
			double t2 = tvecs.at<double>(1, 0) + t3 / 27.5; // and now, magic !

			double r11 = rotVec.at<double>(0, 0);
			double r12 = rotVec.at<double>(0, 1);
			double r13 = rotVec.at<double>(0, 2);
			double r21 = rotVec.at<double>(1, 0);
			double r22 = rotVec.at<double>(1, 1);
			double r23 = rotVec.at<double>(1, 2);
			double r31 = rotVec.at<double>(2, 0);
			double r32 = rotVec.at<double>(2, 1);
			double r33 = rotVec.at<double>(2, 2);


			osg::Matrixd matrixR; // rotation (transposee de rotVec)
			matrixR.set(
				r11,	r21,	r31,	0,
				r12,	r22,	r32,	0,
				r13,	r23,	r33,	0,
				0,		0,		0,		1);

			osg::Matrixd matrixT; // translation
			matrixT.makeTranslate(t1, t2, t3);

			osg::Matrixd matrix90; // rotation de repere entre opencv et osg
			matrix90.makeRotate(osg::Quat(osg::DegreesToRadians(-90.0f), osg::Vec3d(1.0, 0.0, 0.0)));

			mat->setMatrix(matrixS * matrixR * matrixT * matrix90);

			// Calcul d'erreur de reprojection
			double moy = 0;
			for(int i = 0; i < pointsVisage2D.size(); i++)
			{
				double d = sqrt(pow(pointsVisage2D[i].y - imagePoints[i].y, 2) + pow(pointsVisage2D[i].x - imagePoints[i].x, 2));
				distances.push_back(d);
				moy += d;
			}

			moyDistances = moy / pointsVisage2D.size();

			if(moyDistances > 2) // si l'ecart de reproj est trop grand, reset
				resetAuto = true;
		}
		
		backgroundImage->dirty();
		compositeViewer.frame();

	}while(!compositeViewer.done());
}
