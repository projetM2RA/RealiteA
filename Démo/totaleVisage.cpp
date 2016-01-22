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

#define NBRSAVEDIMAGES			3
#define NBRFACEPOINTSDETECTED	6


std::vector<cv::Point2f> dessinerPoints(cv::Mat* imCalibColor, const std::vector<cv::Point3f> & objectPoints, const cv::Mat & rotVec, const cv::Mat & tvecs, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs, bool dessin)
{   
	std::vector<cv::Point2f> imagePoints;
	//Projection
	cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

	// Dessin des points projetes
	if(dessin)
		for(int m = 0; m < objectPoints.size(); m++)
			cv::circle(*imCalibColor, cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(255, 0, 0), 1, 8, 0);

	return imagePoints;
}

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

	std::vector<std::vector<cv::Point2f>> images;
	std::vector<cv::Mat> frames;


	
	//pointsVisage3D.push_back(cv::Point3f(0,0,0)); // centre yeux
	//pointsVisage3D.push_back(cv::Point3f(-60,0,0)); // oeil gauche
	//pointsVisage3D.push_back(cv::Point3f(60,0,0)); // oeil droit
	//pointsVisage3D.push_back(cv::Point3f(0,0,-40)); // nez 1
	//pointsVisage3D.push_back(cv::Point3f(0,0,-60)); // nez 2

	pointsVisage3D.push_back(cv::Point3f(90,0,-680));
	pointsVisage3D.push_back(cv::Point3f(-90,0,-680));	
	pointsVisage3D.push_back(cv::Point3f(0,0,-600));
	pointsVisage3D.push_back(cv::Point3f(600,0,0));
	pointsVisage3D.push_back(cv::Point3f(0,0,0));
	pointsVisage3D.push_back(cv::Point3f(-600,0,0));
	
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

	std::cout << "initialisation des objets 3D..." << std::endl;
	osg::ref_ptr<osg::Node> objet3D = osgDB::readNodeFile("../rsc/objets3D/Glasses.obj");
	osg::ref_ptr<osg::Node> objet3D2 = osgDB::readNodeFile("../rsc/objets3D/Mustache.obj");
	osg::ref_ptr<osg::Node> objet3D3 = osgDB::readNodeFile("../rsc/objets3D/bunny/ItmUsagiHat.obj");
	osg::ref_ptr<osg::Node> masque = osgDB::readNodeFile("../rsc/objets3D/head.obj");

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

	// Paramètres Objets
	/////////////////////////

	// Masque
	osg::StateSet* masqueStateset = masque->getOrCreateStateSet();
	//obectStateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	masqueStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
    osg::ref_ptr<osg::Material> material = new osg::Material;

    material->setAlpha(osg::Material::FRONT_AND_BACK, 0.1); //Making alpha channel
    masqueStateset->setAttributeAndModes( material.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

	masque->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON ); 
	//masque->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 

	// Objet 1
	osg::StateSet* obectStateset = objet3D->getOrCreateStateSet();
	//obectStateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	obectStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

	// Objet 2
	osg::StateSet* obectStateset2 = objet3D2->getOrCreateStateSet();
	obectStateset2->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	obectStateset2->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

	osg::Texture2D* textureMoustache = new osg::Texture2D;
	textureMoustache->setDataVariance(osg::Object::DYNAMIC); 
	osg::Image* texture2ON = osgDB::readImageFile("../rsc/objets3D/MustacheUV-textureMap.bmp");
	if (!texture2ON)
	{
		std::cout << " couldn't find texture, quiting." << std::endl;
		// SORTIE
	}
	textureMoustache->setImage(texture2ON);

	obectStateset2->setTextureAttributeAndModes(0,textureMoustache,osg::StateAttribute::ON);
	objet3D2->setStateSet(obectStateset2);

		// Objet 3
	osg::StateSet* obectStateset3 = objet3D3->getOrCreateStateSet();
	obectStateset3->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	obectStateset3->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

	osg::Texture2D* textureLapin = new osg::Texture2D;
	textureLapin->setDataVariance(osg::Object::DYNAMIC); 
	osg::Image* texture3ON = osgDB::readImageFile("../rsc/objets3D/bunny/earfix.bmp");
	if (!texture3ON)
	{
		std::cout << " couldn't find texture, quiting." << std::endl;
		// SORTIE
	}
	textureLapin->setImage(texture3ON);

	obectStateset3->setTextureAttributeAndModes(0,textureLapin,osg::StateAttribute::ON);
	objet3D3->setStateSet(obectStateset3);

	osg::ref_ptr<osg::MatrixTransform> mat = new osg::MatrixTransform();
	osg::ref_ptr<osg::MatrixTransform> mat1 = new osg::MatrixTransform();
	osg::ref_ptr<osg::MatrixTransform> mat2 = new osg::MatrixTransform();
	osg::ref_ptr<osg::MatrixTransform> mat3 = new osg::MatrixTransform();
	
	// construct the viewer.
	osgViewer::CompositeViewer compositeViewer;
	osgViewer::View* viewer = new osgViewer::View;
	osgViewer::View* viewer2 = new osgViewer::View;

	// add the HUD subgraph.
	group->addChild(cam);

	//mat->addChild(masque);
	mat1->addChild(objet3D);
	mat2->addChild(objet3D2);
	mat3->addChild(objet3D3);
	//group->addChild(mat);
	group->addChild(mat1);
	group->addChild(mat2);
	group->addChild(mat3);

	// Projection

	osg::Matrixd projectionMatrix;

	projectionMatrix.makeFrustum(
		-cameraMatrix.at<double>(0, 2),		vcap.get(CV_CAP_PROP_FRAME_WIDTH) - cameraMatrix.at<double>(0, 2),
		-cameraMatrix.at<double>(1, 2),		vcap.get(CV_CAP_PROP_FRAME_HEIGHT) - cameraMatrix.at<double>(1, 2),
		f,									g);

	double correcteur = (f/2)/(cameraMatrix.at<double>(1, 2)-vcap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);

	osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, g, 0.0f), normal(0.0f, 0.0f, 1.0f);

	// Paramètres de scène
	viewer->setSceneData(group.get());
	viewer->setUpViewInWindow(0, 0, 1920, 1080); 
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

	// Placement masque

	double s = 220;

	osg::Matrixd matrixS; // scale
	matrixS.set(
		s,	0,	0,	0,
		0,	s,	0,	0,
		0,	0,	s,	0,
		0,	0,	0,	1);

	osg::Matrixd matrixRot;
	matrixRot.makeRotate(osg::Quat(osg::DegreesToRadians(180.0f), osg::Vec3d(0.0, 0.0, 1.0)));

	osg::Matrixd matrixRotBis;
	matrixRotBis.makeRotate(osg::Quat(osg::DegreesToRadians(-30.0f), osg::Vec3d(1.0, 0.0, 0.0)));
			
	osg::Matrixd matrixTrans;
	matrixTrans.makeTranslate(3200,0,-300);

	// Placement Objet 1

	double s1 = 350;

	osg::Matrixd matrixS1; // scale
	matrixS1.set(
		s1,	0,	0,	0,
		0,	s1,	0,	0,
		0,	0,	s1,	0,
		0,	0,	0,	1);

	osg::Matrixd matrixRot1;
	matrixRot1.makeRotate(osg::Quat(osg::DegreesToRadians(90.0f), osg::Vec3d(0.0, 0.0, 1.0)));

	osg::Matrixd matrixRot12;
	matrixRot12.makeRotate(osg::Quat(osg::DegreesToRadians(-35.0f), osg::Vec3d(1.0, 0.0, 0.0)));
		
	osg::Matrixd matrixTrans1;
	matrixTrans1.makeTranslate(0,0,-343);

	// Placement Objet 2

	double s2 = 380;

	osg::Matrixd matrixS2; // scale
	matrixS2.set(
		s2,	0,	0,	0,
		0,	s2,	0,	0,
		0,	0,	s2,	0,
		0,	0,	0,	1);

	osg::Matrixd matrixRot2;
	matrixRot2.makeRotate(osg::Quat(osg::DegreesToRadians(90.0f), osg::Vec3d(0.0, 0.0, 1.0)));

	osg::Matrixd matrixRot22;
	matrixRot22.makeRotate(osg::Quat(osg::DegreesToRadians(-35.0f), osg::Vec3d(1.0, 0.0, 0.0)));
		
	osg::Matrixd matrixTrans2;
	matrixTrans2.makeTranslate(-35,0,-860);

	// Placement Objet 3

	double s3 = 500;

	osg::Matrixd matrixS3; // scale
	matrixS3.set(
		s3,	0,	0,	0,
		0,	s3,	0,	0,
		0,	0,	s3,	0,
		0,	0,	0,	1);

	osg::Matrixd matrixRot3;
	matrixRot3.makeRotate(osg::Quat(osg::DegreesToRadians(-55.0f), osg::Vec3d(1.0, 0.0, 0.0)));
			
	osg::Matrixd matrixTrans3;
	matrixTrans3.makeTranslate(0,0,600);

	bool objectAdded = false;

	do
	{       
		detectionVisage = false;

		moyPointsVisage2D.clear();
		pointsVisage2D.clear();

		std::cout << "recherche de pattern" << std::endl
			<< "nbr images sauvegardees : " << images.size() << std::endl;

		vcap >> *frame;
		//frames.push_back(*frame);


		detectionVisage = detecterVisage(frame, &chehra, &pointsVisage2D);

		if(detectionVisage)
		{
			images.push_back(pointsVisage2D);
			nbrLoopSinceLastDetection = 0;
			if(!objectAdded)
			{
				group->addChild(mat);
				group->addChild(mat1);
				group->addChild(mat2);
				group->addChild(mat3);
				objectAdded = true;
			}
		}
		else
			nbrLoopSinceLastDetection++;

		if((images.size() > NBRSAVEDIMAGES || nbrLoopSinceLastDetection > criticalValueOfLoopWithoutDetection) && !images.empty())
			images.erase(images.begin());

		if(images.empty() && objectAdded)
		{
			group->removeChild(mat);
			group->removeChild(mat1);
			group->removeChild(mat2);
			group->removeChild(mat3);
			objectAdded = false;
		}

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

			double t3 = tvecs.at<double>(2, 0);
			double t1 = tvecs.at<double>(0, 0);
			double t2 = tvecs.at<double>(1, 0) + t3 / correcteur; // and now, magic !

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

			mat->setMatrix(matrixS * matrixRot * matrixRotBis * matrixTrans * matrixR * matrixT * matrix90);
			mat1->setMatrix(matrixS1 * matrixRot1 * matrixRot12 * matrixTrans1 * matrixR * matrixT * matrix90);
			mat2->setMatrix(matrixS2 * matrixTrans2 * matrixR * matrixT * matrix90);
			mat3->setMatrix(matrixS3 * matrixRot3 * matrixTrans3 * matrixR * matrixT * matrix90);
		}

		backgroundImage->dirty();

		compositeViewer.frame();

	}while(!compositeViewer.done());
}
