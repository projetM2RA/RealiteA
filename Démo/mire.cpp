#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Camera>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osg/TextureRectangle>
#include <osg/Array>
#include <osg/ShapeDrawable>

#include <osgViewer/CompositeViewer>

#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <iostream>
#include <sstream>

#define COLCHESSBOARD   9
#define ROWCHESSBOARD   6
#define SIZEMIRE		26
#define PI				3.14159265


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
	//if(dessiner)
	//	for(int m = 0; m < objectPoints.size(); m++)
	//		cv::circle(*imCalibColor, cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
	/*
	cv::circle(*imCalibColor, cv::Point(repereMire[0].x, repereMire[0].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
	cv::circle(*imCalibColor, cv::Point(repereMire[1].x, repereMire[1].y), 3, cv::Scalar(0, 255, 0), 1, 8, 0);
	cv::circle(*imCalibColor, cv::Point(repereMire[2].x, repereMire[2].y), 3, cv::Scalar(255, 0, 0), 1, 8, 0);
	cv::circle(*imCalibColor, cv::Point(repereMire[3].x, repereMire[3].y), 3, cv::Scalar(255, 0, 255), 1, 8, 0);
	
	cv::line(*imCalibColor, repereMire[0], repereMire[1], cv::Scalar(0,255,0), 2, 8);
	cv::line(*imCalibColor, repereMire[0], repereMire[2], cv::Scalar(255,0,0), 2, 8);
	cv::line(*imCalibColor, repereMire[0], repereMire[3], cv::Scalar(255,0,255), 2, 8);
	*/

	return imagePoints;
	//return repereMire;
}

bool detecterMire(cv::Mat* imCalibColor, std::vector<cv::Point2f> *pointsMire, cv::Mat *imCalibNext)
{
	bool patternFound = false;
	cv::Mat imNB;

	cv::cvtColor(*imCalibColor, imNB, CV_BGR2GRAY);

	// D�tection de tous les points de la mire avec OpenCV
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

	// Am�lioration de la pr�cision de d�tection des points de la mire
	cv::cornerSubPix(imCalib, (*chessCornersInit)[0], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	cv::cvtColor(*imCalibColor, *imCalibNext, CV_BGR2GRAY);

	// Flot optique
	std::vector<uchar> status;
	std::vector<float> err;
	cv::calcOpticalFlowPyrLK(imCalib, *imCalibNext, (*chessCornersInit)[0], (*chessCornersInit)[1], status, err, cv::Size(31, 31), 3, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1), 0, 0.0001);

	// Calcul des matrices de calibrage
	cv::solvePnP(*chessCorners3D, (*chessCornersInit)[0], *cameraMatrix, *distCoeffs, rvecs, *tvecs);

	// R�sultat final de la matrice de rotation
	cv::Mat rotVec(3, 3, CV_64F);
	cv::Rodrigues(rvecs, rotVec);

	return rotVec;
}

osg::Geode* createHUD(osg::Image* bgImage, int camWidth, int camHeight, double cx, double cy, double near)
{	
	osg::Geometry* geoQuad = new osg::Geometry; 

	osg::Vec3Array* tabSommet = new osg::Vec3Array; 
	tabSommet->push_back(osg::Vec3(-cx, near, -cy)); 
	tabSommet->push_back(osg::Vec3(camWidth - cx, near, -cy)); 
	tabSommet->push_back(osg::Vec3(camWidth - cx, near, camHeight - cy)); 
	tabSommet->push_back(osg::Vec3(-cx, near, camHeight - cy)); 
	geoQuad->setVertexArray(tabSommet); 

	osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0); 
	primitive->push_back(0); 
	primitive->push_back(1); 
	primitive->push_back(2); 
	primitive->push_back(3); 
	geoQuad->addPrimitiveSet(primitive); 

	// Nous cr�ons ensuite une tableau qui contiendra nos coordonn�es de texture. 
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

	// Nous cr�ons une Texture2D. 
	osg::Texture2D* texture = new osg::Texture2D; 

	// Nous associons notre image � notre objet Texture2D. 
	texture->setImage(bgImage); 
	texture->setResizeNonPowerOfTwoHint(false); 

	// Enfin nous activons les texture de notre objet Geometry � travers l'objet statuts. 
	statuts->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON); 

	return noeudGeo;
}

osg::Node* creerPlan()
{
	// Nous cr�ons un objet Geometry dans lequel nous allons construire notre triangle. 
	osg::Geometry* geoTriangle = new osg::Geometry; 

	// Nous cr�ons un tableau de trois sommets. 
	osg::Vec3Array* tabSommet = new osg::Vec3Array; 
	tabSommet->push_back(osg::Vec3(0, 0, 0)); 
	tabSommet->push_back(osg::Vec3(52, 0, 0)); 
	tabSommet->push_back(osg::Vec3(0, 52, 0)); 

	// Nous ajoutons le tableau de sommet a notre objet Geometry. 
	geoTriangle ->setVertexArray(tabSommet); 

	// Nous cr�ons une primitive Triangle et nous ajoutons les sommets selon leur index dans le tableau tabSommet 
	osg::DrawElementsUInt* pPrimitiveSet = 
		new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 ); 
	pPrimitiveSet->push_back(0); 
	pPrimitiveSet->push_back(1); 
	pPrimitiveSet->push_back(2); 

	// Nous ajoutons notre primitive a notre objet Geometry. 
	geoTriangle->addPrimitiveSet(pPrimitiveSet); 

	// On met en place un tableau de couleurs. Dans notre exemple chaque sommet du triangle aura une couleur diff�rente. 
	osg::Vec4Array* tabCouleur = new osg::Vec4Array; 
	tabCouleur->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)); 
	tabCouleur->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)); 
	tabCouleur->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)); 
	geoTriangle->setColorArray(tabCouleur); 

	// Nous nous assurons que notre triangle utilisera bien une couleur par sommet. 
	geoTriangle->setColorBinding(osg::Geometry::BIND_PER_VERTEX); 

	///////////////////////////////////////////////////////////

	// Nous cr�ons un objet Geometry dans lequel nous allons construire notre triangle. 
	osg::Geometry* geoTriangle2 = new osg::Geometry; 

	// Nous cr�ons un tableau de trois sommets. 
	osg::Vec3Array* tabSommet2 = new osg::Vec3Array; 
	tabSommet2->push_back(osg::Vec3(0, 0, 0)); 
	tabSommet2->push_back(osg::Vec3(52, 0, 0)); 
	tabSommet2->push_back(osg::Vec3(0, 0, 52)); 

	// Nous ajoutons le tableau de sommet a notre objet Geometry. 
	geoTriangle2->setVertexArray(tabSommet2); 

	// Nous cr�ons une primitive Triangle et nous ajoutons les sommets selon leur index dans le tableau tabSommet 
	osg::DrawElementsUInt* pPrimitiveSet2 = 
		new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 ); 
	pPrimitiveSet2->push_back(0); 
	pPrimitiveSet2->push_back(1); 
	pPrimitiveSet2->push_back(2); 

	// Nous ajoutons notre primitive a notre objet Geometry. 
	geoTriangle2->addPrimitiveSet(pPrimitiveSet2); 

	// On met en place un tableau de couleurs. Dans notre exemple chaque sommet du triangle aura une couleur diff�rente. 
	osg::Vec4Array* tabCouleur2 = new osg::Vec4Array; 
	tabCouleur2->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)); 
	tabCouleur2->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)); 
	tabCouleur2->push_back(osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f)); 
	geoTriangle2->setColorArray(tabCouleur2); 

	// Nous nous assurons que notre triangle utilisera bien une couleur par sommet. 
	geoTriangle2->setColorBinding(osg::Geometry::BIND_PER_VERTEX); 

	///////////////////////////////////////////////////////////

	// Nous cr�ons un objet Geometry dans lequel nous allons construire notre triangle. 
	osg::Geometry* geoTriangle3 = new osg::Geometry; 

	// Nous cr�ons un tableau de trois sommets. 
	osg::Vec3Array* tabSommet3 = new osg::Vec3Array; 
	tabSommet3->push_back(osg::Vec3(0, 0, 0)); 
	tabSommet3->push_back(osg::Vec3(0, 52, 0)); 
	tabSommet3->push_back(osg::Vec3(0, 0, 52)); 

	// Nous ajoutons le tableau de sommet a notre objet Geometry. 
	geoTriangle3->setVertexArray(tabSommet3); 

	// Nous cr�ons une primitive Triangle et nous ajoutons les sommets selon leur index dans le tableau tabSommet 
	osg::DrawElementsUInt* pPrimitiveSet3 = 
		new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 ); 
	pPrimitiveSet3->push_back(0); 
	pPrimitiveSet3->push_back(1); 
	pPrimitiveSet3->push_back(2); 

	// Nous ajoutons notre primitive a notre objet Geometry. 
	geoTriangle3->addPrimitiveSet(pPrimitiveSet3); 

	// On met en place un tableau de couleurs. Dans notre exemple chaque sommet du triangle aura une couleur diff�rente. 
	osg::Vec4Array* tabCouleur3 = new osg::Vec4Array; 
	tabCouleur3->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)); 
	tabCouleur3->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)); 
	tabCouleur3->push_back(osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f)); 
	geoTriangle3->setColorArray(tabCouleur3); 

	// Nous nous assurons que notre triangle utilisera bien une couleur par sommet. 
	geoTriangle3->setColorBinding(osg::Geometry::BIND_PER_VERTEX); 

	/*---------------------------------/!\----------------------------------*/ 
	// Nous cr�ons un n�ud g�om�trique afin de stocker notre triangle et nous d�sactivons sa lumi�re. 
	osg::Geode* noeudGeo = new osg::Geode; 
	osg::StateSet* status = noeudGeo->getOrCreateStateSet(); 
	status->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
	noeudGeo->addDrawable(geoTriangle); 
	noeudGeo->addDrawable(geoTriangle2); 
	noeudGeo->addDrawable(geoTriangle3); 
	/*----------------------------------------------------------------------*/ 

	return noeudGeo; 
}



void main()
{
	bool patternfound = false;
	bool reset = false;
	bool resetAuto = false;
	int nbImages = 0;
	double moyFinale = 0;
	bool detectionMire = false;

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
	std::vector<std::vector<cv::Point2f>> chessCornersInit(2);
	std::vector<cv::Point3f> chessCorners3D;
	std::vector<double> distances;
	double moyDistances;

	// Creation des coins de la mire
	for(int x = -COLCHESSBOARD / 2; x < COLCHESSBOARD / 2 + COLCHESSBOARD % 2; x++)
		for(int y = -ROWCHESSBOARD / 2; y < ROWCHESSBOARD / 2 + ROWCHESSBOARD % 2; y++)
			chessCorners3D.push_back(cv::Point3f(x * SIZEMIRE, y * SIZEMIRE, 0.0f));  

	// Creation des points a projeter
	for(int x = -COLCHESSBOARD / 2; x < COLCHESSBOARD / 2 + COLCHESSBOARD % 2; x++)
		for(int y = -ROWCHESSBOARD / 2; y < ROWCHESSBOARD / 2 + ROWCHESSBOARD % 2; y++)
			objectPoints.push_back(cv::Point3f(x * SIZEMIRE, y * SIZEMIRE, 0.0f));

	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	
	double NEAR = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2; // NEAR = distance focale ; si pixels carr�s, fx = fy -> np 
	//mais est g�n�ralement diff�rent de fy donc on prend (pour l'instant) par d�faut la valeur m�diane
	double FAR = 2000 * NEAR; // je sais pas pourquoi. au pif.	

	fs.release();

	std::ofstream file;
	file.open ("../rsc/translation.txt");
	file << "t1" << "       " << "t2" << "       " << "t3" << "       " << "t3/46" << std::endl;

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
	osg::ref_ptr<osg::Geode> cam = createHUD(backgroundImage, vcap.get(CV_CAP_PROP_FRAME_WIDTH), vcap.get(CV_CAP_PROP_FRAME_HEIGHT), cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2), NEAR);
	osg::ref_ptr<osg::Node> objet3D = osgDB::readNodeFile("../rsc/objets3D/osg-data-master/avatar.osg");
	osg::ref_ptr<osg::Node> objet3D2 = osgDB::readNodeFile("../rsc/objets3D/osg-data-master/cow.osgt");
	// avatar.osg : poup�e bien
	// axes.osgt : repere tres pratique
	// bignathan.osgt : petit bonhomme bien
	// cessna.osg : avion sous les z -> pas bien
	// cessnafire.osgt : avion avec particules mais mal rep�r�
	// clock.osgt : rien -> pas bien
	// cow.osg : vache -> pas bien
	// cube_mapped_torus.osgt : rien
	// dumptruck.osgt : notre beau camion -> pas bien
	// fountain : tres bien mettre size � 0.2
	//osg::ref_ptr<osg::Node> objet3D = creerPlan();
	osg::StateSet* obectStateset = objet3D->getOrCreateStateSet();
	obectStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	obectStateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
	osg::ref_ptr<osg::MatrixTransform> mat = new osg::MatrixTransform();

	osg::StateSet* obectStateset2 = objet3D2->getOrCreateStateSet();
	obectStateset2->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	obectStateset2->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
	osg::ref_ptr<osg::MatrixTransform> mat2 = new osg::MatrixTransform();


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
	/*
	osg::Sphere* unitSphere = new osg::Sphere(osg::Vec3(0, 0, 0), 5.0);
	osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(unitSphere);
	osg::ref_ptr<osg::MatrixTransform> sphereXForm = new osg::MatrixTransform();

	osg::Geode* unitSphereGeode = new osg::Geode();
	group->addChild(sphereXForm);

	sphereXForm->addChild(unitSphereGeode);
	unitSphereGeode->addDrawable(unitSphereDrawable);*/
	//osg::StateSet* sphereStateset = unitSphereDrawable->getOrCreateStateSet();
	//sphereStateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	//sphereStateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// construct the viewer.
	osgViewer::CompositeViewer compositeViewer;
	osgViewer::View* viewer = new osgViewer::View;
	osgViewer::View* viewer2 = new osgViewer::View;

	// add the HUD subgraph.
	group->addChild(cam);

	mat->addChild(objet3D);
	group->addChild(mat);

	mat2->addChild(objet3D2);
	group->addChild(mat2);

	osg::Matrixd projectionMatrix;

	projectionMatrix.makeFrustum(
		-cameraMatrix.at<double>(0, 2),		vcap.get(CV_CAP_PROP_FRAME_WIDTH) - cameraMatrix.at<double>(0, 2),
		-cameraMatrix.at<double>(1, 2),		vcap.get(CV_CAP_PROP_FRAME_HEIGHT) - cameraMatrix.at<double>(1, 2),
		NEAR,								FAR);

	double correcteur = (NEAR/2)/(cameraMatrix.at<double>(1, 2)-vcap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);

	osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, FAR, 0.0f), normal(0.0f, 0.0f, 1.0f);

	/*
	std::cout << " matrice projection : " << std::endl;

	for(int i = 0; i < 4; i++)
	{
	for(int j = 0; j < 4; j++)
	{
	std::cout << projectionMatrix(j, i) << "          ";
	}
	std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;
	std::cout << " matrice intrinseque : " << std::endl;

	for(int i = 0; i < 3 ; i++)
	{
	for(int j = 0; j < 3; j++)
	{
	std::cout << cameraMatrix.at<double>(i, j) << "          ";
	}
	std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;

	double left = 0, right = 0, top = 0, bottom = 0, near = 0, far = 0;

	projectionMatrix.getFrustum(left, right, bottom, top, near, far);

	std::cout << " Frustrum : " << std::endl <<
	"left : " << left << "     right : " << right << std::endl <<
	"top  : " << top << "     bottom : " << bottom << std::endl <<
	"near : " << near << "     far : " << far << std::endl << std::endl;

	std::cout << "taille de l'image : " << vcap.get(CV_CAP_PROP_FRAME_WIDTH) << " * " << vcap.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;*/

	// set the scene to render
	viewer->setSceneData(group.get());
	viewer->setUpViewInWindow(0, 0, 1920, 1080); 
	viewer->getCamera()->setProjectionMatrix(projectionMatrix);
	viewer->getCamera()->setViewMatrixAsLookAt(eye, target, normal);

	viewer2->setSceneData(group.get());
	viewer2->setUpViewInWindow(1920 / 2, 0, 1920 / 2, 1080 / 2); 
	viewer2->getCamera()->setProjectionMatrix(projectionMatrix);
	osg::Vec3d eye2(4 * NEAR, 3 * NEAR / 2, 0.0f), target2(0.0f, NEAR / 2, 0.0f), normal2(0.0f, 0.0f, 1.0f);
	//osg::Vec3d eye2(0.0f, NEAR / 4, 0.0f), target2(0.0f,0.0f, 0.0f), normal2(0.0f, 0.0f, 1.0f);
	viewer2->getCamera()->setViewMatrixAsLookAt(eye2, target2, normal2);

	compositeViewer.addView(viewer2);
	compositeViewer.addView(viewer);

	compositeViewer.realize();  // set up windows and associated threads.

	double s = 5;

	osg::Matrixd matrixS; // scale
	matrixS.set(
		s,	0,	0,	0,
		0,	s,	0,	0,
		0,	0,	s,	0,
		0,	0,	0,	1);

	double s2 = 10;

	osg::Matrixd matrixRot;
	matrixRot.makeRotate(osg::Quat(osg::DegreesToRadians(90.0f), osg::Vec3d(0.0, 0.0, 1.0)));

	osg::Matrixd matrixTrans;
	matrixTrans.makeTranslate(-52,-40,0);

	osg::Matrixd matrixS2; // scale
	matrixS2.set(
		s2,	0,	0,	0,
		0,	s2,	0,	0,
		0,	0,	s2,	0,
		0,	0,	0,	1);

	osg::Matrixd matrixRot2;
	matrixRot2.makeRotate(osg::Quat(osg::DegreesToRadians(180.0f), osg::Vec3d(0.0, 0.0, 1.0)));

	osg::Matrixd matrixTrans2;
	matrixTrans2.makeTranslate(52,-52,25);

	do
	{       
		group->removeChild(mat);
		group->removeChild(mat2);
		patternfound = false;
		resetAuto = false;
		detectionMire = false;

		imagePoints.clear();
		chessCornersInit[0].clear();
		chessCornersInit[1].clear();
		moyDistances = 0;
		distances.clear();
		imCalibNext.release();

		std::cout << "recherche de mire" << std::endl;

		do
		{
			vcap >> *frame;
			backgroundImage->dirty();
			detectionMire = detecterMire(frame, &chessCornersInit[1], &imCalibNext);
			compositeViewer.frame();
		}while(!detectionMire && !compositeViewer.done());

		if(compositeViewer.done())
			break;

		std::cout << "mire detectee" << std::endl << std::endl;

		group->addChild(mat);
		group->addChild(mat2);

		do
		{           
			vcap >> *frame;

			cv::Mat rotVec = trackingMire(frame, &imCalibNext, &chessCornersInit, &chessCorners3D, &cameraMatrix, &distCoeffs, &tvecs);

			imagePoints = dessinerPoints(frame, objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, false);

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

			mat->setMatrix(matrixS * matrixRot * matrixTrans * matrixR * matrixT * matrix90);
			mat2->setMatrix(matrixS2 * matrixRot2 * matrixTrans2 * matrixR * matrixT * matrix90);
			//sphereXForm->setMatrix(matrixR * matrixT * matrix90);

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

			backgroundImage->dirty();
			compositeViewer.frame();
		}while(!compositeViewer.done() && !resetAuto);

	}while(!compositeViewer.done());
}