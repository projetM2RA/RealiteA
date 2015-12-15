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

#include <osgViewer/Viewer>
#include <osg/Material>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Camera>
#include <osg/Texture2D>
#include <osg/TextureRectangle>
#include <osgGA/TrackballManipulator>
#include <osg/Array>

#include <opencv2/opencv.hpp>

#include <stdio.h>

#define COLCHESSBOARD   9
#define ROWCHESSBOARD   5
#define SIZEMIRE		25

#define NEAR			0.1f
#define FAR				100.0f


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

std::vector<cv::Point2f> dessinerRepere(cv::Mat* imCalibColor, const std::vector<cv::Point3f> & objectPoints, const cv::Mat & rotVec, const cv::Mat & tvecs, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs)
{   
	std::vector<cv::Point2f> imagePoints;
	//Projection
	cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

	// Dessin des points projetes
	for(int m = 0; m < objectPoints.size(); m++)
		cv::circle(*imCalibColor, cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0, 0, 255));

	cv::line(*imCalibColor, imagePoints[0], imagePoints[1], cv::Scalar(0, 255, 0), 2);
	cv::line(*imCalibColor, imagePoints[0], imagePoints[2], cv::Scalar(255, 0, 0), 2);
	cv::line(*imCalibColor, imagePoints[0], imagePoints[3], cv::Scalar(0, 0, 255), 2);

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

osg::Geode* createPyramid()
{
	osg::Geode* pyramidGeode­ = new osg::Geode();
	osg::Geometry* pyramidGeometry = new osg::Geometry();
	pyramidGeode­->addDrawable(pyramidGeometry);

	// Spécification des vertices:
	osg::Vec3Array* pyramidVertices = new osg::Vec3Array;
	pyramidVertices->push_back(osg::Vec3(0, 0, 0)); // Devant gauche 
	pyramidVertices->push_back(osg::Vec3(2, 0, 0)); // Devant droite 
	pyramidVertices->push_back(osg::Vec3(2, 2, 0)); // Derrière droite 
	pyramidVertices->push_back(osg::Vec3(0, 2, 0)); // Derrière gauche
	pyramidVertices->push_back(osg::Vec3(1, 1, 2)); // / Sommet

	// Associe cet ensemble de vertices avec pyramidGeometry lui­même
	// associé avec pyramidGeode
	pyramidGeometry->setVertexArray(pyramidVertices);

	// Crée une primitive QUAD pour la base
	osg::DrawElementsUInt* pyramidBase = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
	pyramidBase->push_back(3);
	pyramidBase->push_back(2);
	pyramidBase->push_back(1);
	pyramidBase->push_back(0);

	// L'ajoute à PyramidGeometry: 
	// pyramidGeometry­>addPrimitiveSet(pyramidBase);
	// Le code pour créer d'autres faces ici!
	// (retiré pour raison d'espace, voir tutoriel précédent)

	// Création des couleurs...
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)); //index 0 rouge
	colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)); //index 1 vert
	colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) ); //index 2 bleu
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); //index 3 blanc

	osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType,4,4> *colorIndexArray;
	colorIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType,4,4>;

	colorIndexArray->push_back(0); // vertex 0 assigné à la couleur 0
	colorIndexArray->push_back(1); // vertex 1 assigné à la couleur 1
	colorIndexArray->push_back(2); // vertex 2 assigné à la couleur 2
	colorIndexArray->push_back(3); // vertex 3 assigné à la couleur 3
	colorIndexArray->push_back(0); // vertex 4 assigné à la couleur 0

	pyramidGeometry->setColorArray(colors);
	//pyramidGeometry->setColorIndices(colorIndexArray);
	pyramidGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	// Comme le mapping des vertices aux coordonnées de texture est 1:1,
	// nous n'avons pas besoin d'utiliser un index de tableau pour mapper
	// les vertices aux coordonnées de textures. On peut le faire direc­
	// tement avec la méthode 'setTexCoordArray' de la classe Geometry.
	// Cette méthode prend une variable qui est un tableau de vecteur de
	// deux dimensions (osg::Vec2). Cette variable a besoin d'avoir le
	// même nombre de vertices que dans notre Geometry. Chaque élément du
	// tableau définit les coordonnées de textures pour correspondre aux
	// vertices dans le tableau de vertices.

	osg::Vec2Array* texcoords = new osg::Vec2Array(5);
	(*texcoords)[0].set(0.00f,0.0f); // coord. de texture pr. vertex 0 
	(*texcoords)[1].set(0.25f,0.0f); // coord. de texture pr. vertex 1 
	(*texcoords)[2].set(0.50f,0.0f); // ""
	(*texcoords)[3].set(0.75f,0.0f); // "" 
	(*texcoords)[4].set(0.50f,1.0f); // ""
	pyramidGeometry->setTexCoordArray(0,texcoords);

	return pyramidGeode­;

}



osg::Geode* createHUD(osg::Image* bgImage, int camWidth, int camHeight, double cx, double cy)
{	
	osg::Geometry* geoQuad = new osg::Geometry; 

	osg::Vec3Array* tabSommet = new osg::Vec3Array; 
	tabSommet->push_back(osg::Vec3(-cx, NEAR, -cy)); 
	tabSommet->push_back(osg::Vec3(camWidth - cx, NEAR, -cy)); 
	tabSommet->push_back(osg::Vec3(camWidth - cx, NEAR, camHeight - cy)); 
	tabSommet->push_back(osg::Vec3(-cx, NEAR, camHeight - cy)); 
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
	cv::Mat cameraMatrix, distCoeffs;
	
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
	osg::ref_ptr<osg::Geode> cam = createHUD(backgroundImage, vcap.get(CV_CAP_PROP_FRAME_WIDTH), vcap.get(CV_CAP_PROP_FRAME_HEIGHT), cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2));
	osg::ref_ptr<osg::Geode> pyramid = createPyramid();

	// construct the viewer.
	osgViewer::Viewer viewer;

	// add the HUD subgraph.
	group->addChild(cam);

	osg::Matrixd projectionMatrix;

	projectionMatrix.makeFrustum(
		-cameraMatrix.at<double>(0, 2), vcap.get(CV_CAP_PROP_FRAME_WIDTH) - cameraMatrix.at<double>(0, 2),
		-cameraMatrix.at<double>(1, 2), vcap.get(CV_CAP_PROP_FRAME_HEIGHT) - cameraMatrix.at<double>(1, 2),
		NEAR, FAR);

	osg::Vec3d eye(0.0f, 0.0f, 0.0f), target(0.0f, 1.0f, 0.0f), normal(0.0f, 0.0f, 1.0f);

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
	viewer.setSceneData(group.get());
	viewer.setUpViewInWindow(0, 0, 1920, 1080); 
	viewer.setUpViewOnSingleScreen();
	viewer.getCamera()->setProjectionMatrix(projectionMatrix);
	viewer.getCamera()->setViewMatrixAsLookAt(eye, target, normal);
	//viewer.setCameraManipulator(new osgGA::TrackballManipulator);
	viewer.realize();  // set up windows and associated threads.

	do
	{           
		vcap >> *frame;

		backgroundImage->dirty();
		viewer.frame();
	}while(!viewer.done());

}
