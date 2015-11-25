#include <opencv2/opencv.hpp>
#include "C:/Libs/Chehra/Chehra.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>

#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6
#define NBPTSFACE		6

void init()
{

}

void dessinerCube(cv::Mat* imCalibColor, const std::vector<cv::Point3f> & objectPoints, const cv::Mat & rotVec, const cv::Mat & tvecs, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs)
{
	std::vector<cv::Point2f> imagePoints;
	//Projection
	cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

	// Dessin des points projetes
	cv::line(*imCalibColor, imagePoints[0], imagePoints[4], cv::Scalar(255, 255, 0), 2, 8);
	cv::line(*imCalibColor, imagePoints[1], imagePoints[5], cv::Scalar(255, 255, 0), 2, 8);
	cv::line(*imCalibColor, imagePoints[2], imagePoints[6], cv::Scalar(255, 255, 0), 2, 8);
	cv::line(*imCalibColor, imagePoints[3], imagePoints[7], cv::Scalar(255, 255, 0), 2, 8);

	cv::line(*imCalibColor, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 255), 2, 8);
	cv::line(*imCalibColor, imagePoints[1], imagePoints[2], cv::Scalar(255, 0, 255), 2, 8);
	cv::line(*imCalibColor, imagePoints[2], imagePoints[3], cv::Scalar(255, 0, 255), 2, 8);
	cv::line(*imCalibColor, imagePoints[3], imagePoints[0], cv::Scalar(255, 0, 255), 2, 8);

	cv::line(*imCalibColor, imagePoints[4], imagePoints[5], cv::Scalar(0, 255, 255), 2, 8);
	cv::line(*imCalibColor, imagePoints[5], imagePoints[6], cv::Scalar(0, 255, 255), 2, 8);
	cv::line(*imCalibColor, imagePoints[6], imagePoints[7], cv::Scalar(0, 255, 255), 2, 8);
	cv::line(*imCalibColor, imagePoints[7], imagePoints[4], cv::Scalar(0, 255, 255), 2, 8);
}

std::vector<cv::Point2f> dessinerPoints(cv::Mat* imCalibColor, const std::vector<cv::Point3f> & objectPoints, const cv::Mat & rotVec, const cv::Mat & tvecs, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs)
{	
	std::vector<cv::Point2f> imagePoints;
	//Projection
	cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

	// Dessin des points projetes
	for(int m = 0; m < objectPoints.size(); m++)
		cv::circle(*imCalibColor, cv::Point((int)imagePoints[m].x, (int)imagePoints[m].y), 3, cv::Scalar(255, 0, 0), 1, 8, 0);

	return imagePoints;
}

void CallBackMouse(int event, int x, int y, int flags, void* userdata)
{
	cv::Point2f* p = (cv::Point2f*) userdata;

	if  ( event == cv::EVENT_LBUTTONDOWN )
    {
         std::cout << x << ", " << y << std::endl;
		 p->x = (float)x;
		 p->y = (float)y;
    }
     /*else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
          std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
     }
     else if ( event == cv::EVENT_MOUSEMOVE )
     {
          std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;

     }*/
}

bool detecterMire(cv::VideoCapture vcap, std::vector<cv::Point2f> *pointsMire, cv::Mat *imCalibNext)
{
	cv::Mat imCalibColor;
	char key = 0;
	bool patternFound = false;
	do
	{
		vcap >> imCalibColor;
		cv::imshow("Projection", imCalibColor);
		cv::cvtColor(imCalibColor, imCalibColor, CV_BGR2GRAY);

		patternFound = cv::findChessboardCorners(imCalibColor, cv::Size(ROWCHESSBOARD, COLCHESSBOARD), *pointsMire, cv::CALIB_CB_FAST_CHECK);
				
		key = (char)cv::waitKey(67);
	}while(!patternFound && key != 27);

	if(patternFound)
		*imCalibNext = imCalibColor;
	
	return patternFound; // normalement, si echap est presse, patternFound = false
}

bool setFace(cv::VideoCapture vcap, std::vector<cv::Point2f> *pointsVisage, std::vector<cv::Point3f> *pointsVisage3D, std::vector<cv::Point3f> *objectPointsVisage, std::vector<cv::Point3f> *cubeObjectPointsVisage, cv::Mat *imCalibNext)
{
	cv::Mat imCalibColor;
	cv::Mat imCalibGray;
	cv::Point2f p;
	char key = 0;
	bool escape = false;
	bool endFace = false;

	do{
		vcap >> imCalibColor;
		cv::imshow("Projection", imCalibColor);
		cv::cvtColor(imCalibColor, imCalibGray, CV_BGR2GRAY);
	
		key = (char)cv::waitKey(67);
		if(key == 27){
				return endFace;
				break;
			}
	}while(key != 32);

	do{
		for(int i=0 ; i<NBPTSFACE ; i++){
			cv::imshow("Projection", imCalibColor);
			std::cout << "Clic gauche sur le point " << i+1 <<" du visage : ";
			cv::setMouseCallback("Projection", CallBackMouse, &p);			
			key = (char)cv::waitKey(0);
			cv::circle(imCalibColor, p, 3, cv::Scalar(0, 0, 255), 1, 8, 0);
			if(key == 27){
				return endFace;
				break;
			}
			(*pointsVisage).push_back(p);
			(*pointsVisage3D).push_back(cv::Point3f(p.x, p.y, 0.f));	
			(*objectPointsVisage).push_back(cv::Point3f(p.x, p.y, 0.f));
			if(i<4)
				(*cubeObjectPointsVisage).push_back(cv::Point3f(p.x, p.y, 0.f));
		}	
		escape = true;
	} while(escape != true && key != 27);

	for(int i=0 ; i<4 ; i++)
		(*cubeObjectPointsVisage).push_back(cv::Point3f((*cubeObjectPointsVisage)[i].x, (*cubeObjectPointsVisage)[i].y, 100.f));

	endFace = true;

	if(endFace)
		*imCalibNext = imCalibGray;
	
	return endFace;
}

bool detecterVisage(cv::VideoCapture vcap, Chehra *chehra, std::vector<cv::Point2f> *pointsVisage, std::vector<cv::Point3f> *pointsVisage3D, std::vector<cv::Point3f> *objectPointsVisage, std::vector<cv::Point3f> *cubeObjectPointsVisage, cv::Mat *imCalibNext)
{
	cv::Mat imCalibColor;
	cv::Mat points;
	char key = 0;
	bool visageFound = false;
	do
	{
		vcap >> imCalibColor;
		cv::imshow("Projection", imCalibColor);
		cv::cvtColor(imCalibColor, imCalibColor, CV_BGR2GRAY);

		visageFound = (*chehra).track(imCalibColor);		
				
		key = (char)cv::waitKey(67);
	}while(key != 27 && visageFound != true);

	if(visageFound){
		*imCalibNext = imCalibColor;	
		points = (*chehra).getTrackedPoints();
		if (points.rows == 98) {
			for(int i = 0; i < 49; i++) {  
				(*pointsVisage).push_back(cv::Point2f(points.at<float>(i,0),points.at<float>(i+49,0)));
				(*pointsVisage3D).push_back(cv::Point3f(points.at<float>(i,0),points.at<float>(i+49,0),0.f));
				(*objectPointsVisage).push_back(cv::Point3f(points.at<float>(i,0),points.at<float>(i+49,0),0.f));
			}
		}

		float headW = ((*pointsVisage)[9].x - (*pointsVisage)[0].x);
		float headH = ((*pointsVisage)[37].y - (*pointsVisage)[9].y)*2;

		(*cubeObjectPointsVisage).push_back(cv::Point3f((*pointsVisage)[0].x-headW,(*pointsVisage)[0].y-headH,0.f));
		(*cubeObjectPointsVisage).push_back(cv::Point3f((*pointsVisage)[9].x+headW,(*pointsVisage)[9].y-headH,0.f));		
		(*cubeObjectPointsVisage).push_back(cv::Point3f((*pointsVisage)[9].x+headW,(*pointsVisage)[9].y+headH,0.f));
		(*cubeObjectPointsVisage).push_back(cv::Point3f((*pointsVisage)[0].x-headW,(*pointsVisage)[0].y+headH,0.f));
		(*cubeObjectPointsVisage).push_back(cv::Point3f((*pointsVisage)[0].x-headW,(*pointsVisage)[0].y-headH,headW));
		(*cubeObjectPointsVisage).push_back(cv::Point3f((*pointsVisage)[9].x+headW,(*pointsVisage)[9].y-headH,headW));
		(*cubeObjectPointsVisage).push_back(cv::Point3f((*pointsVisage)[9].x+headW,(*pointsVisage)[9].y+headH,headW));
		(*cubeObjectPointsVisage).push_back(cv::Point3f((*pointsVisage)[0].x-headW,(*pointsVisage)[0].y+headH,headW));

	}
	
	return visageFound; 
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

int main()
{
	time_t timer = 0;
	time_t start = clock();
	time_t startImage = 0;
	std::cout << "Debut projection" << std::endl << std::endl;

	bool patternfound = false;
	bool detectVisage = false;
	bool reset = false;
	bool endVideo = false;
	bool resetAuto = false;

	int i = 0;
	int nbImages = 0;
	double moyFinale = 0;

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
	cv::Mat rvecsVisage, tvecsVisage;
	
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point3f> objectPoints;
	std::vector<cv::Point3f> cubeObjectPoints;
	std::vector<cv::Point3f> objectPointsVisage;
	std::vector<cv::Point3f> cubeObjectPointsVisage;
	std::vector<std::vector<cv::Point2f>> chessCornersInit(2);
	std::vector<std::vector<cv::Point2f>> pointsVisageInit(2);
	std::vector<cv::Point3f> chessCorners3D;
	std::vector<cv::Point3f> pointsVisage3D;
	std::vector<double> distances;
	std::vector<double> moyDistances;
	

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

	std::ofstream file;
	file.open ("../rsc/error.txt");
	
	cv::VideoCapture vcap(0);
	if(!vcap.isOpened())
	{
		  std::cout << "FAIL!" << std::endl;
		  return -1;
	}
	
	char key = 0;

	do
	{
		// Détection de mire
		//std::cout << "recherche de mire" << std::endl;

		//bool detectionMire = detecterMire(vcap, &chessCornersInit[1], &imCalibNext);

		//std::cout << "mire detectee" << std::endl << std::endl;

		//if(!detectionMire)
		//	break;
		

		// Pointage du visage
		//detectVisage = setFace(vcap, &pointsVisageInit[1], &pointsVisage3D, &objectPointsVisage, &cubeObjectPointsVisage, &imCalibNext);

		detecterVisage(vcap, &chehra, &pointsVisageInit[1], &pointsVisage3D, &objectPointsVisage, &cubeObjectPointsVisage, &imCalibNext);
		std::cout << "visage detectee" << std::endl << std::endl;

		//if(!detectVisage)
		//	break;

		do
		{
			vcap >> imCalibColor;

			if(imCalibColor.empty()){
				endVideo = true;
				break;
			}

			//cv::Mat rotVec = trackingMire(&imCalibColor, &imCalibNext, &chessCornersInit, &chessCorners3D, &cameraMatrix, &distCoeffs, &tvecs);
			cv::Mat rotVecVisage = trackingMire(&imCalibColor, &imCalibNext, &pointsVisageInit, &pointsVisage3D, &cameraMatrix, &distCoeffs, &tvecsVisage);

			/*dessinerCube(&imCalibColor, cubeObjectPoints, rotVec, tvecs, cameraMatrix, distCoeffs);
			imagePoints = dessinerPoints(&imCalibColor, objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs);*/
			dessinerCube(&imCalibColor, cubeObjectPointsVisage, rotVecVisage, tvecsVisage, cameraMatrix, distCoeffs);
			imagePoints = dessinerPoints(&imCalibColor, objectPointsVisage, rotVecVisage, tvecsVisage, cameraMatrix, distCoeffs);
			
			//std::cout << imagePoints << std::endl;

			//Calcul d'erreur de reprojection
			double moy = 0;
			for(int j = 0; j < pointsVisageInit[1].size() ; j++)
			{
				double d = sqrt(pow(pointsVisageInit[0][j].y - imagePoints[j].y, 2) + pow(pointsVisageInit[0][j].x - imagePoints[j].x, 2));
				distances.push_back(d);
				moy += d;
				/*std::cout << "distance point numero " << j << " : " << std::endl
					<< "    subpix : x = " << chessCornersInit[0][j].x << "    y = " << chessCornersInit[0][j].y << std::endl
					<< "    projec : x = " << imagePoints[j].x << "    y = " << imagePoints[j].y << std::endl
					<< " distance : " << d << std::endl << std::endl;*/
			}

			moyDistances.push_back(moy / pointsVisageInit[1].size());
			//std::cout << std::endl << std::endl << "moyenne ecart points image " << i << " : " << moyDistances[i] << std::endl << std::endl;
			//file << "moyenne ecart points image " << i << " : " << moyDistances[i] << " px" << std::endl;

			if(moyDistances[i] > 8){ // si l'ecart de reproj est trop grand, reset
				resetAuto = true;
				break;
			}

			//moyFinale += moyDistances[i];
			i++;
			nbImages++;

			cv::imshow("Projection", imCalibColor);

			key = (char)cv::waitKey(67);

		}while(key != 27 && key != 32 && resetAuto != true);

		if(key == 32 || resetAuto == true)
		{
			patternfound = false;
			detectVisage = false;
			resetAuto = false;
			i = 0;

			std::cout << "RESET" << std::endl;
			
			imagePoints.clear();
			chessCornersInit[0].clear();
			chessCornersInit[1].clear();
			pointsVisageInit[0].clear();
			pointsVisageInit[1].clear();
			pointsVisage3D.clear();
			objectPointsVisage.clear();
			moyDistances.clear();
			distances.clear();
			imCalibNext.release();
		}

	}while(key != 27 && endVideo != true && detectVisage != true);
		
	//std::cout << "moyenne sur toutes images : " << moyFinale / NBRIMAGESCALIB << std::endl;
	file << std::endl << "moyenne sur toutes images : " << moyFinale / nbImages << " px" << std::endl;
	file.close();

	return 0;
}

			