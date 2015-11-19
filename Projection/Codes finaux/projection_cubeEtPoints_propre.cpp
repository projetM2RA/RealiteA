#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>

#define NBRIMAGESCALIB	17
#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6

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

void dessinerPoints(cv::Mat* imCalibColor, const std::vector<cv::Point3f> & objectPoints, const cv::Mat & rotVec, const cv::Mat & tvecs, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs)
{	
	std::vector<cv::Point2f> imagePoints;
	//Projection
	cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

	// Dessin des points projetes
	for(int m = 0; m < objectPoints.size(); m++)
		cv::circle(*imCalibColor, cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
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
				
		key = (char)cv::waitKey(30);
	}while(!patternFound && key != 27);

	if(patternFound)
		*imCalibNext = imCalibColor;
	
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

int main()
{
	time_t timer = 0;
	time_t start = clock();
	time_t startImage = 0;
	std::cout << "Debut projection\t" << std::endl;

	bool patternfound = false;
	bool reset = false;
	int i = 0;

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
	

	// Creation des points a projeter
	for(int x = 0; x < COLCHESSBOARD; x++)
		for(int y = 0; y < ROWCHESSBOARD; y++)
			objectPoints.push_back(cv::Point3f(x * 26.0f, y * 26.0f, 0.0f));

	// Creation des points a projeter
	cubeObjectPoints.push_back(cv::Point3f(50, 25, 0));
	cubeObjectPoints.push_back(cv::Point3f(150, 25, 0));
	cubeObjectPoints.push_back(cv::Point3f(150, 125, 0));
	cubeObjectPoints.push_back(cv::Point3f(50, 125, 0));
	cubeObjectPoints.push_back(cv::Point3f(50, 25, 100));
	cubeObjectPoints.push_back(cv::Point3f(150, 25, 100));
	cubeObjectPoints.push_back(cv::Point3f(150, 125, 100));
	cubeObjectPoints.push_back(cv::Point3f(50, 125, 100));

	// Creation des coins de la mire
	for(int x = 0; x < COLCHESSBOARD; x++)
		for(int y = 0; y < ROWCHESSBOARD; y++)
			chessCorners3D.push_back(cv::Point3f(x * 26.0f, y * 26.0f, 0.0f));	

	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	fs.release();

	cv::VideoCapture vcap(0); 
	if(!vcap.isOpened())
	{
		  std::cout << "FAIL!" << std::endl;
		  return -1;
	}

	char key = 0;

	do
	{
		std::cout << "recherche de mire" << std::endl;

		bool detectionMire = detecterMire(vcap, &chessCornersInit[1], &imCalibNext);

		std::cout << "mire detectee" << std::endl << std::endl;

		if(!detectionMire)
			break;

		do
		{
			vcap >> imCalibColor;

			cv::Mat rotVec = trackingMire(&imCalibColor, &imCalibNext, &chessCornersInit, &chessCorners3D, &cameraMatrix, &distCoeffs, &tvecs);
			
			dessinerCube(&imCalibColor, cubeObjectPoints, rotVec, tvecs, cameraMatrix, distCoeffs);
			dessinerPoints(&imCalibColor, objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs);

			cv::imshow("Projection", imCalibColor);

			key = (char)cv::waitKey(30);

		}while(key != 27 && key != 32);

		if(key == 32)
		{
			patternfound = false;
			
			imagePoints.clear();
			chessCornersInit[0].clear();
			chessCornersInit[1].clear();
			imCalibNext.release();
		}

	}while(key != 27);

	return 0;
}