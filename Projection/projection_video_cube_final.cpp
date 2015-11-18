#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>

#define NBRIMAGESCALIB	17
#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6

int main()
{
	time_t timer = 0;
	time_t start = clock();
	time_t startImage = 0;
	std::cout << "Debut projection\t" << std::endl;

	bool patternfound = false;
	int i = 0;

	cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
	cv::Size winSize(31,31);
	
	cv::Mat cameraMatrix, distCoeffs;
	cv::Mat imCalib;
	cv::Mat imCalibColor;
	cv::Mat imCalibNext;
	cv::Mat rvecs, tvecs;
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point3f> objectPoints;
	std::vector<cv::Point2f> chessCornersInit[2];
	std::vector<cv::Point3f> chessCorners3D;

	// Creation des points a projeter
	objectPoints.push_back(cv::Point3f(50,25,0));
	objectPoints.push_back(cv::Point3f(150,25,0));
	objectPoints.push_back(cv::Point3f(150,125,0));
	objectPoints.push_back(cv::Point3f(50,125,0));
	objectPoints.push_back(cv::Point3f(50,25,100));
	objectPoints.push_back(cv::Point3f(150,25,100));
	objectPoints.push_back(cv::Point3f(150,125,100));
	objectPoints.push_back(cv::Point3f(50,125,100));

	// Creation des coins de la mire
	for(int x=0 ; x<COLCHESSBOARD ; x++)
		for(int y=0 ; y<ROWCHESSBOARD ; y++)
			chessCorners3D.push_back(cv::Point3f(x*26.0f,y*26.0f,0.0f));	

	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	fs.release();

	cv::VideoCapture vcap("../rsc/capture.avi"); 
	if(!vcap.isOpened()){
		  std::cout << "FAIL!" << std::endl;
		  return -1;
	}

	do{
		vcap >> imCalibColor;
		cv::imshow("Projection", imCalibColor);
		cv::cvtColor(imCalibColor, imCalib, CV_BGR2GRAY);
		cv::waitKey();
		timer = clock();
		startImage = clock();
	
		patternfound = cv::findChessboardCorners(imCalib, cv::Size(ROWCHESSBOARD, COLCHESSBOARD), chessCornersInit[0], cv::CALIB_CB_FAST_CHECK);
		
		std::cout << "findChessboardCorners\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
		timer = clock(); 
	} while(!patternfound);

	for(;;)
	{		
		vcap >> imCalibColor;		
						
		if(!imCalibNext.empty())
		{
			cv::swap(imCalib, imCalibNext); // copie de l'ancienne image pour le flot optique
			for(size_t c = 0; c < chessCornersInit[0].size(); c++)
				chessCornersInit[0][c] = chessCornersInit[1][c];
			chessCornersInit[1].clear();
		}
		else
			cv::cornerSubPix(imCalib, chessCornersInit[0], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		cv::cvtColor(imCalibColor, imCalibNext, CV_BGR2GRAY);

		std::vector<uchar> status;
		std::vector<float> err;
		cv::calcOpticalFlowPyrLK(imCalib, imCalibNext, chessCornersInit[0], chessCornersInit[1], status, err, winSize, 3, termcrit, 0, 0.0001);

		cv::solvePnP(chessCorners3D, chessCornersInit[0], cameraMatrix, distCoeffs, rvecs, tvecs);

		cv::Mat rotVec(3, 3, CV_64F);
		cv::Rodrigues(rvecs, rotVec);

		//Projection
		cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

		// Dessin des points projetes
		cv::line(imCalibColor, imagePoints[0], imagePoints[4], cv::Scalar(255,255,0), 2, 8);
		cv::line(imCalibColor, imagePoints[1], imagePoints[5], cv::Scalar(255,255,0), 2, 8);
		cv::line(imCalibColor, imagePoints[2], imagePoints[6], cv::Scalar(255,255,0), 2, 8);
		cv::line(imCalibColor, imagePoints[3], imagePoints[7], cv::Scalar(255,255,0), 2, 8);

		cv::line(imCalibColor, imagePoints[0], imagePoints[1], cv::Scalar(255,0,255), 2, 8);
		cv::line(imCalibColor, imagePoints[1], imagePoints[2], cv::Scalar(255,0,255), 2, 8);
		cv::line(imCalibColor, imagePoints[2], imagePoints[3], cv::Scalar(255,0,255), 2, 8);
		cv::line(imCalibColor, imagePoints[3], imagePoints[0], cv::Scalar(255,0,255), 2, 8);

		cv::line(imCalibColor, imagePoints[4], imagePoints[5], cv::Scalar(0,255,255), 2, 8);
		cv::line(imCalibColor, imagePoints[5], imagePoints[6], cv::Scalar(0,255,255), 2, 8);
		cv::line(imCalibColor, imagePoints[6], imagePoints[7], cv::Scalar(0,255,255), 2, 8);
		cv::line(imCalibColor, imagePoints[7], imagePoints[4], cv::Scalar(0,255,255), 2, 8);

		cv::imshow("Projection", imCalibColor);

		cv::waitKey(67);
	}

	return 0;
}