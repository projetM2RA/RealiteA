#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <fstream>

#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6

int main()
{
	cv::Mat imCalib;
	cv::Mat imCalibColor;
	cv::Mat cameraMatrix, distCoeffs;
	cv::Mat rvecs, tvecs;
	std::vector<cv::Point2f> chessCornersInit;
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point3f> chessCorners3D;
	std::vector<cv::Point3f> objectPoints;

	int stop = 0;

	cv::VideoCapture vcap(0); 
	if(!vcap.isOpened()){
		std::cout << "FAIL!" << std::endl;
		return -1;
	}

	for(int j = 0; j < COLCHESSBOARD; j++)
	{
		for(int k = 0; k < ROWCHESSBOARD; k++)
		{
			cv::Point3f corner(j * 26.0f, k * 26.0f, 0.0f);
			chessCorners3D.push_back(corner);
		}
	}

	// Création des points à projeter
	for(int x = 0; x < COLCHESSBOARD; x++)
		for(int y = 0; y < ROWCHESSBOARD; y++)
			objectPoints.push_back(cv::Point3f(x * 26, y * 26, 0));	

	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	fs.release();	

	//do{
	while(imCalibColor.empty()){
		vcap >> imCalibColor;
		cv::cvtColor(imCalibColor, imCalib, CV_BGR2GRAY);
			
		bool patternfound = cv::findChessboardCorners(imCalib, cv::Size(ROWCHESSBOARD, COLCHESSBOARD), chessCornersInit);

		if(patternfound)
			cv::cornerSubPix(imCalib, chessCornersInit, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		cv::solvePnP(chessCorners3D, chessCornersInit, cameraMatrix, distCoeffs, rvecs, tvecs);

		cv::Mat rotVec(3, 3, CV_64F);
		cv::Rodrigues(rvecs, rotVec);

		cv::Mat matRT = cv::Mat::zeros(4, 4, CV_64F);

		matRT.at<double>(0, 0) = rotVec.at<double>(0, 0);
		matRT.at<double>(0, 1) = rotVec.at<double>(0, 1);
		matRT.at<double>(0, 2) = rotVec.at<double>(0, 2);
		matRT.at<double>(1, 0) = rotVec.at<double>(1, 0);
		matRT.at<double>(1, 1) = rotVec.at<double>(1, 1);
		matRT.at<double>(1, 2) = rotVec.at<double>(1, 2);
		matRT.at<double>(2, 0) = rotVec.at<double>(2, 0);
		matRT.at<double>(2, 1) = rotVec.at<double>(2, 1);
		matRT.at<double>(2, 2) = rotVec.at<double>(2, 2);
		matRT.at<double>(0, 3) = tvecs.at<double>(0);
		matRT.at<double>(1, 3) = tvecs.at<double>(1);
		matRT.at<double>(2, 3) = tvecs.at<double>(2);
		matRT.at<double>(3, 0) = 0;
		matRT.at<double>(3, 1) = 0;
		matRT.at<double>(3, 2) = 0;
		matRT.at<double>(3, 3) = 1;

		// Projection
		cv::projectPoints(objectPoints, rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints);

		// Dessin des points projetés
		for(int m = 0; m < objectPoints.size(); m++)
			cv::circle(imCalibColor, cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0,0,255), 1, 8, 0);
	}

	//} while();

	return 0;
}