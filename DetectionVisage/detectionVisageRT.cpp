#include <opencv2/opencv.hpp>
#include "C:/Libs/Chehra/Chehra.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>


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


int main()
{
	std::cout << "initialisation de Chehra..." << std::endl;
	Chehra chehra;
	std::cout << "done" << std::endl;

	cv::Mat cameraMatrix;
	cv::Mat	distCoeffs;
	cv::Mat rvecs;
	cv::Mat tvecs;
	cv::Mat imColor;
	cv::Mat imCalibColor;
	cv::Mat points;

	std::vector<cv::Point2f> pointsVisage;
	std::vector<cv::Point2f> pointsImage;
	std::vector<cv::Point3f> pointsVisage3D;
	std::vector<cv::Point3f> objectPointsVisage;
	std::vector<cv::Point3f> cubeObjectPointsVisage;

	bool visageFound = false;

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
		vcap >> imColor;
		cv::cvtColor(imColor, imCalibColor, CV_BGR2GRAY);

		visageFound = chehra.track(imCalibColor);		

		if(visageFound){	
			points = chehra.getTrackedPoints();
			//chehra.drawPoints(imColor);
			if (points.rows == 98) {
				for(int i = 0; i < 49; i++) {  
					pointsVisage.push_back(cv::Point2f(points.at<float>(i,0),points.at<float>(i+49,0)));
					pointsVisage3D.push_back(cv::Point3f(points.at<float>(i,0),points.at<float>(i+49,0),0.f));
					objectPointsVisage.push_back(cv::Point3f(points.at<float>(i,0),points.at<float>(i+49,0),0.f));
				}
			}

			float headW = (pointsVisage[9].x - pointsVisage[0].x);
			float headH = (pointsVisage[37].y - pointsVisage[9].y)*2;

			(cubeObjectPointsVisage).push_back(cv::Point3f((pointsVisage)[0].x-headW,(pointsVisage)[0].y-headH,0.f));
			(cubeObjectPointsVisage).push_back(cv::Point3f((pointsVisage)[9].x+headW,(pointsVisage)[9].y-headH,0.f));		
			(cubeObjectPointsVisage).push_back(cv::Point3f((pointsVisage)[9].x+headW,(pointsVisage)[9].y+headH,0.f));
			(cubeObjectPointsVisage).push_back(cv::Point3f((pointsVisage)[0].x-headW,(pointsVisage)[0].y+headH,0.f));
			(cubeObjectPointsVisage).push_back(cv::Point3f((pointsVisage)[0].x-headW,(pointsVisage)[0].y-headH,sqrt(pow((pointsVisage)[0].y - (pointsVisage)[9].y, 2) + pow((pointsVisage)[0].x - (pointsVisage)[9].x, 2))));
			(cubeObjectPointsVisage).push_back(cv::Point3f((pointsVisage)[9].x+headW,(pointsVisage)[9].y-headH,sqrt(pow((pointsVisage)[0].y - (pointsVisage)[9].y, 2) + pow((pointsVisage)[0].x - (pointsVisage)[9].x, 2))));
			(cubeObjectPointsVisage).push_back(cv::Point3f((pointsVisage)[9].x+headW,(pointsVisage)[9].y+headH,sqrt(pow((pointsVisage)[0].y - (pointsVisage)[9].y, 2) + pow((pointsVisage)[0].x - (pointsVisage)[9].x, 2))));
			(cubeObjectPointsVisage).push_back(cv::Point3f((pointsVisage)[0].x-headW,(pointsVisage)[0].y+headH,sqrt(pow((pointsVisage)[0].y - (pointsVisage)[9].y, 2) + pow((pointsVisage)[0].x - (pointsVisage)[9].x, 2))));

			cv::cornerSubPix(imCalibColor, pointsVisage, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			cv::solvePnP(pointsVisage3D, pointsVisage, cameraMatrix, distCoeffs, rvecs, tvecs);
			cv::Mat rotVec(3, 3, CV_64F);
			cv::Rodrigues(rvecs, rotVec);

			dessinerCube(&imColor, cubeObjectPointsVisage, rotVec, tvecs, cameraMatrix, distCoeffs);
			pointsImage = dessinerPoints(&imColor, objectPointsVisage, rotVec, tvecs, cameraMatrix, distCoeffs);
		}

		pointsVisage.clear();
		pointsImage.clear();
		pointsVisage3D.clear();
		objectPointsVisage.clear();
		cubeObjectPointsVisage.clear();
		visageFound = false;

		cv::imshow("Projection", imColor);
		key = (char)cv::waitKey(67);

	}while(key != 27);
				
		
	

	return 0;
}