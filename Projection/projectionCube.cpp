#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <fstream>

#define NBRIMAGESCALIB	17
#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6

int main()
{
	cv::Mat imCalib[NBRIMAGESCALIB];
	cv::Mat cameraMatrix, distCoeffs;

	cv::Mat rvecs, tvecs;
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point3f> objectPoints;

	// Projection

	objectPoints.push_back(cv::Point3f(50,25,0));
	objectPoints.push_back(cv::Point3f(150,25,0));
	objectPoints.push_back(cv::Point3f(150,125,0));
	objectPoints.push_back(cv::Point3f(50,125,0));
	objectPoints.push_back(cv::Point3f(50,25,100));
	objectPoints.push_back(cv::Point3f(150,25,100));
	objectPoints.push_back(cv::Point3f(150,125,100));
	objectPoints.push_back(cv::Point3f(50,125,100));

	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	fs.release();

	for(int i = 0; i < NBRIMAGESCALIB; i++)
	{
		std::ostringstream oss;
		oss << "../rsc/mires/mire" << i + 1 << ".png";
		imCalib[i] = cv::imread(oss.str());
		//cv::cvtColor(imCalib[i], imCalib[i], CV_BGR2GRAY);
	
		std::ostringstream filename;
		filename << "../rsc/mires/RTMatrix" << i + 1 << ".yml";
		cv::FileStorage fs(filename.str(), cv::FileStorage::READ);

		fs["matriceR"] >> rvecs;
		fs["matriceT"] >> tvecs;

		fs.release();

		cv::projectPoints(objectPoints, rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints);

		// Dessin

		cv::line(imCalib[i], imagePoints[0], imagePoints[4], cv::Scalar(255,255,0), 2, 8);
		cv::line(imCalib[i], imagePoints[1], imagePoints[5], cv::Scalar(255,255,0), 2, 8);
		cv::line(imCalib[i], imagePoints[2], imagePoints[6], cv::Scalar(255,255,0), 2, 8);
		cv::line(imCalib[i], imagePoints[3], imagePoints[7], cv::Scalar(255,255,0), 2, 8);

		cv::line(imCalib[i], imagePoints[0], imagePoints[1], cv::Scalar(255,0,255), 2, 8);
		cv::line(imCalib[i], imagePoints[1], imagePoints[2], cv::Scalar(255,0,255), 2, 8);
		cv::line(imCalib[i], imagePoints[2], imagePoints[3], cv::Scalar(255,0,255), 2, 8);
		cv::line(imCalib[i], imagePoints[3], imagePoints[0], cv::Scalar(255,0,255), 2, 8);

		cv::line(imCalib[i], imagePoints[4], imagePoints[5], cv::Scalar(0,255,255), 2, 8);
		cv::line(imCalib[i], imagePoints[5], imagePoints[6], cv::Scalar(0,255,255), 2, 8);
		cv::line(imCalib[i], imagePoints[6], imagePoints[7], cv::Scalar(0,255,255), 2, 8);
		cv::line(imCalib[i], imagePoints[7], imagePoints[4], cv::Scalar(0,255,255), 2, 8);
		
		cv::imshow("image", imCalib[i]);

		cv::waitKey(0);
	}

	return 0;
}
