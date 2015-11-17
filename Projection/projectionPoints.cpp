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

	// Création des points à projeter
	for(int x=0 ; x<COLCHESSBOARD ; x++)
		for(int y=0 ; y<ROWCHESSBOARD ; y++)
			objectPoints.push_back(cv::Point3f(x*26,y*26,0));	

	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	fs.release();

	for(int i = 0; i < NBRIMAGESCALIB; i++)
	{
		std::ostringstream oss;
		oss << "../rsc/mires/mire" << i + 1 << ".png";
		imCalib[i] = cv::imread(oss.str());
	
		std::ostringstream filename;
		filename << "../rsc/mires/RTMatrix" << i + 1 << ".yml";
		cv::FileStorage fs(filename.str(), cv::FileStorage::READ);

		fs["matriceR"] >> rvecs;
		fs["matriceT"] >> tvecs;

		fs.release();

		// Projection
		cv::projectPoints(objectPoints, rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints);

		// Dessin des points projetés
		for(int m=0 ; m<objectPoints.size() ; m++)
			cv::circle(imCalib[i],cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0,0,255), 1, 8, 0);
		
		cv::imshow("image", imCalib[i]);

		cv::waitKey(0);
	}

	return 0;
}