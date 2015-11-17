#include <opencv2/opencv.hpp>
#include <windows.h>
#include <iostream>
#include <sstream>
#include <fstream>

#define NBRIMAGESCALIB	20
#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6

int main()
{
	cv::Mat imCalib[NBRIMAGESCALIB];
	cv::Mat cameraMatrix, distCoeffs;

	cv::Mat rvecs, tvecs;
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point2f> chessCornersInit;
	std::vector<cv::Point3f> objectPoints;
	std::vector<double> distances;
	std::vector<double> moyDistances;
	double moyFinale = 0;

	// Création des points à projeter
	for(int x = 0; x < COLCHESSBOARD; x++)
		for(int y = 0; y < ROWCHESSBOARD; y++)
			objectPoints.push_back(cv::Point3f(x * 26, y * 26, 0));	

	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	fs.release();

	for(int i = 0; i < NBRIMAGESCALIB; i++)
	{
		std::ostringstream oss;
		oss << "../rsc/mires/mire" << i + 1 << ".png";
		imCalib[i] = cv::imread(oss.str(), CV_8UC1);
	
		std::ostringstream filename;
		filename << "../rsc/mires/RTMatrix" << i + 1 << ".yml";
		cv::FileStorage fs(filename.str(), cv::FileStorage::READ);

		fs["matriceR"] >> rvecs;
		fs["matriceT"] >> tvecs;

		fs.release();

		//coordonnes pixels coins
		cv::findChessboardCorners(imCalib[i], cv::Size(ROWCHESSBOARD, COLCHESSBOARD), chessCornersInit);

		cv::cornerSubPix(imCalib[i], chessCornersInit, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


		// Projection
		cv::projectPoints(objectPoints, rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints);


		//cacul des distances entre projections
		double moy = 0;
		for(int j = 0; j < COLCHESSBOARD * ROWCHESSBOARD; j++)
		{
			double d = sqrt(pow(chessCornersInit[j].y - imagePoints[j].y, 2) + pow(chessCornersInit[j].x - imagePoints[j].x, 2));
			distances.push_back(d);
			moy += d;
			std::cout << "distance point numero " << j << " : " << std::endl
				<< "    subpix : x = " << chessCornersInit[j].x << "    y = " << chessCornersInit[j].y << std::endl
				<< "    projec : x = " << imagePoints[j].x << "    y = " << imagePoints[j].y << std::endl
				<< " distance : " << d << std::endl << std::endl;
		}

		moyDistances.push_back(moy / (COLCHESSBOARD * ROWCHESSBOARD));
		std::cout << std::endl << std::endl << "moyenne ecart points image " << i << " : " << moyDistances[i] << std::endl << std::endl;
		moyFinale += moyDistances[i];

		// Dessin des points projetés
		//for(int m = 0; m < objectPoints.size(); m++)
			//cv::circle(imCalib[i], cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0,0,255), 1, 8, 0);
		
		//cv::imshow("image", imCalib[i]);

		//cv::waitKey(0);
		int a;
		std::cin >> a;
	}

	std::cout << "moyenne sur toutes images : " << moyFinale / NBRIMAGESCALIB << std::endl;
	int a;
	std::cin >> a;

	return 0;
}