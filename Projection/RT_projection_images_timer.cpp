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

	cv::Mat imCalib[NBRIMAGESCALIB];
	cv::Mat imCalibRS[NBRIMAGESCALIB];
	cv::Mat imCalibColor[NBRIMAGESCALIB];
	cv::Mat cameraMatrix, distCoeffs;

	cv::Mat rvecs, tvecs;
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point3f> objectPoints;
	std::vector<cv::Point2f> chessCornersInit;
	std::vector<cv::Point3f> chessCorners3D;

	// Creation des points a projeter
	for(int x=0 ; x<COLCHESSBOARD ; x++)
		for(int y=0 ; y<ROWCHESSBOARD ; y++)
			objectPoints.push_back(cv::Point3f(x*26.0f,y*26.0f,0.0f));

	// Creation des coins de la mire
	for(int x=0 ; x<COLCHESSBOARD ; x++)
		for(int y=0 ; y<ROWCHESSBOARD ; y++)
			chessCorners3D.push_back(cv::Point3f(x*26.0f,y*26.0f,0.0f));	

	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	fs.release();

	std::cout << "Initialisations\t" << float(clock()-start)/CLOCKS_PER_SEC << " sec" << std::endl;
	timer = clock();

	for(int i = 0; i < NBRIMAGESCALIB; i++)
	{
		timer = clock();
		startImage = clock();

		std::ostringstream oss;
		oss << "../rsc/mires/mire" << i + 1 << ".png";
		imCalibColor[i] = cv::imread(oss.str());
		cv::cvtColor(imCalibColor[i], imCalib[i], CV_BGR2GRAY);
		//cv::resize(imCalibRS[i], imCalib[i], cv::Size(), 0.5, 0.5, CV_INTER_AREA);

		std::cout << std::endl << "Ouverture/conversion image\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
		timer = clock();
	
		/*std::ostringstream filename;
		filename << "../rsc/mires/RTMatrix" << i + 1 << ".yml";
		cv::FileStorage fsRT(filename.str(), cv::FileStorage::WRITE);*/

		bool patternfound = cv::findChessboardCorners(imCalib[i], cv::Size(ROWCHESSBOARD, COLCHESSBOARD), chessCornersInit);

		std::cout << "findChessboardCorners\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
		timer = clock();

		if(patternfound)
			cv::cornerSubPix(imCalib[i], chessCornersInit, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		std::cout << "cornerSubPix\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
		timer = clock();

		cv::solvePnP(chessCorners3D, chessCornersInit, cameraMatrix, distCoeffs, rvecs, tvecs);

		std::cout << "solvePnP\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
		timer = clock();

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

		std::cout << "calcul matrices\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
		timer = clock();
		
		// Projection
		cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

		std::cout << "projectPoints\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
		timer = clock();

		// Dessin des points projetes
		for(int m=0 ; m<objectPoints.size() ; m++)
			cv::circle(imCalibColor[i],cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0,0,255), 1, 8, 0);
		
		cv::imshow("image", imCalibColor[i]);

		std::cout << std::endl << "Temps total pour l'image" << i+1 << " \t" << float(clock()-startImage)/CLOCKS_PER_SEC << " sec" << std::endl;
		timer = clock();
		
		cv::waitKey(0);
	}

	int a;
	std::cin >> a;

	return 0;
}