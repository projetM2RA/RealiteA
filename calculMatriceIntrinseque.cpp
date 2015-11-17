#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <fstream>

#define NBRIMAGESCALIB	38
#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6

int main()
{
	cv::Mat imCalib[NBRIMAGESCALIB];
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F), distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	cameraMatrix.at<double>(0,0) = 1.0;

	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<std::vector<cv::Point2f>> chessCornersInit;
	std::vector<std::vector<cv::Point3f>> chessCorners3D;

	for(int i = 0; i < NBRIMAGESCALIB; i++)
	{
		std::vector<cv::Point2f> initCorners(ROWCHESSBOARD * COLCHESSBOARD, cv::Point2f(0, 0));
		chessCornersInit.push_back(initCorners);
	}

	for(int i = 0; i < NBRIMAGESCALIB; i++)
	{
		std::ostringstream oss;
		oss << "../rsc/mires/mire" << i + 1 << ".png";
		imCalib[i] = cv::imread(oss.str());
		cv::cvtColor(imCalib[i], imCalib[i], CV_BGR2GRAY);

		bool patternfound = cv::findChessboardCorners(imCalib[i], cv::Size(ROWCHESSBOARD, COLCHESSBOARD), chessCornersInit[i]);

		if(patternfound)
			cv::cornerSubPix(imCalib[i], chessCornersInit[i], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		cv::drawChessboardCorners(imCalib[i], cv::Size(ROWCHESSBOARD, COLCHESSBOARD), cv::Mat(chessCornersInit[i]), patternfound);

		//cv::imshow("image", imCalib[i]);

		//cv::waitKey(0);
	}

	for(int i = 0; i < NBRIMAGESCALIB; i++)
	{
		std::vector<cv::Point3f> initCorners3D;
		for(int j = 0; j < COLCHESSBOARD; j++)
		{
			for(int k = 0; k < ROWCHESSBOARD; k++)
			{
				cv::Point3f corner(j * 26.0f, k * 26.0f, 0.0f);
				initCorners3D.push_back(corner);
			}
		}
		chessCorners3D.push_back(initCorners3D);
	}

	cv::calibrateCamera(chessCorners3D, chessCornersInit, cv::Size(imCalib[0].size()), cameraMatrix, distCoeffs, rvecs, tvecs);
	
    std::string filename = "../rsc/intrinsicMatrix.yml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;

    fs.release();

	return 0;
}