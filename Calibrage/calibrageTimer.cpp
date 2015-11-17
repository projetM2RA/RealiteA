#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>

#define NBRIMAGESCALIB	5
#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6

int main()
{
	time_t timer = 0;
	time_t start = clock();
	std::cout << "Debut calibrage\t" << std::endl;

	cv::Mat imCalib[NBRIMAGESCALIB];
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	cameraMatrix.at<double>(0,0) = 1.0;

	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	std::vector<std::vector<cv::Point2f>> chessCornersInit;
	std::vector<std::vector<cv::Point3f>> chessCorners3D;

	// Initialisation des tailles de mires
	for(int i = 0; i < NBRIMAGESCALIB; i++)
	{
		std::vector<cv::Point2f> initCorners(ROWCHESSBOARD * COLCHESSBOARD, cv::Point2f(0, 0));
		chessCornersInit.push_back(initCorners);
	}

	std::cout << "Initialisation des tailles de mires\t" << float(clock()-start)/CLOCKS_PER_SEC << " sec" << std::endl;
	timer = clock();

	// Détection et affichage des points de la mire
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

	std::cout << "Detection des points de la mire\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
	timer = clock();

	// Calcul des coordonnées 3D des points
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

	std::cout << "Calcul des coordonnes 3D\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
	timer = clock();

	// Calibrage de la caméra
	cv::calibrateCamera(chessCorners3D, chessCornersInit, cv::Size(imCalib[0].size()), cameraMatrix, distCoeffs, rvecs, tvecs);
	
	std::cout << "Calibrage de la cam\t" << float(clock()-timer)/CLOCKS_PER_SEC << " sec" << std::endl;
	timer = clock();
	
	// Sauvegarde de la matrice K
    std::string filename = "../rsc/intrinsicMatrix.yml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;

    fs.release();

	std::cout << std::endl << "Temps total pour " << NBRIMAGESCALIB << " images\t" << float(clock()-start)/CLOCKS_PER_SEC << " sec" << std::endl;
	timer = clock();

	std::system("PAUSE");

	return 0;
}