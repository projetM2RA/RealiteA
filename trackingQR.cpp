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

std::vector<cv::Point2f> dessinerPoints(cv::Mat* imCalibColor, const std::vector<cv::Point3f> & objectPoints, const cv::Mat & rotVec, const cv::Mat & tvecs, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs)
{	
	std::vector<cv::Point2f> imagePoints;
	//Projection
	cv::projectPoints(objectPoints, rotVec, tvecs, cameraMatrix, distCoeffs, imagePoints);

	// Dessin des points projetes
	for(int m = 0; m < objectPoints.size(); m++)
		cv::circle(*imCalibColor, cv::Point(imagePoints[m].x, imagePoints[m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

	return imagePoints;
}

bool detecterMire(cv::VideoCapture vcap, std::vector<cv::Point2f> *pointsMire, cv::Mat *imCalibNext)
{
	cv::Mat imCalibColor;
	char key = 0;
	bool patternFound = false;
	do
	{
		while(imCalibColor.empty())
		{
			vcap >> imCalibColor;
		}
		vcap >> imCalibColor;
		cv::imshow("Projection", imCalibColor);
		cv::cvtColor(imCalibColor, imCalibColor, CV_BGR2GRAY);
			
		patternFound = cv::findChessboardCorners(imCalibColor, cv::Size(ROWCHESSBOARD, COLCHESSBOARD), *pointsMire, cv::CALIB_CB_FAST_CHECK);
				
		key = (char)cv::waitKey(67);
	}while(!patternFound && key != 27);

	if(patternFound)
		*imCalibNext = imCalibColor;
	
	return patternFound; // normalement, si echap est presse, patternFound = false
}

bool detecterQR(cv::VideoCapture vcap, std::vector<cv::Point2f> *pointsQR, std::vector<cv::Point3f> *QRpoint3D, std::vector<cv::Point3f> *tabuseless, cv::Mat *imCalibNext)
{
	cv::Mat imCalibColor;	
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
	
	int A = 0, B= 0, C= 0;
	char key;
	int mark;
	bool patternFound = false;

	do
	{
		while(imCalibColor.empty())
		{
			vcap >> imCalibColor;
		}
		vcap >> imCalibColor;

		cv::Mat edges(imCalibColor.size(),CV_MAKETYPE(imCalibColor.depth(), 1));
		cv::imshow("Projection", imCalibColor);
		cv::cvtColor(imCalibColor, imCalibColor, CV_BGR2GRAY);
		Canny(imCalibColor, edges, 100 , 200, 3);

		cv::findContours( edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		mark = 0;

		cv::vector<cv::Moments> mu(contours.size());
  		cv::vector<cv::Point2f> mc(contours.size());

		for( int i = 0; i < contours.size(); i++ )
		{	
			mu[i] = moments( contours[i], false ); 
			mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
		}

		for( int i = 0; i < contours.size(); i++ )
		{
			int k=i;
			int c=0;

			while(hierarchy[k][2] != -1)
			{
				k = hierarchy[k][2] ;
				c = c+1;
			}
			if(hierarchy[k][2] != -1)
			c = c+1;

			if (c >= 5)
			{	
				if (mark == 0)		A = i;
				else if  (mark == 1)	B = i;		// i.e., A is already found, assign current contour to B
				else if  (mark == 2)	C = i;		// i.e., A and B are already found, assign current contour to C
				mark = mark + 1 ;
			}
		} 

		if (A !=0 && B !=0 && C!=0)
		{
			(*pointsQR).push_back(mc[A]);
			(*QRpoint3D).push_back(cv::Point3f(mc[A].x,mc[A].y,0.f));
			(*tabuseless).push_back(cv::Point3f(mc[A].x,mc[A].y,0.f));
			
			(*pointsQR).push_back(mc[B]);
			(*QRpoint3D).push_back(cv::Point3f(mc[B].x,mc[B].y,0.f));
			(*tabuseless).push_back(cv::Point3f(mc[B].x,mc[B].y,0.f));
			
			(*pointsQR).push_back(mc[C]);
			(*QRpoint3D).push_back(cv::Point3f(mc[C].x,mc[C].y,0.f));
			(*tabuseless).push_back(cv::Point3f(mc[C].x,mc[C].y,0.f));
			

			cv::Point2f D(0.0f,0.0f);
			cv::Point2f E(0.0f,0.0f);
			cv::Point2f F(0.0f,0.0f);

			D.x = (mc[A].x + mc[B].x)/2;
			E.x = (mc[B].x + mc[C].x)/2;
			F.x = (mc[C].x + mc[A].x)/2;

			D.y = (mc[A].y + mc[B].y)/2;
			E.y = (mc[B].y + mc[C].y)/2;
			F.y = (mc[C].y + mc[A].y)/2;

			(*pointsQR).push_back(D);
			(*QRpoint3D).push_back(cv::Point3f(D.x,D.y,0.f));
			(*tabuseless).push_back(cv::Point3f(D.x,D.y,0.f));
	
			(*pointsQR).push_back(E);
			(*QRpoint3D).push_back(cv::Point3f(E.x,E.y,0.f));
			(*tabuseless).push_back(cv::Point3f(E.x,E.y,0.f));
			
			(*pointsQR).push_back(F);
			(*QRpoint3D).push_back(cv::Point3f(F.x,F.y,0.f));
			(*tabuseless).push_back(cv::Point3f(F.x,F.y,0.f));
			

			patternFound = true;
			std::cout << "patternfound" << std::endl;

			break;
		}

		key = (char)cv::waitKey(67);
	}while(patternFound != true && key != 27);

	if(patternFound)
		*imCalibNext = imCalibColor;
	
	return patternFound;
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
	bool endVideo = false;
	bool resetAuto = false;
	int i = 0;
	int nbImages = 0;
	double moyFinale = 0;

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
	std::vector<std::vector<cv::Point2f>> QRpointinit(2);
	std::vector<cv::Point3f> QRpoint3D;
	std::vector<cv::Point3f> tabuseless;
	std::vector<cv::Point3f> chessCorners3D;
	std::vector<double> distances;
	std::vector<double> moyDistances;
	

	// Creation des points a projeter
	for(int x = 0; x < COLCHESSBOARD; x++)
		for(int y = 0; y < ROWCHESSBOARD; y++)
			objectPoints.push_back(cv::Point3f(x * 26.0f, y * 26.0f, 0.0f));

	// Creation des points a projeter
	cubeObjectPoints.push_back(cv::Point3f(52, 26, 0));
	cubeObjectPoints.push_back(cv::Point3f(156, 26, 0));
	cubeObjectPoints.push_back(cv::Point3f(156, 128, 0));
	cubeObjectPoints.push_back(cv::Point3f(52, 128, 0));
	cubeObjectPoints.push_back(cv::Point3f(52, 26, 104));
	cubeObjectPoints.push_back(cv::Point3f(156, 26, 104));
	cubeObjectPoints.push_back(cv::Point3f(156, 128, 104));
	cubeObjectPoints.push_back(cv::Point3f(52, 128, 104));

	// Creation des coins de la mire
	for(int x = 0; x < COLCHESSBOARD; x++)
		for(int y = 0; y < ROWCHESSBOARD; y++)
			chessCorners3D.push_back(cv::Point3f(x * 26.0f, y * 26.0f, 0.0f));	

	cv::FileStorage fs("../rsc/intrinsicMatrix.yml", cv::FileStorage::READ);

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	fs.release();

	std::ofstream file;
	file.open ("../rsc/error.txt");

	cv::VideoCapture vcap(0);
	if(!vcap.isOpened())
	{
		  std::cout << "FAIL!" << std::endl;
		  return -1;
	}

	char key = 0;

	do
	{
		/*std::cout << "recherche de mire" << std::endl;

		bool detectionMire = detecterMire(vcap, &chessCornersInit[1], &imCalibNext);

		std::cout << "mire detectee" << std::endl << std::endl;*/

		bool detectionQR = detecterQR(vcap , &QRpointinit[1], &QRpoint3D, &tabuseless , &imCalibNext);

		if(!detectionQR)
			break;

		do
		{
			vcap >> imCalibColor;

			if(imCalibColor.empty()){
				endVideo = true;
				break;
			}

			cv::Mat rotVec = trackingMire(&imCalibColor, &imCalibNext, &QRpointinit, &QRpoint3D, &cameraMatrix, &distCoeffs, &tvecs);

			//dessinerCube(&imCalibColor, cubeObjectPoints, rotVec, tvecs, cameraMatrix, distCoeffs);
			imagePoints = dessinerPoints(&imCalibColor, tabuseless, rotVec, tvecs, cameraMatrix, distCoeffs);

			// Calcul d'erreur de reprojection
			//double moy = 0;
			//for(int j = 0; j < COLCHESSBOARD * ROWCHESSBOARD; j++)
			//{
			//	double d = sqrt(pow(chessCornersInit[0][j].y - imagePoints[j].y, 2) + pow(chessCornersInit[0][j].x - imagePoints[j].x, 2));
			//	distances.push_back(d);
			//	moy += d;
			//	/*std::cout << "distance point numero " << j << " : " << std::endl
			//		<< "    subpix : x = " << chessCornersInit[0][j].x << "    y = " << chessCornersInit[0][j].y << std::endl
			//		<< "    projec : x = " << imagePoints[j].x << "    y = " << imagePoints[j].y << std::endl
			//		<< " distance : " << d << std::endl << std::endl;*/
			//}

			//moyDistances.push_back(moy / (COLCHESSBOARD * ROWCHESSBOARD));
			////std::cout << std::endl << std::endl << "moyenne ecart points image " << i << " : " << moyDistances[i] << std::endl << std::endl;
			//file << "moyenne ecart points image " << i << " : " << moyDistances[i] << " px" << std::endl;

			//if(moyDistances[i] > 2){ // si l'ecart de reproj est trop grand, reset
			//	resetAuto = true;
			//	break;
			//}

			//moyFinale += moyDistances[i];
			//i++;
			//nbImages++;

			cv::imshow("Projection", imCalibColor);

			key = (char)cv::waitKey(67);

		}while(key != 27 && key != 32 && resetAuto != true);

		if(key == 32 || resetAuto == true)
		{
			patternfound = false;
			resetAuto = false;
			i = 0;
			
			imagePoints.clear();
			chessCornersInit[0].clear();
			chessCornersInit[1].clear();
			QRpointinit[0].clear();
			QRpointinit[1].clear();
			QRpoint3D.clear();
			tabuseless.clear();
			moyDistances.clear();
			distances.clear();
			imCalibNext.release();
		}

	}while(key != 27 && endVideo != true);
		
	//std::cout << "moyenne sur toutes images : " << moyFinale / NBRIMAGESCALIB << std::endl;
	file << std::endl << "moyenne sur toutes images : " << moyFinale / nbImages << " px" << std::endl;
	file.close();

	return 0;
}