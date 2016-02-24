#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <sstream>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>


float cv_distance(cv::Point2f P, cv::Point2f Q)
{
	return sqrt(pow(abs(P.x - Q.x),2) + pow(abs(P.y - Q.y),2)) ; 
}

int main()
{
	cv::Mat imCalibColor;	
	cv::Mat imCalibGray;	
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
	cv::vector<cv::Point2f> pointQR;
	cv::Mat imCalibNext;
	cv::Mat imMQR;
	cv::vector<cv::Mat> tabMarqueur;

	/*cv::vector<cv::Point2f> corners1;
	cv::vector<cv::Point2f> corners2;
	cv::vector<cv::Point2f> corners3;
	cv::vector<cv::Point2f> corners4;
	cv::vector<cv::Point2f> corners5;*/

	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;
	int maxCorners = 600;

	int A = 0, B= 0, C= 0;
	char key;
	int mark;
	bool patternFound = false;
	
	cv::VideoCapture vcap(0/*"../rsc/MQR2.avi"*/);

	for (int i = 1; i < 3; i++)
	{
		std::ostringstream oss;
		oss << "../rsc/QrCodes/Mqr" << i << ".png";
		imMQR = cv::imread(oss.str());
		cv::cvtColor(imMQR, imMQR, CV_BGR2GRAY);
		tabMarqueur.push_back(imMQR);
	}

	do
	{
		while(imCalibColor.empty())
		{
			vcap >> imCalibColor;
		}
		vcap >> imCalibColor;

		cv::Mat edges(imCalibColor.size(),CV_MAKETYPE(imCalibColor.depth(), 1));
		cv::cvtColor(imCalibColor, imCalibGray, CV_BGR2GRAY);
		Canny(imCalibGray, edges, 100 , 200, 3);

		cv::findContours( edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		
		cv::imshow("pointInteret", imCalibColor);

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

			pointQR.push_back(mc[A]);
			cv::circle(imCalibColor, cv::Point(pointQR[0].x, pointQR[0].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
			pointQR.push_back(mc[B]);
			cv::circle(imCalibColor, cv::Point(pointQR[1].x, pointQR[1].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
			pointQR.push_back(mc[C]);
			cv::circle(imCalibColor, cv::Point(pointQR[2].x, pointQR[2].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

			float minDist;
			float dist2;
			cv::Point2f minPoint;

			minDist = sqrt(pow(pointQR[0].x,2)+pow(pointQR[0].y,2));
			minPoint = pointQR[0];

			for (int i = 1; i < 3; i++)
			{
				dist2 = sqrt(pow(pointQR[i].x,2)+pow(pointQR[i].y,2));

				if (dist2 <= minDist)
				{
					minDist = dist2;
					minPoint = pointQR[i];
				}
			}

			float distCrop;
			std::vector<float> tabDistCrop;

			for (int i = 0; i < 3; i++)
			{
				distCrop = sqrt(pow((pointQR[i].x - minPoint.x),2)+pow((pointQR[i].y - minPoint.y),2));
				if (distCrop != 0)
					tabDistCrop.push_back(distCrop);
			}

			cv::Mat imagecropped = imCalibColor;
			cv::Rect ROI(minPoint.x, minPoint.y, tabDistCrop[1], tabDistCrop[0]);
			cv::Mat croppedRef(imagecropped, ROI);
			cv::cvtColor(croppedRef, imagecropped, CV_BGR2GRAY);

			cv::Point2f D(0.0f,0.0f);
			cv::Point2f E(0.0f,0.0f);
			cv::Point2f F(0.0f,0.0f);

			D.x = (mc[A].x + mc[B].x)/2;
			E.x = (mc[B].x + mc[C].x)/2;
			F.x = (mc[C].x + mc[A].x)/2;

			D.y = (mc[A].y + mc[B].y)/2;
			E.y = (mc[B].y + mc[C].y)/2;
			F.y = (mc[C].y + mc[A].y)/2;

			pointQR.push_back(D);
			cv::circle(imCalibColor, cv::Point(pointQR[3].x, pointQR[3].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
			pointQR.push_back(E);
			cv::circle(imCalibColor, cv::Point(pointQR[4].x, pointQR[4].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
			pointQR.push_back(F);
			cv::circle(imCalibColor, cv::Point(pointQR[5].x, pointQR[5].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

			patternFound = true;
			std::cout << "patternfound" << std::endl;
			
			cv::SiftFeatureDetector detector;
			cv::vector<cv::KeyPoint> keypoints1, keypoints2, keypoints3;
			detector.detect(imagecropped, keypoints1);
			detector.detect(tabMarqueur[0], keypoints2);
			detector.detect(tabMarqueur[1], keypoints3);
			

			cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create("SIFT");
			cv::Mat descriptors1, descriptors2, descriptors3, descriptors4, descriptors5;
			descriptor->compute(imagecropped, keypoints1, descriptors1 );
			descriptor->compute(tabMarqueur[0], keypoints2, descriptors2 );
			descriptor->compute(tabMarqueur[1], keypoints3, descriptors3 );

			cv::Mat imgout1; 
			cv::Mat imgout2;  
			cv::FlannBasedMatcher matcher; 
			std::vector< cv::DMatch > matches; 
			std::vector< cv::DMatch > good_matches1;
			std::vector< cv::DMatch > good_matches2;
			double max_dist = 0; double min_dist = 100;

			matcher.match( descriptors2, descriptors1, matches );
	
			for( int i = 0; i < descriptors2.rows; i++ ) 
			{ 
				double dist = matches[i].distance; 
				if( dist < min_dist ) min_dist = dist; 
				if( dist > max_dist ) max_dist = dist; 
			}


			for( int i = 0; i < descriptors2.rows; i++ ) 
				if( matches[i].distance <= 2*min_dist ) 
					good_matches1.push_back( matches[i]); 
			
			//drawMatches(tabMarqueur[0], keypoints2, imagecropped, keypoints1, good_matches1, imgout1); 


			matcher.match( descriptors3, descriptors1, matches );
	
			for( int i = 0; i < descriptors3.rows; i++ ) 
			{ 
				double dist = matches[i].distance; 
				if( dist < min_dist ) min_dist = dist; 
				if( dist > max_dist ) max_dist = dist; 
			}


			for( int i = 0; i < descriptors3.rows; i++ ) 
				if( matches[i].distance <= 2*min_dist ) 
					good_matches2.push_back( matches[i]); 
			
			//drawMatches(tabMarqueur[1], keypoints3, imagecropped, keypoints1, good_matches2, imgout2); 

			if(good_matches1.size() > good_matches2.size())
				std::cout << "cerveau" << std::endl;
			else
				std::cout << "os" << std::endl;

			break;
		}

		key = (char)cv::waitKey(67);
	}while(patternFound != true && key != 27);

	if(patternFound)
		imCalibNext = imCalibColor;
	
	return patternFound;

}