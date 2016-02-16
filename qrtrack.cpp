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
	cv::Mat imQR;
	cv::vector<cv::Mat> tabQR;
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
	
	cv::VideoCapture vcap("../rsc/capture2.avi");

	for (int i = 1; i < 5; i++)
	{
		std::ostringstream oss;
		oss << "../rsc/QrCodes/QR" << i << ".jpg";
		imQR = cv::imread(oss.str());
		cv::cvtColor(imQR, imQR, CV_BGR2GRAY);
		std::cout<< "Bouh!!!!!!" << std::endl;
		tabQR.push_back(imQR);
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

			cv::Mat imagecropped = imCalibColor;
			cv::Rect ROI(280/*pointQR[0].x*/, 260/*pointQR[0].y*/, 253, 218);
			cv::Mat croppedRef(imagecropped, ROI);
			cv::cvtColor(croppedRef, imagecropped, CV_BGR2GRAY);
			cv::threshold(imagecropped, imagecropped, 180, 255, CV_THRESH_BINARY);

			pointQR.push_back(mc[A]);
			cv::circle(imCalibColor, cv::Point(pointQR[0].x, pointQR[0].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
			pointQR.push_back(mc[B]);
			cv::circle(imCalibColor, cv::Point(pointQR[1].x, pointQR[1].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
			pointQR.push_back(mc[C]);
			cv::circle(imCalibColor, cv::Point(pointQR[2].x, pointQR[2].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);

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
			cv::vector<cv::KeyPoint> keypoints1, keypoints2;
			detector.detect(tabQR[3], keypoints1);
			detector.detect(imagecropped, keypoints2);

			cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create("SIFT");
			cv::Mat descriptors1, descriptors2;
			descriptor->compute(tabQR[3], keypoints1, descriptors1 );
			descriptor->compute(imagecropped, keypoints2, descriptors2 );

			cv::FlannBasedMatcher matcher; 
			std::vector< cv::DMatch > matches; 
			matcher.match( descriptors1, descriptors2, matches ); 
			double max_dist = 0; double min_dist = 100;

			for( int i = 0; i < descriptors1.rows; i++ ) 
			{ 
				double dist = matches[i].distance; 
				if( dist < min_dist ) min_dist = dist; 
				if( dist > max_dist ) max_dist = dist; 
			}

			std::vector< cv::DMatch > good_matches;
			for( int i = 0; i < descriptors1.rows; i++ ) 
				if( matches[i].distance <= 2*min_dist ) 
					good_matches.push_back( matches[i]); 
			cv::Mat imgout; 
			drawMatches(tabQR[3], keypoints1, imagecropped, keypoints2, good_matches, imgout); 

			std::vector<cv::Point2f> pt_img1; 
			std::vector<cv::Point2f> pt_img2; 
			for( int i = 0; i < (int)good_matches.size(); i++ ) 
			{ 
				pt_img1.push_back(keypoints1[ good_matches[i].queryIdx ].pt ); 
				pt_img2.push_back(keypoints2[ good_matches[i].trainIdx ].pt ); 
			}
			cv::Mat H = findHomography( pt_img1, pt_img2, CV_RANSAC );

			cv::Mat result; 
			warpPerspective(tabQR[3],result,H,cv::Size(tabQR[3].cols+imagecropped.cols,tabQR[3].rows)); 
			cv::Mat half(result,cv::Rect(0,0,imagecropped.cols,imagecropped.rows)); 
			imagecropped.copyTo(half); 
			imshow( "Result", result );

			break;
		}

		key = (char)cv::waitKey(67);
	}while(patternFound != true && key != 27);

	if(patternFound)
		imCalibNext = imCalibColor;
	
	return patternFound;

}