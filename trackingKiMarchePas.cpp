#include <opencv2/opencv.hpp>

#include <iostream>

cv::Vec3f detectCircle(cv::Mat imgsrc)
{
	cv::Mat imgdst;

	std::vector<cv::Vec3f> circles;
	cv::cvtColor(imgsrc, imgdst, CV_BGR2GRAY);
	cv::GaussianBlur(imgdst, imgdst, cv::Size(9, 9), 2, 2 );
	cv::HoughCircles(imgdst, circles, CV_HOUGH_GRADIENT, 1, imgdst.rows/8, 200, 100, 0, 0 );

	if(!circles.empty())
		return circles[0];

	else
	{
		cv::Vec3f emptyCircle(0, 0, 0);
		return emptyCircle;
	}
}

std::vector<cv::Point2f> detectCircleFeatures(cv::Mat imgsrc, cv::Vec3f circleCoordinates)
{
	cv::Mat imgdst;
	int maxCorner = 25;
	std::vector<cv::Point2f> corners;
	double quality = 0.01, minDistance = 5.0;

	cv::cvtColor(imgsrc, imgdst, CV_BGR2GRAY);
	cv::Rect myROI(circleCoordinates[0] - circleCoordinates[2] - 10,
			circleCoordinates[1] - circleCoordinates[2] - 10,
			2 * circleCoordinates[2] + 20,
			2 * circleCoordinates[2] + 20);

	cv::Mat croppedImage = imgdst(myROI);

	cv::goodFeaturesToTrack(croppedImage, corners, maxCorner, quality, minDistance);

	/// Draw the features detected
	for(size_t i = 0; i < corners.size(); i++)
	{
		corners[i].x += circleCoordinates[0] - circleCoordinates[2] - 10;
		corners[i].y += circleCoordinates[1] - circleCoordinates[2] - 10;
	}

	return corners;
}

void tracking(cv::VideoCapture & vcap)
{
	bool circleDetected = false;
	cv::Mat imgsrc, cur_gray, pre_gray;
	cv::Vec3f circleCoordinates;
	std::vector<cv::Point2f> corners[2];
	cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
	cv::Size subPixWinSize(10,10), winSize(31,31);

	do
	{
		vcap >> imgsrc;

		circleCoordinates = detectCircle(imgsrc);

		if(circleCoordinates[2] != 0)
		{
			circleDetected = true;
			corners[0]= detectCircleFeatures(imgsrc, circleCoordinates);
		}
	}while(!circleDetected);

	cv::cvtColor(imgsrc, pre_gray, CV_BGR2GRAY);

	for(;;)
	{
		vcap >> imgsrc;
		if (imgsrc.empty())
			break;

		if(!cur_gray.empty())
		{
			cv::swap(pre_gray, cur_gray); // copie de l'ancienne image pour le flot optique
			for(size_t i = 0; i < corners[0].size(); i++)
				corners[0][i] = corners[1][i];
			corners[1].clear();
		}

		else
			cv::cornerSubPix(pre_gray, corners[0], subPixWinSize, cv::Size(-1,-1), termcrit);

		cv::cvtColor(imgsrc, cur_gray, CV_BGR2GRAY);

		std::vector<uchar> status;
		std::vector<float> err;
		cv::calcOpticalFlowPyrLK(pre_gray, cur_gray, corners[0], corners[1], status, err, winSize, 3, termcrit, 0, 0.0001);

		/// Draw the features detected
		for(size_t i = 0; i < corners[0].size(); i++)
		{
			cv::Point center(cvRound(corners[0][i].x), cvRound(corners[0][i].y));
		    int radius = 5;
		    // circle center
		    cv::circle(pre_gray, center, 1, cv::Scalar(0,255,0), -1, 8, 0 );
		    // circle outline
		    cv::circle(pre_gray, center, radius, cv::Scalar(0,0,255), 1, 8, 0 );
		}

		/// Draw the features detected
		for(size_t i = 0; i < corners[1].size(); i++)
		{
			cv::Point center(cvRound(corners[1][i].x), cvRound(corners[1][i].y));
		    int radius = 5;
		    // circle center
		    cv::circle(cur_gray, center, 1, cv::Scalar(0,255,0), -1, 8, 0 );
		    // circle outline
		    cv::circle(cur_gray, center, radius, cv::Scalar(0,0,255), 1, 8, 0 );
		}

		cv::imshow("Image entree", imgsrc);
		cv::imshow("pre_gray", pre_gray);
		cv::imshow("cur_gray", cur_gray);

		cv::waitKey(0);
	}
}

int main()
{
	cv::VideoCapture vcap("/home/isen/Vidéos/m1cible.avi");
	if (!vcap.isOpened()) return 1;
	tracking(vcap);
	return 0;
}
