#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace cv;

// ESPACE pour lancer le record - ESCAPE pour stop

int main ()
{
	int record = 0;
	
	VideoCapture vcap(0); 
	if(!vcap.isOpened()){
		  cout << "FAIL!" << endl;
		  return -1;
	}
	
	int framerate = 15;
	
	double width = vcap.get(CV_CAP_PROP_FRAME_WIDTH);
	double height = vcap.get(CV_CAP_PROP_FRAME_HEIGHT);

	Mat emptyMat((int)height, (int)width, CV_8UC1);
	emptyMat = cv::Scalar(0,0,0);
	
	cout << "Camera properties\n";
	cout << "width = " << width << endl <<"height = "<< height << endl;

	VideoWriter video("../rsc/testitest.avi",CV_FOURCC('D','I','V','X'), framerate, cvSize((int)width,(int)height) );

	for(;;)
	{	
		Mat frame;		
		vcap >> frame;		
		
		imshow("moi", frame);
		char c = (char)waitKey(33);
		if(c == 32){
			record = 1;
			cout << "Debut de l'enregistrement" << endl;
		}
		else if(c == 27){
			video << emptyMat;
			break;
		}
		
		if(record == 1)
			video << frame;
	}

	return 0;
}
