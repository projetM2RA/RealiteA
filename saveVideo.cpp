#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace cv;

int main ()
{
	int record = 0;

	VideoCapture vcap(0); 
	if(!vcap.isOpened()){
		  cout << "FAIL!" << endl;
		  return -1;
	}
	
	int framerate = 25;
	
	double width = vcap.get(CV_CAP_PROP_FRAME_WIDTH);
	double height = vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
	
	cout << "Camera properties\n";
	cout << "width = " << width << endl <<"height = "<< height << endl;

	VideoWriter video("../rsc/capture.avi",CV_FOURCC('D','I','V','X'), framerate, cvSize((int)width,(int)height) );

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
		else if(c == 27)
			break;
		
		if(record == 1)
			video << frame;
	}

	return 0;
}
