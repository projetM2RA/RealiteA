#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

using namespace cv;
using namespace std;

#define NBIMAGES  10
#define SAVEDPATH "C:/Users/Nico/Desktop/snapshot/"

int main()
{	
	int nbimg = 0;
	int stop = 0;

	do{
		std::ostringstream oss;
		oss << SAVEDPATH << "mire" << nbimg + 1 << ".png";
		
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);

		VideoCapture vcap(0); 
		if(!vcap.isOpened()){
			  cout << "FAIL!" << endl;
			  return -1;
		}
		
		for(;;){
		   Mat out;
		   while (out.empty())
		   {
			   vcap >> out;
		   }		
		   namedWindow("webCamflux",WINDOW_AUTOSIZE);
		   imshow( "webCamflux", out );	   
		   char c = (char)waitKey(33);
		   if(c == 32){
				try {
					imwrite(oss.str(), out, compression_params);
					cout << "Image " << oss.str() << " saved" << endl;
					nbimg++;
					break;
				 }
				catch (runtime_error& ex) {
					fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
					return 1;
				}
		   }
		   else if(c == 27){ 
			   stop = 1;
			   return 1;
		   }
		}
	} while (nbimg < NBIMAGES || stop == 0);

    return 0;
}

