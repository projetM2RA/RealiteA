#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

#include <Chehra.h>

#define NBIMAGES  20
#define SAVEDPATH "../rsc/visage/"

bool detectVisage(cv::Mat img, Chehra chehra, int nbimg)
{
	cv::Mat imgGray;
	bool visageFound = false;

	cv::destroyWindow("Image " + nbimg+1);
	cv::cvtColor(img, imgGray, CV_BGR2GRAY);
	visageFound = chehra.track(imgGray);
	
	if(visageFound)
	{
		chehra.drawPoints(img);
		imshow( "Sauvegarde", img );
	}

	return visageFound;
}

int main()
{	
	int nbimg = 0;
	int stop = 0;

	std::cout << "initialisation de Chehra..." << std::endl;
	Chehra chehra;
	std::cout << "done" << std::endl;

	VideoCapture vcap(0); 
	if(!vcap.isOpened()){
		 std::cout << "FAIL!" << std::endl;
		 return -1;
	}
		
	char key = 0;

	do{
		std::ostringstream oss;
		oss << SAVEDPATH << "mire" << nbimg + 1 << ".png";
				
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
				
 		Mat img;
		   
		vcap >> img;		  
		imshow( "webCamflux", img );
		key = (char)waitKey(30);		

		if(key == 32){

			bool visageFound = detectVisage(img, chehra, nbimg);

			if(visageFound == false)
			{
				std::cout << "Pas de detection du visage !" << std::endl;
			}
			else
			{
				imwrite(oss.str(), img, compression_params);
				std::cout << "Image " << oss.str() << " saved" << std::endl;
				nbimg++;
				visageFound = false;
			}
		}
	} while (key != 27);

    return 0;
}

