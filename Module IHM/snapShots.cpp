#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

#include <Chehra.h>

#define COLCHESSBOARD	9
#define ROWCHESSBOARD	6
#define SAVEDPATH "../rsc/mires/"

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

bool detectMire(cv::Mat img, int nbimg)
{
    cv::Mat imgGray;
    std::vector<cv::Point2f> chessCorners;
    bool mireFound = false;

    cv::destroyWindow("Image " + nbimg+1);
    cv::cvtColor(img, imgGray, CV_BGR2GRAY);
    mireFound = cv::findChessboardCorners(imgGray, cv::Size(ROWCHESSBOARD, COLCHESSBOARD), chessCorners);

    if(mireFound)
    {
        for(int m = 0; m < chessCorners.size(); m++)
            cv::circle(img, cv::Point(chessCorners[m].x, chessCorners[m].y), 3, cv::Scalar(0, 0, 255), 1, 8, 0);
        imshow( "Sauvegarde", img );
    }

    return mireFound;
}

int main()
{
    int nbimg = 0;

//    std::cout << "initialisation de Chehra..." << std::endl;
//    Chehra chehra;
//    std::cout << "done" << std::endl;

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

            bool visageFound = detectMire(img, nbimg);

            if(visageFound == false)
            {
                std::cout << "Pas de detection !" << std::endl;
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

