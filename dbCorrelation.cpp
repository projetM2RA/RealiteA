void WebcamDevice::dbCorrelation()
{
    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints1, keypoints2, keypoints3;
    detector.detect(_frameCropped, keypoints1);
    detector.detect(_markersModels[0], keypoints2);
    detector.detect(_markersModels[1], keypoints3);

    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create("SIFT");
    cv::Mat descriptors1, descriptors2, descriptors3;
    descriptor->compute(_frameCropped, keypoints1, descriptors1 );
    descriptor->compute(_markersModels[0], keypoints2, descriptors2 );
    descriptor->compute(_markersModels[1], keypoints3, descriptors3 );

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
}